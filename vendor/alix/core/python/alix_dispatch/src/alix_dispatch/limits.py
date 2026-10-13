"""Controller speed limits — what a `v%` is a percentage OF.

The gap this closes: `movel(pose, v, ...)` sends v as a PERCENTAGE, and the
controller scales it against its own TCP speed constraint. A config carrying
`cleaning_line_speed_mps: 0.8` therefore commanded nothing at all — nothing
wrote that number to the arm. Measured on hardware 2026-08-20: `toplid_right`
at v=100 % peaked at 0.495 m/s and cruised at 0.215, against a config claiming
0.8, because the arm's own constraint was sitting near 0.5.

    movej:  effective = v% x realtime_adjust x JOINT speed limit
    movel:  effective = v% x realtime_adjust x TCP  speed constraint

`realtime_adjust` is the pendant's global override. It multiplies everything
and must be verified at 100 % by eye before any timing measurement.

CORRECTION 2026-08-29: this used to say it "is NOT readable through the SDK".
That is too strong — `libalgo_get_rt_global_ratio` is an exported symbol, with
`libalgo_set_global_real_time_speed` beside it. What is VERIFIED is only that
it reads 0 offline, with no controller attached, like every other
controller-resident value in that library. Whether it reflects the PENDANT's
ratio once an arm is connected is UNTESTED — one probe on hardware settles it.
Until then, keep verifying by eye, but do not repeat the claim that no such
call exists.

Ported from `speed_limits.py` (C17), with one addition: every write is READ
BACK — see the note on the ratio below for why a return code is not evidence.

⚠ RealMan advise against changing the TCP values: the defaults are
algorithm-designed and preserve the ability to stop immediately on command.
Raising `line_acc` is the single most effective lever on stroke time, but its
cost in stopping distance has never been measured. Treat that as open.
"""

from __future__ import annotations

import contextlib
from typing import Callable, Dict, List, Optional

#: C2's law: `line_acc = max(1.6, 3v)`, and the same for angular at max(4, 3w).
ACC_SPEED_RATIO = 3.0
LINE_ACC_FLOOR = 1.60
ANGULAR_ACC_FLOOR = 4.00

#: THE MARGIN IS DEFENSIVE, AND THE REASON IS NOT WHAT THIS FILE USED TO SAY.
#:
#: What the vendor publishes (develop.realman-robotics.com/en/robot/json/
#: armConfig/, re-fetched 2026-09-05): "the ratio of the maximum linear
#: acceleration to the maximum linear speed of the end effector must be >= 3,
#: **otherwise, abnormal motion may occur**", and the same sentence for the
#: angular pair. Note both halves: the bound is >= 3, so a pair exactly on 3 is
#: LEGAL; and the stated consequence is abnormal motion, NOT a refusal.
#:
#: This file used to claim the controller "rejects a pair on the boundary with
#: a bare ret=1 ... measured 2026-08-19". That measurement does not exist. The
#: string quoted as the controller's answer — "line_acc 1.800 is below 3 x
#: line_speed 0.600 = 1.800" — is OUR OWN `SystemExit` from
#: `speed_limits.py:150-153`, whose own text says the controller *would* reject
#: it. Our pre-flight guard refused those rungs; the arm was never asked. No
#: setter has ever returned a non-zero code in any run ledger.
#:
#: The margin stays, and so does the read-back, because the correction makes
#: them MORE important, not less: if the controller accepts an out-of-ratio
#: pair, nothing warns you and the failure mode is "abnormal motion" during a
#: stroke. 0.01 is the smallest margin the 2-decimal wire format can carry; at
#: 0.8 m/s it costs under 1 % of acceleration.
LINE_ACC_MARGIN = 0.01

#: Factory defaults, read back on a never-used arm of the same model — and
#: since 2026-08-29 confirmed from a SECOND, independent source: they are the
#: algorithm library's own compiled defaults, readable with no arm attached.
#: `libalgo_get_tcp_config()` after `rm_algo_init_sys_data(RM_MODEL_RM_75_E,
#: RM_MODEL_RM_ISF_E)` returns exactly [0.25, 1.6, 0.6, 4.0] in this order
#: (they live at `robot+0x134..0x140`, and the vendor's own key names
#: `arm_line_speed / arm_line_acc / arm_angular_speed / arm_angular_acc` fix
#: the order). That matches the `limits_before` recorded on hardware in
#: runs/20260820T200357_toplid_right byte for byte, so the constant below is
#: now cross-checked against the library AND a real arm.
FACTORY = {"line_speed": 0.250, "line_acc": 1.600,
           "angular_speed": 0.600, "angular_acc": 4.000}

_GETTERS = {"line_speed": "rm_get_arm_max_line_speed",
            "line_acc": "rm_get_arm_max_line_acc",
            "angular_speed": "rm_get_arm_max_angular_speed",
            "angular_acc": "rm_get_arm_max_angular_acc"}
_SETTERS = {"line_speed": "rm_set_arm_max_line_speed",
            "line_acc": "rm_set_arm_max_line_acc",
            "angular_speed": "rm_set_arm_max_angular_speed",
            "angular_acc": "rm_set_arm_max_angular_acc"}


class LimitError(RuntimeError):
    """A limit could not be set, or could not be verified after setting."""


def line_acc_for(line_speed: float) -> float:
    """C2's linear pair, clear of the ratio boundary."""
    return max(LINE_ACC_FLOOR, ACC_SPEED_RATIO * float(line_speed) + LINE_ACC_MARGIN)


def angular_acc_for(angular_speed: float) -> float:
    """C2's angular pair, clear of the ratio boundary."""
    return max(ANGULAR_ACC_FLOOR,
               ACC_SPEED_RATIO * float(angular_speed) * (1 + 1e-9))


def read(robot) -> Dict[str, Optional[float]]:
    """Everything a commanded `v%` will be scaled against."""
    out: Dict[str, Optional[float]] = {}
    for key, getter in _GETTERS.items():
        try:
            ret, val = getattr(robot, getter)()
            out[key] = float(val) if ret == 0 else None
        except Exception:
            out[key] = None
    return out


def _check_ratio(values: Dict[str, float]) -> None:
    """Refuse a pair below the vendor's ratio, before it reaches the wire.

    The vendor's consequence for acc/speed < 3 is "abnormal motion may occur",
    not a refusal — so this check is the only thing standing between a typo and
    a stroke executed with a misconfigured envelope.
    """
    for spd, acc, what in (("line_speed", "line_acc", "linear"),
                           ("angular_speed", "angular_acc", "angular")):
        v_s, v_a = values.get(spd), values.get(acc)
        if not (isinstance(v_s, float) and isinstance(v_a, float) and v_s > 0):
            continue
        # Compared with a tolerance: an exactly-3x pair does not always divide
        # back to 3 in binary floating point (3 * 0.70 is 2.0999999999999996).
        if v_a / v_s < ACC_SPEED_RATIO * (1 - 1e-9):
            raise LimitError(
                f"{what}: acceleration/speed = {v_a:.3f}/{v_s:.3f} = "
                f"{v_a / v_s:.3f}, below the required {ACC_SPEED_RATIO:.0f}. "
                f"Raise {acc} to at least {ACC_SPEED_RATIO * v_s:.3f}.")


def _write_order(values: Dict[str, float],
                 have: Dict[str, Optional[float]]) -> List[tuple]:
    """The order these writes must go out in, per pair.

    Going UP, acceleration leads: speed first would leave acc/speed below 3
    until the second write lands. Coming DOWN, speed leads, for the same reason
    mirrored. Keys the caller did not pass are not written.
    """
    order: List[tuple] = []
    for spd, acc in (("line_speed", "line_acc"), ("angular_speed", "angular_acc")):
        pair = [(k, values[k]) for k in (spd, acc) if k in values]
        if len(pair) == 2:
            want_s, have_s = float(values[spd]), have.get(spd)
            rising = not isinstance(have_s, float) or want_s > have_s
            order += [pair[1], pair[0]] if rising else [pair[0], pair[1]]
        else:
            order += pair
    # anything outside the two known pairs keeps the caller's own order
    known = {"line_speed", "line_acc", "angular_speed", "angular_acc"}
    order += [(k, v) for k, v in values.items() if k not in known]
    return order


def apply(robot, *, verify: bool = True, tol: float = 1e-3,
          **values: float) -> Dict[str, Optional[float]]:
    """Set limits and return the PREVIOUS values, for restoration.

    These are GLOBAL controller state, shared with the Web GUI and every other
    program on the arm, so a caller that changes them owns putting them back.

    Every write is READ BACK. `ret=0` is not evidence — see LINE_ACC_MARGIN:
    the vendor's documented consequence of a bad pair is "abnormal motion",
    with no promise of a return code, so the only proof a limit took is the
    getter.

    Writes are ORDERED, not iterated in the order the caller happened to pass
    them: raising speed before its acceleration walks the arm through an
    intermediate pair that breaks the ratio, which is exactly the state the
    vendor warns about. `restore()` has always done this; `apply()` did not.
    """
    before = read(robot)
    merged = {k: values.get(k, before.get(k))
              for k in ("line_speed", "line_acc", "angular_speed", "angular_acc")}
    _check_ratio({k: v for k, v in merged.items() if isinstance(v, float)})

    for key in values:
        if key not in _SETTERS:
            raise KeyError(f"not a settable limit: {key!r}")
    for key, want in _write_order(values, before):
        ret = getattr(robot, _SETTERS[key])(float(want))
        if ret != 0:
            raise LimitError(f"{_SETTERS[key]}({want}) -> ret={ret}")
    if not verify:
        return before

    now = read(robot)
    wrong = {k: (values[k], now.get(k)) for k in values
             if not isinstance(now.get(k), float)
             or abs(now[k] - float(values[k])) > tol}
    if wrong:
        detail = ", ".join(f"{k}: asked {a}, holds {b}" for k, (a, b) in wrong.items())
        raise LimitError(
            f"the controller did not take the limits it accepted — {detail}. "
            "A bare ret=0 with the old value still in force is the documented "
            "failure here, which is why this reads back.")
    return before


def restore(robot, previous: Optional[Dict[str, Optional[float]]]) -> List[str]:
    """Put the limits back. Never raises — it runs while unwinding.

    Order matters: raise acceleration BEFORE speed and lower speed BEFORE
    acceleration, or an intermediate pair violates acc >= 3 x speed and the
    controller refuses the very write that would have restored it.
    """
    warnings: List[str] = []
    if not previous:
        return warnings
    now = read(robot)
    order = _write_order({k: v for k, v in previous.items()
                          if isinstance(v, float)}, now)
    for key, want in order:
        if not isinstance(want, float):
            continue
        try:
            ret = getattr(robot, _SETTERS[key])(float(want))
            if ret != 0:
                warnings.append(f"{key} not restored to {want} (ret={ret})")
        except Exception as exc:
            warnings.append(f"{key} restore raised {exc!r}")
    return warnings


def describe(limits: Dict[str, Optional[float]], v_pct: int) -> str:
    """What a commanded percentage means in physical units.

    A TARGET, not an achieved speed: the pendant's real-time override
    multiplies it and is unreadable, and a short segment cannot accelerate
    into it (A.1).
    """
    ls = limits.get("line_speed")
    if not isinstance(ls, float):
        return "line speed unreadable"
    return (f"movel at {v_pct}% of a {ls:.3f} m/s constraint = "
            f"{ls * v_pct / 100:.3f} m/s target "
            f"(line_acc {limits.get('line_acc')}, "
            f"angular {limits.get('angular_speed')} rad/s)")


# --------------------------------------------------------------------------
# PER-JOINT VELOCITY CAPS.
#
# The reason these exist here: on `toplid_right` the binding constraint is J1,
# not the TCP. The controller's own redundancy resolution swings the RM75's
# arm angle ~149 deg across one stroke, which costs J1 0.72 deg per mm of tool
# travel and saturates it at 98.7 % — while J4, the one joint the redundancy
# cannot reach, sits at 68 %. Raising `line_speed` raises `line_acc` and scales
# that whole profile up until the controller refuses with a bare ret=1.
#
# Capping J1 asks the controller a question nothing else can: does it RETIME a
# trajectory to respect a joint limit, or does it refuse? The offline solver
# cannot answer — measured 2026-08-20, both its resolvers over-read the
# redundancy-affected joints by 2-14x and only J4 is trustworthy — so the
# controller has to be asked directly.
# --------------------------------------------------------------------------

#: The pendant documents the joint SETTER range as 1-180 deg/s, while J3..J7
#: hold 225. So a joint above 180 can be lowered and then NOT put back through
#: this API. Refuse to touch such a joint rather than strand it degraded.
JOINT_SETTER_MAX_DEG_S = 180.0


def read_joints(robot):
    """`{"joint_speed": [...], "joint_acc": [...]}` — the per-joint envelope."""
    out = {}
    for key, getter in (("joint_speed", "rm_get_joint_max_speed"),
                        ("joint_acc", "rm_get_joint_max_acc")):
        try:
            ret, vals = getattr(robot, getter)()
            out[key] = list(vals) if ret == 0 else None
        except Exception:
            out[key] = None
    return out


def apply_joint_speed(robot, caps, *, verify: bool = True, tol: float = 0.5):
    """Cap named joints' velocity. Returns the PREVIOUS array for restoration.

    *caps* is ``{joint_number_1_to_7: deg_per_s}``.

    Refuses a joint whose CURRENT limit exceeds what the setter can write back,
    because lowering it would leave the arm degraded with no API path to
    restore it. J1 and J2 hold 180 and are safe; J3..J7 hold 225 and are not.
    """
    before = read_joints(robot)
    current = before.get("joint_speed")
    if not current:
        raise LimitError("cannot read the joint speed limits — refusing to "
                         "change one we could not restore")
    for joint, want in caps.items():
        if not 1 <= int(joint) <= 7:
            raise LimitError(f"joint {joint} is not in 1..7")
        was = float(current[int(joint) - 1])
        if was > JOINT_SETTER_MAX_DEG_S + 1e-9:
            raise LimitError(
                f"J{joint} currently holds {was:.0f} deg/s, above the "
                f"{JOINT_SETTER_MAX_DEG_S:.0f} deg/s the setter accepts — "
                "lowering it would leave no way to put it back. Refusing.")
        if float(want) > was:
            raise LimitError(
                f"J{joint}: {want} deg/s is ABOVE its current {was:.0f}. These "
                "limits are the machine's safety configuration; this path only "
                "lowers them.")
        ret = robot.rm_set_joint_max_speed(int(joint), float(want))
        if ret != 0:
            raise LimitError(f"rm_set_joint_max_speed({joint}, {want}) -> ret={ret}")
    if not verify:
        return before
    now = read_joints(robot).get("joint_speed") or []
    wrong = {j: (w, now[int(j) - 1] if len(now) >= int(j) else None)
             for j, w in caps.items()
             if len(now) < int(j) or abs(now[int(j) - 1] - float(w)) > tol}
    if wrong:
        raise LimitError(
            "the controller did not take the joint caps it accepted — "
            + ", ".join(f"J{j}: asked {a}, holds {b}" for j, (a, b) in wrong.items()))
    return before


def restore_joints(robot, previous) -> List[str]:
    """Put every joint speed back. Never raises; reports what did not take."""
    warnings: List[str] = []
    want = (previous or {}).get("joint_speed")
    if not want:
        return warnings
    for i, value in enumerate(want, start=1):
        try:
            ret = robot.rm_set_joint_max_speed(i, float(value))
            if ret != 0:
                warnings.append(f"J{i} speed NOT restored to {value} (ret={ret})")
        except Exception as exc:
            warnings.append(f"J{i} speed restore raised {exc!r}")
    now = read_joints(robot).get("joint_speed") or []
    for i, value in enumerate(want, start=1):
        if len(now) >= i and abs(now[i - 1] - float(value)) > 0.5:
            warnings.append(f"J{i} reads {now[i-1]} after restoring {value}")
    return warnings


# ---------------------------------------------------------------------------
# The envelope a TRACK REPLAY runs under
# ---------------------------------------------------------------------------
#: Newton's ruling, 2026-09-05: "we can set them higher when replaying a plan,
#: say, linear cap at 1.5 m/s, and angular to 5 rad/s, of course the
#: acceleration has to be more than 3x, we will use 4x".
#:
#: What the vendor publishes about these four numbers, checked 2026-09-05:
#:   * NO RANGE. develop.realman-robotics.com/en/robot/json/armConfig/ gives
#:     units and a 0.001 resolution for all four and specifies no minimum or
#:     maximum; neither does rm_interface.h:466-517, nor any pendant page.
#:   * The ratio law, quoted: "the ratio of the maximum linear acceleration to
#:     the maximum linear speed of the end effector must be >= 3, otherwise,
#:     abnormal motion may occur", with the same sentence for the angular pair.
#:     4x clears it.
#:   * 1.8 m/s is the RM75's BODY spec ("TCP Line Speed/m/s <=1.8", Basic
#:     Parameters, beside payload 5 kg and reach 610 mm) — a machine maximum,
#:     not an API cap. 1.5 sits under it.
#: The angular pair has no published ceiling of any kind, so 5.0 rad/s is
#: bounded by nothing but this project's judgement.
REPLAY_LINE_SPEED = 1.5          # m/s
REPLAY_ANGULAR_SPEED = 5.0       # rad/s
REPLAY_ACC_RATIO = 4.0           # Newton's 4x, against the vendor's >= 3


def replay_caps(line_speed: float = REPLAY_LINE_SPEED,
                angular_speed: float = REPLAY_ANGULAR_SPEED,
                ratio: float = REPLAY_ACC_RATIO) -> Dict[str, float]:
    """The four values, with each acceleration derived from its own speed."""
    if ratio < ACC_SPEED_RATIO:
        raise LimitError(f"ratio {ratio} is below the vendor's minimum "
                         f"{ACC_SPEED_RATIO:.0f} (armConfig: 'abnormal motion may occur')")
    return {"line_speed": float(line_speed),
            "line_acc": round(float(line_speed) * ratio, 3),
            "angular_speed": float(angular_speed),
            "angular_acc": round(float(angular_speed) * ratio, 3)}


@contextlib.contextmanager
def guard(robot, caps: Optional[Dict[str, float]],
          log: Callable[[str], None] = print,
          verdict: Optional[Dict[str, object]] = None):
    """Hold a Cartesian envelope for one replay, and ALWAYS put back what the
    controller had.

    These four values are global, persistent, shared with the Web GUI and every
    other program on the arm, and they survive power cycles — so whatever is
    left set is what the next operator silently gets. Nothing else on the
    replay path restores them.

    `caps=None` is a no-op, so a caller can turn the whole thing off without
    branching. Yields the previous values (None when disabled).

    THE RESTORE VERDICT IS RETURNED, NOT ONLY PRINTED (H31, 2026-09-08). Pass
    a dict as *verdict* and it is filled in on the way out with
    ``{restored, wanted, readback, warnings, applied}`` so the caller can put
    a ``restored{...}`` block in the run record. Until 2026-09-08 the only
    evidence a leaked 1.5 m/s / 20 rad/s^2 envelope had ever been put back was
    a console line, and the dual path has no tee: 41 of 41 caps-raising runs
    in the ledger recorded `limits_cached: true` and every one of the 126
    after-snapshots read the same 0.800 the BEFORE snapshot did, because both
    were served from one cache. An out-parameter is used rather than a return
    value because a context manager's ``yield`` value is already spoken for
    (it is `before`, and callers - and test_replay_caps.py - depend on it).

    The previous values are read BEFORE the first write and restored on every
    exit path including a partial apply: `apply()` writes one parameter at a
    time, so a failure on the third write leaves the first two standing, and a
    restore that only runs on success would strand them. A cap whose current
    value cannot be READ is refused outright — raising a limit we could never
    put back is the one outcome worth failing the run for.
    """
    if verdict is not None:
        verdict.clear()
        verdict.update({"restored": None, "wanted": None, "readback": None,
                        "warnings": [], "applied": dict(caps) if caps else None})
    if not caps:
        if verdict is not None:
            verdict["restored"] = True          # nothing was raised, nothing to undo
        yield None
        return
    before = read(robot)
    missing = [k for k in caps if not isinstance(before.get(k), float)]
    if missing:
        raise LimitError(
            f"cannot read the current {', '.join(sorted(missing))} — refusing to "
            f"raise a limit that could not be put back")
    try:
        apply(robot, verify=True, **caps)
        log("** Cartesian envelope for this replay: "
            + ", ".join(f"{k} {before[k]:g}->{caps[k]:g}" for k in sorted(caps))
            + " (restored after)")
        yield before
    finally:
        warnings = list(restore(robot, before))
        for w in warnings:
            log(f"** {w}")
        now = read(robot)
        back = {k: now.get(k) for k in caps}
        wanted = {k: before[k] for k in caps}
        ok = all(isinstance(v, float) and abs(v - before[k]) <= 1e-3
                 for k, v in back.items())
        if verdict is not None:
            verdict.update({"restored": bool(ok), "wanted": wanted,
                            "readback": back, "warnings": warnings})
        if ok:
            log("** Cartesian envelope restored to "
                + ", ".join(f"{k} {before[k]:g}" for k in sorted(caps)))
        else:
            log(f"** CARTESIAN ENVELOPE NOT RESTORED: wanted {before}, controller "
                f"holds {back} — reset it from the pendant before anyone else runs")
