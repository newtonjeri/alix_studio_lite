"""A recorded Program moved for the UGV's offset (the user, 23 and 25 Nov 2026): "the adjustment of the plans with the
same cartesian magnitude of offset (x, y and the yaw): FK of every individual point, application of the offset, IK of
the solution, ensuring we maintain the same path". For testing first (tools/program_offset.py, its page); the GUI and
the backend later.

THE OFFSET is the UGV's: where it stands now against where it stood when the Program was recorded, in base_link (x
forward, y left, yaw counter-clockwise seen from above, about base_link's origin, the floor point between the wheels).
The commode (or the test box) stays, so the hand's path moves the other way: every hand pose T in base_link becomes
inv(Trans(dx, dy) Rz(yaw)) T. The UGV 20 mm back and 20 mm left -> the path 20 mm forward and 20 mm right.

THE FRAME (the user, 25 Nov: the frame the RealMan applications use): the IK is solved for the vendor's Arm_Tip in the
arm's own base frame (<X>_base_link, the vendor's "Base"). The URDF has no Arm_Tip link: its nearest frame is <X>_Link7,
2.8 mm long along the tool axis (ALIX's d7 NOTE: joint 6 -> flange 0.164 m in the URDF, 0.1612 m on the RM75-6FB), so
Arm_Tip = <X>_Link7 moved 2.8 mm back along its z axis (ARM_TIP_FROM_LINK7). Checked 25 Nov against the vendor's own FK
(rm_algo_forward_kinematics) on both arms at poles 0, 50, 150, 200 and 300 mm, 200 poses each: within 0.0002 mm and
0.0 deg (Link7 itself 2.8 mm off; tests/data/vendor_arm_tip_fk.json pins it). The pole only lifts the arm's base
(right (45, -124, 648.5 + pole) mm, left (45, +124, ...)): the Arm_Tip and the joints relative to the base do not
depend on it.

THE SOLVE, every 5 ms line (study 23 Nov, ALIX Resource/reports/2026-11-23_program_offset_study): the Arm_Tip moved by
that move, written in the arm's base frame at the Program's pole; then the configuration NEAREST THE TAUGHT LINE (in the
joints) that puts the Arm_Tip there: constrained least squares on ALIX's URDF Jacobian, run until the tip is within
0.01 mm / 0.0006 deg AND the arm's shape has settled; seeded with the taught line plus the change the line before took.
Tried and dropped in the study: the vendor's IK at the line's own arm angle (flips the shoulder where the arm angle
passes 0) and the plain IK (the elbow drifts). The timing stays the file's: the same lines, 5 ms apart.

JUDGED on the lines as written (millidegrees), against the taught file itself (hand-made files are often over the
acceleration bar already):
- every line solved: else the Program cannot follow (where, and why: past reach = the arm straight, a joint limit);
- the hand where the moved path puts it (within 0.05 mm, 0.05 deg);
- singularity as defined (ALIX's four vendor types, alix_workspace.redundancy.singular_margin_deg): the smallest margin,
  taught and moved;
- Lite's limit readings (motion.checks): speed, acceleration and joint-limit shares, the 20 Hz jerk, the largest step;
- FOLLOWS: all of that within 10 % of the taught file's readings and inside Lite's hard bars; RETIME: the path is there
  but a share rises more (the timing must be redone); CANNOT: a line unsolved, a joint at its limit, or a step over 10
  deg.
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass, replace

import numpy as np

from .. import vendor_paths

CLOCK_S = 0.005
TOL_M, TOL_RAD, TOL_NULL, MAX_IT, MAX_STEP = 1e-5, 1e-5, 1e-5, 80, 0.1
HAND_OK_MM, HAND_OK_DEG = 0.05, 0.05
RISE = 1.10                   #: a reading more than 10 % above the taught file's: re-time
LIMIT_MARGIN_DEG = 0.5        #: Lite's L1
STEP_MAX_DEG = 10.0           #: Lite's L4
JERK_REF_RAD_S3 = 1500.0      #: ALIX joint_limits.yaml (Lite's C2 reference)
TOUCH_MM_S, TOUCH_MIN_S = 8.0, 0.2   #: a pause of the hand tip: under 8 mm/s (over 100 ms) for at least 0.2 s
TOUCH_SAME_MM = 15.0         #: two pauses this near each other are one touch
#: The vendor's Arm_Tip in the URDF's <X>_Link7 frame: 2.8 mm back along the tool axis, the same orientation (module doc).
ARM_TIP_FROM_LINK7 = np.array([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, -0.0028], [0.0, 0.0, 0.0, 1.0]])
LINK7_FROM_ARM_TIP = np.linalg.inv(ARM_TIP_FROM_LINK7)
SINGULAR_GATE_DEG = 20.0     #: ALIX's generation gate (alix_taskgen.singularity): a margin under it is worth a note


class ProgramError(ValueError):
    """A file or a setting the tool cannot use; the text says which."""


@dataclass(frozen=True)
class UgvOffset:
    """The UGV's offset from where it stood when the Program was recorded (base_link: mm forward, mm left, deg
    counter-clockwise)."""
    dx_mm: float = 0.0
    dy_mm: float = 0.0
    yaw_deg: float = 0.0

    def path_move(self) -> np.ndarray:
        """The 4 x 4 move of the hand path in base_link: inv(Trans(dx, dy) Rz(yaw))."""
        t = math.radians(self.yaw_deg)
        T = np.eye(4)
        T[:2, :2] = [[math.cos(t), -math.sin(t)], [math.sin(t), math.cos(t)]]
        T[0, 3], T[1, 3] = self.dx_mm / 1000.0, self.dy_mm / 1000.0
        return np.linalg.inv(T)

    def words(self) -> str:
        def part(v, pos, neg, unit="mm"):
            return f"{abs(v):g} {unit} {pos if v > 0 else neg}" if v else None
        ugv = [p for p in (part(self.dx_mm, "forward", "back"), part(self.dy_mm, "left", "right"),
                           part(self.yaw_deg, "counter-clockwise", "clockwise", "deg")) if p]
        if not ugv:
            return "no offset: the Program stays as it is"
        M = self.path_move()
        turn = -self.yaw_deg
        path = [p for p in (part(round(M[0, 3] * 1000, 2), "forward", "back"), part(round(M[1, 3] * 1000, 2), "left", "right"))
                if p]
        tail = f" and turns {abs(turn):g} deg {'counter-clockwise' if turn > 0 else 'clockwise'} about the robot's centre" \
            if turn else ""
        return f"UGV {', '.join(ugv)} -> the hand path moves {', '.join(path) or '0 mm'}{tail}"


# ─────────────────────────────────────────────────────────────── the file

def read_program(text: str) -> tuple[np.ndarray, list[dict]]:
    """A Program file (one JSON object a line with "point": seven joints in millidegrees) -> (N x 7 int64, the rows)."""
    rows = []
    for k, line in enumerate(text.splitlines(), 1):
        s = line.strip()
        if not s:
            continue
        try:
            d = json.loads(s)
        except json.JSONDecodeError:
            raise ProgramError(f"Line {k} is not a Program line (JSON with \"point\"): {s[:60]}") from None
        if not isinstance(d, dict) or not isinstance(d.get("point"), list) or len(d["point"]) != 7:
            raise ProgramError(f"Line {k} has no seven-joint \"point\": {s[:60]}")
        rows.append(d)
    if len(rows) < 2:
        raise ProgramError("The file holds fewer than two Program lines.")
    return np.array([d["point"] for d in rows], dtype=np.int64), rows


def write_program(rows: list[dict], md: np.ndarray) -> str:
    """The rows with their points replaced, written as the input was (compact JSON, one a line)."""
    out = []
    for d, q in zip(rows, md):
        e = dict(d)
        e["point"] = [int(v) for v in q]
        out.append(json.dumps(e, separators=(",", ":")))
    return "\n".join(out) + "\n"


# ─────────────────────────────────────────────────────────────── the arm

class Arm:
    """One arm at one pole height: its base in base_link, the IK for its Arm_Tip (through Link7's Jacobian), and its
    hand tip (<X>_tip_frame) for the pictures only."""

    def __init__(self, side: str, pole_m: float):
        if side not in ("left", "right"):
            raise ProgramError("The arm is left or right.")
        if not 0.0 <= pole_m <= 0.30:
            raise ProgramError(f"A pole height of {pole_m * 1000:.0f} mm is outside 0 to 300 mm.")
        vendor_paths.activate()
        from alix_workspace.ik import IKSolver

        from ..server import robot_fk

        m = robot_fk.model()
        pfx = "L" if side == "left" else "R"
        self.side, self.pole_m, self.m = side, pole_m, m
        self.T_base = m.fk(m.chain("base_link", f"{pfx}_base_link"), [pole_m])
        self.solver = IKSolver(m, f"{pfx}_base_link", f"{pfx}_Link7")
        self.ch_tip = m.chain(f"{pfx}_base_link", f"{pfx}_tip_frame")
        self.ch_link7 = m.chain(f"{pfx}_base_link", f"{pfx}_Link7")
        self.ch_elbow = m.chain(f"{pfx}_base_link", f"{pfx}_Link4")
        lo, hi = self.solver.lower, self.solver.upper
        self.lower_deg, self.upper_deg = np.degrees(lo), np.degrees(hi)
        # the reach, from the URDF at the straight arm: the shoulder (Link2's origin, fixed in the arm's base) and the
        # wrist centre (Link6's origin, fixed in the Arm_Tip's frame: 161.2 mm behind it, the RM75's d7); a target wrist
        # farther than the straight arm is past reach
        z = np.zeros(7)
        self.shoulder = m.fk(m.chain(f"{pfx}_base_link", f"{pfx}_Link2"), z[:2])[:3, 3]
        T6 = m.fk(m.chain(f"{pfx}_base_link", f"{pfx}_Link6"), z[:6])
        self.wrist_in_tip = np.linalg.inv(self.arm_tip(z)[0]) @ T6[:, 3]
        self.reach_m = float(np.linalg.norm(T6[:3, 3] - self.shoulder))

    def arm_tip(self, q_deg: np.ndarray) -> np.ndarray:
        """(N, 4, 4): the vendor's Arm_Tip in the arm's base frame (the vendor's Base), as rm_algo_forward_kinematics."""
        return self.m.fk_batch(self.ch_link7, np.radians(np.atleast_2d(q_deg))) @ ARM_TIP_FROM_LINK7

    def fk(self, chain, q_deg: np.ndarray) -> np.ndarray:
        """(N, 4, 4) in base_link."""
        n = sum(1 for j in chain if j.type != "fixed")
        T = self.m.fk_batch(chain, np.radians(np.atleast_2d(q_deg)[:, :n]))
        return self.T_base @ T

    def past_reach(self, T_tip: np.ndarray) -> bool:
        """Whether the Arm_Tip at ``T_tip`` (arm base frame) puts the wrist centre beyond the straight arm."""
        w = (T_tip @ self.wrist_in_tip)[:3]
        return float(np.linalg.norm(w - self.shoulder)) > self.reach_m - 0.0005

    def nearest(self, q_taught: np.ndarray, T_tip: np.ndarray, q_start: np.ndarray) -> tuple[np.ndarray, bool]:
        """The configuration nearest ``q_taught`` (rad) with the Arm_Tip at ``T_tip`` (arm base frame). ALIX's IKSolver
        works on Link7, so the Arm_Tip target is handed to it as the Link7 pose it implies (a fixed 2.8 mm)."""
        T_target = T_tip @ LINK7_FROM_ARM_TIP
        sv, q, I7 = self.solver, q_start.copy(), np.eye(7)
        for _ in range(MAX_IT):
            J, T = sv._jacobian(q)
            e = sv._pose_error(T, T_target)
            Jp = J.T @ np.linalg.inv(J @ J.T + 1e-10 * np.eye(6))
            null = (I7 - Jp @ J) @ (q_taught - q)
            if np.linalg.norm(e[:3]) < TOL_M and np.linalg.norm(e[3:]) < TOL_RAD and np.linalg.norm(null) < TOL_NULL:
                return q, True
            dq = Jp @ e + null
            big = float(np.abs(dq).max())
            if big > MAX_STEP:
                dq *= MAX_STEP / big
            q = np.clip(q + dq, sv.lower, sv.upper)
        J, T = sv._jacobian(q)
        e = sv._pose_error(T, T_target)
        return q, bool(np.linalg.norm(e[:3]) < 1e-4 and np.linalg.norm(e[3:]) < 1e-4)


# ─────────────────────────────────────────────────────────────── readings

def _readings(md: np.ndarray, side: str, arm: Arm) -> dict:
    from alix_workspace.redundancy import singular_margin_deg

    from . import checks, limits as L

    lim = L.load_limits(side)
    u = checks.util_series(md, lim)
    q = md / 1000.0
    out = {}
    for key in ("speed", "acceleration", "joint_limit"):
        vals = [(float(u[f"{key}_J{j}"].max()), j) for j in range(1, 8)]
        top, j = max(vals)
        out[key] = dict(share=round(top, 3), joint=j)
    if len(q) > 4:
        jerk = np.abs(np.diff(checks.bandlimit(q, CLOCK_S), n=3, axis=0)) / CLOCK_S ** 3          # deg/s^3
        share = np.radians(jerk).max(axis=0) / JERK_REF_RAD_S3
        out["jerk"] = dict(share=round(float(share.max()), 3), joint=int(np.argmax(share)) + 1)
    else:
        out["jerk"] = dict(share=0.0, joint=1)
    margin = checks.commanded_margin_deg(q, lim).min(axis=0)
    out["joint_margin_deg"] = dict(min=round(float(margin.min()), 2), joint=int(np.argmin(margin)) + 1)
    out["step_deg"] = round(float(np.abs(np.diff(q, axis=0)).max()), 3)
    sm = singular_margin_deg(q)
    i = int(np.argmin(sm))
    out["singular_margin_deg"] = dict(min=round(float(sm[i]), 2), at_s=round(i * CLOCK_S, 3),
                                      type=_singular_type(q[i]))
    return out


def _singular_type(q: np.ndarray) -> str:
    def at(v, target):
        return abs(abs(v) - target)
    t = [max(at(q[1], 0), at(q[5], 0)), at(q[3], 0), max(at(q[1], 0), at(q[2], 90)), max(at(q[5], 0), at(q[4], 90))]
    return ("type 1 (J2 and J6 at 0)", "type 2 (J4 at 0: the arm straight)", "type 3 (J2 at 0, J3 at 90)",
            "type 4 (J6 at 0, J5 at 90)")[int(np.argmin(t))]


def touch_points(tip_mm: np.ndarray) -> list[tuple[int, int]]:
    """The pauses of the hand tip: [(first line, last line)], each at least TOUCH_MIN_S under TOUCH_MM_S; pauses in
    a row within TOUCH_SAME_MM of each other are one."""
    w = 20
    if len(tip_mm) <= w:
        return []
    sp = np.r_[np.zeros(w), np.linalg.norm(tip_mm[w:] - tip_mm[:-w], axis=1) / (w * CLOCK_S)]
    low, runs, start = sp < TOUCH_MM_S, [], None
    for i, s in enumerate(low):
        if s and start is None:
            start = i
        if (not s or i == len(low) - 1) and start is not None:
            if (i - start) * CLOCK_S >= TOUCH_MIN_S:
                if runs and np.linalg.norm(tip_mm[(start + i) // 2] - tip_mm[sum(runs[-1]) // 2]) < TOUCH_SAME_MM:
                    runs[-1] = (runs[-1][0], i)
                else:
                    runs.append((start, i))
            start = None
    return runs


# ─────────────────────────────────────────────────────────────── the move

def shift_program(md: np.ndarray, side: str, pole_m: float, offset: UgvOffset, progress=None) -> dict:
    """The Program ``md`` (N x 7 millidegrees) moved for the UGV's ``offset``: {"md": the moved lines, "report": ...}."""
    arm = Arm(side, pole_m)
    q0 = md / 1000.0
    n = len(q0)
    D = offset.path_move()
    D_ab = np.linalg.inv(arm.T_base) @ D @ arm.T_base                 # the move seen from the arm's base
    Ttip = arm.arm_tip(q0)                                            # the vendor's Arm_Tip in the arm's base frame
    qr = np.radians(q0)
    new = np.empty_like(q0)
    failed, why = [], {}
    delta = np.zeros(7)
    for i in range(n):
        q, ok = arm.nearest(qr[i], D_ab @ Ttip[i], qr[i] + delta)
        if not ok:
            failed.append(i)
            if arm.past_reach(D_ab @ Ttip[i]):
                why.setdefault("past reach (the arm straight: J4 at 0)", i)
            elif np.any(np.isclose(q, arm.solver.lower) | np.isclose(q, arm.solver.upper)):
                why.setdefault("a joint at its limit", i)
            else:
                why.setdefault("no solution near the taught arm", i)
            new[i] = np.degrees(qr[i] + delta)
            continue
        new[i] = np.degrees(q)
        delta = q - qr[i]
        if progress and i % 500 == 0:
            progress(i, n)
    md_new = np.round(new * 1000.0).astype(np.int64)
    q1 = md_new / 1000.0

    # the Arm_Tip where the moved path puts it, judged on the lines as written
    want = D_ab @ Ttip
    got = arm.arm_tip(q1)
    err_mm = np.linalg.norm(got[:, :3, 3] - want[:, :3, 3], axis=1) * 1000.0
    cosang = np.clip((np.einsum("nij,nij->n", got[:, :3, :3], want[:, :3, :3]) - 1.0) / 2.0, -1.0, 1.0)
    err_deg = np.degrees(np.arccos(cosang))
    ok_lines = np.setdiff1d(np.arange(n), failed)
    tip0 = arm.fk(arm.ch_tip, q0)[:, :3, 3] * 1000.0
    tip1 = arm.fk(arm.ch_tip, q1)[:, :3, 3] * 1000.0
    e0 = arm.fk(arm.ch_elbow, q0)[:, :3, 3] * 1000.0
    e1 = arm.fk(arm.ch_elbow, q1)[:, :3, 3] * 1000.0

    taught, moved = _readings(md, side, arm), _readings(md_new, side, arm)
    touches = []
    for a, b in touch_points(tip0):
        c = (a + b) // 2
        touches.append(dict(from_s=round(a * CLOCK_S, 2), to_s=round(b * CLOCK_S, 2),
                            taught_mm=[round(float(v), 1) for v in tip0[c]], moved_mm=[round(float(v), 1) for v in tip1[c]],
                            change_mm=[round(float(v), 1) for v in tip1[c] - tip0[c]]))
    verdict, reasons = _verdict(taught, moved, failed, why, err_mm[ok_lines], err_deg[ok_lines])
    step = max(1, n // 1500)
    report = dict(
        side=side, pole_mm=round(pole_m * 1000.0, 1), lines=n, seconds=round(n * CLOCK_S, 3),
        ik_frame=f"Arm_Tip (the vendor's) in {arm.side[0].upper()}_base_link: the URDF's {arm.side[0].upper()}_Link7 moved 2.8 mm "
                 "back along its tool axis",
        offset=dict(dx_mm=offset.dx_mm, dy_mm=offset.dy_mm, yaw_deg=offset.yaw_deg), words=offset.words(),
        path_move=dict(dx_mm=round(float(D[0, 3]) * 1000.0, 3), dy_mm=round(float(D[1, 3]) * 1000.0, 3),
                       turn_deg=round(-offset.yaw_deg, 3)),
        verdict=verdict, reasons=reasons,
        lines_unsolved=len(failed),
        first_unsolved=None if not failed else dict(line=failed[0] + 1, t_s=round(failed[0] * CLOCK_S, 3),
                                                     why=min(why, key=why.get)),
        hand_error_mm=round(float(err_mm[ok_lines].max()) if len(ok_lines) else 0.0, 4),
        hand_error_deg=round(float(err_deg[ok_lines].max()) if len(ok_lines) else 0.0, 4),
        joint_change_deg=[round(float(v), 2) for v in np.abs(q1 - q0).max(axis=0)],
        elbow_move_mm=round(float(np.linalg.norm(e1 - e0, axis=1).max()), 1),
        first_line_change_deg=round(float(np.abs(q1[0] - q0[0]).max()), 2),
        last_line_change_deg=round(float(np.abs(q1[-1] - q0[-1]).max()), 2),
        taught=taught, moved=moved, touches=touches,
        picture=dict(t_s=[round(i * CLOCK_S, 3) for i in range(0, n, step)],
                     tip_taught=[[round(float(v), 1) for v in tip0[i]] for i in range(0, n, step)],
                     tip_moved=[[round(float(v), 1) for v in tip1[i]] for i in range(0, n, step)],
                     joints_taught=[[round(float(v), 2) for v in q0[i]] for i in range(0, n, step)],
                     joints_moved=[[round(float(v), 2) for v in q1[i]] for i in range(0, n, step)]),
    )
    return dict(md=md_new, report=report)


def _verdict(taught: dict, moved: dict, failed: list, why: dict, err_mm, err_deg) -> tuple[str, list]:
    if failed:
        where = ", ".join(f"{k} from {v * CLOCK_S:.2f} s" for k, v in sorted(why.items(), key=lambda kv: kv[1]))
        return "CANNOT", [f"{len(failed)} lines have no arm pose near the taught one: {where}"]
    out = []
    if len(err_mm) and (err_mm.max() > HAND_OK_MM or err_deg.max() > HAND_OK_DEG):
        out.append(f"the hand is {err_mm.max():.2f} mm, {err_deg.max():.3f} deg from the moved path")
    if moved["joint_margin_deg"]["min"] < LIMIT_MARGIN_DEG <= taught["joint_margin_deg"]["min"]:
        out.append(f"J{moved['joint_margin_deg']['joint']} comes {moved['joint_margin_deg']['min']:.2f} deg from its limit")
    if moved["step_deg"] > STEP_MAX_DEG:
        out.append(f"a step of {moved['step_deg']:.1f} deg between two lines (the arm's shape jumps)")
    if out:
        return "CANNOT", out
    rises = []
    for k in ("speed", "acceleration", "jerk"):
        a, b = taught[k]["share"], moved[k]["share"]
        if b > RISE * a + 1e-9 and b > 0.05:
            rises.append(f"{k} J{moved[k]['joint']} {a:.2f} -> {b:.2f} of its bar")
    if moved["speed"]["share"] > 1.0 >= taught["speed"]["share"]:
        rises.append(f"J{moved['speed']['joint']} over its speed rating ({moved['speed']['share']:.2f})")
    sm0, sm1 = taught["singular_margin_deg"]["min"], moved["singular_margin_deg"]["min"]
    notes = []
    if sm1 < SINGULAR_GATE_DEG and sm1 < sm0 - 1.0:
        notes.append(f"nearer a singularity than the taught file: {sm1:.1f} deg ({moved['singular_margin_deg']['type']} "
                     f"at {moved['singular_margin_deg']['at_s']:.2f} s) against {sm0:.1f}")
    if rises:
        return "RETIME", rises + notes
    return "FOLLOWS", notes
