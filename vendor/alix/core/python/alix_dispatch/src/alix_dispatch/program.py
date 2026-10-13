"""Task config -> an ordered program of controller calls. ROS-free.

This is mode 1 (contract C16.1): the config IS the program. Every stage of
`stage_sequence` becomes exactly one typed call, in order, and the program is
fully resolved — joint targets in degrees, Cartesian targets in the
CONTROLLER's frame at the pole height in effect, speeds as SDK percentages.

TWO RULES ARE STRUCTURAL HERE, both bought with defects:

1. **An unrecognised stage FAILS.** F81: the reference runner classified any
   stage it did not recognise as "scene", skipped it, and still reported PASS —
   so `flush_press` moved nothing and passed, and bin/glove path phases were
   dropped silently. A skip is a verdict, never a default.

2. **No execution parameter may come from a code default** (C16.4). The
   per-move blend radius `r` has a slot in the config contract and no config
   writes it, so this builder does NOT invent one: moves carry `r=None` and
   the program reports `blend_unspecified`. A caller that wants a blend must
   pass one explicitly and own it.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence

import numpy as np

from alix_dispatch import frames

# Stages that legitimately command no device. Anything NOT here and not a
# device stage is a FAILURE, not a skip (rule 1 above).
SCENE_STAGES = frozenset({
    "setup_washroom_scene", "current_state", "allow_collision",
    "forbid_collision", "update_fixture_scene", "update_bin_scene",
    "confirm_bin_lowered", "confirm_bin_extended", "confirm_bin_stowed",
})

STROKE_STAGE = "execute_path"
#: A second stroke stage over its own sequence and tool frame.
#:
#: NOT FOR CLEANING TASKS (Newton, 2026-08-21). A cleaning task emits points and
#: ONE sequence, both in the primary frame: the partner frame is a GENERATION-
#: time parameter used to find reachable poses, never a second path stage. The
#: pads sit 90 deg apart on one hand with one hand state, so which pad contacts
#: is carried by the ORIENTATION of a primary-frame pose — there is nothing for
#: a second stage to switch to.
#:
#: Mechanism and robot tasks are a different shape and keep it.
STROKE_STAGE_2 = "execute_path_2"
STROKE_STAGES = (STROKE_STAGE, STROKE_STAGE_2)


class DispatchError(RuntimeError):
    """The config asks for something this dispatcher cannot faithfully issue."""


@dataclass
class Call:
    stage: str
    device: str            # arm | pole | hand | scene | config
    kind: str              # movej | movej_p | movel | movel_chain | lift | hand | scene
    detail: Dict = field(default_factory=dict)
    arm: Optional[str] = None   # "left"/"right" — which arm this call is for


@dataclass
class Program:
    task: str
    side: str
    calls: List[Call]
    warnings: List[str] = field(default_factory=list)

    @property
    def motion_calls(self) -> List[Call]:
        """Calls that MOVE something — not scene ops, not configuration."""
        return [c for c in self.calls if c.device not in ("scene", "config")]

    def summary(self) -> str:
        by = {}
        for c in self.calls:
            by[c.device] = by.get(c.device, 0) + 1
        return ", ".join(f"{k}={v}" for k, v in sorted(by.items()))



# The controller's tool-frame name is the URDF link with the `_frame` token
# dropped: `L_glove_frame_2` -> `L_glove_2`. `rm_frame_t.frame_name` is a
# fixed char buffer, so an over-long name is refused rather than truncated —
# two frames could collide silently.
CONTROLLER_FRAME_NAME_MAX = 11

#: Links the controller does not need a tool frame FOR, because it already has
#: the frame they are measured against.
#:
#: `*_ConnectorLink` is a URDF link, not a controller concept. The controller's
#: own tool frame is Arm_Tip, and ConnectorLink is a known 15.3 mm along its Z
#: (`LINK7_TO_CONNECTOR_Z`, hardware-measured). So a task authored against
#: ConnectorLink is dispatched on Arm_Tip with that offset applied to the poses
#: — `_tool_pose_shift` below — rather than by inventing a second controller
#: frame for a point the controller can already reach. (Newton, 2026-08-22:
#: "no need of creating a connect frame in the controller".)
#: {authored ik_frame: (controller tool, tool-side shift in metres)}. The shift
#: is `T_controller_tool - T_authored_frame`, both measured from the flange, so
#: a pose authored for the left column arrives as a pose for the right one:
#: `P_tool = P_authored . Translate(shift)`.
#:
#: ConnectorLink is +15.3 mm along Arm_Tip's Z (`verify_task_motion.
#: LINK7_TO_CONNECTOR_Z`, fitted to recorded controller TCP over ~200 samples
#: per arm, residual 0.008 mm), so commanding it on Arm_Tip means sending the
#: pose 15.3 mm back. `test_reference_frames` re-derives this number from
#: `tool_frames()` so the two cannot drift.
CONTROLLER_TOOL_ALIAS = {
    "R_ConnectorLink": ("R_arm_tip", (0.0, 0.0, -0.0153)),
    "L_ConnectorLink": ("L_arm_tip", (0.0, 0.0, -0.0153)),
}


def controller_tool_shift(urdf_link: str):
    """The tool-side translation to apply when the controller drives a
    DIFFERENT frame from the one the task was authored against."""
    entry = CONTROLLER_TOOL_ALIAS.get(urdf_link)
    return tuple(entry[1]) if entry else (0.0, 0.0, 0.0)


def controller_frame_name(urdf_link: str) -> str:
    entry = CONTROLLER_TOOL_ALIAS.get(urdf_link)
    name = (entry[0] if entry else urdf_link).replace("_frame", "")
    if len(name) > CONTROLLER_FRAME_NAME_MAX:
        raise DispatchError(
            f"{urdf_link!r} -> {name!r} is {len(name)} chars; the controller "
            f"accepts {CONTROLLER_FRAME_NAME_MAX}")
    return name


def _device_of(stage: dict) -> str:
    """arm | pole | hand | scene — and never a silent default."""
    g = (stage.get("planning_group") or "").strip()
    name = stage.get("stage")
    if g.startswith("pole_") or g == "dual_poles":
        return "pole"
    if g.startswith("inspire_hand_") or g == "dual_hands":
        return "hand"
    if g.startswith("rm75_arm_") or g == "dual_arms" or g.startswith("arm_pole_"):
        return "arm"
    if name in STROKE_STAGES:
        return "arm"
    if not g and name in SCENE_STAGES:
        return "scene"
    raise DispatchError(
        f"stage {name!r} (planning_group={g!r}) is not a device stage and not a known "
        f"scene stage — refusing to skip it (F81). Add it to SCENE_STAGES if it "
        f"genuinely commands nothing, or give it a planning_group.")


def _prismatic_value(joints: Dict[str, float]) -> float:
    if len(joints) != 1:
        raise DispatchError(f"expected one prismatic joint, got {sorted(joints)}")
    return float(next(iter(joints.values())))


def build_program(cfg, states, model, *, motion: Optional[dict] = None,
                  blend_pct: Optional[int] = None,
                  pole_start_m: Optional[float] = None) -> Program:
    """Resolve a task config into an ordered controller program.

    `states` resolves named SRDF group states; `model` is a ROS-free UrdfModel
    used only for the reference-frame -> controller-frame transform.
    """
    if cfg.is_compound:
        return _build_compound(cfg, states, model, blend_pct=blend_pct,
                               pole_start_m=pole_start_m)

    prefix = cfg.prefix
    arm_group = f"rm75_arm_{cfg.side}"
    resolver = frames.FrameResolver(model, cfg.side, cfg.reference_frame,
                                    tool_shift=controller_tool_shift(cfg.ik_frame))
    calls: List[Call] = []
    warnings: List[str] = []
    pole_m = pole_start_m
    arm_joint_names = [f"{prefix}_joint{i}" for i in range(1, 8)]

    # EVERY Cartesian target in this program is the IK FRAME's pose, not the
    # flange's — `L_glove_frame_2` sits ~180 mm beyond the flange. The
    # controller must be told which tool it is commanding BEFORE any Cartesian
    # move, or it drives the flange there instead and the target reads as
    # out of reach. Measured on the emulator: without this, toplid points sit
    # up to 898 mm from the arm base against an 867.7 mm flange reach, and the
    # chain lands 126.7 mm short.
    if cfg.ik_frame:
        calls.append(Call("select_tool_frame", "config", "set_tool_frame", {
            "tool_frame": controller_frame_name(cfg.ik_frame),
            "ik_frame": cfg.ik_frame}))
    else:
        warnings.append("no ik_frame in the config: the controller's current "
                        "tool frame is whatever a previous task left selected")

    def need_pole() -> float:
        if pole_m is None:
            raise DispatchError(
                "a Cartesian target is resolved before any pole stage set the "
                "height — the pole carries the arm base, so this would displace "
                "every waypoint (C16.1). Pass pole_start_m or order the config so "
                "the pole moves first.")
        return pole_m

    pole_state = {cfg.side: pole_m}
    for stage in cfg.stage_sequence:
        name = stage.get("stage")
        device = _device_of(stage)
        if device == "scene":
            calls.append(Call(name, "scene", "scene", {"note": "commands no device"}))
            continue
        calls.extend(_build_stage(cfg, stage, states, resolver, pole_state,
                                  cfg.side, blend_pct, warnings,
                                  motion or {}))

    if blend_pct is None and not _blend_from_motion(motion):
        warnings.append(
            "blend_unspecified: no config carries a per-move `r` and this builder "
            "does not invent one (C16.4). Chain corners will use whatever the "
            "controller defaults to unless a blend is passed explicitly.")
    return Program(cfg.task, cfg.side, calls, warnings)






def enforce_device_serialization(program: Program) -> tuple:
    """F9, as a checked invariant rather than a hoped-for property.

    **Concurrent lift + PLANNED arm move is unsupported in BOTH directions** —
    the command issued second TRUNCATES the first. Measured model-free on both
    arms at every speed and dispatch offset (RM_API2 C16,
    `test_arm_pole_baseline`), whose verdict was blunt: *"a working concurrent
    combination exists — NONE; sequence them instead"*. The arm-truncated case
    latches 16384 on the moving joints and `rm_clear_system_err()` returns 0
    WITHOUT clearing it; the pole-truncated case stops silently ~25 mm in with
    no error and no event. Vendor-confirmed Gen-3 defect, no fix ETA.

    The hand is NOT restricted: planned arm move + concurrent hand is proven
    (C6, 4/4 phases finishing during arm motion). This dispatcher still issues
    hands sequentially — allowed, just not required.

    Returns `(ok, order, problems)`. The dangerous case this actually catches
    is a chain left OPEN: `connect=1` returns immediately even in blocking
    mode, so a pole command issued after an unclosed chain lands on an arm that
    is still moving — which is exactly the F9 fault, arriving through a door
    that looks sequential.
    """
    problems = []
    order = []
    # Per ARM, because F9 is per controller: cross-arm concurrency is routine
    # and hardware-proven, and must not be serialized away here.
    moving: Dict[Optional[str], bool] = {}
    for call in program.calls:
        if call.device in ("scene", "config"):
            continue
        order.append((call.stage, call.device, call.arm))
        arm = call.arm

        # Anything issued while that arm is still moving is the F9 shape: the
        # command issued SECOND truncates the first.
        if moving.get(arm):
            if call.device == "pole":
                problems.append(
                    f"{call.stage}: pole commanded while the {arm} arm is still "
                    "moving — F9, the second command truncates the first (arm "
                    "latches 16384, or the pole stops silently ~25 mm in)")
            else:
                problems.append(
                    f"{call.stage}: issued while the {arm} arm is still moving "
                    "(the preceding chain never closed with connect=0)")

        if call.kind == "movel_chain":
            moves = call.detail.get("moves", [])
            still = bool(moves) and moves[-1].get("connect") != 0
            if still:
                problems.append(
                    f"{call.stage}: chain does not close with connect=0 — "
                    "connect=1 returns IMMEDIATELY even in blocking mode, so the "
                    "call returns while the arm keeps moving")
            moving[arm] = still
        elif call.device == "arm":
            moving[arm] = call.detail.get("connect") == 1
        # pole and hand calls do not leave the ARM moving; the hand is
        # explicitly allowed to overlap arm motion (C6, 4/4 phases).
    return (not problems), order, problems


def _pose_in_reference_frame(view, pose_name: str) -> np.ndarray:
    """A `cartesian_poses` entry -> 4x4 in the reference frame.

    TWO FORMS are legal and both appear in the tree:
      * ABSOLUTE  `[x, y, z, qx, qy, qz, qw]`
      * RELATIVE  `{translation: [...], rotation: [r, p, y] degrees}` — a delta
        from `start_pose`, the same convention `cleaning_points` use, with the
        rotation LEFT-multiplied (`R_final = R_delta @ R_start`).
    The relative form is used by compound tasks (`lid_open`'s `retract_pose`),
    so a dispatcher that only understood the absolute form would fail on
    exactly the dual-arm tasks it was written for.
    """
    entry = view.cartesian_poses[pose_name]
    if isinstance(entry, dict):
        base = view.cartesian_poses.get("start_pose")
        if base is None:
            raise DispatchError(
                f"{pose_name!r} is a relative pose but the config has no "
                "`start_pose` to resolve it against")
        T0 = frames.pose_from_xyz_quat(base)
        T = np.eye(4)
        T[:3, 3] = T0[:3, 3] + np.array(entry.get("translation", [0, 0, 0]), dtype=float)
        T[:3, :3] = frames.rpy_deg_to_R(entry.get("rotation", [0, 0, 0])) @ T0[:3, :3]
        return T
    return frames.pose_from_xyz_quat(entry)


def _build_stage(view, stage: dict, states, resolver, pole_m: Dict[str, Optional[float]],
                 side: str, blend_pct: Optional[int],
                 warnings: List[str],
                 motion: Optional[dict] = None) -> List[Call]:
    """The calls for ONE device stage. Shared by the single-arm and compound
    builders so there is exactly one implementation of what a stage means."""
    name = stage.get("stage")
    device = _device_of(stage)
    prefix = "L" if side == "left" else "R"
    arm_group = f"rm75_arm_{side}"
    arm_joint_names = [f"{prefix}_joint{i}" for i in range(1, 8)]

    def need_pole() -> float:
        if pole_m.get(side) is None:
            raise DispatchError(
                f"{name!r}: a Cartesian target on the {side} arm is resolved before "
                "any pole stage set that arm's height — the pole carries the arm "
                "base, so this would displace every waypoint (C16.1)")
        return pole_m[side]

    if device == "pole":
        st = states.group_state(stage["planning_group"], stage["pose_name"])
        pole_m[side] = _prismatic_value(st)
        return [Call(name, "pole", "lift", {
            "height_m": pole_m[side], "state": stage["pose_name"],
            "speed_pct": view.speed_pct("pole", stage)}, arm=side)]

    if device == "hand":
        st = states.group_state(stage["planning_group"], stage["pose_name"])
        if not st:
            raise DispatchError(
                f"hand state {stage['pose_name']!r} not defined for group "
                f"{stage['planning_group']!r} in the SRDF")
        return [Call(name, "hand", "hand", {
            "state": stage["pose_name"], "joints_rad": st,
            "speed_pct": view.speed_pct("hand", stage)}, arm=side)]

    if name in STROKE_STAGES:
        motion = motion or {}
        calls = []
        if name == STROKE_STAGE_2 and str(view.params.get("task_type") or "") == "cleaning":
            raise DispatchError(
                f"{name!r} is not valid on a CLEANING task: cleaning emits points "
                "and ONE sequence, both in the primary frame, and uses the partner "
                "frame only at generation time to find reachable poses. A second "
                "path stage here means the config was built the wrong way.")
        # BAND 2 RUNS ON THE PARTNER PAD. The stage names its own ik_frame, and
        # it is a different contact pad from band 1's — so the tool frame must
        # be selected before the stroke, exactly as band 1's is before its own.
        # Without this the second band would be commanded through the FIRST
        # band's pad: every waypoint offset by the distance between two pads,
        # and nothing would report it.
        stage_frame = str(stage.get("ik_frame") or "").strip().strip('"')
        if name == STROKE_STAGE_2 and stage_frame and stage_frame != view.ik_frame:
            calls.append(Call(f"select_tool_frame_{name}", "config", "set_tool_frame", {
                "tool_frame": controller_frame_name(stage_frame),
                "ik_frame": stage_frame}, arm=side))
        call = _cleaning_chain(view, resolver, need_pole(), stage, blend_pct,
                               warnings, line_speed=motion.get("line_speed"),
                               per_segment=motion.get("per_segment", False),
                               auto_blend_cap=motion.get("auto_blend_cap"),
                               turn=motion.get("turn"),
                               sequence_key=("cleaning_sequence_2"
                                             if name == STROKE_STAGE_2 else None))
        call.arm = side
        calls.append(call)
        return calls

    pose_name = stage.get("pose_name")
    if pose_name is None:
        raise DispatchError(f"arm stage {name!r} names no pose_name")

    if pose_name in view.cartesian_poses:
        T_ref = _pose_in_reference_frame(view, pose_name)
        T_ctrl = resolver.to_controller(T_ref, need_pole())
        kind = "movel" if name.startswith("move_to_start") else "movej_p"
        return [Call(name, "arm", kind, {
            "pose": frames.to_sdk_pose(T_ctrl), "pose_name": pose_name,
            "speed_pct": view.speed_pct("arm", stage), "r": blend_pct,
            "connect": 0, "pole_m": need_pole()}, arm=side)]

    joints = states.group_state(stage.get("planning_group") or arm_group, pose_name)
    missing = [j for j in arm_joint_names if j not in joints]
    if missing:
        raise DispatchError(
            f"named state {pose_name!r} does not define {missing} — cannot issue movej")
    return [Call(name, "arm", "movej", {
        "joints_deg": [math.degrees(joints[j]) for j in arm_joint_names],
        "state": pose_name, "speed_pct": view.speed_pct("arm", stage),
        "r": blend_pct, "connect": 0}, arm=side)]


def _build_compound(cfg, states, model, *, motion: Optional[dict] = None,
                    blend_pct: Optional[int] = None,
                    pole_start_m: Optional[float] = None) -> Program:
    """A two-arm task, dispatched in the order the config states.

    The config already serialises the arms — one `stage_sequence`, each motion
    stage naming its `arm` — so this walks that one sequence and resolves each
    stage against the per-arm view. Nothing here decides an ordering, which is
    the point: an ordering invented at dispatch time would be a second opinion
    about a task the config has already sequenced.

    Each arm keeps its OWN pole height. The pole carries the arm base, so one
    shared height would displace every Cartesian target on the other arm.
    """
    views = {side: cfg.arm_view(side) for side in cfg.arms}
    resolvers = {side: frames.FrameResolver(model, side, v.reference_frame)
                 for side, v in views.items()}
    calls: List[Call] = []
    warnings: List[str] = []
    pole_m = {side: pole_start_m for side in views}
    tool_selected = set()

    for stage in cfg.stage_sequence:
        name = stage.get("stage")
        device = _device_of(stage)
        if device == "scene":
            # Carry the arm when the config names one: `allow_collision` and
            # `forbid_collision` are per-arm contact licences, and dropping the
            # arm made them invisible to any per-arm analysis (custody.py).
            calls.append(Call(name, "scene", "scene", {"note": "commands no device"},
                              arm=stage.get("arm")))
            continue

        side = stage.get("arm")
        if side is None:
            raise DispatchError(
                f"compound stage {name!r} commands a device but names no `arm` — "
                f"refusing to guess which arm it belongs to")
        if side not in views:
            raise DispatchError(f"stage {name!r} names arm {side!r}, not in {sorted(views)}")
        view = views[side]

        # Select each arm's tool frame once, before its first Cartesian move.
        if side not in tool_selected and view.ik_frame:
            calls.append(Call("select_tool_frame", "config", "set_tool_frame", {
                "tool_frame": controller_frame_name(view.ik_frame),
                "ik_frame": view.ik_frame}, arm=side))
            tool_selected.add(side)

        sub = _build_stage(view, stage, states, resolvers[side], pole_m, side,
                           blend_pct, warnings, motion or {})
        calls.extend(sub)

    if blend_pct is None and not _blend_from_motion(motion):
        warnings.append(
            "blend_unspecified: no config carries a per-move `r` and this builder "
            "does not invent one (C16.4).")
    return Program(cfg.task, "dual", calls, warnings)


# How far from tangent a movec span may sit and still be priced as a U-turn.
# A tangent turn has no corner left; 5 degrees of residual costs under 0.4 % of
# the turn speed, which is inside the noise on a_lat itself.
_TANGENT_TOL_RAD = math.radians(5.0)


def _untangent_speed(order, pts, absolute, i, j, r_pct, turn, line_speed):
    """v% a NON-tangent arc span may carry — the worse of its two real corners.

    Returns None when `alix_taskgen` is unavailable, in which case the caller
    leaves the commanded speed alone and the warning is all the operator gets.
    """
    try:
        from alix_taskgen import junction_limits as jl
    except Exception:
        return None
    if not line_speed:
        return None
    pos = [absolute(pts[n])[:3, 3] for n in order]
    a_lat = float((turn or {}).get("a_lat") or jl.line_acc(float(line_speed)))
    best = None
    for k, prev, nxt in ((i, i - 1, i + 1), (j, j - 1, j + 1)):
        if prev < 0 or nxt >= len(order):
            continue
        u, w = pos[k] - pos[prev], pos[nxt] - pos[k]
        L_in = float(np.linalg.norm(u))
        L_out = float(np.linalg.norm(w))
        if L_in <= 0.0 or L_out <= 0.0:
            continue
        phi = jl.corner_deviation(u, w)
        if phi < jl.TURN_DEVIATION_RAD:
            continue
        cut = min(jl.blend_cut(max(int(r_pct or 10), 10), L_in, L_out),
                  min(L_in, L_out))
        v = jl.turn_speed(jl.fillet_radius(cut, phi), a_lat, float(line_speed))
        best = v if best is None else min(best, v)
    if best is None:
        return None
    return int(round(100.0 * best / float(line_speed)))


def _arc_tangency(p_prev, p_start, p_via, p_end, p_next):
    """How far a movec span misses being TANGENT to the moves either side.

    Returns `(entry_dev, exit_dev)` in radians. Zero at both ends means the arc
    flows into and out of its neighbours with no corner left, which is the ONLY
    case where a turn policy's U-turn speed is honest — that speed is
    `sqrt(a_lat * pitch/2)` for a semicircle, and it says nothing about a
    corner the arc did not remove.

    This matters because the two are easy to confuse. The spans in the corpus
    bulge a 48.5 mm hop by 3 mm, which is a 99.5 mm radius and reads fast, but
    the arc still meets both rows at 90 degrees. Priced as a U-turn it would be
    commanded at 0.28 m/s into a corner that holds 0.16.
    """
    def _dev(a, b):
        na, nb = np.linalg.norm(a), np.linalg.norm(b)
        if na < 1e-9 or nb < 1e-9:
            return 0.0
        return float(np.arccos(np.clip(float(a @ b) / (na * nb), -1.0, 1.0)))

    A, V, B = np.asarray(p_start), np.asarray(p_via), np.asarray(p_end)
    u, w = V - A, B - A
    nrm = np.cross(u, w)
    if np.linalg.norm(nrm) < 1e-12:          # collinear: the span is a straight
        t_in = t_out = w
    else:
        # Circumcentre of A, V, B.
        O = A + (np.cross((np.linalg.norm(u) ** 2) * w - (np.linalg.norm(w) ** 2) * u,
                          nrm) / (2.0 * float(nrm @ nrm)))
        # Tangents, oriented along travel (A -> V -> B).
        t_in = np.cross(nrm, A - O)
        t_out = np.cross(nrm, B - O)
        if float(t_in @ (V - A)) < 0:
            t_in, t_out = -t_in, -t_out
    entry = _dev(A - np.asarray(p_prev), t_in) if p_prev is not None else 0.0
    exit_ = _dev(t_out, np.asarray(p_next) - B) if p_next is not None else 0.0
    return entry, exit_


def _apply_step_limit(pct, r_at, seg, a_lat, jl, line_speed, *, enforce=True):
    """A.2.2 across every junction, relaxed to a fixed point. Returns the count.

    THE CONTRACT WAS RIGHT AND I WAS NOT. I shipped this as a diagnostic on the
    reasoning that the 2026-08-20 run's dead stops (r=0, arrivals at rest with
    no cut at all) proved the controller brakes along the straight. It does —
    but only to a STOP AT a waypoint. That says nothing about changing speed
    THROUGH a blend, which is what A.2.2 is actually about, and the two are not
    the same manoeuvre.

    The 2026-08-21 SIM run settled it. `execute_path` was rejected ret=1 and
    the arm stopped 12.3 mm short of point15 — exactly the r=16 cut. That
    junction stepped 0.689 -> 0.200 m/s, which needs 90.2 mm of cut and had
    12.8 mm. Seven times over, and the controller refused the chain rather
    than absorbing it.

    Two rules, both from A.2.2's own sentence:

      * the LOWER SPEED binds, never a larger r. Raising r to the cap is what
        the old code did, and on a 48.5 mm hop r=50 lands on the emulator's
        FREEZE signature (r >= 25 near a < 90 mm segment) — trading a refusal
        for a stall.
      * one junction's clamp changes its neighbours' steps, so relax to a
        fixed point rather than sweeping once.

    Operates on the INTEGER PERCENTS that go on the wire, in place, and steps
    the faster side down one percent at a time. Anything finer would be a
    number the controller never sees.
    """
    n = len(seg)
    short = 0
    for _ in range(4096):
        changed = False
        short = 0
        for i in range(1, n):
            cut = min(jl.blend_cut(max(r_at[i], 10), seg[i - 1], seg[i]),
                      min(seg[i - 1], seg[i]))
            v_in = pct[i - 1] / 100.0 * line_speed
            v_out = pct[i] / 100.0 * line_speed
            if jl.step_cut_required(v_in, v_out, a_lat) <= cut + 1e-12:
                continue
            short += 1
            if not enforce:
                continue
            k = i - 1 if pct[i - 1] > pct[i] else i
            if pct[k] > 1:
                pct[k] -= 1
                changed = True
        if not changed:
            break
    return short


def _blend_from_motion(motion) -> bool:
    """Whether the motion policy computes a per-move `r` of its own.

    `blend_unspecified` means "nobody chose a blend, so the controller's own
    default decides the corners". That is false the moment `--auto-blend` or a
    turn policy is in play — both derive an r per junction — and leaving the
    warning up made a run that HAD chosen its corners read as though it had not.
    """
    m = motion or {}
    return bool(m.get("auto_blend_cap") or m.get("turn"))


def _segment_policy(order, pts, absolute, line_speed, r_pct_cap, warnings,
                    turn=None, arc_segs=None):
    """Per-move speed and per-junction blend, from the contract's own laws.

    Returns ``(v_frac, r_at)`` — `v_frac[i]` is the fraction of the WRITTEN TCP
    constraint that the move arriving at `order[i]` may command, and `r_at[i]`
    is the A.2 blend radius PERCENTAGE for the corner at `order[i]`.

    Two laws, both already implemented in `alix_taskgen.junction_limits`, which
    is imported rather than re-derived here — a second copy of A.1/A.2 that
    drifted from the first is exactly what C17 exists to prevent:

      A.1   v_reach(L)  a segment too short to accelerate into the cap cannot
                        be commanded at it; asking anyway just means the arm
                        never gets there.
      A.2   cut = c(r) * (r/100) * min(L_in, L_out), with c(r) PIECEWISE and
                        DECLINING — 1.70 at r=10, 1.57 at r=25, 1.33 at r=50.
      A.2.2 the corner must carry the speed STEP: cut >= |v_out^2-v_in^2|/(2a).

    Turning points are deliberately NOT given a per-segment speed: they are
    short hops that never reach any cap, and on this corpus they are traced by
    `rm_movec`, which A.2.1 says takes no A.2 cut at all.
    """
    try:
        from alix_taskgen import junction_limits as jl
    except Exception as exc:
        warnings.append(f"per-segment speed/blend needs alix_taskgen "
                        f"({exc!r}); every move keeps the global v and r")
        return None, None

    pos = [absolute(pts[n])[:3, 3] for n in order]
    seg = [float(np.linalg.norm(pos[i] - pos[i - 1])) for i in range(1, len(pos))]
    if not seg:
        return None, None

    if turn:
        return _turn_policy(order, pos, seg, line_speed, r_pct_cap, warnings,
                            turn, jl, arc_segs or set())

    # Per-segment reachable speed (A.1), as a fraction of the written cap.
    v_seg = [jl.v_reach(L, cap=line_speed) for L in seg]
    v_frac = [1.0] + [min(1.0, v / line_speed) if line_speed > 0 else 1.0
                      for v in v_seg]

    # Per-junction blend: the SMALLEST r whose A.2 cut carries the A.2.2 step.
    # Smallest, not largest: a cut is arc-length REMOVED from the path, so an
    # oversized one rounds off surface the tool is supposed to clean.
    r_at = [0] * len(order)
    for i in range(1, len(seg)):
        v_in, v_out = v_seg[i - 1], v_seg[i]
        a = jl.line_acc(max(v_in, v_out))
        need = jl.step_cut_required(v_in, v_out, a)
        chosen = 0
        for r in range(10, int(r_pct_cap) + 1):
            have = min(jl.blend_cut(r, seg[i - 1], seg[i]),
                       min(seg[i - 1], seg[i]))
            if have >= need:
                chosen = r
                break
        else:
            # Infeasible at every r. A.2.2 is explicit: the LOWER SPEED binds —
            # never a larger r, and never "run it anyway".
            ok, detail = jl.check_junction(v_in, v_out, seg[i - 1], seg[i],
                                           r_pct_cap)
            bind = detail.get("v_bind")
            if bind is not None:
                v_frac[i] = min(v_frac[i], bind / line_speed if line_speed else 1.0)
            chosen = int(r_pct_cap)
        r_at[i] = chosen
    return v_frac, r_at


def _turn_policy(order, pos, seg, line_speed, r_pct_cap, warnings, turn, jl,
                 arc_segs):
    """Cruise on the straights, RADIUS at the corners — the three-field policy.

    The contract prices a whole path at its worst junction: one
    `cleaning_line_speed_mps` for every move in it. This does the opposite.
    Each junction is priced by its own geometry, and each segment by the two
    junctions it runs between.

    Policy fields (`motion["turn"]`), all optional but `radius_m`:

      radius_m       the corner radius asked for. NOT a speed — A.2.1 turns a
                     radius into a speed and A.2 turns it into a blend `r`, so
                     radius is the only one of the three that is an input.
      speed_cap_mps  a ceiling on any turn, for when the geometry would permit
                     more than the operator wants to give it.
      overtravel_m   how far past the workpiece a turnaround may reach. With
                     room a serpentine turn becomes `uturn_plan`'s tangent
                     semicircle, which deletes both corners and costs NO
                     surface. Without it every corner is a fillet, and every
                     fillet eats the vertex it replaces.
      a_lat          the lateral-acceleration budget. RM_API2 has no separate
                     setter, so this is the WRITTEN `line_acc`, and using it
                     for lateral accel is an ASSUMPTION, not a measurement.

    A.2.2 is applied here as a DIAGNOSTIC, not a gate, and that is a deliberate
    departure worth stating. Its model confines a speed step to the blend
    itself, which makes an 0.80 -> 0.28 corner entry infeasible at every r and
    binds the row to 0.53 m/s. The 2026-08-20 run is evidence against that
    model: `r=0` on every corner produced 32 dead STOPS at the waypoints, and
    arriving at a waypoint at rest with no cut at all can only happen by
    braking along the straight before it. So the controller does look ahead.
    `strict_step` restores the contract's reading for anyone who wants it.
    """
    cruise = float(turn.get("cruise_mps") or line_speed)
    a_lat = float(turn.get("a_lat") or jl.line_acc(cruise))
    radius_m = turn.get("radius_m")
    speed_cap = turn.get("speed_cap_mps")
    overtravel = float(turn.get("overtravel_m") or 0.0)
    max_hop = float(turn.get("max_hop_m") or 0.120)
    surface_tol = turn.get("surface_tol_m")
    # A.2.2 is ENFORCED unless explicitly opted out of. It used to default to
    # a diagnostic on my reading that the controller pre-decelerates along the
    # straight — the 2026-08-21 SIM run refuted that, see `_apply_step_limit`.
    strict = not bool(turn.get("allow_step"))
    n = len(seg)

    # Deviation at each interior junction. order[i] sits between seg[i-1] and
    # seg[i]; the ends are at rest and are not junctions at all.
    phi = [0.0] * (n + 1)
    for i in range(1, n):
        phi[i] = jl.corner_deviation(pos[i] - pos[i - 1], pos[i + 1] - pos[i])

    # A serpentine turnaround is a SHORT hop that turns at both ends. It is the
    # only shape a single tangent arc can replace, so it is the only one
    # offered the overtravel budget.
    hop = [False] * n
    for j in range(n):
        if seg[j] <= max_hop and j >= 1 and j + 1 <= n - 1:
            if phi[j] >= jl.TURN_DEVIATION_RAD and phi[j + 1] >= jl.TURN_DEVIATION_RAD:
                hop[j] = True

    plans = [None] * (n + 1)
    seg_cap = [cruise] * n
    vj = [0.0] * (n + 1)          # junction speeds; both ends at rest
    uturns = 0
    # A U-turn is only on offer where an ARC will actually trace it. Pricing a
    # junction as a tangent semicircle when the chain will send two movels
    # straight through it is the same mispricing `_arc_tangency` catches on
    # spans — invisible here, because there is no span to check. Measured
    # 2026-08-21: 18 junctions priced as U-turns against ONE arc in the file.
    orphan = 0
    for i in range(1, n):
        if hop[i - 1] and (i - 1) in arc_segs:
            pitch = seg[i - 1]
        elif hop[i] and i in arc_segs:
            pitch = seg[i]
        else:
            pitch = None
            if (hop[i - 1] or hop[i]) and overtravel > 0.0:
                orphan += 1
        plan = jl.corner_plan(
            phi[i], seg[i - 1], seg[i], a_lat=a_lat, cap=cruise,
            radius_target=radius_m, r_max=float(r_pct_cap),
            overtravel_m=overtravel, pitch_m=pitch,
            **({} if surface_tol is None
               else {"surface_tol_m": float(surface_tol)}))
        plans[i] = plan
        v = plan["v_mps"]
        # The cap is a TURN cap. A junction that barely deviates is not a turn
        # and holds cruise; clamping it too would slow the straights, which is
        # the opposite of what the policy exists to do (measured: it dropped
        # this path's four near-collinear junctions from 0.80 to 0.35).
        if speed_cap is not None and plan.get("is_turn"):
            v = min(v, float(speed_cap))
        vj[i] = v
        if plan.get("adopted") == "uturn":
            uturns += 1
            # The arc IS the hop, so the hop carries the turn speed end to end
            # and the two junctions it lands on are tangent — no corner left.
            if hop[i - 1]:
                seg_cap[i - 1] = min(seg_cap[i - 1], v)
            if hop[i]:
                seg_cap[i] = min(seg_cap[i], v)
        elif hop[i - 1] or hop[i]:
            k = i - 1 if hop[i - 1] else i
            seg_cap[k] = min(seg_cap[k], v)

    # Per-segment peak: the trapezoid that starts at one junction speed and
    # ends at the other. This is A.1 generalised — `v_reach` is the special
    # case where both ends are at rest, which is what a path with no turn
    # policy is, and it is why a 48 mm hop used to read 0.28 m/s no matter how
    # gently the path arrived at it.
    v_peak = []
    for k in range(n):
        v_in, v_out = vj[k], vj[k + 1]
        reach = math.sqrt(max(0.0,
                              (2.0 * a_lat * seg[k] + v_in * v_in + v_out * v_out) / 2.0))
        v_peak.append(min(seg_cap[k], reach))

    # Blend radius. The corner plan already chose the smallest r that reaches
    # the asked-for radius.
    r_at = [0] * len(order)
    for i in range(1, n):
        plan = plans[i]
        r = int(round(plan["r_pct"])) if plan else 0
        if plan and plan.get("adopted") == "uturn":
            # Tangent: the fillet radius is infinite, so any r cuts nothing.
            # It must still be NON-ZERO — r=0 is what produced the 32 dead
            # stops, because the controller reads it as "arrive at this
            # waypoint", not as "do not round this corner".
            r = 10
        r_at[i] = max(r, 1)

    # QUANTISE FIRST, then enforce. `v` goes on the wire as an INTEGER percent,
    # and checking A.2.2 against the continuous plan is checking a number that
    # is never sent. Rounding the slow side of a junction down and the fast
    # side up widens the very step being limited: measured 2026-08-21 on
    # junction 23, planned 0.358 -> 0.607 (compliant) went out as 44 % -> 76 %
    # = 0.352 -> 0.608, which is not.
    pct = [max(1, min(100, int(math.floor(100.0 * v / line_speed))))
           if line_speed > 0 else 100 for v in v_peak]
    step_short = _apply_step_limit(pct, r_at, seg, a_lat, jl, line_speed,
                                   enforce=strict)
    v_frac = [1.0] + [p / 100.0 for p in pct]

    if orphan:
        warnings.append(
            "%d turnaround(s) have the shape for a tangent U-turn but no "
            "`arc_moves` span to trace one, so they stay priced as fillets. "
            "rm_movec is what draws the semicircle; without a span the chain "
            "sends two movels straight through the corner. Generate the spans "
            "with tools/tangent_turnarounds.py (it screens them for reach)."
            % orphan)

    lost = sum(p["surface_lost_m"] for p in plans[1:n] if p)
    turns = sum(1 for p in plans[1:n] if p and p.get("is_turn"))
    # Report the TURN speeds only. Folding the near-collinear junctions in
    # made the range top out at cruise no matter what the turns were doing,
    # which hid whether `speed_cap_mps` had bound at all.
    turn_v = [vj[i] for i in range(1, n)
              if plans[i] and plans[i].get("is_turn")] or [0.0]
    warnings.append(
        "turn policy: %d/%d junctions turn; %d taken as tangent U-turns "
        "(overtravel %.0f mm), %d as fillets discarding %.1f mm of corner in "
        "total; turns run %.3f-%.3f m/s against a %.2f m/s cruise%s; "
        "a_lat=%.2f m/s^2 ASSUMED equal to line_acc"
        % (turns, max(0, n - 1), uturns, overtravel * 1000.0, turns - uturns,
           lost * 1000.0, min(turn_v), max(turn_v), cruise,
           ("" if speed_cap is None else
            " (turn cap %.2f, %s)" % (
                float(speed_cap),
                "BINDING" if max(turn_v) >= float(speed_cap) - 1e-9
                else "not reached")),
           a_lat))
    if step_short:
        warnings.append(
            "A.2.2 %s: %d junction(s) asked for a speed step larger than "
            "their blend cut. %s"
            % ("enforced" if strict else "NOT ENFORCED", step_short,
               "Speeds bound down to what each cut can carry; r is left where "
               "the corner plan put it."
               if strict else
               "Left alone on --turn-allow-step. The 2026-08-21 SIM run is "
               "evidence against doing this: the chain was rejected ret=1 and "
               "the arm stopped inside the first such blend."))
    bound = [i for i in range(1, n)
             if plans[i] and plans[i].get("fillet", {}).get("offset_bound")
             and plans[i].get("adopted") == "fillet"]
    if bound and radius_m:
        tol = float(surface_tol if surface_tol is not None else jl.SURFACE_TOL_M)
        got = max(plans[i]["radius_m"] for i in bound)
        warnings.append(
            "%d turn(s) bound by SURFACE, not by the %.0f mm radius asked for: "
            "the biggest fillet that stays within %.1f mm of its vertex is "
            "%.1f mm. A fillet's offset grows as sec(phi/2), so a sharper "
            "corner buys less radius for the same tolerance. Raise "
            "--turn-surface-tol to trade coverage for speed, or give the turn "
            "room with --turn-overtravel and pay nothing."
            % (len(bound), float(radius_m) * 1000.0, tol * 1000.0,
               got * 1000.0))

    extrap = [i for i in range(1, n)
              if plans[i] and plans[i].get("fillet", {}).get("extrapolated_r")]
    if extrap:
        warnings.append(
            "%d junction(s) blend below r=10, outside A.2's measured domain "
            "(c(r) is fitted over r=10..50 and clamps there). Taken anyway "
            "because the alternative is worse — a near-reversal at r=10 leaves "
            "the surface by more than the pad is wide — but c DECLINES with r, "
            "so the real cut is probably LARGER than modelled at these."
            % len(extrap))
    deficits = [p["uturn"]["radius_deficit_m"] for p in plans[1:n]
                if p and p.get("uturn", {}).get("radius_deficit_m")]
    if deficits:
        warnings.append(
            "%d turnaround(s) capped by row pitch, short of the asked radius "
            "by up to %.1f mm: a tangent semicircle's radius IS pitch/2 and "
            "more overtravel does not raise it. Exceeding it needs a "
            "multi-arc teardrop, which rm_movec cannot express in one call."
            % (len(deficits), max(deficits) * 1000.0))
    return v_frac, r_at


def _cleaning_chain(cfg, resolver, pole_m: float, stage: dict,
                    blend_pct: Optional[int], warnings: List[str],
                    line_speed: Optional[float] = None,
                    per_segment: bool = False,
                    auto_blend_cap: Optional[int] = None,
                    sequence_key: Optional[str] = None,
                    turn: Optional[dict] = None) -> Call:
    """`cleaning_points` + a sequence -> one chained movel program.

    *sequence_key* names which sequence to walk; `None` means the task's own
    `cleaning_sequence`. A group-frame task's second band passes
    `cleaning_sequence_2`, and it indexes the SAME `cleaning_points` map
    against the SAME `start_pose` — one delta convention, one anchor. Only the
    pad presenting the poses differs, and that is selected by the caller.
    """
    poses = cfg.cartesian_poses
    if "start_pose" not in poses:
        raise DispatchError("cleaning stroke needs `start_pose` in cartesian_poses")
    T0 = frames.pose_from_xyz_quat(poses["start_pose"])
    p0, R0 = T0[:3, 3], T0[:3, :3]

    def absolute(point: dict) -> np.ndarray:
        T = np.eye(4)
        T[:3, 3] = p0 + np.array(point.get("translation", [0, 0, 0]), dtype=float)
        # R_final = R_delta @ R_start_pose, left-multiplied as the runtime does.
        T[:3, :3] = frames.rpy_deg_to_R(point.get("rotation", [0, 0, 0])) @ R0
        return T

    pts = cfg.cleaning_points
    order: List[str] = []
    extended = 0
    sequence = (cfg.doc.get(sequence_key) or []) if sequence_key else cfg.cleaning_sequence
    if sequence_key and not sequence:
        raise DispatchError(
            f"{stage.get('stage')!r} drives {sequence_key} but the config carries "
            f"no such sequence — the stage would command nothing")
    for entry in sequence:
        if isinstance(entry, dict):          # C16.4 extended form
            extended += 1
            seg = entry.get("seg") or entry.get("arc") or entry.get("scrub") or []
            names = list(seg)
        else:
            names = list(entry)
        for n in names:
            if n not in pts:
                raise DispatchError(f"cleaning_sequence names unknown point {n!r}")
            if not order or order[-1] != n:
                order.append(n)
    if extended:
        warnings.append(f"{extended} extended sequence entries (seg/arc/scrub) "
                        "seen; per-move v/r are carried through unchanged")

    # ARC SPANS (Newton, 2026-08-20): "turns we use the movec instead of
    # blending". A span names two ENDPOINTS of `cleaning_points` plus a via,
    # and what it claims is every move BETWEEN those endpoints. The config's
    # `cleaning_points` and `cleaning_sequence` are untouched — MoveIt still
    # plans the dense polyline (C17) — and only the hardware chain below
    # collapses a span into one `rm_movec`.
    runs = _arc_runs(order, pts, getattr(cfg, "arc_moves", []) or [], warnings)

    # PER-MOVE speed and blend, when asked for. Without this every move in the
    # chain carried ONE global v and ONE global r — and with no config carrying
    # `r`, `int(r or 0)` sent r=0 on every corner, which is a full stop. The
    # 2026-08-20 hardware run measured 32 dead-stop episodes, 9 % of the
    # stroke below 10 mm/s.
    v_frac = r_at = None
    if (per_segment or auto_blend_cap or turn) and line_speed:
        # Which SEGMENTS a movec will cover — a span from order[i] to order[j]
        # traces segments i..j-1. The turn policy needs this to know which
        # turnarounds it may price as arcs.
        arc_segs = {k for i, (j, _) in runs.items() for k in range(i, j)}
        v_frac, r_at = _segment_policy(order, pts, absolute, float(line_speed),
                                       auto_blend_cap or 50, warnings,
                                       turn=turn, arc_segs=arc_segs)

    base_v = cfg.speed_pct("arm", stage)

    def _move_to(name: str, i: int) -> dict:
        v = base_v
        if (per_segment or turn) and v_frac is not None:
            # A.1: a segment too short to accelerate into the cap cannot be
            # commanded at it. 1 % is the SDK floor.
            v = max(1, min(100, int(round(base_v * v_frac[i]))))
        r = blend_pct
        if (auto_blend_cap or turn) and r_at is not None:
            r = r_at[i]
        return {
            "point": name,
            "pose": frames.to_sdk_pose(
                resolver.to_controller(absolute(pts[name]), pole_m)),
            "v_pct": v,
            "r": r,
            "connect": 1,
            "kind": "movel",
        }

    moves = [_move_to(order[0], 0)]
    untangent: List[tuple] = []
    i = 0
    while i < len(order) - 1:
        run = runs.get(i)
        if run is None:
            moves.append(_move_to(order[i + 1], i + 1))
            i += 1
            continue
        j, span = run
        move = _move_to(order[j], j)
        move["kind"] = "movec"
        move["via"] = frames.to_sdk_pose(resolver.to_controller(
            _span_via_pose(span, pts, absolute, order[i], order[j]), pole_m))
        # A.2.1: "An arc is geometry, not a blended corner, so it takes no
        # A.2 cut at all."
        #
        # That clause is about the arc's OWN geometry — an arc is not a corner
        # and has no vertex to round off. Sending r=0 at the arc's far END is a
        # different statement, and a wrong one: the end of an arc is still a
        # junction with whatever follows, and r=0 is what the controller reads
        # as "arrive at this waypoint". Measured 2026-08-21 on toplid_right,
        # the file's 9 arc spans were costing 9 dead stops, one each, which is
        # the opposite of what a movec turn is for. So the arc's exit blend
        # comes from the policy whenever there is one; r=0 survives only when
        # nothing computed a blend at all, which is the old whole-path default.
        move["r"] = r_at[j] if r_at is not None else 0
        move["arc"] = {k: span.get(k) for k in
                       ("sagitta_mm", "radius_mm", "v_arc_mps", "residual_mm")
                       if span.get(k) is not None}
        # The waypoints this one command replaces. A ring quarter collapses
        # five moves into one; a turnaround collapses none. Recorded so a
        # reader can see what the controller is no longer being told.
        move["spans"] = [order[k] for k in range(i, j + 1)]

        # A turn policy prices a turnaround as a TANGENT arc — but ONLY when it
        # was given overtravel to place one. Without room, every corner was
        # already costed as the fillet it is and there is nothing to verify.
        # With room, check the span really is tangent before letting it carry
        # the U-turn speed: otherwise the corners the policy assumed away are
        # still there, and we would be commanding the arc's speed into a corner
        # that cannot hold it.
        if turn and float(turn.get("overtravel_m") or 0.0) > 0.0:
            via_xyz = _span_via_pose(span, pts, absolute, order[i],
                                     order[j])[:3, 3]
            entry, exit_ = _arc_tangency(
                absolute(pts[order[i - 1]])[:3, 3] if i > 0 else None,
                absolute(pts[order[i]])[:3, 3], via_xyz,
                absolute(pts[order[j]])[:3, 3],
                absolute(pts[order[j + 1]])[:3, 3] if j + 1 < len(order) else None)
            worst = max(entry, exit_)
            if worst > _TANGENT_TOL_RAD:
                untangent.append((order[i], order[j], worst))
                # CLAMP, do not merely warn. The corner the arc failed to
                # remove is still there, and it holds sqrt(a_lat * R_fillet),
                # not the U-turn speed this move is carrying. Commanding the
                # difference into a real corner is the shape of every abort
                # this dispatcher has seen.
                v_ok = _untangent_speed(order, pts, absolute, i, j, move["r"],
                                        turn, line_speed)
                if v_ok is not None:
                    move["v_pct"] = max(1, min(move["v_pct"], v_ok))
        moves.append(move)
        i = j

    # Chained: connect=1 queues, and the CLOSING connect=0 executes the whole
    # chain and yields exactly one arrival event. N unchained moves would buy
    # N-1 full stops (C4 / C16.2).
    moves[-1]["connect"] = 0

    if untangent:
        worst = max(w for _, _, w in untangent)
        warnings.append(
            "TURN POLICY MISPRICED: %d of the %d arc span(s) are NOT tangent to "
            "the moves beside them — worst %.0f deg at %s->%s. The policy "
            "costed these as U-turns, so they carry sqrt(a_lat*pitch/2); the "
            "corner that is actually still there holds far less. Each has been "
            "CLAMPED back to its own corner speed, so this run is safe but no "
            "faster than one with no overtravel at all. To collect the U-turn, "
            "regenerate the vias as tangent semicircles — "
            "tools/tangent_turnarounds.py, which also screens them for reach."
            % (len(untangent), sum(1 for m in moves if m["kind"] == "movec"),
               math.degrees(worst), untangent[0][0], untangent[0][1]))

    n_arc = sum(1 for m in moves if m["kind"] == "movec")
    collapsed = len(order) - len(moves)
    if n_arc:
        exits = sorted({m["r"] for m in moves if m["kind"] == "movec"})
        how = ("exit blend r=%s" % "..".join(str(x) for x in
               ({exits[0], exits[-1]} if exits else {0}))) if r_at is not None \
              else "r=0 per A.2.1 — each arc ENDS AT REST, see _cleaning_chain"
        warnings.append(
            f"{n_arc} span(s) dispatch as rm_movec ({how}), collapsing "
            f"{collapsed} intermediate waypoint(s); the rest are movel")
    return Call(stage.get("stage") or STROKE_STAGE, "arm", "movel_chain", {
        "moves": moves, "points": len(order), "pole_m": pole_m,
        "line_speed_mps": cfg.cleaning_line_speed_mps,
        "arc_moves": n_arc, "collapsed_points": collapsed,
    })


def _span_via_pose(span, pts, absolute, name_from, name_to):
    """The 6-DOF pose `rm_movec` needs for a span's via.

    Two producers, two honest shapes:

    * ``arc_moves_for_sequence`` (rings) names a via that IS one of the ring's
      own waypoints — ``{"via": "point7"}``. Its authored rotation is real, so
      use that point's own pose rather than interpolating one.
    * ``turnaround_arcs`` (serpentines) bulges to a position that is NOT a
      waypoint — ``{"via_translation": [...]}``. Nothing authored a rotation
      for it, so take the half-way slerp between the two endpoints, the same
      interpolation the runtime does between waypoints (F75). Either
      endpoint's orientation alone would snap the tool through the turn.

    Resolving the name HERE is why the generator need not duplicate a via into
    both keys — one shape per producer, one reader.
    """
    name = span.get("via")
    if name is not None and name in pts:
        return absolute(pts[name])
    T = absolute({"translation": list(span["via_translation"])})
    T[:3, :3] = frames.slerp_R(absolute(pts[name_from])[:3, :3],
                               absolute(pts[name_to])[:3, :3], 0.5)
    return T


def _arc_runs(order, pts, spans, warnings):
    """Map each arc span onto the contiguous run of `order` it replaces.

    Returns ``{start_index: (end_index, span)}``.

    A span names two endpoints; the run is every point walked between them.
    Matching the endpoint PAIR against an EDGE of `cleaning_sequence` was
    correct only for the one-edge turnaround: a ring span such as
    ``(point1, point6)`` covers five edges, so it matched no edge and was
    dropped with a warning — `rm_movec` therefore never reached the annular
    families at all, the ones CLEANING_MOTION_SPEC §3 calls its biggest win.
    Found 2026-08-20.

    Scanned left to right over `order`, never by name lookup: toplid_right
    walks point5 eleven times, so a name pair alone is ambiguous and would arc
    the wrong turnaround. Every occurrence of a span's run is claimed
    independently — a turnaround edge revisited on each row still arcs on each
    row — and a run may not overlap one already claimed.
    """
    runs: dict = {}
    claimed: List[tuple] = []
    # DEDUPLICATE FIRST. `turnaround_arcs` emits one entry per qualifying edge
    # OCCURRENCE, so a serpentine that turns at the same two points on several
    # rows carries that pair several times (toplid_right ships three such
    # pairs). Matching is by NAME and already claims every occurrence, so the
    # repeats are redundant — left in, they matched runs the first copy had
    # taken and reported as spurious "overlap" conflicts.
    unique, seen_spans, duplicates = [], set(), 0
    for span in spans:
        key = (tuple(span.get("arc") or []), span.get("via"),
               tuple(span.get("via_translation") or ()))
        if key in seen_spans:
            duplicates += 1
            continue
        seen_spans.add(key)
        unique.append(span)
    if duplicates:
        warnings.append(
            f"{duplicates} duplicate arc_moves span(s) collapsed; each distinct "
            "span already arcs every occurrence of its run")
    for span in unique:
        pair = list(span.get("arc") or [])
        if len(pair) != 2:
            warnings.append(f"arc_moves span {pair} does not name two "
                            "endpoints; ignored")
            continue
        a, b = pair
        if a not in pts or b not in pts:
            warnings.append(f"arc_moves span {pair} names an unknown point; "
                            "ignored")
            continue
        via_name = span.get("via")
        if via_name is None and span.get("via_translation") is None:
            warnings.append(f"arc_moves span {pair} carries neither `via` nor "
                            "`via_translation`; ignored")
            continue
        if via_name is not None and via_name not in pts:
            warnings.append(f"arc_moves span {pair} names via {via_name!r}, "
                            "which is not a cleaning point; ignored")
            continue
        # HOW LONG A RUN THE SPAN CLAIMS is decided by its own via, not by a
        # guess. An off-path `via_translation` is the serpentine bulge over ONE
        # hop; a named `via` is a waypoint ON the arc, so the run is whatever
        # stretch passes through it. Getting this wrong is not academic: a
        # permissive "shortest run a->b" matcher let toplid_right's
        # (point11, point9) claim a multi-edge stretch and starve three other
        # turnaround spans, which then reported as overlaps (caught 2026-08-20
        # against the shipped config, which must stay at 12 arcs).
        single_edge = via_name is None
        hits = 0
        overlapped = 0
        i = 0
        while i < len(order) - 1:
            if order[i] != a:
                i += 1
                continue
            if single_edge:
                j = i + 1 if order[i + 1] == b else -1
            else:
                # Forward run a -> b that revisits no point and passes through
                # the named via. An arc is one sweep of a circle: a run that
                # comes back on itself is not one, and a run that misses the
                # via is not the arc that was fitted.
                seen = {a}
                saw_via = False
                j = i + 1
                while j < len(order) and order[j] != b:
                    if order[j] in seen:
                        j = -1
                        break
                    seen.add(order[j])
                    saw_via = saw_via or order[j] == via_name
                    j += 1
                if 0 <= j < len(order) and not saw_via:
                    j = -1
            if j < 0 or j >= len(order):
                i += 1
                continue
            if any(not (j <= s or i >= e) for s, e in claimed):
                overlapped += 1            # a span already claimed these moves
                i += 1
                continue
            runs[i] = (j, span)
            claimed.append((i, j))
            hits += 1
            i = j
        if not hits:
            warnings.append(
                f"arc_moves span {pair} overlaps {overlapped} run(s) another "
                "span already claimed; ignored" if overlapped else
                f"arc_moves span {pair} matched no run of cleaning_sequence; "
                "ignored")
    return runs
