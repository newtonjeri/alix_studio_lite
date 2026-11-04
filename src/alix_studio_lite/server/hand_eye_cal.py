"""The right wrist camera's eye-in-hand calibration on the lab robot, ROS-free (the user, 4 Nov 2026: "Let us run the
camera calibration, we will use the rest pose, inspection pose 1 and inspection pose 4, to avoid collision with the
environment ... in those 3 poses we can have orientation changes, so that they give us different views"; the lab's
technician had re-fixed the camera on its mount that afternoon).

RealMan's method as ALIX ported it (ALIX Resource/reports/2026-10-13_hand_eye_calibration; alix_perception_core.hand_eye,
vendored): a chessboard (here 10 x 7 squares of 30 mm, 9 x 6 inner corners) lying still on the closed lid, seen from 15 or
more arm poses whose END frame (``R_CameraHolderLink``, the J7 output the URDF hangs the D435 on) turns more than 30 deg
about each of its axes; per image the board's pose in the colour camera; the camera in the END frame by
``cv2.calibrateHandEye`` (five methods), refined over every corner; the board's spread in base_link as the error the
robot sees. ALIX's capture tool reads ROS topics and TF; Lite reads the D435 itself (camera_d435) and the arm's joints
from its controller through the same URDF (robot_fk), and writes ALIX's session format (``samples.yaml`` + ``images/``),
so ALIX's own offline solve (``tools/perception/compute_hand_eye.py``, vendored) solves it, here or in ALIX.

THE VIEWS (``plan``; the user's choice, 4 Nov: Lite plans them; the moves are verified offline, not confirmed one by one:
"you can do that offline, because we know how movej works ... you also have the scene ... the walls"): around each of the
three named poses (ALIX's group states rest_pose, inspection_pose_1 and inspection_pose_4; the right pole at
``POLE_MM``), turns of the wrist (J5, J6, J7) with a little of J1 and J4, each kept only when
- the whole board (``board_spec``; 10 x 7 squares of 30 mm unless told otherwise), turned ANY way on the lid and up to
  the plan's slack (``SLACKS_M``) from the lid's middle, projects inside the colour image (1280 x 720 at the D435's
  factory focal length, ``MARGIN_PX`` in), between ``RANGE_M``, tilted ``TILT_DEG`` to the image (RealMan: never
  square-on);
- every right-arm frame stays ``ABOVE_LID_M`` above the lid's top over the commode;
- the arm's straight joint path to it from the view before is clear of the room's walls, the commode with its lid
  closed, the board lying on it (``with_board``) and the left arm where it stands (``collision_check.check_states``, S2's own check; the hand touching the
  commode refuses it here, it is not cleaning; the bin's events are left out: the dustbin is not on the lab robot);
then ``VIEWS_PER_POSE`` a pose that turn the END frame as differently as possible (RealMan: more than 30 deg about
each of its axes across the set), 15 views in all, rest's first, then pose 1's, then pose 4's; the plan ends at rest.
From rest the camera looks straight down 330 mm right of the commode: its views turn toward the board.

THE RUN (``run_session``, one job; Real asks once, as every real run does): every view in turn: the move checked again
from where the arms stand (``leg_clear``: a MoveJ is straight in the joints, so the path checked is the path run), the
MoveJ at ``CAL_SPEED_PCT``, the arm still (two readings ``STILL_WINDOW_S`` apart within ``STILL_DEG``, ALIX's capture
tool's rule), two colour frames 0.3 s apart with the board found in both and moved under ``STILL_PX`` (ALIX's), the
joints and pole read then, the END frame's pose by FK, the image (PNG) and its row saved at once; back to rest; the
solve. Offline before it: Visual sim shows the tour in 3D (``tour_md``), Hardware sim runs the same MoveJs in the
controllers' SIM mode. SOLVE (``solve``): ALIX's compute_hand_eye.run: the five methods, the refinement,
leave-one-out, ALIX's acceptance (board spread <= 3 mm RMS, reprojection <= 1.5 px, the rotations enough).

NOTHING IS APPLIED: the result says the correction (mm and deg in the END frame, and as the URDF mount's <origin>);
applying it is the user's decision (ALIX's report), and the place references (place_correct, taken with the camera
where it was) are re-taken after it.
"""
from __future__ import annotations

import json
import math
import threading
import time
from datetime import datetime
from pathlib import Path

import numpy as np

from .. import vendor_paths
from .jobs import UserError

BOARD_COLS, BOARD_ROWS, BOARD_SQUARE_M = 11, 8, 0.015      # the lab's board: inner corners of 12 x 9 squares of 15 mm
BOARD_PLATE_MM = 200.0                 # ... on a 200 x 200 mm plate: its footnote "GP200 12*9-15" (the user, 4 Nov)
# The user, 4 Nov: ALIX's A3 board (12 x 9 squares of 30 mm, 360 x 270 mm) is never seen whole from these three poses
# (at the inspection poses' 150 mm pole the camera is 0.5 m above the lid; at 300 mm only 11 views fit, 29 deg about
# the END frame's Y); the right pole at 250 mm. The lab's GP200 plate (180 x 135 mm of squares) gives 15 views, 50 / 43
# / 178 deg about X / Y / Z, 0.54-0.69 m (a square 20-25 px), and may lie 80 mm from the lid's middle. Any other board
# is given by its squares, its measured square and its plate (``board_spec``; the A3 print of 10 x 7 squares of 30 mm,
# Resources/Calibration/board_10x7_30mm.svg, say); its rotation on the lid is free (the user's photo, 4 Nov: "it is
# just for illustration, not the exact placement position on top of the lid").
BOARD_CENTRE_M = (0.70, 0.0, 0.452)    # on the closed lid: 4 Nov's frames put its top at 0.449 m, x 0.49-1.00, y +-0.17
SLACKS_M = (0.08, 0.06, 0.04, 0.02, 0.0)   # how far from the lid's middle it may lie: the largest a full plan allows
BOARD_BORDER_M, BOARD_THICK_M = 0.015, 0.010    # a print's backing round its squares (when no plate size is given), the
                                                 # thickness: the obstacle
BASES = ("rest_pose", "inspection_pose_1", "inspection_pose_4")
POLE_MM = 250.0                        # the user, 4 Nov (the inspection poses' own 150 mm puts the camera too near)
IMAGE_W, IMAGE_H, FOCAL_PX = 1280, 720, 904.5   # the D435's colour stream as Lite opens it (3 Nov: fx 904.5)
MARGIN_PX = 20
RANGE_M = (0.35, 0.85)
TILT_DEG = (15.0, 55.0)                # RealMan: tilted to the image, never square-on; past 55 deg the corners blur
ABOVE_LID_M = 0.08
LID_FOOTPRINT = ((0.45, 1.10), (-0.25, 0.25))   # base_link x, y over which the arm keeps ABOVE_LID_M over the lid
VIEWS_PER_POSE = 5
CAL_SPEED_PCT = 40                     # the user, 4 Nov: "Let us set the speed at 40%"
VIEW_HOLD_S = 1.0                      # the 3D preview's stop at each view (the stillness wait and two frames 0.3 s apart)
BOARD_RETRIES = 1                      # a view whose board is not found is taken once more, 1 s later, then left out
PATH_STEP_DEG = 2.0                    # a path's samples: its largest joint change in steps of at most this
OFFSETS_DEG = dict(j1=(-15.0, 0.0, 15.0), j4=(-10.0, 0.0, 10.0), j5=(-40.0, -20.0, 0.0, 20.0, 40.0),
                   j6=(-30.0, -15.0, 0.0, 15.0, 30.0), j7=(-90.0, -60.0, -30.0, 0.0, 30.0, 60.0, 90.0))
LIMIT_MARGIN_DEG = 3.0
STILL_DEG, STILL_WINDOW_S, STILL_TIMEOUT_S, STILL_PX = 0.011, 0.5, 10.0, 0.5
END_FRAME = "R_CameraHolderLink"
COLOUR_FRAME, MOUNT_FRAME, DEPTH_FRAME = "camera_color_optical_frame", "camera_bottom_screw_frame", "camera_depth_optical_frame"
CHECK_FRAMES = ("R_Link4", "R_Link5", "R_Link6", "R_Link7", "R_CameraHolderLink", "camera_link", "IRH_plam",
                "IRH_right_thumb_tip", "IRH_right_index_tip", "IRH_right_middle_tip", "IRH_right_ring_tip",
                "IRH_right_little_tip")


# ── the board and the camera ──────────────────────────────────────────────

def board_spec(squares_x: int = BOARD_COLS + 1, squares_y: int = BOARD_ROWS + 1, square_mm: float = BOARD_SQUARE_M * 1000,
               plate_mm: float | None = BOARD_PLATE_MM) -> dict:
    """A chessboard by its squares along and across, its MEASURED square and its plate (the longer side, mm; None: a
    print on a backing BOARD_BORDER_M wider than its squares): {squares, cols, rows (inner corners), square_m, size_m,
    radius_m (the squares' half diagonal: what any rotation of them sweeps), plate_m (the obstacle's half diagonal)}.
    RealMan's boards are even x odd squares (ALIX's canonical_order tells the board's ends apart by its two outer
    corners' colours)."""
    sx, sy, sq = int(squares_x), int(squares_y), float(square_mm) / 1000.0
    if sx < 4 or sy < 4 or (sx + sy) % 2 == 0:
        raise UserError(f"A board of {sx} x {sy} squares cannot be used: one count must be even and the other odd "
                        "(so its two outer corners differ in colour), at least 4 each.", "Count the squares again.")
    if not 0.010 <= sq <= 0.060:
        raise UserError(f"A square of {square_mm:g} mm is not a calibration board's.", "Measure ten squares with calipers and divide by ten.")
    radius = math.hypot(sx * sq, sy * sq) / 2.0
    if plate_mm is not None:
        plate = float(plate_mm) / 1000.0
        if plate < max(sx, sy) * sq or plate > 0.6:
            raise UserError(f"A plate of {plate_mm:g} mm cannot hold {sx} x {sy} squares of {square_mm:g} mm.",
                            "Measure the plate's longer side.")
        plate_half = plate / math.sqrt(2.0)                  # a square plate's half diagonal (a longer one: its side's)
    else:
        plate, plate_half = None, radius + BOARD_BORDER_M
    return dict(squares=[sx, sy], cols=sx - 1, rows=sy - 1, square_m=sq, size_m=[sx * sq, sy * sq], radius_m=radius,
                plate_m=plate, plate_half_m=max(plate_half, radius))


def he_board(board: dict):
    vendor_paths.activate()
    from alix_perception_core import hand_eye as he

    return he.Board(int(board["cols"]), int(board["rows"]), float(board["square_m"]))


def board_outline(board: dict, slack: float) -> np.ndarray:
    """What the camera must see whole, whatever the board's turn on the lid and up to ``slack`` from the lid's middle:
    a circle round the middle of the board's half diagonal + ``slack`` (16 points, base_link)."""
    r = float(board["radius_m"]) + float(slack)
    cx, cy, cz = BOARD_CENTRE_M
    a = np.linspace(0.0, 2.0 * np.pi, 16, endpoint=False)
    return np.stack([cx + r * np.cos(a), cy + r * np.sin(a), np.full(16, cz)], axis=1)


def with_board(scene: dict, board: dict, slack: float) -> dict:
    """The collision scene with the board on the closed lid as one more box (the user, 4 Nov: "we know that the
    checkerboard will be placed on the lid"): a square as wide as the board turned any way with its border, ``slack``
    more each side, BOARD_THICK_M high; every link of the arm, the hand too, touching it is a collision. Only with the
    measured room's walls (collision_check takes the room's boxes from there); without them the lid alone stands for it."""
    if not scene.get("walls"):
        return dict(scene)
    side = 2.0 * (float(board.get("plate_half_m") or board["radius_m"] + BOARD_BORDER_M) + float(slack))
    cx, cy, cz = BOARD_CENTRE_M
    box = dict(name="calibration board", center=[cx, cy, cz + BOARD_THICK_M / 2], size=[side, side, BOARD_THICK_M])
    return dict(scene, walls=[w for w in scene["walls"] if w.get("name") != "calibration board"] + [box])


def nominal_K() -> np.ndarray:
    return np.array([[FOCAL_PX, 0.0, IMAGE_W / 2], [0.0, FOCAL_PX, IMAGE_H / 2], [0.0, 0.0, 1.0]])


MIN_AXIS_ROTATION_DEG, MIN_SAMPLES = 30.0, 15      # RealMan's (alix_perception_core.hand_eye's): > 30 deg about each axis


def diversity(T_base_end) -> dict:
    """ALIX's hand_eye.diversity without OpenCV (planning and the 3D need none): how much the END frame turned
    between samples, per END-frame axis (RealMan: > 30 deg about each of X, Y, Z), and the largest single turn."""
    from scipy.spatial.transform import Rotation

    Rs = [np.asarray(T, float)[:3, :3] for T in T_base_end]
    per_axis, largest = np.zeros(3), 0.0
    for i in range(len(Rs)):
        for j in range(i + 1, len(Rs)):
            v = np.degrees(Rotation.from_matrix(Rs[i].T @ Rs[j]).as_rotvec())
            per_axis = np.maximum(per_axis, np.abs(v))
            largest = max(largest, float(np.linalg.norm(v)))
    ok = bool(len(Rs) >= MIN_SAMPLES and np.all(per_axis >= MIN_AXIS_ROTATION_DEG))
    return dict(samples=len(Rs), max_rotation_about_end_xyz_deg=[round(float(a), 1) for a in per_axis],
                largest_relative_rotation_deg=round(largest, 1), enough=ok)


def need_opencv(what: str) -> None:
    """The capture and the solve read the board with OpenCV (ALIX's hand_eye and compute_hand_eye import cv2): refused
    before anything moves when Lite's Python has none. Planning, the 3D tour and Hardware sim need none."""
    try:
        import cv2  # noqa: F401
    except ImportError:
        raise UserError(f"{what} needs OpenCV in Lite's Python (it finds the board in the images and solves), and it is "
                        "not installed.", "Run python3 tools/setup_lite.py (its optional packages bring it), then start Lite "
                        "again. Planning, the 3D tour and Hardware sim work without it.", code="setup") from None


# ── the robot's frames ───────────────────────────────────────────────────

def named_poses() -> dict:
    """{name: [7 deg]} of the right arm's three calibration poses, from ALIX's group states (vendored)."""
    import yaml

    gs = vendor_paths.ALIX / "ros/manipulation/alix_moveit_config/config/group_states"
    arms = yaml.safe_load((gs / "arms_group_states.yaml").read_text(encoding="utf-8"))["group_states"]["rm75_arm_right"]
    poles = yaml.safe_load((gs / "arm_poles_group_states.yaml").read_text(encoding="utf-8"))["group_states"]["arm_pole_right"]
    out = {}
    for name in BASES:
        st = arms.get(name) or poles.get(name)
        if st is None:
            raise UserError(f"ALIX's group states have no right-arm pose {name!r}.")
        out[name] = [round(math.degrees(float(st[f"R_joint{i}"])), 3) for i in range(1, 8)]
    return out


def limits_deg() -> tuple:
    from ..motion.limits import load_limits

    lo, hi = zip(*load_limits("right").pos_deg)
    return np.asarray(lo, float), np.asarray(hi, float)


def links_at(q_deg, pole_mm: float = POLE_MM, left: dict | None = None) -> dict:
    """Every link's pose in base_link with the right arm at ``q_deg`` (the left as ``left`` reads, if given)."""
    from . import robot_fk

    readings = {"right": {"q_deg": list(q_deg), "pole_mm": float(pole_mm)}}
    if left:
        readings["left"] = left
    values, _ = robot_fk.joint_values(readings)
    return robot_fk.model().link_world_transforms(values)


def nominal_frames() -> dict:
    """The URDF's nominal camera in the END frame (ALIX's session header ``nominal``): T_end_cam (colour optical),
    T_end_mount (the mount's screw frame), T_end_depth (depth optical); the same at any arm pose."""
    L = links_at([0.0] * 7)
    inv_end = np.linalg.inv(L[END_FRAME])
    return dict(T_end_cam=inv_end @ L[COLOUR_FRAME], T_end_mount=inv_end @ L[MOUNT_FRAME], T_end_depth=inv_end @ L[DEPTH_FRAME])


# ── one candidate view ───────────────────────────────────────────────────

def view_of(T_base_cam: np.ndarray, board: dict, slack: float, K: np.ndarray | None = None, size=(IMAGE_W, IMAGE_H)) -> dict:
    """Whether the board (turned any way, ``slack`` off the lid's middle) is seen whole from the colour camera at
    ``T_base_cam``: {ok, why, margin_px, range_m, tilt_deg}."""
    K = nominal_K() if K is None else K
    inv = np.linalg.inv(T_base_cam)
    P = (inv[:3, :3] @ board_outline(board, slack).T).T + inv[:3, 3]
    if np.any(P[:, 2] <= 0.05):
        return dict(ok=False, why="the board is behind the camera")
    uv = (K @ (P / P[:, 2:3]).T).T[:, :2]
    margin = float(min(uv[:, 0].min(), uv[:, 1].min(), size[0] - uv[:, 0].max(), size[1] - uv[:, 1].max()))
    centre = inv[:3, :3] @ np.asarray(BOARD_CENTRE_M) + inv[:3, 3]
    rng = float(np.linalg.norm(centre))
    n_cam = inv[:3, :3] @ np.array([0.0, 0.0, 1.0])
    tilt = float(np.degrees(np.arccos(min(1.0, abs(n_cam[2])))))       # the board's normal against the optical axis
    out = dict(margin_px=round(margin, 1), range_m=round(rng, 3), tilt_deg=round(tilt, 1))
    if margin < MARGIN_PX:
        return dict(out, ok=False, why=f"the board leaves the image ({margin:.0f} px in, {MARGIN_PX} needed)")
    if not RANGE_M[0] <= rng <= RANGE_M[1]:
        return dict(out, ok=False, why=f"the board {rng:.2f} m away (between {RANGE_M[0]} and {RANGE_M[1]} m)")
    if not TILT_DEG[0] <= tilt <= TILT_DEG[1]:
        return dict(out, ok=False, why=f"the board tilted {tilt:.0f} deg to the image ({TILT_DEG[0]:.0f}-{TILT_DEG[1]:.0f} needed)")
    return dict(out, ok=True, why="")


def above_lid(L: dict) -> tuple:
    """(ok, the lowest clearance over the lid, m): every CHECK_FRAMES frame over LID_FOOTPRINT at least ABOVE_LID_M
    above the lid's top."""
    top = BOARD_CENTRE_M[2]
    (x0, x1), (y0, y1) = LID_FOOTPRINT
    low = math.inf
    for name in CHECK_FRAMES:
        T = L.get(name)
        if T is None:
            continue
        x, y, z = T[:3, 3]
        if x0 <= x <= x1 and y0 <= y <= y1:
            low = min(low, z - top)
    return (low >= ABOVE_LID_M, low)


def candidates(base: str, q0, board: dict, slack: float, left: dict | None = None) -> list[dict]:
    """Every offset of OFFSETS_DEG around ``q0`` within the joint limits whose view and lid clearance pass."""
    import itertools

    lo, hi = limits_deg()
    out = []
    q0 = np.asarray(q0, float)
    for d1, d4, d5, d6, d7 in itertools.product(*(OFFSETS_DEG[k] for k in ("j1", "j4", "j5", "j6", "j7"))):
        q = q0 + np.array([d1, 0.0, 0.0, d4, d5, d6, d7])
        if np.any(q < lo + LIMIT_MARGIN_DEG) or np.any(q > hi - LIMIT_MARGIN_DEG):
            continue
        if abs(q[5]) < 10.0:                    # the wrist at its singularity (J6 near 0): no view taken there
            continue
        L = links_at(q, POLE_MM, left)
        v = view_of(L[COLOUR_FRAME], board, slack)
        if not v["ok"]:
            continue
        ok, low = above_lid(L)
        if not ok:
            continue
        out.append(dict(base=base, q_deg=[round(float(a), 3) for a in q], offsets_deg=dict(j1=d1, j4=d4, j5=d5, j6=d6, j7=d7),
                        T_base_end=L[END_FRAME], above_lid_m=round(low, 3) if math.isfinite(low) else None,
                        **{k: v[k] for k in ("margin_px", "range_m", "tilt_deg")}))
    return out


def _rot_deg(Ra, Rb) -> float:
    c = (np.trace(Ra.T @ Rb) - 1.0) / 2.0
    return float(np.degrees(np.arccos(np.clip(c, -1.0, 1.0))))


def choose(per_base: dict, n: int = VIEWS_PER_POSE, refused: set | None = None) -> list[dict]:
    """``n`` views a pose, turning the END frame as differently as possible over the whole set (greedy: each next view
    the one farthest, by its smallest rotation from those chosen; a pose's first view the one nearest the pose)."""
    refused = refused or set()
    chosen: list[dict] = []
    for base, cands in per_base.items():
        pool = [c for c in cands if (base, tuple(c["q_deg"])) not in refused]
        if not pool:
            continue
        first = min(pool, key=lambda c: sum(abs(v) for v in c["offsets_deg"].values()))
        picked = [first]
        while len(picked) < n:
            best, best_d = None, -1.0
            for c in pool:
                if any(c is p for p in picked):
                    continue
                d = min(_rot_deg(c["T_base_end"][:3, :3], p["T_base_end"][:3, :3]) for p in chosen + picked)
                if d > best_d:
                    best, best_d = c, d
            if best is None:
                break
            picked.append(best)
        # the pose's views in a short tour from its first: each next the nearest in the joints
        tour, rest = [picked[0]], picked[1:]
        while rest:
            k = min(range(len(rest)), key=lambda i: float(np.abs(np.subtract(rest[i]["q_deg"], tour[-1]["q_deg"])).max()))
            tour.append(rest.pop(k))
        chosen.extend(tour)
    return chosen


def path(qa, qb) -> list:
    """A straight joint path from ``qa`` to ``qb`` (the MoveJ's), sampled every PATH_STEP_DEG at most."""
    qa, qb = np.asarray(qa, float), np.asarray(qb, float)
    k = max(2, int(math.ceil(float(np.abs(qb - qa).max()) / PATH_STEP_DEG)) + 1)
    return [qa + (qb - qa) * (t / (k - 1)) for t in range(k)]


def check_paths(views: list[dict], q_start, ends_at, robot_state: dict, room: dict) -> list[dict]:
    """Each leg of the tour (from ``q_start`` through ``views`` to ``ends_at``) against the room, the commode with its
    lid closed and the left arm where it stands: [{leg, ok, why}] (leg i goes to views[i]; the last to ``ends_at``)."""
    from . import collision_check, staging

    home = dict(room["home"])
    left = robot_state.get("left") or {}
    if left.get("q_deg") is not None:
        home = staging.start_values("left", home, dict(pole_mm=left.get("pole_mm"), hand=left.get("hand"), q_deg=left["q_deg"]))
    hand = list((robot_state.get("right") or {}).get("hand") or [1000] * 6)
    stops = [q_start] + [v["q_deg"] for v in views] + [ends_at]
    q, leg_of = [], []
    for i in range(len(stops) - 1):
        for p in path(stops[i], stops[i + 1]):
            q.append([float(a) for a in p])
            leg_of.append(i)
    n = len(q)
    times = [0.02 * k for k in range(n)]
    arms = {"right": dict(pole=[POLE_MM] * n, hand=[hand] * n, q=q, known=[True] * n)}
    out = collision_check.check_states(times, arms, home, room["scene"], room["fid"], ["lid_closed"] * n,
                                       int(room.get("cubicle_mm") or 1400))
    bad: dict = {}
    for e in out.get("events") or []:
        what = [w for w in e.get("what") or [] if w != "bin"]      # the dustbin is not on the lab robot (4 Nov)
        if not what:
            continue
        i0 = int(round(e["first_t"] / 0.02))
        leg = leg_of[min(i0, n - 1)]
        hits = sorted({str(p.get("object")) for p in e.get("pairs") or [] if p.get("what") != "bin"})
        on = ", ".join(("the board on the lid" if "board" in h else h) for h in hits) or ", ".join(what)
        bad.setdefault(leg, f"the arm would touch {on}")
    touched = (out.get("contact") or {}).get("right") or {}
    if touched.get("samples"):
        i0 = int(round(float(touched.get("first_t") or 0.0) / 0.02))
        bad.setdefault(leg_of[min(i0, n - 1)], "the hand would touch the commode")
    return [dict(leg=i, ok=i not in bad, why=bad.get(i, "")) for i in range(len(stops) - 1)]


def plan(robot_state: dict, room: dict, board: dict | None = None, per_pose: int = VIEWS_PER_POSE) -> dict:
    """The session's views (module docstring): {views, poses, diversity, refused, ends_at, board, slack_m, scene, ...}.
    Runs in the worker process (FCL). ``robot_state``: {side: {q_deg, pole_mm, hand}} as read now; ``room``: {home,
    scene, fid, cubicle_mm} (the board is put in its scene here); ``board``: board_spec's. The largest of SLACKS_M a
    full plan allows (every pose its views, RealMan's 30 deg about each axis) is the board's tolerance on the lid."""
    vendor_paths.activate()
    t0 = time.monotonic()
    board = board or board_spec()
    poses = named_poses()
    left = robot_state.get("left") or None
    for slack in SLACKS_M:
        per_base = {b: candidates(b, poses[b], board, slack, left) for b in BASES}
        views = choose(per_base, per_pose)
        if len(views) == per_pose * len(BASES) and diversity([v["T_base_end"] for v in views])["enough"]:
            break
    else:
        raise UserError(f"No full set of views sees a {board['squares'][0]} x {board['squares'][1]} board of "
                        f"{board['square_m'] * 1000:g} mm squares from rest, inspection pose 1 and inspection pose 4 "
                        f"({sum(len(c) for c in per_base.values())} views fit, {per_pose * len(BASES)} with enough wrist turn needed).",
                        "Use a smaller board (fewer or smaller squares).")
    room = dict(room, scene=with_board(room["scene"], board, slack))
    right = robot_state.get("right") or {}
    q_start = right.get("q_deg") or poses["rest_pose"]
    refused: set = set()
    notes = []
    for _ in range(8):
        views = choose(per_base, per_pose, refused)
        legs = check_paths(views, q_start, poses["rest_pose"], robot_state, room)
        bad = [g for g in legs if not g["ok"]]
        if not bad:
            break
        for g in bad:
            if g["leg"] < len(views):
                v = views[g["leg"]]
                refused.add((v["base"], tuple(v["q_deg"])))
                notes.append(f"{v['base']} {v['offsets_deg']}: {g['why']}")
            else:
                notes.append(f"the way back to rest: {g['why']}")
        if all(g["leg"] >= len(views) for g in bad):
            break
    else:
        legs = check_paths(views, q_start, poses["rest_pose"], robot_state, room)
    if any(not g["ok"] for g in legs):
        raise UserError("No collision-free tour of the calibration views was found: " + "; ".join(notes[-3:]) + ".",
                        "Check the room's walls in Prepare and that the left arm stands at its rest, then plan again.")
    div = diversity([v["T_base_end"] for v in views])
    return dict(views=[{k: v[k] for k in v if k != "T_base_end"} | dict(index=i, T_base_end=v["T_base_end"].tolist())
                       for i, v in enumerate(views)],
                poses=poses, candidates={b: len(c) for b, c in per_base.items()}, diversity=div,
                refused=notes, ends_at=poses["rest_pose"], pole_mm=POLE_MM, speed_pct=CAL_SPEED_PCT,
                board=dict(board, centre_m=list(BOARD_CENTRE_M)), slack_m=slack, scene=room["scene"],
                seconds=round(time.monotonic() - t0, 2))


# ── the session ──────────────────────────────────────────────────────────

class Session:
    """One calibration session: its folder (``samples.yaml``, ``images/``, ``plan.json``, ``hand_eye_result.yaml``), its
    plan, the views taken. One a robot at a time (``SESSIONS``)."""

    def __init__(self, folder: Path, planned: dict, room: dict | None = None):
        self.folder = Path(folder)
        self.plan = planned
        self.board = dict(planned["board"])  # board_spec's, the square as measured
        # {home, scene (with the board on the lid), fid, cubicle_mm}: each move is checked again from where it starts
        self.room = None if room is None else dict(room, scene=planned.get("scene") or room["scene"])
        self.square_m = float(self.board["square_m"])
        self.rows: list[dict] = []
        self.attempted: list[int] = []   # the planned views moved to (taken or left out), in order
        self.missed: list[int] = []      # the planned views whose board was not found (left out of the solve)
        self.header: dict | None = None
        self.notes: list[str] = []
        self.result: dict | None = None
        self.lock = threading.Lock()
        self.folder.mkdir(parents=True, exist_ok=True)
        (self.folder / "plan.json").write_text(json.dumps(planned, indent=1), encoding="utf-8")

    @property
    def next_index(self) -> int:
        """The first planned view not moved to yet (a run goes on from here after a STOP)."""
        done = set(self.attempted)
        return next((v["index"] for v in self.plan["views"] if v["index"] not in done), len(self.plan["views"]))

    def view(self) -> dict:
        from_div = None
        if len(self.rows) >= 2:
            from_div = diversity([np.asarray(r["T_base_end"], float) for r in self.rows])
        return dict(folder=str(self.folder), views=len(self.plan["views"]), taken=len(self.rows), next=self.next_index,
                    missed=list(self.missed), speed_pct=CAL_SPEED_PCT,
                    planned=[{k: v[k] for k in ("index", "base", "offsets_deg", "range_m", "tilt_deg", "margin_px")}
                             for v in self.plan["views"]],
                    rows=[{k: r[k] for k in ("index", "view", "board_pose_rms_px", "board_tilt_deg", "board_range_m", "still_px")}
                          for r in self.rows],
                    diversity=from_div, planned_diversity=self.plan.get("diversity"), notes=self.notes[-6:],
                    result=summary(self.result) if self.result else None, square_mm=round(self.square_m * 1000, 3),
                    board=dict(squares=self.board["squares"], square_mm=round(self.square_m * 1000, 3),
                               plate_mm=None if self.board.get("plate_m") is None else round(self.board["plate_m"] * 1000),
                               tolerance_mm=round(float(self.plan.get("slack_m") or 0.0) * 1000),
                               centre_m=list(BOARD_CENTRE_M), radius_m=round(float(self.board["radius_m"]), 4)))

    def write(self) -> None:
        vendor_paths.activate()
        import yaml

        che = _compute()
        data = dict(self.header or {}, samples=self.rows)
        tmp = self.folder / "samples.yaml.tmp"
        tmp.write_text(yaml.safe_dump(che.plain(data), sort_keys=False, width=140), encoding="utf-8")
        tmp.replace(self.folder / "samples.yaml")

    def header_from(self, camera) -> dict:
        c = camera._cintr
        dist = (camera.colour_distortion or {}).get("coeffs") or [0.0] * 5
        nom = nominal_frames()
        self.header = dict(created=datetime.now().isoformat(timespec="seconds"), tool="ALIX Studio Lite hand_eye_cal",
                           board=dict(cols=int(self.board["cols"]), rows=int(self.board["rows"]), square_m=self.square_m),
                           end_frame=END_FRAME, camera_frame=COLOUR_FRAME, base_frame="base_link",
                           camera_info=dict(width=int(c.width), height=int(c.height),
                                            K=[c.fx, 0.0, c.cx, 0.0, c.fy, c.cy, 0.0, 0.0, 1.0], D=list(dist),
                                            distortion_model=(camera.colour_distortion or {}).get("model")),
                           camera=getattr(camera, "device", None), nominal=nom)
        return self.header


SESSIONS: dict = {}          # id(robot) -> Session


def _compute():
    """ALIX's compute_hand_eye (vendored tools/perception), as a module."""
    import importlib.util
    import sys

    vendor_paths.activate()
    path_ = vendor_paths.ALIX / "tools/perception/compute_hand_eye.py"
    name = "alix_compute_hand_eye"
    if name not in sys.modules:
        spec = importlib.util.spec_from_file_location(name, path_)
        mod = importlib.util.module_from_spec(spec)
        sys.modules[name] = mod
        spec.loader.exec_module(mod)
    return sys.modules[name]


def folder_for(robot) -> Path:
    log = getattr(robot, "log", None)
    root = Path(log.root).parent if log is not None and getattr(log, "root", None) else Path.cwd()
    return root / "Calibration" / datetime.now().strftime("hand_eye_%Y%m%d_%H%M%S")


def readings_now(robot) -> dict:
    """{side: {q_deg, pole_mm, hand}} as each linked arm reads now (a fresh read); {} for an arm not linked."""
    out = {}
    for side in ("left", "right"):
        link = robot.links.get(side)
        if link is None:
            continue
        fb = link.read_fresh() if hasattr(link, "read_fresh") else (link.view().get("feedback") or {})
        out[side] = dict(q_deg=fb.get("joints_deg"), pole_mm=fb.get("pole_mm"), hand=fb.get("hand"))
    return out


def wait_still(robot, side: str = "right", job=None) -> dict:
    """The arm's joints twice STILL_WINDOW_S apart within STILL_DEG (ALIX's capture tool's rule); its last reading."""
    link = robot.links[side]
    t_end = time.monotonic() + STILL_TIMEOUT_S
    prev = link.read_fresh()
    while time.monotonic() < t_end:
        (job.sleep if job is not None else time.sleep)(STILL_WINDOW_S)
        cur = link.read_fresh()
        a, b = prev.get("joints_deg"), cur.get("joints_deg")
        if a and b and float(np.max(np.abs(np.subtract(a, b)))) < STILL_DEG:
            return cur
        prev = cur
    raise UserError(f"The right arm did not come to rest within {STILL_TIMEOUT_S:g} s (joints still changing).",
                    "Check the arm, then take this view again.")


def capture(camera, board: dict, out: Path, index: int) -> tuple:
    """Two colour frames 0.3 s apart, the board found in both and still (ALIX's capture tool): (the second image, its
    corners, the shift px, the second shot: its depth is kept with the view)."""
    import cv2

    vendor_paths.activate()
    from alix_perception_core import hand_eye as he

    b = he.Board(int(board["cols"]), int(board["rows"]), float(board["square_m"]))
    s1 = camera.shot()
    time.sleep(0.3)
    s2 = camera.shot()
    if s1.colour is None or s2.colour is None:
        raise UserError("The wrist camera sent no colour frame.", "Check its USB cable, then take this view again.")
    c1 = he.detect(cv2.cvtColor(s1.colour, cv2.COLOR_BGR2GRAY), b)
    c2 = he.detect(cv2.cvtColor(s2.colour, cv2.COLOR_BGR2GRAY), b)
    if c1 is None or c2 is None:
        cv2.imwrite(str(out / f"images/miss_{index:02d}.png"), s2.colour)
        raise UserError("The board was not found in the image (the whole board in view, both its outer corners, no glare?).",
                        f"The image is kept as images/miss_{index:02d}.png; take this view again, or skip it.")
    shift = float(np.max(np.linalg.norm(c1.reshape(-1, 2) - c2.reshape(-1, 2), axis=1)))
    if shift > STILL_PX:
        raise UserError(f"The board moved {shift:.2f} px between two frames 0.3 s apart (vibration?).", "Take this view again.")
    return s2.colour, c2, shift, s2


def depth_at_board(aligned, corners, T_cam_board, board) -> dict | None:
    """The depth aligned to the colour image at the board's corners against the corners' distance from the board's
    pose (PnP on the colour image): the median and spread of depth - PnP, mm. A check of the depth against the colour
    camera from the same view, independent of the arm. None without an aligned depth or with too few readings."""
    if aligned is None:
        return None
    a = np.asarray(aligned)
    px = np.round(np.asarray(corners, float).reshape(-1, 2)).astype(int)
    inside = (px[:, 0] >= 0) & (px[:, 0] < a.shape[1]) & (px[:, 1] >= 0) & (px[:, 1] < a.shape[0])
    obj = np.c_[board.object_points(), np.ones(len(px))]
    z = (np.asarray(T_cam_board, float) @ obj.T)[2] * 1000.0
    d = np.where(inside, a[np.clip(px[:, 1], 0, a.shape[0] - 1), np.clip(px[:, 0], 0, a.shape[1] - 1)], 0).astype(float)
    ok = inside & (d > 0)
    if ok.sum() < 10:
        return None
    diff = d[ok] - z[ok]
    return dict(median_mm=round(float(np.median(diff)), 1), spread_mm=round(float(np.percentile(np.abs(diff - np.median(diff)), 90)), 1),
                corners=int(ok.sum()))


def record(session: Session, k: int, v: dict, image, corners, shift: float, q_deg, pole_mm, shot=None) -> dict:
    """A view's row as ALIX's capture tool writes it (the END frame's pose from the joints read at the capture, by the
    URDF's FK; the board's pose in the camera for its tilt and range), its image saved, the session's samples.yaml
    written at once. The shot's depth is kept beside the image (the user, 4 Nov: "why did we not capture the depth
    image frames as well?"): the native depth (``NN_depth.npy``, z16 mm, its intrinsics in the row) and the depth
    aligned to the colour image (``NN_aligned_depth.npz``, ``depth_mm``), as Lite's check records keep them. ALIX's
    solve reads the colour image only; the aligned depth gives each view's depth-against-board check."""
    import cv2

    from . import robot_fk

    vendor_paths.activate()
    from alix_perception_core import hand_eye as he

    values, _ = robot_fk.joint_values({"right": {"q_deg": list(q_deg), "pole_mm": float(pole_mm)}})
    T_base_end = robot_fk.model().link_world_transforms(values)[END_FRAME]
    info = session.header["camera_info"]
    K = np.asarray(info["K"], float).reshape(3, 3)
    D = np.asarray(info["D"], float)
    T_cam_board, rms = he.board_pose(corners, K, D, he_board(session.board))
    (session.folder / "images").mkdir(parents=True, exist_ok=True)
    with session.lock:
        stem = f"images/{len(session.rows):02d}"
        rel = f"{stem}.png"
        cv2.imwrite(str(session.folder / rel), image)
        row = dict(index=len(session.rows), view=int(k), base=v["base"], offsets_deg=v["offsets_deg"], image=rel,
                   captured=datetime.now().isoformat(timespec="seconds"), T_base_end=T_base_end,
                   joints={**{f"R_joint{i + 1}": math.radians(float(q_deg[i])) for i in range(7)},
                           "R_sliding_plate_joint": float(pole_mm) / 1000.0},
                   board_pose_rms_px=round(float(rms), 3), board_tilt_deg=round(float(he.board_tilt_deg(T_cam_board)), 1),
                   board_range_m=round(float(np.linalg.norm(T_cam_board[:3, 3])), 3), still_px=round(float(shift), 3))
        if shot is not None and getattr(shot, "depth", None) is not None:
            np.save(session.folder / f"{stem}_depth.npy", np.asarray(shot.depth))
            di = shot.intrinsics
            row["depth"] = f"{stem}_depth.npy"
            row["depth_intrinsics"] = dict(fx=di.fx, fy=di.fy, cx=di.cx, cy=di.cy, width=di.width, height=di.height,
                                           unit="mm" if np.asarray(shot.depth).dtype == np.uint16 else "m")
        if shot is not None and getattr(shot, "aligned", None) is not None:
            np.savez_compressed(session.folder / f"{stem}_aligned_depth.npz", depth_mm=np.asarray(shot.aligned))
            row["aligned_depth"] = f"{stem}_aligned_depth.npz"
            check = depth_at_board(shot.aligned, corners, T_cam_board, he_board(session.board))
            if check is not None:
                row["depth_minus_board_mm"] = check
        session.rows.append(row)
        session.write()
    return row


def leg_clear(robot, session: Session, target_deg, what: str) -> None:
    """The move about to be sent, from where the right arm stands NOW to ``target_deg``, with the left arm where it
    stands now (both read fresh), against the session's room (check_paths, in the worker process: FCL); refused
    (UserError, nothing moved) when it is not clear. The plan checked the tour from view to view; this checks the
    move itself (the arms may have been moved since)."""
    from . import workers

    if session.room is None:
        raise UserError("This session has no room to check the moves against.", "Plan the views again.")
    now = readings_now(robot)
    q0 = (now.get("right") or {}).get("q_deg")
    if not q0:
        raise UserError("The right arm's joints were not read.", "Check its connection, then try again.")
    now.setdefault("right", {})["pole_mm"] = POLE_MM
    legs = workers.pool().submit(check_paths, [], q0, list(target_deg), now, session.room).result(timeout=120)
    bad = [g for g in legs if not g["ok"]]
    if bad:
        raise UserError(f"Not moved: the move to {what} would not be clear ({bad[0]['why']}).",
                        "Check where the arms stand (the left at its rest), then plan the views again.")


def camera_of(robot):
    from . import mechanism

    return mechanism.kit(robot).camera


def movej(robot, real, target_deg, sim: bool) -> None:
    """One MoveJ of the right arm at CAL_SPEED_PCT (ALIX's park move, checked against the pose within 2 deg): the
    controller's REAL mode, or its SIM mode in Hardware sim (it runs the move, the arm does not)."""
    originals = real.set_run_mode("SIM" if sim else "REAL")
    try:
        real.move_to([round(float(a) * 1000) for a in target_deg], speed=CAL_SPEED_PCT, restore_pct=robot.found_speed("right"))
    finally:
        real.restore_run_mode(originals)


def run_session(robot, session: Session, mode: str):
    """The whole tour as one job on the right arm (the user, 4 Nov: "No need of us to confirm every movement, you can do
    that offline, because we know how movej works"): from the first view not moved to yet, every planned view (the
    move checked again from where the arms stand, the MoveJ at CAL_SPEED_PCT, the capture), back to rest, then the
    solve. STOP ends it between moves or in one; a later run goes on from the view after the last one moved to.
    Hardware sim: the same moves in the controllers' SIM mode, no pole move and no capture (the camera sees the real
    arm, which stands still). A board not found is tried once more, then the view is left out and the run goes on."""
    from contextlib import nullcontext

    if mode not in ("real", "hwsim"):
        raise UserError("Visual sim does not use the robot: Show the tour in 3D, or switch to Hardware sim or Real.",
                        code="mode")
    if robot.links.get("right") is None:
        raise UserError("The right arm is not connected.", "Connect it in Prepare, then try again.")
    sim = mode != "real"
    if not sim:
        need_opencv("The camera calibration on the robot")
    cam = None if sim else camera_of(robot)
    if cam is not None and not cam.available().get("ok"):
        raise UserError("The wrist camera (D435) is not connected.", "Plug it in, then try again.", code="camera")
    views = session.plan["views"]
    todo = list(views) if sim else [v for v in views if v["index"] >= session.next_index]     # a dry run: all, nothing kept
    n = len(views)

    def work(job):
        robot._same_mode(mode, "the arm did not move")
        real = robot._real("right")
        with (cam.using() if cam is not None else nullcontext()):
            if cam is not None:
                if session.header is None:
                    cam.open()
                    session.header_from(cam)
                pole_now, _ = robot._measured("right")
                if pole_now is None or abs(float(pole_now) - POLE_MM) > 5.0:
                    job.progress(f"Right pole to {POLE_MM:.0f} mm", 0, n)
                    real.pole_to(POLE_MM)
            else:
                job.log(f"Hardware sim: the moves run in the controller's SIM mode; the pole is not moved (the run puts it "
                        f"at {POLE_MM:.0f} mm) and nothing is captured.")
            reached = []
            for v in todo:
                k = v["index"]
                job.check()
                job.progress(f"View {k + 1} of {n}: {v['base'].replace('_', ' ')}, the wrist turned", k, n)
                leg_clear(robot, session, v["q_deg"], f"view {k + 1}")
                movej(robot, real, v["q_deg"], sim)
                if sim:
                    reached.append(k)
                    continue
                for attempt in range(BOARD_RETRIES + 1):
                    fb = wait_still(robot, "right", job)
                    try:
                        image, corners, shift, shot = capture(cam, session.board, session.folder, k)
                        break
                    except UserError as exc:
                        if attempt == BOARD_RETRIES:
                            session.missed.append(k)
                            note = f"view {k + 1}: left out ({exc})"
                            session.notes.append(note)
                            job.log(note, "warn")
                            image = None
                        else:
                            job.sleep(1.0)
                session.attempted.append(k)
                if image is None:
                    continue
                after = robot.links["right"].read_fresh()
                q, pole = after.get("joints_deg") or fb.get("joints_deg"), after.get("pole_mm") or fb.get("pole_mm")
                row = record(session, k, v, image, corners, shift, q, pole, shot)
                note = (f"view {k + 1}: board found, PnP {row['board_pose_rms_px']:.2f} px, tilted {row['board_tilt_deg']:.0f} deg, "
                        f"{row['board_range_m']:.2f} m away")
                session.notes.append(note)
                job.log(note)
            job.check()
            job.progress("Back to rest", n, n)
            leg_clear(robot, session, session.plan["ends_at"], "rest")
            movej(robot, real, session.plan["ends_at"], sim)
        if sim:
            return dict(mode="hwsim", reached=len(reached), views=n,
                        said=f"Hardware sim: the controller ran all {len(reached)} MoveJs and the way back to rest; nothing moved.")
        out = dict(mode="real", taken=len(session.rows), missed=list(session.missed), views=n)
        if len(session.rows) >= 3:
            try:
                out["result"] = solve(session)
            except UserError as exc:          # the views stay taken: Solve again re-solves them
                out["solve_error"] = str(exc)
                job.log(f"not solved: {exc}", "warn")
        return out

    return robot._start("move", f"RIGHT camera calibration: {len(todo)} view(s) at {CAL_SPEED_PCT} %", work, "right", mode)


def back_to_rest(robot, session: Session, mode: str):
    """The right arm back at rest (the plan's ``ends_at``) after a STOP, the move checked from where the arms stand."""
    if robot.links.get("right") is None:
        raise UserError("The right arm is not connected.", "Connect it in Prepare, then try again.")

    def work(job):
        robot._same_mode(mode, "the arm did not move")
        leg_clear(robot, session, session.plan["ends_at"], "rest")
        movej(robot, robot._real("right"), session.plan["ends_at"], mode != "real")
        return dict(at="rest")

    return robot._start("move", "RIGHT back to rest (camera calibration)", work, "right", mode)


def leg_lines(qa, qb, rating_deg_s) -> np.ndarray:
    """A MoveJ from ``qa`` to ``qb`` as 5 ms lines (millideg) for the 3D: straight in the joints, eased, its slowest
    joint at CAL_SPEED_PCT of its rating at the peak (the controller's MoveJ synchronises the joints)."""
    qa, qb = np.asarray(qa, float), np.asarray(qb, float)
    rate = np.asarray(rating_deg_s, float) * CAL_SPEED_PCT / 100.0
    t = max(0.2, float(np.max(np.abs(qb - qa) / rate)) * math.pi / 2)        # an eased move peaks at pi/2 of its mean
    k = max(2, int(round(t / 0.005)))
    f = 0.5 - 0.5 * np.cos(np.pi * np.arange(k) / (k - 1))
    return np.round((qa + (qb - qa) * f[:, None]) * 1000.0).astype(np.int64)


def tour_md(session: Session, rating_deg_s) -> np.ndarray:
    """The tour from its first view to rest as 5 ms lines (the 3D's ``execution_timeline`` adds the move to the first
    view from where the arm stands): each view held VIEW_HOLD_S, each move between at CAL_SPEED_PCT."""
    views = session.plan["views"]
    stops = [v["q_deg"] for v in views] + [session.plan["ends_at"]]
    hold = int(round(VIEW_HOLD_S / 0.005))
    parts = []
    for i, q in enumerate(stops):
        if i:
            parts.append(leg_lines(stops[i - 1], q, rating_deg_s)[1:])
        if i < len(views):
            parts.append(np.repeat(np.round(np.asarray([q], float) * 1000.0).astype(np.int64), hold, axis=0))
    return np.concatenate(parts, axis=0)


SOLVE_TIMEOUT_S = 600

# ALIX's solve in a Python of its own: the lab's Lite of 4 Nov evening had found the board in every image, then
# its solve met an OpenCV without calibrateHandEye. The running Lite had imported OpenCV and torch on several threads
# at once (that start's plan check met torch half imported). A fresh process imports them alone.
_SOLVE = r"""
import importlib.util, json, sys
from pathlib import Path
sys.path.insert(0, sys.argv[1])
from alix_studio_lite import vendor_paths
vendor_paths.activate()
try:
    import cv2
except ImportError:
    print("@@SETUP@@OpenCV is not installed in this Python"); sys.exit(0)
if not hasattr(cv2, "calibrateHandEye"):
    print(f"@@SETUP@@this Python's OpenCV ({getattr(cv2, '__version__', '?')}, {getattr(cv2, '__file__', '?')}) has no "
          "calibrateHandEye"); sys.exit(0)
spec = importlib.util.spec_from_file_location("alix_compute_hand_eye", vendor_paths.ALIX / "tools/perception/compute_hand_eye.py")
che = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = che
spec.loader.exec_module(che)
try:
    res = che.run(Path(sys.argv[2]), quiet=True)
except SystemExit as exc:
    print(f"@@FAILED@@{exc}"); sys.exit(0)
print("@@RESULT@@" + json.dumps(che.plain(res)))
"""


def solve(session: Session) -> dict:
    """ALIX's offline solve of the session (compute_hand_eye.run), in a Python process of its own (``_SOLVE``): writes
    hand_eye_result.yaml beside the samples."""
    import subprocess
    import sys

    if len(session.rows) < 3:
        raise UserError(f"Only {len(session.rows)} view(s) taken; RealMan's method asks for 15 to 20.", "Take more views.")
    src = Path(__file__).resolve().parents[2]
    try:
        done = subprocess.run([sys.executable, "-c", _SOLVE, str(src), str(session.folder)], capture_output=True,
                              text=True, timeout=SOLVE_TIMEOUT_S)
    except subprocess.TimeoutExpired:
        raise UserError(f"The calibration was not solved within {SOLVE_TIMEOUT_S} s.", "Press Solve again.") from None
    said = {k: line[len(k):] for line in done.stdout.splitlines() for k in ("@@RESULT@@", "@@SETUP@@", "@@FAILED@@")
            if line.startswith(k)}
    if "@@SETUP@@" in said:
        raise UserError(f"Solving the calibration needs OpenCV's calibrateHandEye: {said['@@SETUP@@']}.",
                        "Run python3 tools/setup_lite.py (its optional packages bring OpenCV), then press Solve again; "
                        "the views taken are kept.", code="setup")
    if "@@FAILED@@" in said:
        raise UserError(f"The calibration could not be solved: {said['@@FAILED@@']}.")
    if "@@RESULT@@" not in said:
        tail = (done.stderr or done.stdout or "").strip().splitlines()[-1:] or ["no output"]
        raise UserError(f"The calibration's solve stopped: {tail[0][:300]}.", "Press Solve again; the views taken are kept.")
    res = json.loads(said["@@RESULT@@"])
    session.result = res
    return summary(res)


def summary(res: dict) -> dict:
    """The result for the page: the refined correction, the verdict and its evidence."""
    best = (res.get("runs") or {}).get("factory") or next(iter((res.get("runs") or {}).values()), {})
    r = best.get("refined") or {}
    vn = r.get("vs_nominal") or {}
    return dict(accepted=bool((best.get("verdict") or {}).get("accepted")), samples=best.get("samples"),
                diversity=best.get("diversity"),
                translation_mm_in_end=vn.get("translation_mm_in_end"), translation_norm_mm=vn.get("translation_norm_mm"),
                rotation_deg=vn.get("rotation_deg"), rotation_rotvec_deg_in_camera=vn.get("rotation_rotvec_deg_in_camera"),
                board_spread=r.get("board_spread"), reprojection_rms_px=r.get("reprojection_rms_px"),
                reprojection_worst_px=r.get("reprojection_worst_px"),
                end_frame_correction=r.get("end_frame_correction"), urdf_mount_origin=r.get("urdf_mount_origin"),
                method_agreement=best.get("method_agreement"), leave_one_out=best.get("leave_one_out"),
                skipped=res.get("skipped"), file=str(Path(res.get("data") or ".") / "hand_eye_result.yaml"))
