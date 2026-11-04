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

THE VIEWS (``plan``; the user's choice, 4 Nov: Lite plans them, the technician confirms each move): around each of the
three named poses (ALIX's group states rest_pose, inspection_pose_1 and inspection_pose_4; the right pole at
``POLE_MM``), turns of the wrist (J5, J6, J7) with a little of J1 and J4, each kept only when
- the whole board (10 x 7 squares of 30 mm, see BOARD_COLS), with ``BOARD_SLACK_M`` of slack for where it was laid,
  projects inside the colour image (1280 x 720 at the D435's factory focal length, ``MARGIN_PX`` in), between
  ``RANGE_M``, tilted ``TILT_DEG`` to the image (RealMan: never square-on);
- every right-arm frame stays ``ABOVE_LID_M`` above the lid's top over the commode;
- the arm's straight joint path to it from the view before is clear of the room's walls, the commode with its lid
  closed and the left arm where it stands (``collision_check.check_states``, S2's own check; the hand touching the
  commode refuses it here, it is not cleaning; the bin's events are left out: the dustbin is not on the lab robot);
then ``VIEWS_PER_POSE`` a pose that turn the END frame as differently as possible (RealMan: more than 30 deg about
each of its axes across the set), 15 views in all, rest's first, then pose 1's, then pose 4's; the plan ends at rest.
From rest the camera looks straight down 330 mm right of the commode: its views turn toward the board.

A VIEW (``run_view``, a job, Real only, each confirmed): the MoveJ at ``CAL_SPEED_PCT``; the arm still (two readings
``STILL_WINDOW_S`` apart within ``STILL_DEG``, ALIX's capture tool's rule); two colour frames 0.3 s apart, the board
found in both and moved under ``STILL_PX`` (ALIX's); the joints and pole read then, the END frame's pose by FK; the
image (PNG) and its row saved at once. SOLVE (``solve``): ALIX's compute_hand_eye.run: the five methods, the
refinement, leave-one-out, ALIX's acceptance (board spread <= 3 mm RMS, reprojection <= 1.5 px, the rotations enough).

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

BOARD_COLS, BOARD_ROWS, BOARD_SQUARE_M = 9, 6, 0.030       # inner corners: 10 x 7 squares of 30 mm (300 x 210 mm)
# The user, 4 Nov: ALIX's A3 board (12 x 9 squares of 30 mm, 360 x 270 mm) is never seen whole from these three poses
# (at the inspection poses' 150 mm pole the camera is 0.5 m above the lid; at 300 mm only 11 views fit, 29 deg about
# the END frame's Y): 10 x 7 squares of 30 mm on A3 (Resources/Calibration/board_10x7_30mm.svg, ALIX's
# make_chessboard.py), the right pole at 250 mm: 15 views, 47 / 40 / 177 deg about X / Y / Z.
BOARD_CENTRE_M = (0.70, 0.0, 0.452)    # on the closed lid: 4 Nov's frames put its top at 0.449 m, x 0.49-1.00, y +-0.17
BOARD_LONG_ALONG = "x"                 # its long side from the robot outward (12 x 30 = 360 mm: the lid is 340 mm wide)
BOARD_SLACK_M = 0.04                   # where it was laid: the plan keeps 40 mm more than the board in view
BASES = ("rest_pose", "inspection_pose_1", "inspection_pose_4")
POLE_MM = 250.0                        # the user, 4 Nov (the inspection poses' own 150 mm puts the camera too near)
IMAGE_W, IMAGE_H, FOCAL_PX = 1280, 720, 904.5   # the D435's colour stream as Lite opens it (3 Nov: fx 904.5)
MARGIN_PX = 20
RANGE_M = (0.35, 0.85)
TILT_DEG = (15.0, 55.0)                # RealMan: tilted to the image, never square-on; past 55 deg the corners blur
ABOVE_LID_M = 0.08
LID_FOOTPRINT = ((0.45, 1.10), (-0.25, 0.25))   # base_link x, y over which the arm keeps ABOVE_LID_M over the lid
VIEWS_PER_POSE = 5
CAL_SPEED_PCT = 20
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

def board():
    vendor_paths.activate()
    from alix_perception_core import hand_eye as he

    return he.Board(BOARD_COLS, BOARD_ROWS, BOARD_SQUARE_M)


def board_size_m() -> tuple:
    """(long, short) of the printed squares: 12 x 9 squares of 30 mm."""
    return (BOARD_COLS + 1) * BOARD_SQUARE_M, (BOARD_ROWS + 1) * BOARD_SQUARE_M


def board_in_base() -> np.ndarray:
    """The board as the plan expects it (4 x 4, base_link): flat on the closed lid, its normal up."""
    T = np.eye(4)
    T[:3, 3] = BOARD_CENTRE_M
    return T


def board_outline(slack: float = BOARD_SLACK_M) -> np.ndarray:
    """The board's four corners with ``slack`` (4 x 3, base_link)."""
    long_m, short_m = board_size_m()
    a, b = long_m / 2 + slack, short_m / 2 + slack
    if BOARD_LONG_ALONG != "x":
        a, b = b, a
    cx, cy, cz = BOARD_CENTRE_M
    return np.array([[cx - a, cy - b, cz], [cx + a, cy - b, cz], [cx + a, cy + b, cz], [cx - a, cy + b, cz]])


def nominal_K() -> np.ndarray:
    return np.array([[FOCAL_PX, 0.0, IMAGE_W / 2], [0.0, FOCAL_PX, IMAGE_H / 2], [0.0, 0.0, 1.0]])


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

def view_of(T_base_cam: np.ndarray, K: np.ndarray | None = None, size=(IMAGE_W, IMAGE_H)) -> dict:
    """Whether the board (with its slack) is seen whole from the colour camera at ``T_base_cam``: {ok, why, margin_px,
    range_m, tilt_deg}."""
    K = nominal_K() if K is None else K
    inv = np.linalg.inv(T_base_cam)
    P = (inv[:3, :3] @ board_outline().T).T + inv[:3, 3]
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


def candidates(base: str, q0, left: dict | None = None) -> list[dict]:
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
        v = view_of(L[COLOUR_FRAME])
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
        bad.setdefault(leg, f"{', '.join(what)} at {e['first_t']:.2f} s of the tour")
    touched = (out.get("contact") or {}).get("right") or {}
    if touched.get("samples"):
        i0 = int(round(float(touched.get("first_t") or 0.0) / 0.02))
        bad.setdefault(leg_of[min(i0, n - 1)], "the hand would touch the commode")
    return [dict(leg=i, ok=i not in bad, why=bad.get(i, "")) for i in range(len(stops) - 1)]


def plan(robot_state: dict, room: dict, per_pose: int = VIEWS_PER_POSE) -> dict:
    """The session's views (module docstring): {views, poses, diversity, checked, refused, ends_at, board, ...}. Runs
    in the worker process (FCL). ``robot_state``: {side: {q_deg, pole_mm, hand}} as read now; ``room``: {home, scene,
    fid, cubicle_mm}."""
    vendor_paths.activate()
    from alix_perception_core import hand_eye as he

    t0 = time.monotonic()
    poses = named_poses()
    left = robot_state.get("left") or None
    per_base = {b: candidates(b, poses[b], left) for b in BASES}
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
    div = he.diversity([v["T_base_end"] for v in views])
    return dict(views=[{k: v[k] for k in v if k != "T_base_end"} | dict(index=i, T_base_end=v["T_base_end"].tolist())
                       for i, v in enumerate(views)],
                poses=poses, candidates={b: len(c) for b, c in per_base.items()}, diversity=div,
                refused=notes, ends_at=poses["rest_pose"], pole_mm=POLE_MM, speed_pct=CAL_SPEED_PCT,
                board=dict(cols=BOARD_COLS, rows=BOARD_ROWS, square_m=BOARD_SQUARE_M, centre_m=list(BOARD_CENTRE_M),
                           long_along=BOARD_LONG_ALONG, slack_m=BOARD_SLACK_M),
                seconds=round(time.monotonic() - t0, 2))


# ── the session ──────────────────────────────────────────────────────────

class Session:
    """One calibration session: its folder (``samples.yaml``, ``images/``, ``plan.json``, ``hand_eye_result.yaml``), its
    plan, the views taken. One a robot at a time (``SESSIONS``)."""

    def __init__(self, folder: Path, planned: dict, square_mm: float, room: dict | None = None):
        self.folder = Path(folder)
        self.plan = planned
        self.room = room                 # {home, scene, fid, cubicle_mm}: each move is checked again from where it starts
        self.square_m = float(square_mm) / 1000.0
        self.rows: list[dict] = []
        self.header: dict | None = None
        self.notes: list[str] = []
        self.result: dict | None = None
        self.lock = threading.Lock()
        self.folder.mkdir(parents=True, exist_ok=True)
        (self.folder / "plan.json").write_text(json.dumps(planned, indent=1), encoding="utf-8")

    @property
    def next_index(self) -> int:
        return len(self.rows)

    def view(self) -> dict:
        from_div = None
        if len(self.rows) >= 2:
            vendor_paths.activate()
            from alix_perception_core import hand_eye as he

            from_div = he.diversity([np.asarray(r["T_base_end"], float) for r in self.rows])
        return dict(folder=str(self.folder), views=len(self.plan["views"]), taken=len(self.rows), next=self.next_index,
                    planned=[{k: v[k] for k in ("index", "base", "offsets_deg", "range_m", "tilt_deg", "margin_px")}
                             for v in self.plan["views"]],
                    rows=[{k: r[k] for k in ("index", "view", "board_pose_rms_px", "board_tilt_deg", "board_range_m", "still_px")}
                          for r in self.rows],
                    diversity=from_div, planned_diversity=self.plan.get("diversity"), notes=self.notes[-6:],
                    result=summary(self.result) if self.result else None, square_mm=round(self.square_m * 1000, 3))

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
                           board=dict(cols=BOARD_COLS, rows=BOARD_ROWS, square_m=self.square_m),
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


def capture(camera, square_m: float, out: Path, index: int) -> tuple:
    """Two colour frames 0.3 s apart, the board found in both and still (ALIX's capture tool): (the second image,
    its corners, the board's pose in the camera, PnP rms px, the shift px)."""
    import cv2

    vendor_paths.activate()
    from alix_perception_core import hand_eye as he

    b = he.Board(BOARD_COLS, BOARD_ROWS, float(square_m))
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
    return s2.colour, c2, shift


def record(session: Session, k: int, v: dict, image, corners, shift: float, q_deg, pole_mm) -> dict:
    """A view's row as ALIX's capture tool writes it (the END frame's pose from the joints read at the capture, by the
    URDF's FK; the board's pose in the camera for its tilt and range), its image saved, the session's samples.yaml
    written at once."""
    import cv2

    from . import robot_fk

    vendor_paths.activate()
    from alix_perception_core import hand_eye as he

    values, _ = robot_fk.joint_values({"right": {"q_deg": list(q_deg), "pole_mm": float(pole_mm)}})
    T_base_end = robot_fk.model().link_world_transforms(values)[END_FRAME]
    info = session.header["camera_info"]
    K = np.asarray(info["K"], float).reshape(3, 3)
    D = np.asarray(info["D"], float)
    T_cam_board, rms = he.board_pose(corners, K, D, he.Board(BOARD_COLS, BOARD_ROWS, session.square_m))
    (session.folder / "images").mkdir(parents=True, exist_ok=True)
    with session.lock:
        rel = f"images/{len(session.rows):02d}.png"
        cv2.imwrite(str(session.folder / rel), image)
        row = dict(index=len(session.rows), view=int(k), base=v["base"], offsets_deg=v["offsets_deg"], image=rel,
                   captured=datetime.now().isoformat(timespec="seconds"), T_base_end=T_base_end,
                   joints={**{f"R_joint{i + 1}": math.radians(float(q_deg[i])) for i in range(7)},
                           "R_sliding_plate_joint": float(pole_mm) / 1000.0},
                   board_pose_rms_px=round(float(rms), 3), board_tilt_deg=round(float(he.board_tilt_deg(T_cam_board)), 1),
                   board_range_m=round(float(np.linalg.norm(T_cam_board[:3, 3])), 3), still_px=round(float(shift), 3))
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


def run_view(robot, session: Session, mode: str, index: int | None = None):
    """The next planned view (or ``index``): a job on the right arm (module docstring). Real only."""
    from . import mechanism

    if mode != "real":
        raise UserError("The camera calibration takes its views on the real robot only.",
                        "Switch to Real Machine Mode; Visual sim shows the plan.")
    if robot.links.get("right") is None:
        raise UserError("The right arm is not connected.", "Connect it in Prepare, then try again.")
    cam = mechanism.kit(robot).camera
    if not cam.available().get("ok"):
        raise UserError("The wrist camera (D435) is not connected.", "Plug it in, then try again.", code="camera")
    k = session.next_index if index is None else int(index)
    views = session.plan["views"]
    if not 0 <= k < len(views):
        raise UserError("Every planned view is taken.", "Solve the calibration.")
    v = views[k]
    target = [round(float(a) * 1000) for a in v["q_deg"]]

    def work(job):
        robot._same_mode(mode, "the arm did not move")
        real = robot._real("right")
        with cam.using():
            if session.header is None:
                cam.open()
                session.header_from(cam)
            pole_now, _ = robot._measured("right")
            if pole_now is None or abs(float(pole_now) - POLE_MM) > 5.0:
                job.progress(f"Right pole to {POLE_MM:.0f} mm")
                real.pole_to(POLE_MM)
            job.progress(f"Checking the move to view {k + 1} from where the arms stand")
            leg_clear(robot, session, v["q_deg"], f"view {k + 1}")
            originals = real.set_run_mode("REAL")
            try:
                job.progress(f"View {k + 1} of {len(views)}: {v['base']} turned {v['offsets_deg']} ({CAL_SPEED_PCT} % speed)")
                real.move_to(target, speed=CAL_SPEED_PCT, restore_pct=robot.found_speed("right"))
            finally:
                real.restore_run_mode(originals)
            fb = wait_still(robot, "right", job)
            (session.folder / "images").mkdir(parents=True, exist_ok=True)
            image, corners, shift = capture(cam, session.square_m, session.folder, k)
            after = robot.links["right"].read_fresh()
            q, pole = after.get("joints_deg") or fb.get("joints_deg"), after.get("pole_mm") or fb.get("pole_mm")
            row = record(session, k, v, image, corners, shift, q, pole)
            note = (f"view {k + 1}: board found, PnP {row['board_pose_rms_px']:.2f} px, tilted {row['board_tilt_deg']:.0f} deg, "
                    f"{row['board_range_m']:.2f} m away")
            session.notes.append(note)
            job.log(note)
            return dict(view=k, taken=len(session.rows), row={kk: row[kk] for kk in ("board_pose_rms_px", "board_tilt_deg",
                                                                                    "board_range_m", "still_px")})

    return robot._start("move", f"RIGHT calibration view {k + 1} of {len(views)}", work, "right", mode)


def back_to_rest(robot, session: Session, mode: str):
    """The tour's end: the right arm back at rest (the plan's ``ends_at``), at the calibration's speed."""
    target = [round(float(a) * 1000) for a in session.plan["ends_at"]]

    def work(job):
        robot._same_mode(mode, "the arm did not move")
        real = robot._real("right")
        leg_clear(robot, session, session.plan["ends_at"], "rest")
        originals = real.set_run_mode("REAL" if mode == "real" else "SIM")
        try:
            real.move_to(target, speed=CAL_SPEED_PCT, restore_pct=robot.found_speed("right"))
        finally:
            real.restore_run_mode(originals)
        return dict(at="rest")

    return robot._start("move", "RIGHT back to rest (calibration)", work, "right", mode)


def solve(session: Session) -> dict:
    """ALIX's offline solve of the session (compute_hand_eye.run): writes hand_eye_result.yaml beside the samples."""
    if len(session.rows) < 3:
        raise UserError(f"Only {len(session.rows)} view(s) taken; RealMan's method asks for 15 to 20.", "Take more views.")
    che = _compute()
    try:
        res = che.run(session.folder, quiet=True)
    except SystemExit as exc:
        raise UserError(f"The calibration could not be solved: {exc}.") from None
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
