"""Eye-in-hand calibration of the wrist D435 — RealMan's method, fitted to alix.

Source: RealMan's developer guide "Hand-eye Calibration"
(github.com/RealManRobot/rm_docs, docs/en/AI/developerGuide/hand/index.md, commit f5444ba,
2025-05-23). Its code repository, github.com/RealManRobot/hand_eye_calibration, no longer
exists (checked 2026-10-13); the guide still prints the method and its key code:

  * a chessboard FIXED in the workspace, seen from 15-20 arm poses with large rotations
    (> 30 deg about each of the end frame's X, Y and Z), the board tilted to the image plane;
  * per image: chessboard corners (subpixel) and the board's pose in the camera;
  * the end-frame pose of each sample in the arm's base frame;
  * `cv2.calibrateHandEye(R_end2base, t_end2base, R_board2cam, t_board2cam, CALIB_HAND_EYE_TSAI)`
    -> the camera in the END frame. RealMan states the translation within 1 cm.

What alix changes, and why:

  * the END frame is `R_CameraHolderLink` (the J7 output the URDF hangs the D435 on) and its
    pose comes from the robot's own TF, not the controller's pose readout, so the result is
    the extrinsic that makes OUR transform chain (the one perception uses) right;
  * intrinsics: the camera's factory CameraInfo by default; RealMan's `calibrateCamera`
    re-estimate is available for comparison;
  * all five OpenCV methods are solved and their spread reported — agreement is evidence;
  * a joint refinement minimises the reprojection error of every corner in every image
    over the camera-in-end AND the board-in-base transforms (seeded by the closed forms);
  * the consistency checks RealMan's guide does not print: the board's pose in the base
    frame from every sample (it never moved, so its spread IS the calibration error seen
    through the arm), leave-one-out stability, and the correction against the URDF nominal.

ROS-free (numpy, OpenCV, scipy). `tools/perception/capture_hand_eye.py` collects samples on
the robot; `tools/perception/compute_hand_eye.py` runs this module on them.
"""
from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np

METHODS = {
    "tsai": cv2.CALIB_HAND_EYE_TSAI,          # RealMan's choice
    "park": cv2.CALIB_HAND_EYE_PARK,
    "horaud": cv2.CALIB_HAND_EYE_HORAUD,
    "andreff": cv2.CALIB_HAND_EYE_ANDREFF,
    "daniilidis": cv2.CALIB_HAND_EYE_DANIILIDIS,
}
MIN_AXIS_ROTATION_DEG = 30.0      # RealMan: "greater than 30 deg" about each end-frame axis
MIN_SAMPLES = 15                  # RealMan: 15-20 poses
MIN_BOARD_TILT_DEG = 15.0         # RealMan: the board at an angle to the image plane, never square-on


@dataclass(frozen=True)
class Board:
    """A chessboard by its INNER corners (RealMan's XX, YY) and square size (RealMan's L).
    RealMan's default print: 12 x 9 squares of 30 mm = 11 x 8 inner corners. `cols + rows`
    must be odd so the board has one black and one white outer corner (`canonical_order`)."""
    cols: int = 11
    rows: int = 8
    square_m: float = 0.030

    def object_points(self) -> np.ndarray:
        objp = np.zeros((self.cols * self.rows, 3), np.float64)
        objp[:, :2] = np.mgrid[0:self.cols, 0:self.rows].T.reshape(-1, 2) * self.square_m
        return objp


# ── small transform helpers ──────────────────────────────────────────────
def make_T(R: np.ndarray, t) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(t, float).reshape(3)
    return T


def inv(T: np.ndarray) -> np.ndarray:
    R, t = T[:3, :3], T[:3, 3]
    return make_T(R.T, -R.T @ t)


def rot_angle_deg(R: np.ndarray) -> float:
    return float(np.degrees(np.arccos(np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0))))


def rotvec(R: np.ndarray) -> np.ndarray:
    return cv2.Rodrigues(np.asarray(R, float))[0].reshape(3)


def from_rotvec(v) -> np.ndarray:
    return cv2.Rodrigues(np.asarray(v, float).reshape(3, 1))[0]


def mean_rotation(Rs) -> np.ndarray:
    U, _, Vt = np.linalg.svd(np.sum(Rs, axis=0))
    R = U @ Vt
    if np.linalg.det(R) < 0:
        U[:, -1] *= -1
        R = U @ Vt
    return R


def rpy_from_matrix(R: np.ndarray) -> tuple[float, float, float]:
    """URDF fixed-axis roll, pitch, yaw: R = Rz(yaw) @ Ry(pitch) @ Rx(roll)."""
    pitch = float(np.arcsin(-np.clip(R[2, 0], -1.0, 1.0)))
    if abs(np.cos(pitch)) > 1e-9:
        roll = float(np.arctan2(R[2, 1], R[2, 2]))
        yaw = float(np.arctan2(R[1, 0], R[0, 0]))
    else:                                            # gimbal lock: put it all in yaw
        roll, yaw = 0.0, float(np.arctan2(-R[0, 1], R[1, 1]))
    return roll, pitch, yaw


def matrix_from_rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr, cp, sp, cy, sy = np.cos(roll), np.sin(roll), np.cos(pitch), np.sin(pitch), np.cos(yaw), np.sin(yaw)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


# ── the image side ───────────────────────────────────────────────────────
def detect(gray: np.ndarray, board: Board) -> np.ndarray | None:
    """Inner corners (N,1,2) float32 in the board's OWN row-major order, or None.

    The sector-based detector (`findChessboardCornersSB`) is more accurate than RealMan's
    `findChessboardCorners` + `cornerSubPix`; the latter is the fallback. Either numbers the
    corners from whichever end suits the image, so a camera rolled past ~90 deg (RealMan asks
    for large rotations about Z) sees the board turned 180 deg and the solve breaks;
    `canonical_order` pins corner 0 to the board's BLACK outer corner square."""
    size = (board.cols, board.rows)
    ok, corners = cv2.findChessboardCornersSB(gray, size, flags=cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY)
    if not ok:
        ok, corners = cv2.findChessboardCorners(gray, size, None)
        if not ok:
            return None
        criteria = (cv2.TERM_CRITERIA_MAX_ITER | cv2.TERM_CRITERIA_EPS, 30, 0.001)
        corners = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), criteria)
    return canonical_order(gray, corners.astype(np.float32), board)


def _outer_square_mean(gray: np.ndarray, grid: np.ndarray, r: int, c: int, dr: int, dc: int) -> float | None:
    a = grid[r, c]
    centre = a - 0.5 * (grid[r, c + dc] - a) - 0.5 * (grid[r + dr, c] - a)
    half = 0.2 * min(np.linalg.norm(grid[r, c + dc] - a), np.linalg.norm(grid[r + dr, c] - a))
    x0, x1 = int(round(centre[0] - half)), int(round(centre[0] + half)) + 1
    y0, y1 = int(round(centre[1] - half)), int(round(centre[1] + half)) + 1
    if half < 1 or x0 < 0 or y0 < 0 or x1 > gray.shape[1] or y1 > gray.shape[0]:
        return None
    return float(gray[y0:y1, x0:x1].mean())


def canonical_order(gray: np.ndarray, corners: np.ndarray, board: Board) -> np.ndarray | None:
    """Corners ordered so corner 0 sits at the board's black outer corner square, or None when
    the board cannot tell its ends apart (its outer squares out of the image, or no contrast).
    Needs `cols + rows` odd (an even x odd count of squares, RealMan's 12 x 9 is): only then
    are the two diagonal outer squares different colours."""
    if (board.cols + board.rows) % 2 == 0:
        raise ValueError(f"a {board.cols}x{board.rows} inner-corner board is 180-deg symmetric; use cols+rows odd")
    grid = corners.reshape(board.rows, board.cols, 2).astype(np.float64)
    # 1. handedness: a printed board seen from its front keeps x -> y clockwise in the image
    #    (image y points down), so the board's z points AWAY from the camera. A planar target
    #    fits a mirrored labelling just as well (the pose then views it from behind), so the
    #    detector's order is not trusted for this.
    ex, ey = grid[0, -1] - grid[0, 0], grid[-1, 0] - grid[0, 0]
    if ex[0] * ey[1] - ex[1] * ey[0] < 0:
        grid = grid[::-1].copy()
    # 2. the 180-deg turn: corner 0 at the black outer corner square
    first = _outer_square_mean(gray, grid, 0, 0, 1, 1)
    last = _outer_square_mean(gray, grid, board.rows - 1, board.cols - 1, -1, -1)
    if first is None or last is None or abs(first - last) < 30.0:
        return None
    if first > last:
        grid = grid[::-1, ::-1].copy()
    return grid.reshape(-1, 1, 2).astype(np.float32)


def board_pose(corners: np.ndarray, K: np.ndarray, dist: np.ndarray, board: Board) -> tuple[np.ndarray, float]:
    """(T_cam_board, reprojection RMS px). The board's +z points out of the printed face
    or into it depending on the corner order; the hand-eye solve does not care."""
    objp = board.object_points()
    ok, rvec, tvec = cv2.solvePnP(objp, corners, K, dist, flags=cv2.SOLVEPNP_IPPE)
    if not ok:
        ok, rvec, tvec = cv2.solvePnP(objp, corners, K, dist, flags=cv2.SOLVEPNP_ITERATIVE)
    rvec, tvec = cv2.solvePnPRefineLM(objp, corners, K, dist, rvec, tvec)
    proj, _ = cv2.projectPoints(objp, rvec, tvec, K, dist)
    rms = float(np.sqrt(np.mean(np.sum((proj.reshape(-1, 2) - corners.reshape(-1, 2)) ** 2, axis=1))))
    return make_T(from_rotvec(rvec), tvec), rms


def board_tilt_deg(T_cam_board: np.ndarray) -> float:
    """Angle between the board's normal and the optical axis (0 = square-on, RealMan's 'wrong pose')."""
    n = T_cam_board[:3, 2]
    return float(np.degrees(np.arccos(min(1.0, abs(float(n[2]))))))


def calibrate_intrinsics(corner_sets, board: Board, image_size) -> tuple[np.ndarray, np.ndarray, float]:
    """RealMan's step 2: `cv2.calibrateCamera` over the board images -> (K, dist, RMS px)."""
    objp = board.object_points().astype(np.float32)
    rms, K, dist, _, _ = cv2.calibrateCamera([objp] * len(corner_sets), list(corner_sets),
                                             tuple(image_size), None, None)
    return K, dist.reshape(-1), float(rms)


# ── the motion side ──────────────────────────────────────────────────────
def diversity(T_base_end) -> dict:
    """How much the END frame rotated between samples, per end-frame axis (RealMan: > 30 deg
    about each of X, Y, Z), and the largest single relative rotation."""
    Rs = [T[:3, :3] for T in T_base_end]
    per_axis = np.zeros(3)
    largest = 0.0
    for i in range(len(Rs)):
        for j in range(i + 1, len(Rs)):
            rel = Rs[i].T @ Rs[j]
            v = np.degrees(rotvec(rel))
            per_axis = np.maximum(per_axis, np.abs(v))
            largest = max(largest, rot_angle_deg(rel))
    ok = bool(len(Rs) >= MIN_SAMPLES and np.all(per_axis >= MIN_AXIS_ROTATION_DEG))
    return dict(samples=len(Rs), max_rotation_about_end_xyz_deg=[round(float(a), 1) for a in per_axis],
                largest_relative_rotation_deg=round(largest, 1), enough=ok)


def solve(T_base_end, T_cam_board, method: str = "tsai") -> np.ndarray:
    """RealMan's step 4: the camera in the END frame (T_end_cam) by `cv2.calibrateHandEye`."""
    R, t = cv2.calibrateHandEye([T[:3, :3] for T in T_base_end], [T[:3, 3].reshape(3, 1) for T in T_base_end],
                                [T[:3, :3] for T in T_cam_board], [T[:3, 3].reshape(3, 1) for T in T_cam_board],
                                method=METHODS[method])
    return make_T(R, t)


def board_in_base(T_base_end, T_end_cam, T_cam_board) -> list[np.ndarray]:
    return [Tb @ T_end_cam @ Tc for Tb, Tc in zip(T_base_end, T_cam_board)]


def board_spread(T_base_board) -> dict:
    """The board never moved: the spread of its pose in the base frame over the samples is the
    calibration error as the arm sees it (plus the arm's own kinematic error)."""
    P = np.array([T[:3, 3] for T in T_base_board])
    centre = P.mean(axis=0)
    d = 1000.0 * np.linalg.norm(P - centre, axis=1)
    Rm = mean_rotation([T[:3, :3] for T in T_base_board])
    a = np.array([rot_angle_deg(Rm.T @ T[:3, :3]) for T in T_base_board])
    return dict(translation_rms_mm=float(np.sqrt(np.mean(d ** 2))), translation_max_mm=float(d.max()),
                rotation_rms_deg=float(np.sqrt(np.mean(a ** 2))), rotation_max_deg=float(a.max()),
                board_in_base=make_T(Rm, centre))


def reprojection(T_base_end, T_end_cam, T_base_board, corner_sets, K, dist, board: Board) -> dict:
    """Corners predicted through the WHOLE chain (one board pose, the arm, the extrinsic) against
    the detected ones: the error the runtime will see when it projects a known point."""
    objp = board.object_points()
    per = []
    for Tb, c in zip(T_base_end, corner_sets):
        T = inv(T_end_cam) @ inv(Tb) @ T_base_board
        proj, _ = cv2.projectPoints(objp, rotvec(T[:3, :3]), T[:3, 3], K, dist)
        per.append(float(np.sqrt(np.mean(np.sum((proj.reshape(-1, 2) - c.reshape(-1, 2)) ** 2, axis=1)))))
    return dict(rms_px=float(np.sqrt(np.mean(np.square(per)))), worst_px=float(max(per)), per_sample_px=per)


def refine(T_base_end, corner_sets, K, dist, board: Board, T_end_cam0, T_base_board0) -> tuple[np.ndarray, np.ndarray]:
    """Joint least squares over the camera-in-end and board-in-base transforms (12 parameters),
    minimising every corner's reprojection error in every image (Huber, 1 px)."""
    from scipy.optimize import least_squares
    objp = board.object_points()
    obs = [c.reshape(-1, 2).astype(np.float64) for c in corner_sets]
    inv_end = [inv(T) for T in T_base_end]

    def unpack(x):
        return make_T(from_rotvec(x[0:3]), x[3:6]), make_T(from_rotvec(x[6:9]), x[9:12])

    def residuals(x):
        Tec, Tbb = unpack(x)
        out = []
        iTec = inv(Tec)
        for ie, o in zip(inv_end, obs):
            T = iTec @ ie @ Tbb
            proj, _ = cv2.projectPoints(objp, rotvec(T[:3, :3]), T[:3, 3], K, dist)
            out.append((proj.reshape(-1, 2) - o).ravel())
        return np.concatenate(out)

    x0 = np.concatenate([rotvec(T_end_cam0[:3, :3]), T_end_cam0[:3, 3], rotvec(T_base_board0[:3, :3]), T_base_board0[:3, 3]])
    res = least_squares(residuals, x0, loss="huber", f_scale=1.0, x_scale="jac", max_nfev=200)
    return unpack(res.x)


def compare(T_measured: np.ndarray, T_nominal: np.ndarray) -> dict:
    """The measured camera against the URDF's nominal, both in the END frame."""
    dt = 1000.0 * (T_measured[:3, 3] - T_nominal[:3, 3])
    dR = T_nominal[:3, :3].T @ T_measured[:3, :3]               # in the nominal camera's own axes
    return dict(translation_mm_in_end=[round(float(v), 2) for v in dt], translation_norm_mm=round(float(np.linalg.norm(dt)), 2),
                rotation_deg=round(rot_angle_deg(dR), 3),
                rotation_rotvec_deg_in_camera=[round(float(v), 3) for v in np.degrees(rotvec(dR))])


def leave_one_out(T_base_end, T_cam_board, method: str = "park") -> dict:
    sols = []
    for k in range(len(T_base_end)):
        idx = [i for i in range(len(T_base_end)) if i != k]
        sols.append(solve([T_base_end[i] for i in idx], [T_cam_board[i] for i in idx], method))
    P = np.array([S[:3, 3] for S in sols])
    Rm = mean_rotation([S[:3, :3] for S in sols])
    d = 1000.0 * np.linalg.norm(P - P.mean(axis=0), axis=1)
    a = [rot_angle_deg(Rm.T @ S[:3, :3]) for S in sols]
    return dict(method=method, translation_max_mm=float(d.max()), rotation_max_deg=float(max(a)))


def mount_origin(T_end_cam_measured: np.ndarray, T_mount_cam_nominal: np.ndarray) -> dict:
    """The URDF `<origin>` of the camera mount (END -> camera_bottom_screw_frame) that puts the
    colour optical frame where the calibration measured it; the D435's internal frames stay nominal."""
    T = T_end_cam_measured @ inv(T_mount_cam_nominal)
    return dict(xyz=[round(float(v), 5) for v in T[:3, 3]], rpy=[round(float(v), 5) for v in rpy_from_matrix(T[:3, :3])])


def end_frame_correction(T_end_cam_measured: np.ndarray, T_end_cam_nominal: np.ndarray) -> dict:
    """The small transform C, in the END frame, with T_measured = C @ T_nominal: what a fixed
    joint between `R_CameraHolderLink` and the (unchanged, nominal) camera mount would carry.
    Readable where the full mount `<origin>` is not: the nominal pitch is exactly -90 deg, where
    URDF roll and yaw trade off and a 2 deg change rewrites both."""
    C = T_end_cam_measured @ inv(T_end_cam_nominal)
    rpy = rpy_from_matrix(C[:3, :3])
    return dict(xyz=[round(float(v), 5) for v in C[:3, 3]], rpy=[round(float(v), 6) for v in rpy],
                xyz_mm=[round(1000.0 * float(v), 2) for v in C[:3, 3]], rpy_deg=[round(float(np.degrees(v)), 3) for v in rpy])


def corrected_camera_to_base(T_base_cam_recorded: np.ndarray, T_end_cam_nominal: np.ndarray,
                             T_end_cam_measured: np.ndarray) -> np.ndarray:
    """Re-express a frame recorded with the nominal extrinsic through the measured one."""
    return T_base_cam_recorded @ inv(T_end_cam_nominal) @ T_end_cam_measured


def calibrate(samples: list[dict], board: Board, K: np.ndarray, dist: np.ndarray,
              T_end_cam_nominal: np.ndarray | None = None) -> dict:
    """Everything for one session. `samples`: dicts with `T_base_end` (4x4) and `corners` (N,1,2).
    Returns the per-method solutions, the refined one, and the consistency evidence."""
    Tbe = [np.asarray(s["T_base_end"], float) for s in samples]
    corners = [np.asarray(s["corners"], np.float32) for s in samples]
    poses = [board_pose(c, K, dist, board) for c in corners]
    Tcb = [p[0] for p in poses]
    out = dict(samples=len(samples), diversity=diversity(Tbe),
               board_pose_rms_px=[round(p[1], 3) for p in poses],
               board_tilt_deg=[round(board_tilt_deg(T), 1) for T in Tcb],
               board_range_m=[round(float(np.linalg.norm(T[:3, 3])), 3) for T in Tcb],
               methods={})
    for name in METHODS:
        try:
            Tec = solve(Tbe, Tcb, name)
        except cv2.error as exc:                      # RealMan's "Not enough informative motions"
            out["methods"][name] = dict(error=str(exc).strip().splitlines()[-1])
            continue
        if not np.all(np.isfinite(Tec)):
            out["methods"][name] = dict(error="non-finite solution")
            continue
        spread = board_spread(board_in_base(Tbe, Tec, Tcb))
        rep = reprojection(Tbe, Tec, spread["board_in_base"], corners, K, dist, board)
        entry = dict(T_end_cam=Tec, board_spread={k: v for k, v in spread.items() if k != "board_in_base"},
                     reprojection_rms_px=rep["rms_px"], reprojection_worst_px=rep["worst_px"],
                     board_in_base=spread["board_in_base"])
        if T_end_cam_nominal is not None:
            entry["vs_nominal"] = compare(Tec, T_end_cam_nominal)
        out["methods"][name] = entry
    good = {k: v for k, v in out["methods"].items() if "T_end_cam" in v}
    if not good:
        return out
    seed = min(good.values(), key=lambda v: v["reprojection_rms_px"])
    Tec, Tbb = refine(Tbe, corners, K, dist, board, seed["T_end_cam"], seed["board_in_base"])
    spread = board_spread(board_in_base(Tbe, Tec, Tcb))
    rep = reprojection(Tbe, Tec, Tbb, corners, K, dist, board)
    out["refined"] = dict(T_end_cam=Tec, board_in_base=Tbb,
                          board_spread={k: v for k, v in spread.items() if k != "board_in_base"},
                          reprojection_rms_px=rep["rms_px"], reprojection_worst_px=rep["worst_px"],
                          reprojection_per_sample_px=[round(v, 3) for v in rep["per_sample_px"]])
    if T_end_cam_nominal is not None:
        out["refined"]["vs_nominal"] = compare(Tec, T_end_cam_nominal)
    P = np.array([v["T_end_cam"][:3, 3] for v in good.values()])
    Rm = mean_rotation([v["T_end_cam"][:3, :3] for v in good.values()])
    out["method_agreement"] = dict(
        translation_max_mm=float(1000.0 * np.max(np.linalg.norm(P - P.mean(axis=0), axis=1))),
        rotation_max_deg=float(max(rot_angle_deg(Rm.T @ v["T_end_cam"][:3, :3]) for v in good.values())))
    if len(samples) > 3:
        try:
            out["leave_one_out"] = leave_one_out(Tbe, Tcb, "park")
        except cv2.error:
            pass
    return out
