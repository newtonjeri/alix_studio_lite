"""The commode's PLACE read from one depth frame: where it stands (x, y, height, yaw) against where the scene puts it, by
the render and compare of its parts' state (seat_state.fit_fixture_state). The fixture's surfaces, with the lid and the
seat at the angles the parts reading found, are moved over a search about the scene place and rendered into the camera
that took the frame; the place whose render the depth explains best is the commode's.

Each pixel where the render has a surface and the camera a depth scores: explained (within tol_m) by how near, -1 when
the camera sees PAST the surface (nothing is there), nothing when it sees something NEARER (a glove, an arm: occlusion);
the pixels within hand_mask_m of a hand frame are left out. A 10 mm grid in x and y (the height by the depth shift, as
fit_fixture_state's), then a pattern search down to 0.6 mm and 0.125 deg, then a parabola through the last steps.

WHAT IT CAN AND CANNOT SAY (2026-10-31: the 1155 frames ALIX Studio Lite recorded at the lab, 21-31 Oct; its
tools/placement_replay.py):
- ITS ABSOLUTE PLACE IS OFF BY THE CAMERA'S MOUNT. The wrist camera's mount is a CAD value (no hand-eye calibration was
  captured: Resource/reports/2026-10-13_hand_eye_calibration). From inspection pose 4 and pose 1 the same commode, the
  same day, reads 10.7 / -14.8 mm and -3.2 deg apart (26 Oct, the lid open and the seat down; 11.4 / -14.0 mm, -2.4 deg
  with the seat open). So a place is compared only with one read from the SAME arm pose, where the mount's error is the
  same and cancels: what this gives is a CHANGE of place.
- THE PARTS' STATE MOVES IT. The real lid, shut, lies otherwise on the bowl than the model's: from pose 4 the commode
  with its lid shut reads 27 mm nearer the robot than with the lid open. A place is compared only with one read in the
  same state of the lid and the seat (``state_key``), and the parts are fitted at the angle they rest at (0 closed,
  the model's stop open), never at the angle a frame reads: a standing lid read at 105 deg in one frame and 110 in the
  next moved the place 6 mm.
- A PART IN A HAND IS NO STATE. A lid held lifted by the hand moves with the hand, and the frame gives no place.
- WITHIN THAT: the frames of one check agree to 0.4 / 0.6 / 0.3 mm and 0.2 deg (median); the twelve starts of Close the
  seat on 30 Oct, over three hours, to 0.24 / 0.25 mm (their standard deviation); the change from one day to the next,
  read in two states from one pose, agrees to a millimetre (26 to 27 Oct, pose 1, the seat down and the seat open:
  x +2.0 and +2.0, y -3.3 and -3.6 mm; 24 to 26 Oct, pose 4, three states: x +2.1 to +3.0 mm); the camera moved by a
  known 20 to 30 mm and 2 to 3 deg is found to 0.1 to 0.6 mm and 0.3 deg.
- WHERE IT IS WEAK: the lid shut seen from pose 4 (Open the lid's start: the right glove hides the lid's front edge)
  holds the commode's length loosely, its frames up to 5 to 8 mm apart along it (``sharp``: x 1 to 3 against 30 to 40
  in the other states); the lid shut seen from pose 1 gives nothing to use; an open seat that rests otherwise (lifted
  with the lid, against opened on its own) moves the reading by up to 5 mm.

Pure numpy plus `region3d.render_hypothesis` (OpenCV), no ROS. About 200 to 350 renders, 1 to 2 s a frame.
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Optional

import numpy as np


@dataclass(frozen=True)
class PlaceParams:
    stride: int = 2                  # render and compare at 1/stride resolution
    tol_m: float = 0.015             # a pixel within this of the render is explained (by how near); beyond it, seen past
    hand_mask_m: float = 0.14        # depth points this close to a hand frame never count
    reach_m: float = 0.04            # the first search, x and y about the start place, on a grid of
    grid_m: float = 0.01
    height_m: float = 0.015          # its height, by the depth shift, on a grid of
    height_step_m: float = 0.0025
    steps_m: tuple = (0.005, 0.0025, 0.00125, 0.000625)     # then a pattern search, x and y (height by half), and yaw
    yaw_steps_deg: tuple = (1.0, 0.5, 0.25, 0.125)
    min_px: int = 5000               # fewer rendered pixels with a depth (at 1/stride) = no place
    open_rest_deg: Optional[float] = None   # the angle an open part rests at (None: the model's open stop): a part
                                     # against its stop rests at one angle, and the place is fitted with it, never
                                     # with the angle each frame reads (31 Oct: the standing lid read 105 in one frame
                                     # and 110 in the next moved the place 6 mm along the commode)
    centre_body: tuple = (0.27, 0.0, 0.0)   # the commode's middle in its body frame: the yaw turns about the vertical
                                            # through it (commode_c's body runs 0 at the back to 0.535 m at the front)


DEFAULT_PLACE = PlaceParams()


def _rotz(deg: float) -> np.ndarray:
    t = math.radians(deg)
    c, s = math.cos(t), math.sin(t)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def moved(fixture_pose, dx: float, dy: float, dz: float, dyaw_deg: float, centre_body=DEFAULT_PLACE.centre_body) -> np.ndarray:
    """The fixture's pose (body -> world) moved by (dx, dy, dz) m in the world frame and turned dyaw_deg about the
    vertical through its middle."""
    pose = np.asarray(fixture_pose, dtype=float)
    o = pose[:3, :3] @ np.asarray(centre_body, dtype=float) + pose[:3, 3]
    R = _rotz(dyaw_deg)
    D = np.eye(4)
    D[:3, :3] = R
    D[:3, 3] = o - R @ o + np.array([dx, dy, dz])
    return D @ pose


@dataclass
class Placement:
    """Where the frame says the commode stands against the fixture pose it was given. ``ok`` False = no place (the reason
    says why)."""
    ok: bool = False
    dx_m: float = 0.0
    dy_m: float = 0.0
    dz_m: float = 0.0
    dyaw_deg: float = 0.0
    lid_deg: Optional[float] = None
    seat_deg: Optional[float] = None
    score: float = 0.0
    px: int = 0                      # rendered pixels with a depth, at the place found
    hand_px: int = 0
    sharp: dict = field(default_factory=dict)   # per axis: the score lost 3 mm (x, y), 1.5 mm (z), 0.5 deg away, per 1000
    renders: int = 0
    reason: str = ""
    time_ms: float = 0.0

    def pose(self, fixture_pose, centre_body=DEFAULT_PLACE.centre_body) -> np.ndarray:
        return moved(fixture_pose, self.dx_m, self.dy_m, self.dz_m, self.dyaw_deg, centre_body)

    def as_dict(self) -> dict:
        out = {k: (round(v, 5) if isinstance(v, float) else v) for k, v in self.__dict__.items()}
        out["sharp"] = {k: round(float(v), 2) for k, v in self.sharp.items()}
        return out


def at_rest(lid_deg, seat_deg, lid_closed_below: float = 10.0, open_from: float = 80.0) -> bool:
    """The lid and the seat where they rest: each closed (under ``lid_closed_below``) or open against its stop (from
    ``open_from``). A part between is held, or falling: its frame gives no place."""
    def rests(a):
        return a is not None and (float(a) < lid_closed_below or float(a) >= open_from)
    return rests(lid_deg) and rests(seat_deg)


def state_key(lid_deg, seat_deg, lid_closed_below: float = 10.0) -> str:
    """The parts' state a place is compared within: "lid shut" | "lid open, seat down" | "lid open, seat open"."""
    if float(lid_deg) < lid_closed_below:
        return "lid shut"
    return "lid open, seat down" if float(seat_deg) < lid_closed_below else "lid open, seat open"


class _Scorer:
    """One frame's depth against the fixture's render at a place: the depth measured once (the hands left out), the
    model's visible cells made per pair of angles, each place rendered and scored."""

    def __init__(self, depth, intrinsics, camera_to_world, fixture_pose, fixture, hands, params: PlaceParams):
        from . import seat_state as sst

        get = intrinsics.get if isinstance(intrinsics, dict) else (lambda k: getattr(intrinsics, k))
        self.fx, self.fy, self.cx, self.cy = (float(get(k)) for k in ("fx", "fy", "cx", "cy"))
        d = np.asarray(depth, dtype=np.float32)
        self.h, self.w = d.shape
        self.cam = np.asarray(camera_to_world, dtype=float)
        self.pose0 = np.asarray(fixture_pose, dtype=float)
        self.fixture, self.params = fixture, params
        self.meas, self.hand_px, *_ = sst._measure(d, {"fx": self.fx, "fy": self.fy, "cx": self.cx, "cy": self.cy},
                                                   self.cam, list(hands),
                                                   sst.SeatFitParams(stride=int(params.stride), hand_mask_m=params.hand_mask_m))
        self.dz_per_m = float(np.linalg.inv(np.linalg.inv(self.pose0) @ self.cam)[2, 2])   # as fit_fixture_state's
        self.renders, self.angles, self.pts = 0, None, None

    def model(self, lid_deg: float, seat_deg: float) -> "_Scorer":
        if (lid_deg, seat_deg) != self.angles:
            self.pts = _visible_cells(self.fixture, float(lid_deg), float(seat_deg), self.pose0, self.cam,
                                      self.fx, self.fy, self.cx, self.cy, self.w, self.h)
            self.angles = (lid_deg, seat_deg)
        return self

    def __call__(self, x, shifts=(0.0,)) -> tuple:
        """-> ([the score at place x with the whole render moved by each of ``shifts`` m in height], pixels scored)."""
        from .region3d import render_hypothesis

        self.renders += 1
        r = render_hypothesis(self.pts, np.linalg.inv(moved(self.pose0, *x, self.params.centre_body)) @ self.cam,
                              self.fx, self.fy, self.cx, self.cy, self.w, self.h, int(self.params.stride))
        m = self.meas[:r.shape[0], :r.shape[1]]
        ok = (r > 0) & (m > 0)
        e = (m[ok] - r[ok]).astype(np.float64)
        tol = float(self.params.tol_m)
        out = []
        for z in shifts:
            ee = e - z * self.dz_per_m if z else e
            near = np.abs(ee) <= tol
            out.append(float((1.0 - np.abs(ee[near]) / tol).sum() - (ee > tol).sum()))
        return out, int(ok.sum())


def _search(sc: _Scorer, start, params: PlaceParams, reach_m: float) -> tuple:
    """-> (place, score, pixels, at the grid's edge): the grid about ``start`` (x and y within ``reach_m``, the height
    by the depth shift), the pattern search from its best. Only a full reach's grid has an edge that says the commode
    may be further: a short grid about a place found only starts the pattern search, which walks on from it."""
    x0 = [float(v) for v in start]
    zs = np.arange(-params.height_m, params.height_m + 1e-9, params.height_step_m)
    grid = np.arange(-reach_m, reach_m + 1e-9, params.grid_m)
    best = None
    for gx in grid:
        for gy in grid:
            ss, _ = sc([x0[0] + gx, x0[1] + gy, x0[2], x0[3]], zs)
            i = int(np.argmax(ss))
            if best is None or ss[i] > best[0]:
                best = (ss[i], [x0[0] + gx, x0[1] + gy, x0[2] + float(zs[i]), x0[3]], max(abs(gx), abs(gy)))
    x = best[1]
    (f,), px = sc(x)                 # the grid took the height by the depth shift: score its best exactly
    for st, ys in zip(params.steps_m, params.yaw_steps_deg):
        better = True
        while better:
            better = False
            for j, dv in ((0, st), (0, -st), (1, st), (1, -st), (2, st / 2), (2, -st / 2), (3, ys), (3, -ys)):
                c = list(x)
                c[j] += dv
                (fc,), n = sc(c)
                if fc > f + 1e-9:
                    f, x, px, better = fc, c, n, True
    return x, f, px, reach_m >= params.reach_m - 1e-9 and best[2] >= reach_m - 1e-9


def _placement(sc: _Scorer, x, f: float, px: int, edge: bool, params: PlaceParams, t0: float) -> Placement:
    """The place found, a parabola through the last steps, each axis's sharpness; refused under min_px or at the edge."""
    out = Placement(lid_deg=sc.angles[0], seat_deg=sc.angles[1], score=float(f), px=int(px), hand_px=sc.hand_px)
    sub = list(x)
    for j, dv, name in ((0, 0.003, "x"), (1, 0.003, "y"), (2, 0.0015, "z"), (3, 0.5, "yaw")):
        (fm,), _ = sc([v - dv if i == j else v for i, v in enumerate(x)])
        (fp,), _ = sc([v + dv if i == j else v for i, v in enumerate(x)])
        den = fm - 2.0 * f + fp
        if den < 0.0:
            sub[j] = x[j] + float(np.clip(dv * 0.5 * (fm - fp) / den, -dv, dv))
        out.sharp[name] = float(-den / 2.0 / max(abs(f), 1.0) * 1000.0)     # the score lost dv away, per 1000
    out.dx_m, out.dy_m, out.dz_m, out.dyaw_deg = sub
    out.renders, out.time_ms = sc.renders, (time.time() - t0) * 1000.0
    s = int(params.stride)
    if out.px < params.min_px:
        out.reason = f"only {out.px} depth pixels on the commode ({params.min_px} needed at 1/{s}; {out.hand_px} under the hands)"
    elif edge:
        out.reason = f"the place found is at the search's edge ({params.reach_m * 1000:.0f} mm from where it started)"
    else:
        out.ok = True
        out.reason = (f"the commode {sub[0] * 1000:+.1f} / {sub[1] * 1000:+.1f} / {sub[2] * 1000:+.1f} mm, {sub[3]:+.2f} deg "
                      f"from the place given (lid {out.lid_deg:.0f}, seat {out.seat_deg:.0f} deg) over {out.px} pixels at 1/{s}")
    return out


def fit_placement(depth, intrinsics, camera_to_world, fixture_pose, fixture, lid_deg: float, seat_deg: float,
                  hands=(), params: PlaceParams = DEFAULT_PLACE, start=(0.0, 0.0, 0.0, 0.0)) -> Placement:
    """The commode's place in this depth frame (metres) against ``fixture_pose`` (body -> world, the frame
    ``camera_to_world`` is in), its lid and seat at ``lid_deg`` / ``seat_deg`` from closed, the search about ``start``
    (dx, dy, dz m, dyaw deg against ``fixture_pose``). ``fixture`` is a seat_state.HingedFixture."""
    t0 = time.time()
    if lid_deg is None or seat_deg is None:
        return Placement(lid_deg=lid_deg, seat_deg=seat_deg, reason="no angle for the lid or the seat")
    sc = _Scorer(depth, intrinsics, camera_to_world, fixture_pose, fixture, hands, params).model(lid_deg, seat_deg)
    x, f, px, edge = _search(sc, start, params, params.reach_m)
    return _placement(sc, x, f, px, edge, params, t0)


def read(depth, intrinsics, camera_to_world, fixture_pose, fixture, hands=(), parts_params=None,
         params: PlaceParams = DEFAULT_PLACE, closed_below: float = 10.0, open_from: float = 80.0,
         start=(0.0, 0.0, 0.0, 0.0), rounds: int = 3) -> tuple:
    """The parts' state and the commode's place together, from ``start`` (dx, dy, dz m, dyaw deg against
    ``fixture_pose``: where it was last read from this arm pose in this state, or the scene place). The parts are read
    (seat_state.fit_fixture_state with ``parts_params``) where the commode is taken to stand, and decide its STATE: a
    part closed (under ``closed_below``) is fitted at 0 deg, one open (from ``open_from``) at its rest angle
    (``open_rest_deg``); the place is searched with them, and the parts read again where it was found, until neither
    moves (``rounds`` at most; a place at the search's edge is searched again from there). Read away from where the
    commode stands, a standing seat can read 55 deg: that round fits with the angle read, and the next reads it again.
    The parts must rest, judged where the commode stands: a part between moves with what holds it, and gives no
    place. -> (Placement, the last parts reading)."""
    from . import seat_state as sst

    t0 = time.time()
    pp = parts_params or sst.SeatFitParams(height_search_m=0.005, height_step_m=0.0025, seat_open_min_deg=open_from)
    pose0 = np.asarray(fixture_pose, dtype=float)
    sc = _Scorer(depth, intrinsics, camera_to_world, pose0, fixture, hands, params)

    def fitted(a, part):
        if a < closed_below:
            return 0.0
        if a >= open_from:
            return float(params.open_rest_deg if params.open_rest_deg is not None else fixture.open_deg(part))
        return float(a)                   # between: only to bring the place near, never a place of its own

    x, angles, edge, f, px, whole = [float(v) for v in start], None, True, 0.0, 0, None
    for r in range(int(rounds)):
        whole = sst.fit_fixture_state(depth, intrinsics, camera_to_world, moved(pose0, *x, params.centre_body), fixture,
                                      0.0, list(hands), pp)
        if whole.lid_deg is None:
            return Placement(reason=f"the parts cannot be read ({whole.reason})"), whole
        use = (fitted(whole.lid_deg, "lid"), fitted(whole.seat_deg, "seat"))
        if use == angles and not edge:
            break
        angles = use
        x, f, px, edge = _search(sc.model(*angles), x, params, params.reach_m if (r == 0 or edge) else params.grid_m)
    if not at_rest(whole.lid_deg, whole.seat_deg, closed_below, open_from):
        return Placement(lid_deg=whole.lid_deg, seat_deg=whole.seat_deg,
                         reason=f"the lid at {whole.lid_deg:.0f} deg, the seat at {whole.seat_deg:.0f}: a part between "
                                f"closed and open moves with what holds it, no place"), whole
    return _placement(sc.model(*angles), x, f, px, edge, params, t0), whole


def _visible_cells(fixture, lid_deg, seat_deg, pose, cam, fx, fy, cx, cy, w, h, margin_px: int = 60) -> np.ndarray:
    """The model's cells (the body, the lid and the seat at their angles, in the body frame) that face the camera and
    fall in its image (with a margin for the search), at the place given: a render projects only these."""
    from . import seat_state as sst

    pts, nrm = [fixture.body], [fixture.normals("body")]
    for name, deg in (("lid", lid_deg), ("seat", seat_deg)):
        _cells, _origin, axis, open_deg = fixture.parts[name]
        n = fixture.normals(name)
        pts.append(fixture.placed(name, deg))
        nrm.append(None if n is None else n @ sst._rotation(axis, math.copysign(deg, open_deg)).T)
    P = np.vstack(pts)
    Pw = P @ pose[:3, :3].T + pose[:3, 3]
    keep = np.ones(len(P), dtype=bool)
    if all(n is not None for n in nrm):                  # the cells facing away from the camera are never seen
        keep = ((np.vstack(nrm) @ pose[:3, :3].T) * (Pw - cam[:3, 3])).sum(axis=1) < 0.0
    cf = np.linalg.inv(cam)
    c = Pw @ cf[:3, :3].T + cf[:3, 3]
    z = np.maximum(c[:, 2], 1e-3)
    u, v = fx * c[:, 0] / z + cx, fy * c[:, 1] / z + cy
    keep &= (c[:, 2] > 0.1) & (u > -margin_px) & (u < w + margin_px) & (v > -margin_px) & (v < h + margin_px)
    return P[keep]


def change(a: Placement, b: Placement, fixture_pose, centre_body=DEFAULT_PLACE.centre_body) -> dict:
    """The commode's move from place ``a`` to place ``b`` (both read against ``fixture_pose`` from the same arm pose, in
    the same state): the rigid move in the world frame (``matrix``, 4 x 4: b = matrix @ a) and its parts about the
    commode's middle as ``a`` places it (dx, dy, dz m; dyaw deg)."""
    A, B = a.pose(fixture_pose, centre_body), b.pose(fixture_pose, centre_body)
    D = B @ np.linalg.inv(A)
    o = A[:3, :3] @ np.asarray(centre_body, dtype=float) + A[:3, 3]
    dp = D[:3, :3] @ o + D[:3, 3] - o
    return {"matrix": D, "dx_m": float(dp[0]), "dy_m": float(dp[1]), "dz_m": float(dp[2]),
            "dyaw_deg": float(math.degrees(math.atan2(D[1, 0], D[0, 0])))}
