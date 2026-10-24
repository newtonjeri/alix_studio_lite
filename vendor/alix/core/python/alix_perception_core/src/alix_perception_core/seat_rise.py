"""Is the seat LIFTED? Read by comparing with the seat's own closed frame, from any viewpoint.

The gap check's seat question measures the ring's height in a fixed ROI after masking the hands
(perception_node `gap_check`, P20). On the robot that ROI is where the LEFT hand holds the lifted
seat: in all four seat_open runs of 22 Oct 11:58/11:59/12:00 and 23 Oct 15:08 the left hand lifted
the seat and the check answered "refused" ("only 0 % of the seat ROI lies in the 0.38-0.47 m seat
band after masking the hands"), which counted as NOT lifted and started a false re-lift.

So the seat is compared with ITSELF. A seat_closed check leaves a REFERENCE: the closed frame's
points in the world frame, each labelled (ring / open lid / distance from the hinge). A later frame
is read by projecting those points into ITS camera (its own pose and intrinsics, a depth buffer
keeping the nearest) and comparing, along each of its rays, what the ray hits now with what it hit
then: higher on the ring near the hinge = the seat rose there, lower = the rim shows where the ring
was. Nothing depends on the camera coming back to the same place (Newton 2026-10-23: "position or
view point should never affect the classification"); what limits an answer is VISIBILITY — enough of
the ring near the hinge seen, where the lifting hand never is. Measured on the four runs
(Resource/reports/2026-10-23_seat_check_occlusion): every usable ring pixel 0.03-0.15 m from the hinge
moved by >= 8 mm and a rotation about the hinge fitted 29-40 deg within 9 mm of the model's hinge;
two closed frames differ by 0 +/- 3 mm (91-98 % within 4 mm).

    ref, action, why = update_reference(None, depth, cam_to_world, intrinsics, seat_top, geometry, t)
    reading = read_seat(ref, depth_now, cam_to_world_now, intrinsics_now, geometry, t_now, hands=hands)
    reading.state  # "seat_lifted" | "seat_closed" | "" (no answer, `reason` says why)

THE GUARDS (from the records):
* a lid lifted to about -30 deg turns on the SAME hinge and mimics a lifted seat (10 Oct 17:38 ->
  17:41: the lid was closed and lifted again between the frames, the ring "rose" everywhere and the
  fit gave 36 deg). The reference keeps the OPEN LID's points (above 0.55 m); a ray that now sees
  FARTHER than the lid was means the lid left, and with fewer than 80 % of the lid's seen points
  unchanged (+/- 10 mm) the reference is refused. A ray that now sees NEARER is occlusion — the glove,
  the left arm, the raised seat itself — and is left out, never counted as a moved lid.
* only the HINGE-FIT reading can be mimicked by that lid, so only it needs the lid seen: with the open
  lid out of view the closed reading (the ring as it was) and the rim reading (the ring gone, the rim
  20 mm below showing) still answer, and a lift by the fit alone does not.
* a glove over the seat front with the seat down (10 Oct 19:09) leaves the ring near the hinge
  unchanged (0-1 % rose): a pixel that rose more than 0.10 m is a hand or an arm, not the seat, and
  points within 0.14 m of a hand frame never count.

Pure numpy, no ROS and no scipy (ALIX Studio Lite vendors this file unchanged). The fixture enters
only as a `SeatGeometry`: the hinge, the direction from the hinge to the seat front, and the seat's
footprint (its mesh cells placed closed, or a fixed xy box when the meshes cannot be loaded).
"""
from __future__ import annotations

import math
import warnings
from dataclasses import dataclass, field
from typing import Iterable, Optional

import numpy as np


@dataclass(frozen=True)
class SeatRiseParams:
    """The rule's numbers (Newton's decisions of 2026-10-23 and the records' measurements)."""
    ring_band_m: float = 0.015          # a ring point lies this close to the registered seat top in the closed frame
    footprint_margin_m: float = 0.015   # ... inside the seat's footprint grown by this
    region_m: tuple = (0.03, 0.15)      # the decision region: ring points this far from the hinge (fixture frame)
    hand_rise_m: float = 0.10           # a pixel that rose more than this is a hand or an arm, not the seat
    hand_mask_m: float = 0.14           # points this close to a hand frame never count (the gap check's mask)
    min_pixels: int = 1500              # usable pixels needed for any answer
    closed_change_m: float = 0.004      # closed: a pixel within this of the closed frame is unchanged ...
    closed_frac: float = 0.85           # ... and this share of the usable pixels is
    moved_m: float = 0.008              # lifted: a pixel moved by at least this ...
    moved_frac: float = 0.60            # ... this share of the usable pixels did ...
    min_deg: float = 10.0               # ... and the hinge fit says at least this (Newton: 10 deg) ...
    hinge_tol_m: float = 0.040          # ... about a hinge this close to the model's,
    down_frac: float = 0.60             # ... or this share moved DOWN (the rim shows where the ring was)
    min_fit_points: int = 200           # fewer moved-up pixels than this = no fit
    max_age_s: float = 900.0            # the reference is used at most this long after its newest frame
    lid_min_z_m: float = 0.55           # the open lid's points: above this in the closed frame
    lid_unchanged_m: float = 0.010      # a seen lid point within this (along the ray) is unchanged ...
    lid_unchanged_frac: float = 0.80    # ... and fewer than this share unchanged = the lid moved: refused
    lid_min_pixels: int = 1500          # the open lid counts as seen with this many unoccluded points
    max_frames: int = 5                 # the reference is the median of up to this many closed frames
    replace_m: float = 0.006            # a closed frame this far (median, on the ring) from the reference replaces it


DEFAULT_PARAMS = SeatRiseParams()


# ── fixture geometry ─────────────────────────────────────────────────────────
@dataclass
class SeatGeometry:
    """Where the seat is: the hinge (a point on its axis, world frame), the horizontal unit vector
    from the hinge towards the seat front, and the seat's xy footprint in the closed state."""
    hinge: np.ndarray
    toward_seat: np.ndarray
    footprint_xy: Optional[np.ndarray] = None      # (N, 2) world xy of the seat's cells, closed
    box_xy: Optional[tuple] = None                 # (x0, x1, y0, y1) when there are no cells
    _grid: Optional[tuple] = field(default=None, repr=False, compare=False)

    @classmethod
    def from_fixture(cls, fixture_pose, hinge_body, axis_body, seat_cells_body=None, box_xy=None) -> "SeatGeometry":
        """From the fixture's body frame: `fixture_pose` (4x4 body -> world), the seat joint's
        origin and axis in the body, and the seat's cells placed closed (body, (N, 3))."""
        pose = np.asarray(fixture_pose, dtype=float)
        hinge = pose[:3, :3] @ np.asarray(hinge_body, dtype=float) + pose[:3, 3]
        axis = pose[:3, :3] @ np.asarray(axis_body, dtype=float)
        toward = np.cross(axis, [0.0, 0.0, 1.0])
        footprint = None
        if seat_cells_body is not None and len(seat_cells_body):
            cells = np.asarray(seat_cells_body, dtype=float) @ pose[:3, :3].T + pose[:3, 3]
            footprint = cells[:, :2]
            if np.dot(cells.mean(axis=0) - hinge, toward) < 0:
                toward = -toward
        elif box_xy is not None:
            centre = np.array([0.5 * (box_xy[0] + box_xy[1]), 0.5 * (box_xy[2] + box_xy[3]), hinge[2]])
            if np.dot(centre - hinge, toward) < 0:
                toward = -toward
        toward = toward / max(np.linalg.norm(toward), 1e-9)
        return cls(hinge=hinge, toward_seat=toward, footprint_xy=footprint, box_xy=None if footprint is not None else box_xy)

    def distance_from_hinge(self, points: np.ndarray) -> np.ndarray:
        """Horizontal distance from the hinge axis towards the seat front (metres), per point."""
        return (np.asarray(points, dtype=float) - self.hinge) @ self.toward_seat

    def inside(self, xy: np.ndarray, margin_m: float) -> np.ndarray:
        """True where a world xy lies within `margin_m` of the seat's footprint (or inside the box);
        the margin holds to within the 2.5 mm grid the footprint is rasterised on."""
        xy = np.asarray(xy, dtype=float).reshape(-1, 2)
        if self.footprint_xy is None:
            if self.box_xy is None:
                return np.zeros(len(xy), dtype=bool)
            x0, x1, y0, y1 = self.box_xy
            return (xy[:, 0] >= x0) & (xy[:, 0] <= x1) & (xy[:, 1] >= y0) & (xy[:, 1] <= y1)
        cell = 0.0025
        if self._grid is None or self._grid[0] != margin_m:
            # The footprint rasterised on a 2.5 mm grid and grown by the margin (a disc), so a lookup
            # is an index, with no KD-tree: the seat's cells are 1 cm apart, the margin 15 mm.
            lo = self.footprint_xy.min(axis=0) - margin_m - cell
            shape = np.ceil((self.footprint_xy.max(axis=0) + margin_m + cell - lo) / cell).astype(int) + 1
            occ = np.zeros(shape, dtype=bool)
            ij = np.floor((self.footprint_xy - lo) / cell).astype(int)
            occ[ij[:, 0], ij[:, 1]] = True
            r = int(math.ceil(margin_m / cell))
            grown = np.zeros_like(occ)
            for di in range(-r, r + 1):
                for dj in range(-r, r + 1):
                    if di * di + dj * dj > r * r:
                        continue
                    grown[max(0, di):shape[0] + min(0, di), max(0, dj):shape[1] + min(0, dj)] |= \
                        occ[max(0, -di):shape[0] - max(0, di), max(0, -dj):shape[1] - max(0, dj)]
            self._grid = (margin_m, lo, grown)
        _, lo, grown = self._grid
        ij = np.floor((xy - lo) / cell).astype(int)
        ok = (ij[:, 0] >= 0) & (ij[:, 1] >= 0) & (ij[:, 0] < grown.shape[0]) & (ij[:, 1] < grown.shape[1])
        out = np.zeros(len(xy), dtype=bool)
        out[ok] = grown[ij[ok, 0], ij[ok, 1]]
        return out


# ── frames ───────────────────────────────────────────────────────────────────
def _k(intrinsics) -> tuple:
    """(fx, fy, cx, cy) from an Intrinsics-like object, a dict, or such a 4-sequence."""
    if isinstance(intrinsics, (tuple, list)) and len(intrinsics) == 4:
        return tuple(float(v) for v in intrinsics)
    get = intrinsics.get if isinstance(intrinsics, dict) else (lambda k: getattr(intrinsics, k))
    return float(get("fx")), float(get("fy")), float(get("cx")), float(get("cy"))


def deproject(depth: np.ndarray, intrinsics, camera_to_world) -> tuple[np.ndarray, np.ndarray]:
    """-> (world points (H, W, 3), valid (H, W)) for a depth image in metres (0 / nan = no depth)."""
    d = np.asarray(depth, dtype=np.float64)
    fx, fy, cx, cy = _k(intrinsics)
    h, w = d.shape
    v, u = np.mgrid[0:h, 0:w]
    valid = np.isfinite(d) & (d > 0.05) & (d < 3.0)
    dd = np.where(valid, d, 0.0)
    m = np.asarray(camera_to_world, dtype=np.float64)
    x = (u - cx) * dd / fx
    y = (v - cy) * dd / fy
    pts = np.stack([x, y, dd], axis=-1) @ m[:3, :3].T + m[:3, 3]
    return pts, valid


def project(points: np.ndarray, intrinsics, camera_to_world, shape) -> tuple[np.ndarray, np.ndarray]:
    """World points (N, 3) into a camera: -> (for each target pixel (H*W,), the index of the NEAREST
    point that lands on it or -1, and that point's depth along the optical axis (nan where none))."""
    h, w = shape
    fx, fy, cx, cy = _k(intrinsics)
    m = np.asarray(camera_to_world, dtype=np.float64)
    pc = (np.asarray(points, dtype=np.float64) - m[:3, 3]) @ m[:3, :3]          # world -> camera
    z = pc[:, 2]
    with np.errstate(divide="ignore", invalid="ignore"):
        u = np.rint(fx * pc[:, 0] / z + cx)
        v = np.rint(fy * pc[:, 1] / z + cy)
    ok = (z > 0.05) & (u >= 0) & (u < w) & (v >= 0) & (v < h)
    src = np.flatnonzero(ok)
    pix = (v[ok] * w + u[ok]).astype(np.int64)
    order = np.lexsort((z[ok], pix))                 # by pixel, nearest first
    pix_sorted = pix[order]
    first = np.ones(len(order), dtype=bool)
    first[1:] = pix_sorted[1:] != pix_sorted[:-1]
    winner = np.full(h * w, -1, dtype=np.int64)
    winner[pix_sorted[first]] = src[order[first]]
    depth = np.full(h * w, np.nan)
    depth[pix_sorted[first]] = z[src[order[first]]]
    return winner, depth


def pose_difference(a, b) -> tuple[float, float]:
    """(translation metres, rotation degrees) between two 4x4 camera poses."""
    a = np.asarray(a, dtype=float); b = np.asarray(b, dtype=float)
    dt = float(np.linalg.norm(a[:3, 3] - b[:3, 3]))
    c = (np.trace(a[:3, :3].T @ b[:3, :3]) - 1.0) / 2.0
    return dt, float(math.degrees(math.acos(max(-1.0, min(1.0, c)))))


def _near_hands(points: np.ndarray, hands: Iterable, radius: float) -> np.ndarray:
    """True for each point (N, 3) within `radius` of any hand position."""
    out = np.zeros(len(points), dtype=bool)
    for c in hands or ():
        out |= np.linalg.norm(points - np.asarray(c, dtype=float)[:3], axis=1) <= radius
    return out


def fit_hinge(s: np.ndarray, z_rel: np.ndarray, iters: int = 20, k: float = 2.5, bins: int = 10) -> tuple[float, float, int]:
    """Robust line z_rel = a * s + b through the lifted seat's points (s: distance from the model's
    hinge towards the seat front, z_rel: height above the closed seat top). Started from a line
    through the per-bin MEDIANS along s (a hand or an edge in a fifth of the points does not move a
    median), then least squares on the points within k MADs of the line, repeated until the set
    settles. -> (a, b, inliers); the lifted angle is atan(a) and the fitted hinge, where the line
    meets the closed top, lies at s = -b / a."""
    s = np.asarray(s, dtype=float); z = np.asarray(z_rel, dtype=float)
    if len(s) < 2:
        return 0.0, 0.0, 0
    edges = np.quantile(s, np.linspace(0.0, 1.0, bins + 1))
    which = np.clip(np.searchsorted(edges, s, side="right") - 1, 0, bins - 1)
    ms = np.array([np.median(s[which == i]) for i in range(bins) if (which == i).any()])
    mz = np.array([np.median(z[which == i]) for i in range(bins) if (which == i).any()])
    a, b = np.polyfit(ms, mz, 1) if len(ms) >= 2 else (0.0, float(np.median(z)))
    use = np.ones(len(s), dtype=bool)
    for _ in range(iters):
        res = z - (a * s + b)
        mad = 1.4826 * float(np.median(np.abs(res[use] - np.median(res[use])))) + 1e-4
        new = np.abs(res) <= k * mad
        if new.sum() < 2 or np.array_equal(new, use):
            break
        use = new
        A = np.stack([s[use], np.ones(int(use.sum()))], axis=1)
        a, b = np.linalg.lstsq(A, z[use], rcond=None)[0]
    return float(a), float(b), int(use.sum())


# ── the reference ────────────────────────────────────────────────────────────
@dataclass
class SeatReference:
    """The seat closed: the median of up to `max_frames` closed frames as world points, each
    labelled — on the ring (near the registered seat top, inside the footprint), on the open lid,
    and its distance from the hinge — with the frames it was built from and the newest one's time.
    The median is taken in the first frame's view; later frames are projected into it."""
    frames: list                  # [(depth (H, W) metres, camera_to_world 4x4, (fx, fy, cx, cy))]
    views: list                   # each frame as the first frame's camera sees it (H, W; nan = no point)
    points: np.ndarray            # (N, 3) world, the valid points of the median frame
    ring: np.ndarray              # (N,) bool
    lid: np.ndarray               # (N,) bool
    s: np.ndarray                 # (N,) distance from the hinge towards the seat front
    seat_top_m: float
    time_s: float

    @property
    def camera_to_world(self) -> np.ndarray:
        return self.frames[0][1]

    def summary(self) -> dict:
        return {"frames": len(self.frames), "seat_top_m": round(self.seat_top_m, 4), "time_s": round(self.time_s, 3),
                "ring_px": int(self.ring.sum()), "lid_px": int(self.lid.sum()),
                "camera_xyz": [round(float(v), 4) for v in self.camera_to_world[:3, 3]]}


def _in_view(depth, camera_to_world, intrinsics, view_cam, view_k, shape) -> np.ndarray:
    """A depth frame as the camera `view_cam` would see its points (nan where none lands)."""
    pts, valid = deproject(depth, intrinsics, camera_to_world)
    _, d = project(pts[valid], view_k, view_cam, shape)
    return d.reshape(shape)


def _view(frames: list, frame) -> np.ndarray:
    """`frame` as the first of `frames` sees it (the first one itself, as it is)."""
    depth, cam, k = frame
    if not frames:
        return np.where(np.isfinite(depth) & (depth > 0.05) & (depth < 3.0), depth, np.nan)
    depth0, cam0, k0 = frames[0]
    return _in_view(depth, cam, k, cam0, k0, np.asarray(depth0).shape)


def _build(frames: list, views: list, seat_top_m: float, geometry: SeatGeometry, time_s: float, hands,
           params: SeatRiseParams) -> SeatReference:
    depth0, cam0, k0 = frames[0]
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", RuntimeWarning)          # a pixel with no depth in any frame stays nan
        median = np.nanmedian(np.stack(views), axis=0) if len(views) > 1 else views[0]
    pts, valid = deproject(np.nan_to_num(median, nan=0.0), k0, cam0)
    points = pts[valid]
    ring = (np.abs(points[:, 2] - seat_top_m) <= params.ring_band_m)
    idx = np.flatnonzero(ring)
    ring[idx] = geometry.inside(points[idx, :2], params.footprint_margin_m)
    lid = points[:, 2] >= params.lid_min_z_m
    if hands:
        lid &= ~_near_hands(points, hands, params.hand_mask_m)
    return SeatReference(frames=list(frames), views=list(views), points=points, ring=ring, lid=lid,
                         s=geometry.distance_from_hinge(points), seat_top_m=float(seat_top_m), time_s=float(time_s))


def _compare(ref: SeatReference, depth, camera_to_world, intrinsics) -> dict:
    """The reference seen from this frame's camera: per pixel of this frame, the nearest reference
    point on its ray (index, or -1), that point's depth, and this frame's own points."""
    frame = np.asarray(depth, dtype=np.float64)
    winner, ref_depth = project(ref.points, intrinsics, camera_to_world, frame.shape)
    pts, valid = deproject(frame, intrinsics, camera_to_world)
    return {"winner": winner, "ref_depth": ref_depth, "now_depth": np.where(valid, frame, np.nan).reshape(-1),
            "points": pts.reshape(-1, 3), "valid": valid.reshape(-1)}


def update_reference(ref: Optional[SeatReference], depth, camera_to_world, intrinsics, seat_top_m: float,
                     geometry: SeatGeometry, time_s: float, hands=(), params: SeatRiseParams = DEFAULT_PARAMS
                     ) -> tuple[Optional[SeatReference], str, str]:
    """Add a CLOSED frame, from wherever the camera is. -> (the reference, 'started' | 'added' |
    'replaced' | 'refused', why).

    A frame whose view of the ring differs from the reference by more than `replace_m` (median,
    along its own rays) replaces it — the fixture moved; otherwise it joins the median (the newest
    `max_frames`). Refused when the open lid is not in view (its points are the lid guard's
    evidence) or too little ring is seen."""
    frame = (np.asarray(depth, dtype=np.float64), np.asarray(camera_to_world, dtype=float), _k(intrinsics))
    action, frames, views = "started", [frame], [_view([], frame)]
    if ref is not None:
        c = _compare(ref, depth, camera_to_world, intrinsics)
        on = (c["winner"] >= 0) & c["valid"]
        on[on] = ref.ring[c["winner"][on]]
        dz = np.abs(c["points"][on, 2] - ref.points[c["winner"][on], 2])
        if len(dz) < params.min_pixels or float(np.median(dz)) > params.replace_m:
            action = "replaced"
        else:
            action = "added"
            keep_n = params.max_frames - 1                 # the first frame (the view) and the newest others
            frames = ref.frames[:1] + (ref.frames[1:] + [frame])[-keep_n:]
            views = ref.views[:1] + (ref.views[1:] + [_view(ref.frames, frame)])[-keep_n:]
    new = _build(frames, views, seat_top_m, geometry, time_s, hands, params)
    if int(new.lid.sum()) < params.lid_min_pixels:
        return ref, "refused", (f"the open lid is not in view ({int(new.lid.sum())} points above "
                                f"{params.lid_min_z_m:.2f} m, {params.lid_min_pixels} needed): the lid guard could not run")
    if int(new.ring.sum()) < params.min_pixels:
        return ref, "refused", f"only {int(new.ring.sum())} ring points in the closed frame ({params.min_pixels} needed)"
    return new, action, f"{len(new.frames)} closed frame(s), {int(new.ring.sum())} ring points, {int(new.lid.sum())} open-lid points"


# ── the reading ──────────────────────────────────────────────────────────────
@dataclass
class SeatRise:
    """What the comparison says. `state` is "seat_lifted", "seat_closed" or "" (no answer)."""
    state: str = ""
    confidence: float = 0.0
    reason: str = ""
    reference_valid: bool = False
    lid_guard: str = ""                        # "unchanged" | "moved" | "unseen" (fewer than lid_min_pixels seen)
    angle_deg: Optional[float] = None
    hinge_offset_m: Optional[float] = None     # fitted hinge minus the model's, towards the seat front
    hinge_xyz: Optional[list] = None
    usable_px: int = 0
    region_px: int = 0
    unchanged_frac: float = 0.0
    moved_frac: float = 0.0
    up_frac: float = 0.0
    down_frac: float = 0.0
    hand_frac: float = 0.0                     # share of the seen decision region the hand covers (masked or risen > hand_rise_m)
    lid_unchanged_frac: Optional[float] = None # over the lid points seen (occluded ones left out)
    lid_px: int = 0                            # lid points seen, not occluded
    lid_occluded_px: int = 0                   # lid points something nearer hides (the glove, an arm, the raised seat)
    pose_dt_mm: Optional[float] = None         # how far the camera is from the reference's first frame: diagnostic only
    pose_dr_deg: Optional[float] = None
    age_s: Optional[float] = None
    reference_frames: int = 0
    median_change_mm: Optional[float] = None

    def as_dict(self) -> dict:
        out = {}
        for k, v in self.__dict__.items():
            out[k] = round(v, 4) if isinstance(v, float) else v
        return out


def read_seat(ref: Optional[SeatReference], depth, camera_to_world, intrinsics, geometry: SeatGeometry,
              now_s: float, hands=(), params: SeatRiseParams = DEFAULT_PARAMS) -> SeatRise:
    """The seat's state from this frame against the closed reference, or no answer with the reason."""
    out = SeatRise()
    if ref is None:
        out.reason = "no closed reference at this placement"
        return out
    out.reference_frames = len(ref.frames)
    out.age_s = float(now_s - ref.time_s)
    dt, dr = pose_difference(ref.camera_to_world, camera_to_world)
    out.pose_dt_mm, out.pose_dr_deg = dt * 1000.0, dr
    if out.age_s > params.max_age_s or out.age_s < -1.0:
        out.reason = f"the closed frame is {out.age_s:.0f} s old (limit {params.max_age_s:.0f} s)"
        return out
    c = _compare(ref, depth, camera_to_world, intrinsics)
    win, valid, now_pts = c["winner"], c["valid"], c["points"]
    seen = (win >= 0) & valid                                  # a reference point on the ray, and depth now
    hand_now = np.zeros(len(win), dtype=bool)
    if hands:
        si = np.flatnonzero(seen)
        hand_now[si] = _near_hands(now_pts[si], hands, params.hand_mask_m)
    # THE LID GUARD, along this frame's rays: farther than the lid was = it left; nearer = occluded
    on_lid = seen & ~hand_now
    on_lid[on_lid] = ref.lid[win[on_lid]]
    along = c["now_depth"][on_lid] - c["ref_depth"][on_lid]
    occluded = along < -params.lid_unchanged_m
    out.lid_occluded_px = int(occluded.sum())
    out.lid_px = int((~occluded).sum())
    if out.lid_px >= params.lid_min_pixels:
        out.lid_unchanged_frac = float(np.mean(np.abs(along[~occluded]) <= params.lid_unchanged_m))
        if out.lid_unchanged_frac < params.lid_unchanged_frac:
            out.lid_guard = "moved"
            out.reason = (f"the lid moved since the closed frame ({out.lid_unchanged_frac * 100:.0f} % of its "
                          f"{out.lid_px} seen points unchanged, {params.lid_unchanged_frac * 100:.0f} % needed): a lid "
                          f"turning on the seat's hinge mimics a lifted seat")
            return out
        out.lid_guard = "unchanged"
    else:
        out.lid_guard = "unseen"
    out.reference_valid = True
    # THE DECISION REGION: the ring near the hinge, where the lifting hand never is
    region = seen.copy()
    region[region] = ref.ring[win[region]]
    ri = np.flatnonzero(region)
    s = ref.s[win[ri]]
    ri = ri[(s >= params.region_m[0]) & (s <= params.region_m[1])]
    out.region_px = int(len(ri))
    dz = now_pts[ri, 2] - ref.points[win[ri], 2]
    handlike = (dz > params.hand_rise_m) | hand_now[ri]
    use, ch = ri[~handlike], dz[~handlike]
    out.usable_px = int(len(use))
    out.hand_frac = float(handlike.mean()) if len(ri) else 0.0
    if out.usable_px < params.min_pixels:
        out.reason = (f"only {out.usable_px} usable ring pixels near the hinge ({params.min_pixels} needed; "
                      f"the hand covers {out.hand_frac * 100:.0f} % of the {out.region_px} seen)")
        return out
    out.median_change_mm = float(np.median(ch)) * 1000.0
    out.unchanged_frac = float(np.mean(np.abs(ch) <= params.closed_change_m))
    out.moved_frac = float(np.mean(np.abs(ch) >= params.moved_m))
    out.up_frac = float(np.mean(ch >= params.moved_m))
    out.down_frac = float(np.mean(ch <= -params.moved_m))
    counts = (f"{out.usable_px} ring pixels near the hinge: {out.unchanged_frac * 100:.0f} % unchanged, "
              f"{out.moved_frac * 100:.0f} % moved >= {params.moved_m * 1000:.0f} mm "
              f"({out.down_frac * 100:.0f} % down), hand {out.hand_frac * 100:.0f} %")
    if out.unchanged_frac >= params.closed_frac:
        out.state, out.confidence = "seat_closed", out.unchanged_frac
        out.reason = f"closed as in the reference: {counts}"
        return out
    fit_ok, fit_text = False, "no hinge fit"
    up = use[ch >= params.moved_m]
    if len(up) >= params.min_fit_points:
        a, b, n = fit_hinge(geometry.distance_from_hinge(now_pts[up]), now_pts[up, 2] - ref.seat_top_m)
        if abs(a) > 1e-6:
            out.angle_deg = float(math.degrees(math.atan(a)))
            out.hinge_offset_m = float(-b / a)
            out.hinge_xyz = [round(float(v), 4) for v in geometry.hinge + out.hinge_offset_m * geometry.toward_seat]
            fit_ok = out.angle_deg >= params.min_deg and abs(out.hinge_offset_m) <= params.hinge_tol_m
            fit_text = (f"hinge fit {out.angle_deg:.1f} deg about a hinge {out.hinge_offset_m * 1000:+.0f} mm from the model's "
                        f"({n} pixels; needs >= {params.min_deg:.0f} deg within {params.hinge_tol_m * 1000:.0f} mm)")
    rim = out.down_frac >= params.down_frac
    if out.moved_frac >= params.moved_frac and rim:
        out.state, out.confidence = "seat_lifted", out.moved_frac
        out.reason = f"lifted: {counts}; the rim shows where the ring was"
        return out
    if out.moved_frac >= params.moved_frac and fit_ok:
        if out.lid_guard != "unchanged":
            # only a lid turning on the seat's hinge can fake this reading, and its points are not seen
            out.reason = (f"a hinge fit says lifted ({fit_text}) but the open lid is not seen "
                          f"({out.lid_px} points, {params.lid_min_pixels} needed), so a lid lowered onto the hinge "
                          f"cannot be ruled out: {counts}")
            return out
        out.state, out.confidence = "seat_lifted", out.moved_frac
        out.reason = f"lifted: {counts}; {fit_text}"
        return out
    out.reason = f"neither closed nor lifted: {counts}; {fit_text}"
    return out
