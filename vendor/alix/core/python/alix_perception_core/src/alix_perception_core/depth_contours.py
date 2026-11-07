"""THE LID READ FROM THE DEPTH IMAGE'S ISO-DISTANCE CONTOURS (the user, 2026-11-06: "contours should be applied in the
depth image, not in the colored image, because contours are points joining or lines joining points with the same
distances ... we will have a 1 centimeter tolerance").

No robot model, no scene place, no camera pose: everything is measured in the camera's own frame, from the depth
alone, so the reading holds whatever the placement, the pole height or the camera's mount. The fixture fit
(seat_state / fixture_place) renders the commode's model at the scene's place and could not fit a closed lid read
20 mm off on 6 Nov (a new robot: its camera pose is the old robot's model); the same frames read closed here.

What the contour map shows, and the two measures taken from it:
- A closed lid is one plane: straight, parallel, evenly spaced iso-lines. Its angle to the FLOOR's plane, both read in
  the same frame, is the lid's angle: closed at an inspection 0.1-1.5 deg, resting on the seat after a closing push
  6-17 deg (the fit read 10 on the same frames), held lifted 30-37, standing open 68-71 (the lab's frames, 5-6 Nov).
- An open lid shows the bowl: concentric closed iso-lines converging on the drain. A region enclosed by nearer pixels,
  deeper than its rim by cavity_min_m over cavity_min_frac of the image, whose deepest point lies well inside it (a
  floor seen through a gap between nearer things is enclosed too, but its deepest point lies on the gap's far edge):
  28-35 cm over 5.5-21 % of the image in every open frame, none in a closed one, at most 13 cm and 3 % under a lid
  held lifted. The D435 returns 0 for anything nearer than its range (the glove): on a cavity's rim a hole is a NEAR thing.
- The floor is the farthest large plane (the lid is near, the wall between); every frame of the lab's checks has it.
- Standing faces (planes at standing_deg to the floor, parallel ones apart by more than 15 mm counted once) are read
  for the record: the seat standing against the open lid is a second face 18-22 mm in front of the lid's.

The 1 cm tolerance is the plane fits' and the bands'; the D435's noise at these distances is 3-5 mm. Pure numpy plus
OpenCV's contours and distance transform, 1/stride resolution, about 0.3 s a frame."""
from __future__ import annotations

import math
import time
from dataclasses import dataclass, field, asdict
from typing import Optional

import numpy as np


@dataclass(frozen=True)
class ContourParams:
    band_m: float = 0.01             # the tolerance: a plane's inliers, and the iso-distance bands
    stride: int = 2                  # read at 1/stride resolution
    hand_mask_m: float = 0.0         # depth within this of a hand frame is left out; 0 (the default): nothing is, the
                                     # glove is a hole already (too near for the D435) and a hand masked out of a lid it
                                     # rests on bites the lid's patch (its solidity, 6 Nov)
    cavity_min_m: float = 0.20       # the bowl: an enclosed region this much deeper than its rim ...
    cavity_min_frac: float = 0.02    # ... over this share of the image (a lid held lifted shows at most 13 cm, 3 %)
    lid_height_m: tuple = (0.25, 0.80)   # a lid's plane this far above the floor; a step or a table is no lid
    standing_no_lid_deg: float = 60.0    # a central plane steeper than this with no bowl in view is a wall, not the lid
    cavity_centred: float = 0.08     # the deepest point's distance from the rim, in radii of the region: a floor seen
                                     # through a gap is deepest at the gap's far edge (0), the drain lies inside (0.12-0.4
                                     # over the lab's views; from inspection pose 4 the drain sits near the far rim)
    cavity_round: float = 0.55       # the region's largest inscribed radius in radii of its area: a strip of floor seen
                                     # between the lid's edge and the glove is long and thin (0.1-0.45), the bowl 0.65-0.86
    lid_solidity_min: float = 0.80   # the lid plane's largest patch, holes filled, its area over its convex hull's: a
                                     # closed lid is solid; the seat ring around a bowl cut by the frame's edge is a C
                                     # (0.4-0.7), no lid, so no "closed" from it (Isaac's edge views, 6 Nov)
    cavity_rms_min_m: float = 0.02   # the region's points off their own best plane, RMS: a funnel is no plane (5-10 cm),
                                     # a wall or a floor patch at the frame's edge is one (under 1 cm)
    lifted_deg: tuple = (10.0, 55.0) # a plane at this angle to the floor over a visible bowl is the lid held lifted ...
    lifted_over_bowl: float = 0.30   # ... when this share of its pixels lies within the bowl's outline (Isaac, 6 Nov: the
                                     # commode's sloped body read as a lifted lid without it)
    cavity_above_floor_m: float = 0.10   # the cavity's pixels lie, in the median, this far above the floor's plane: the
                                     # bowl's walls stand on the commode (its bulk 20-30 cm up, the trap 3-5 cm), the floor
                                     # seen past a closed lid's edge or through a gap is the floor (0-2 cm)
    hole_margin_px: int = 0          # pixels this close to a hole count as the hole; 0: off (eroding the depth around the
                                     # glove's hole made the hole enclose the bowl and the ring together, 6 Nov)
    floor_min_frac: float = 0.010    # a plane's inliers, as a share of the image, to count as the floor ...
    lid_min_frac: float = 0.020      # ... as the lid's plane (a sliver of a lid at the frame's corner is no lid) ...
    face_min_frac: float = 0.006     # ... as a standing face
    standing_deg: tuple = (55.0, 85.0)
    roi: tuple = (0.12, 0.88, 0.12, 0.88)   # rows and columns (fractions) the lid's plane is looked for in
    ransac_iters: int = 300
    seed: int = 0


@dataclass
class LidContours:
    """One frame's reading. ``state``: lid_closed / lid_lifted / lid_open, or "" when the frame cannot be judged
    (``reason`` says why: no floor, no lid plane)."""
    state: str = ""
    confidence: float = 0.0
    lid_floor_deg: Optional[float] = None    # the central plane's angle to the floor (the lid's when it covers the bowl)
    cavity_m: float = 0.0                    # the deepest enclosed cavity: its depth below its rim ...
    cavity_frac: float = 0.0                 # ... and its share of the image
    cavity_centred: Optional[float] = None
    cavity_round: Optional[float] = None
    cavity_floor_m: Optional[float] = None   # the cavity's pixels' median height above the floor's plane
    cavity_rms_m: Optional[float] = None     # the cavity's points off their best plane (RMS)
    lifted_plane_deg: Optional[float] = None # a lid-sized plane at lifted_deg over the bowl (the lid held lifted)
    standing_faces: int = 0
    standing_offsets_mm: list = field(default_factory=list)
    floor_px: int = 0
    floor_depth_m: Optional[float] = None    # the floor's distance from the camera along its normal
    lid_px: int = 0
    lid_height_m: Optional[float] = None     # the central plane's centroid above the floor
    lid_solidity: Optional[float] = None     # the plane's largest patch: its area over its convex hull's (a ring: low)
    reason: str = ""
    time_ms: float = 0.0
    masks: Optional[dict] = None             # with keep_masks: the floor's and the lid's inlier masks, the cavity's
                                             # contour and the band image, for a picture of the reading (never recorded)

    def as_dict(self) -> dict:
        d = asdict(self)
        d.pop("masks", None)
        for k in ("lid_floor_deg", "cavity_m", "cavity_frac", "cavity_centred", "cavity_round", "cavity_floor_m", "cavity_rms_m",
                  "lifted_plane_deg", "floor_depth_m", "lid_height_m", "lid_solidity", "confidence", "time_ms"):
            if d.get(k) is not None:
                d[k] = round(float(d[k]), 4)
        return d


def _ransac_plane(Q: np.ndarray, tol: float, iters: int, rng: np.random.Generator, sample: int = 20000):
    """-> (normal toward the camera, a point, the inlier mask over Q) of the plane most of ``Q`` lies on within
    ``tol``, refined by least squares over its inliers; None with too few points."""
    n_pts = len(Q)
    if n_pts < 50:
        return None
    pick = Q if n_pts <= sample else Q[rng.choice(n_pts, sample, replace=False)]
    best_n, best_c, best_count = None, None, -1
    for start in range(0, iters, 100):                           # batches of 100 hypotheses against the sample
        k = min(100, iters - start)
        idx = rng.integers(0, len(pick), size=(k, 3))
        a, b, c = pick[idx[:, 0]], pick[idx[:, 1]], pick[idx[:, 2]]
        n = np.cross(b - a, c - a)
        nn = np.linalg.norm(n, axis=1)
        ok = nn > 1e-9
        if not ok.any():
            continue
        n = n[ok] / nn[ok][:, None]
        a = a[ok]
        dist = np.abs(np.einsum("ij,mj->im", n, pick) - np.einsum("ij,ij->i", n, a)[:, None])   # |n . p - n . a|
        counts = (dist < tol).sum(axis=1)
        j = int(np.argmax(counts))
        if counts[j] > best_count:
            best_count, best_n, best_c = int(counts[j]), n[j], a[j]
    if best_n is None:
        return None
    inl = np.abs((Q - best_c) @ best_n) < tol
    S = Q[inl]
    if len(S) < 3:
        return None
    c = S.mean(axis=0)
    _, _, vt = np.linalg.svd(S - c, full_matrices=False)
    n = vt[2]
    if n @ c > 0:                                               # the normal toward the camera (the camera at the origin)
        n = -n
    inl = np.abs((Q - c) @ n) < tol
    return n, c, inl


def _angle_deg(n1, n2) -> float:
    return float(math.degrees(math.acos(min(1.0, max(-1.0, abs(float(n1 @ n2)))))))


def read_lid(depth, intrinsics, hands=(), camera_to_world=None, closed_below_deg: float = 10.0,
             params: ContourParams = ContourParams(), keep_masks: bool = False) -> LidContours:
    """The lid's state from one depth frame (metres, the camera's frame). ``intrinsics`` sized to it (fx, fy, cx, cy as
    attributes or keys); ``hands`` the hand frames' positions in the world and ``camera_to_world`` the camera's pose,
    used only to leave the hands' pixels out (None: nothing masked); ``closed_below_deg`` the angle under which the lid
    is closed (the fit's gap_lid_fit_min_deg, or a closing's allowance)."""
    import cv2

    t0 = time.monotonic()
    out = LidContours()
    rng = np.random.default_rng(params.seed)
    get = intrinsics.get if isinstance(intrinsics, dict) else (lambda k: getattr(intrinsics, k))
    s = max(1, int(params.stride))
    d = np.asarray(depth, dtype=np.float32)
    fx, fy, cx, cy = (float(get(k)) for k in ("fx", "fy", "cx", "cy"))
    h, w = d.shape
    valid = np.isfinite(d) & (d > 0.15) & (d < 3.0)
    v, u = np.indices(d.shape)
    P = np.stack([(u - cx) * d / fx, (v - cy) * d / fy, d], axis=-1).astype(np.float64)
    if hands is not None and len(hands) and camera_to_world is not None and params.hand_mask_m > 0:
        T = np.asarray(camera_to_world, dtype=np.float64)
        Wp = P @ T[:3, :3].T + T[:3, 3]
        hand = np.zeros(d.shape, dtype=bool)
        for c in hands:
            hand |= np.linalg.norm(Wp - np.asarray(c, dtype=np.float64)[:3], axis=-1) <= params.hand_mask_m
        valid &= ~hand                                           # a hand is a near thing: a hole, as the glove is
    d = np.where(valid, d, 0.0)
    n_valid = int(valid.sum())
    if n_valid < 8000:
        out.reason = f"only {n_valid} depth pixels"
        out.time_ms = (time.monotonic() - t0) * 1000
        return out
    sub = (slice(None, None, s), slice(None, None, s))         # the plane fits read every s-th pixel
    Ps, vs, ds = P[sub], valid[sub], d[sub]
    px_scale = 1.0 / (s * s)
    n_fit = float(h * w) * px_scale                                 # the pixels the plane fits read
    floor_min, lid_min, face_min = (params.floor_min_frac * n_fit, params.lid_min_frac * n_fit, params.face_min_frac * n_fit)
    tol = float(params.band_m)

    # THE FLOOR: the farthest large plane among the points beyond the nearest third
    far = vs & (ds >= np.percentile(ds[vs], 35))
    Qf = Ps[far]
    cands = []
    for _ in range(4):
        f = _ransac_plane(Qf, tol * 1.2, params.ransac_iters, rng)
        if f is None:
            break
        n, c, inl = f
        if inl.sum() >= floor_min:
            cands.append((float(-(n @ c)), n, c, int(inl.sum())))   # the plane's distance from the camera
        Qf = Qf[~inl]
        if len(Qf) < floor_min:
            break
    floor = None
    if cands:                                                    # the largest of the far ones (a tilted sliver of the floor
        farthest = max(x[0] for x in cands)                      # can read a little farther than the floor itself)
        floor = max((x for x in cands if x[0] >= farthest - 0.10), key=lambda x: x[3])
    if floor is None:
        out.reason = "no floor plane in view"
        out.time_ms = (time.monotonic() - t0) * 1000
        return out
    f_depth, fn, fc, f_px = floor
    out.floor_px, out.floor_depth_m = int(f_px / px_scale), f_depth
    off_floor = np.abs((Ps - fc) @ fn) > 1.5 * tol

    # THE CENTRAL PLANE (the lid when it covers the bowl): the largest plane in the ROI off the floor
    r0, r1, c0, c1 = params.roi
    roi = np.zeros(d.shape, dtype=bool)
    roi[int(r0 * h):int(r1 * h), int(c0 * w):int(c1 * w)] = True
    rois = roi[sub]
    Qc = Ps[vs & rois & off_floor]
    lid = _ransac_plane(Qc, tol, params.ransac_iters, rng)
    if lid is not None and lid[2].sum() < lid_min:
        lid = None
    if lid is not None:
        out.lid_px = int(lid[2].sum() / px_scale)
        out.lid_floor_deg = _angle_deg(lid[0], fn)
        out.lid_height_m = float(abs((lid[1] - fc) @ fn))
        lm = np.zeros(Ps.shape[:2], dtype=np.uint8)
        lm[vs & rois & off_floor] = lid[2]
        lm = cv2.morphologyEx(lm, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
        n_cc, lab, st, _ = cv2.connectedComponentsWithStats(lm)
        if n_cc > 1:
            j = 1 + int(np.argmax(st[1:, cv2.CC_STAT_AREA]))
            cs_, _ = cv2.findContours((lab == j).astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if cs_:                                                  # the patch with its holes filled (a hand masked out
                big = max(cs_, key=cv2.contourArea)                  # of the lid's middle is no bite): its outline's area
                hull_a = cv2.contourArea(cv2.convexHull(big))        # over its hull's
                out.lid_solidity = float(cv2.contourArea(big) / hull_a) if hull_a > 0 else None

    # THE CAVITY: iso-distance bands; a region enclosed by nearer pixels (or holes), deeper than its rim, its deepest
    # point well inside
    m = int(params.hole_margin_px)
    valid_c = valid if m <= 0 else cv2.erode(valid.astype(np.uint8), np.ones((2 * m + 1, 2 * m + 1), np.uint8)).astype(bool)
    B = np.where(valid_c, np.floor(d / tol).astype(np.int32), -1)
    in_roi = B[roi & valid_c]
    levels = np.unique(in_roi) if in_roi.size else np.zeros(0, dtype=np.int32)
    min_area = 1500
    kernel = np.ones((9, 9), np.uint8)
    best = None
    for lvl in levels:
        mask = ((B >= lvl) & valid_c).astype(np.uint8)
        cs, hier = cv2.findContours(mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
        if hier is None:
            continue
        for i, cnt in enumerate(cs):
            if hier[0][i][3] != -1:
                continue
            a = cv2.contourArea(cnt)
            if a < min_area:
                continue
            x, y, ww, hh = cv2.boundingRect(cnt)
            if x <= 1 or y <= 1 or x + ww >= w - 1 or y + hh >= h - 1:
                continue                                             # touches the frame's edge: not enclosed (a region open
                                                                     # to the edge is anything farther than what stands before it)
            inside = np.zeros(d.shape, np.uint8)
            cv2.drawContours(inside, [cnt], -1, 1, -1)
            ring = (cv2.dilate(inside, kernel) > 0) & (inside == 0)
            rb = B[ring]
            if rb.size == 0 or ((rb == -1) | (rb < lvl)).mean() < 0.9:
                continue                                             # bounded by farther pixels: no rim
            din = np.where((inside > 0) & valid_c, d, 0.0)
            k_deep = np.unravel_index(int(np.argmax(din)), din.shape)
            extent = float(din.max() - lvl * tol)
            dist = cv2.distanceTransform(inside, cv2.DIST_L2, 5)
            radius = max(math.sqrt(a / math.pi), 1.0)
            roundness = float(dist.max() / radius)
            if roundness < params.cavity_round:
                continue                                             # a strip, not a bowl
            centred = float(dist[k_deep] / radius)
            if centred < params.cavity_centred:
                continue
            sel = (inside > 0) & valid
            Q = P[sel]
            above = float(np.median((Q - fc) @ fn))                   # the region's height above the floor, in the median
            if above < params.cavity_above_floor_m:
                continue                                             # it is the floor: a gap, not the bowl
            Qs = Q[:: max(1, len(Q) // 4000)]
            qc = Qs.mean(axis=0)
            _, _, qvt = np.linalg.svd(Qs - qc, full_matrices=False)
            rms = float(np.sqrt(np.mean(((Qs - qc) @ qvt[2]) ** 2)))
            if rms < params.cavity_rms_min_m:
                continue                                             # a plane (a wall, a floor patch), not a funnel
            if best is None or extent > best[0]:
                best = (extent, a / (h * w), centred, roundness, above, cnt, rms)
    if best is not None:
        out.cavity_m, out.cavity_frac, out.cavity_centred, out.cavity_round, out.cavity_floor_m = best[:5]
        out.cavity_rms_m = best[6]
    if keep_masks:
        floor_mask = np.zeros(d.shape, dtype=bool); floor_mask[sub] = ~off_floor & vs
        lid_mask = np.zeros(d.shape, dtype=bool)
        if lid is not None:
            lm = np.zeros(Ps.shape[:2], dtype=bool); lm[vs & rois & off_floor] = lid[2]; lid_mask[sub] = lm
        out.masks = dict(floor=floor_mask, lid=lid_mask, bands=B, cavity=best[5] if best is not None else None,
                         floor_plane=(fn, fc), points=P, valid=valid)

    # STANDING FACES, for the record (the seat standing against the open lid is a second one)
    faces = []
    rest_mask = vs & rois & off_floor
    over_bowl = None                                                 # the bowl's outline (its hull), at the fits' resolution
    if best is not None:                                             # not the bowl's own sloping walls
        bowl = np.zeros(d.shape, np.uint8)
        cv2.drawContours(bowl, [best[5]], -1, 1, -1)
        rest_mask &= cv2.dilate(bowl, np.ones((15, 15), np.uint8))[sub] == 0
        hull = np.zeros(d.shape, np.uint8)
        cv2.drawContours(hull, [cv2.convexHull(best[5])], -1, 1, -1)
        over_bowl = hull[sub] > 0

    def lies_over_bowl(inlier_mask) -> bool:
        if over_bowl is None or inlier_mask.sum() == 0:
            return False
        return float((inlier_mask & over_bowl).sum() / inlier_mask.sum()) >= params.lifted_over_bowl
    if lid is not None:
        rest_mask &= np.abs((Ps - lid[1]) @ lid[0]) > 1.2 * tol
        ang = out.lid_floor_deg
        if params.standing_deg[0] <= ang <= params.standing_deg[1]:
            faces.append((lid[0], lid[1]))
    Qr = Ps[rest_mask]
    where = np.flatnonzero(rest_mask.ravel())                        # each remaining point's pixel
    lifted_plane = None
    if lid is not None and params.lifted_deg[0] <= out.lid_floor_deg <= params.lifted_deg[1]:
        lm_full = np.zeros(Ps.shape[:2], dtype=bool); lm_full[vs & rois & off_floor] = lid[2]
        if lies_over_bowl(lm_full):
            lifted_plane = (out.lid_floor_deg, int(lid[2].sum()))
    for _ in range(4):
        f = _ransac_plane(Qr, tol, params.ransac_iters, rng)
        if f is None or f[2].sum() < face_min:
            break
        n, c, inl = f
        ang = _angle_deg(n, fn)
        if params.standing_deg[0] <= ang <= params.standing_deg[1]:
            faces.append((n, c))
        if params.lifted_deg[0] <= ang <= params.lifted_deg[1] and inl.sum() >= lid_min \
                and params.lid_height_m[0] <= float(abs((c - fc) @ fn)) <= params.lid_height_m[1] + 0.3 \
                and (lifted_plane is None or inl.sum() > lifted_plane[1]):
            im = np.zeros(Ps.shape[:2], dtype=bool); im.ravel()[where[inl]] = True
            if lies_over_bowl(im):
                lifted_plane = (ang, int(inl.sum()))                 # a lid-sized plane held at a lifted angle over the bowl
        Qr, where = Qr[~inl], where[~inl]
        if len(Qr) < face_min:
            break
    if lifted_plane is not None:
        out.lifted_plane_deg = float(lifted_plane[0])
    kept = []
    for n, c in faces:
        if not any(_angle_deg(n, n2) < 15.0 and abs(float((c - c2) @ n2)) < 0.015 for n2, c2 in kept):
            kept.append((n, c))
    out.standing_faces = len(kept)
    out.standing_offsets_mm = [int(round(abs(float((c - kept[0][1]) @ kept[0][0])) * 1000)) for _, c in kept[1:]]

    # THE STATE
    thr = float(closed_below_deg)
    if out.cavity_m >= params.cavity_min_m and out.cavity_frac >= params.cavity_min_frac:
        if out.lifted_plane_deg is not None:                         # the bowl shows under a lid held part way (Isaac, 6 Nov:
            ang = float(out.lifted_plane_deg)                        # from pose 1 the bowl shows under a lid at 40-50 deg)
            out.state = "lid_lifted"
            out.confidence = float(min(1.0, max(0.2, abs(ang - thr) / 2.0)))
            out.reason = (f"the lid lifted at {ang:.1f} deg to the floor (from {thr:g}), the bowl showing under it "
                          f"{out.cavity_m * 100:.0f} cm deep over {out.cavity_frac * 100:.0f} %")
        else:
            out.state = "lid_open"
            out.confidence = 1.0 if out.standing_faces else 0.8
            out.reason = (f"the bowl open: a cavity {out.cavity_m * 100:.0f} cm deep over {out.cavity_frac * 100:.0f} % of the image"
                          + (f", {out.standing_faces} standing face(s)" if out.standing_faces else ", no standing face"))
    elif lid is None:
        out.reason = "no plane within the band in the central region"
    elif float(out.lid_floor_deg) > params.standing_no_lid_deg:
        out.reason = f"the central plane stands at {out.lid_floor_deg:.0f} deg to the floor with no bowl in view: a wall, not the lid"
    elif not (params.lid_height_m[0] <= float(out.lid_height_m) <= params.lid_height_m[1]):
        out.reason = f"the central plane lies {out.lid_height_m:.2f} m above the floor: not a lid"
    else:
        ang = float(out.lid_floor_deg)
        out.confidence = float(min(1.0, max(0.2, abs(ang - thr) / 2.0)))   # 2 deg clear of the threshold is sure: the
        #                                                                     plane's angle repeats within ~0.5 deg
        if ang < thr and out.lid_solidity is not None and out.lid_solidity < params.lid_solidity_min:
            out.state, out.confidence = "", 0.0
            out.reason = (f"a flat plane at {ang:.1f} deg with a bite out of it (solidity {out.lid_solidity:.2f}): a ring "
                          "around a bowl cut by the frame's edge, not a lid")
        elif ang < thr:
            out.state = "lid_closed"
            out.reason = f"the lid closed at {ang:.1f} deg to the floor (under {thr:g})" + (
                f", a cavity of {out.cavity_m * 100:.0f} cm over {out.cavity_frac * 100:.1f} % disregarded" if out.cavity_m > 0 else ", no cavity")
        else:
            out.state = "lid_lifted"
            out.reason = f"the lid lifted at {ang:.1f} deg to the floor (from {thr:g})" + (
                f", the bowl showing {out.cavity_m * 100:.0f} cm deep over {out.cavity_frac * 100:.1f} %" if out.cavity_m > 0 else "")
    out.time_ms = (time.monotonic() - t0) * 1000
    return out
