"""The seat's STATE read from one frame: its hinge angle, by render and compare with the fixture model.

The gap check's band reading measures the ring's height in a fixed ROI; with the seat lifted the left
hand fills that ROI, and on 22 Oct 11:58/11:59/12:00 and 23 Oct 15:08 it answered "refused" with the
seat held at about -30 deg — a shallow check that never looked at the seat itself. This one does:
every hypothesis places the fixture's own parts (the region meshes, calibrated, at the session's
registration) with the seat and the lid at candidate hinge angles, renders them into the camera that
took the frame (its pose and intrinsics, the nearest surface per pixel), and the measured depth says
which one it is. No earlier frame, no ROI, no camera pose of its own: any viewpoint that sees the seat.

The hypotheses (angles from closed, every `step_deg`):
    seat d  — the seat at d, the lid standing open      (d >= min_deg: seat_lifted; below: seat_closed)
    both d  — the seat AND the lid at d, the lid on top   (the seat came up with the lid: seat_lifted)
    lid d   — the lid at d, the seat down                 (a lid lowered onto the hinge mimics a lifted
                                                           seat; it says seat_closed)
Each scores, over the pixels where the hypotheses differ, the pixels it EXPLAINS (within tol_m) minus
the pixels it is CONTRADICTED by — the camera seeing past a surface the hypothesis puts there. A
surface measured NEARER than the render is occlusion (the glove, the left arm) and costs nothing; the
pixels within hand_mask_m of a hand frame are left out. The best lifted hypothesis against the best
seat-down one gives the margin; within +/- min_margin there is no answer.

Measured on every recorded seat question (Resource/reports/2026-10-23_seat_check_occlusion): the four
seat_open runs read the seat at 30-35 deg (margin +0.34..+0.40), every closed seat at 0 (-0.52..-1.07),
a fully open seat at 110; with the lid lifted to -30 the seat under it is mostly hidden and the "came
along" question stays inside the margin there (an answer is withheld, never guessed).

Pure numpy plus `region3d.render_hypothesis` (OpenCV), no ROS.
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Optional

import numpy as np


@dataclass(frozen=True)
class SeatFitParams:
    stride: int = 3                  # render and compare at 1/stride resolution
    step_deg: float = 5.0            # candidate hinge angles, from closed to open
    tol_m: float = 0.015             # a pixel within this of a hypothesis's surface is explained by it
    zone_m: float = 0.010            # pixels where the hypotheses differ by more than this decide
    min_zone_px: int = 1000          # fewer deciding pixels in view (at 1/stride) = no answer
    min_margin: float = 0.20         # the best lifted vs the best seat-down hypothesis, relative
    min_deg: float = 10.0            # Newton 2026-10-23: a lift is at least 10 deg
    hand_mask_m: float = 0.14        # depth points this close to a hand frame never count
    lid_min_deg: float = 10.0        # the lid lifted from 10 deg (the seat's rule; the 50 mm bar is ~6.5 deg at its front)
    lid_open_min_deg: float = 80.0   # the lid open (standing) from 80 deg; between the two: lid_lifted
    height_search_m: float = 0.010   # the fixture fit also finds the fixture's height within +/- this of the registration
    height_step_m: float = 0.005     # (2026-10-24: the simulator's commode sat 9 mm below the calibrated model; a closed
                                     # lid then read "open" — the seat's top is only ~10 mm below the closed lid's)
    seat_open_min_deg: float = 0.0   # the seat open (standing) from this angle, lifted between min_deg and it; 0 = no
                                     # open class, a seat lifted from min_deg (2026-10-31: the gap check's parts reading)


DEFAULT_FIT = SeatFitParams()


def _rotation(axis, deg: float) -> np.ndarray:
    a = np.asarray(axis, dtype=float)
    a = a / max(np.linalg.norm(a), 1e-12)
    t = math.radians(deg)
    k = np.array([[0.0, -a[2], a[1]], [a[2], 0.0, -a[0]], [-a[1], a[0], 0.0]])
    return np.eye(3) + math.sin(t) * k + (1.0 - math.cos(t)) * k @ k


@dataclass
class HingedFixture:
    """The fixture's surfaces in its BODY frame: the fixed body and each hinged part (its cells in
    the part's hinge frame, the joint origin and axis, the signed angle that opens it)."""
    body: np.ndarray
    parts: dict                     # name -> (cells (N, 3) hinge-relative, origin (3,), axis (3,), open_deg signed)

    @classmethod
    def from_locator(cls, locator) -> "HingedFixture":
        """From a `region3d.RegionLocator` (its calibrated cells) and its scene.yaml joints."""
        import yaml
        with open(locator.scene_yaml) as handle:
            scene = (yaml.safe_load(handle) or {}).get("fixture") or {}
        parts = {}
        for name in ("lid", "seat"):
            joint = (scene.get(name) or {}).get("joint") or {}
            if name not in locator.fields:
                continue
            parts[name] = (np.asarray(locator.fields[name].centers, dtype=float),
                           np.asarray(joint.get("origin") or [0.0, 0.0, 0.0], dtype=float),
                           np.asarray(joint.get("axis") or [0.0, 1.0, 0.0], dtype=float),
                           float((joint.get("angles_deg") or {}).get("open", -110.0)))
        return cls(body=np.asarray(locator.fields["body"].centers, dtype=float), parts=parts)

    def placed(self, name: str, opened_deg: float) -> np.ndarray:
        """`name`'s cells in the body frame with the part `opened_deg` away from closed (kept: they do not depend on
        the camera, and a new camera pose renders every angle again)."""
        memo = self.__dict__.setdefault("_placed", {})
        key = (name, round(float(opened_deg), 3))
        if key not in memo:
            cells, origin, axis, open_deg = self.parts[name]
            memo[key] = cells @ _rotation(axis, math.copysign(opened_deg, open_deg)).T + origin
        return memo[key]

    def open_deg(self, name: str) -> float:
        return abs(self.parts[name][3])


@dataclass
class SeatFit:
    """What the depth says the seat is. `state` "seat_lifted" | "seat_closed" | "" (no answer)."""
    state: str = ""
    confidence: float = 0.0
    reason: str = ""
    family: str = ""                # the best hypothesis: "seat" | "both" | "lid"
    seat_deg: Optional[float] = None
    lid_deg: Optional[float] = None
    margin: float = 0.0
    lifted_score: int = 0
    down_score: int = 0
    zone_px: int = 0
    hand_px: int = 0
    best_by_family: dict = field(default_factory=dict)
    time_ms: float = 0.0

    def as_dict(self) -> dict:
        return {k: (round(v, 4) if isinstance(v, float) else v) for k, v in self.__dict__.items()}


@dataclass
class PartAnswer:
    """One part's state from the fixture fit. `state` "" = no answer (the reason says why)."""
    state: str = ""
    confidence: float = 0.0
    deg: Optional[float] = None
    margin: float = 0.0
    reason: str = ""
    classes: dict = field(default_factory=dict)     # class -> [best score, its hypothesis "family deg"]

    def as_dict(self) -> dict:
        return {k: (round(v, 4) if isinstance(v, float) else v) for k, v in self.__dict__.items()}


@dataclass
class FixtureFit:
    """The lid's and the seat's state from one frame: the best hypothesis and, per part, the best of each
    class against the best of the others."""
    lid: PartAnswer = field(default_factory=PartAnswer)
    seat: PartAnswer = field(default_factory=PartAnswer)
    family: str = ""
    lid_deg: Optional[float] = None
    seat_deg: Optional[float] = None
    zone_px: int = 0
    hand_px: int = 0
    height_offset_m: float = 0.0     # the fixture's height found by the fit, against the registration given
    seat_hidden: bool = False        # the lid reads lifted: the seat is under it, and its question is not judged
    cached: bool = False
    reason: str = ""
    time_ms: float = 0.0

    def as_dict(self) -> dict:
        out = {k: (round(v, 4) if isinstance(v, float) else v) for k, v in self.__dict__.items() if k not in ("lid", "seat")}
        out.update(lid=self.lid.as_dict(), seat=self.seat.as_dict())
        return out


# The hypotheses' renders depend on the camera, the placement and the model only, not on the frame: the frames
# of one check (3, up to 5, the camera still) and every later check from the same inspect pose (24 Oct: the
# camera came back to within 0.1 mm) render them once. Keyed by the pose to 1 mm / ~0.06 deg, far inside the
# fit's 15 mm tolerance.
_RENDERS: dict = {}
_RENDERS_MAX = 4


def _pose_key(T) -> bytes:
    T = np.asarray(T, dtype=float)
    return np.concatenate([np.round(T[:3, :3], 3).ravel(), np.round(T[:3, 3], 3)]).tobytes()


def _render_key(cam, fixture_pose, registration_m, fx, fy, cx, cy, w, h, fixture, params):
    return (_pose_key(cam), _pose_key(fixture_pose),
            round(float(registration_m), 4), round(fx, 3), round(fy, 3), round(cx, 3), round(cy, 3), w, h, id(fixture),
            int(params.stride), float(params.step_deg), float(params.zone_m))


def _hypotheses(cam, fixture_pose, fixture: HingedFixture, registration_m, fx, fy, cx, cy, w, h, params):
    """-> (hyps {(family, deg): depth image at 1/stride}, the zone where they differ, lid_open, cached).
    Families: ("seat", d) the seat at d, the lid standing open; ("lid", d) the lid at d, the seat down;
    ("both", d) the seat and the lid at d."""
    from .region3d import render_hypothesis
    key = _render_key(cam, fixture_pose, registration_m, fx, fy, cx, cy, w, h, fixture, params)
    hit = _RENDERS.get(key)
    if hit is not None:
        return hit + (True,)
    s = int(params.stride)
    body_from_cam = np.linalg.inv(np.asarray(fixture_pose, dtype=float)) @ np.asarray(cam, dtype=float)
    lift = np.array([0.0, 0.0, float(registration_m)])

    def render(points):
        return render_hypothesis(points + lift, body_from_cam, fx, fy, cx, cy, w, h, s)

    seat_open, lid_open = fixture.open_deg("seat"), fixture.open_deg("lid")
    angles = [float(a) for a in np.arange(0.0, seat_open + 1e-6, params.step_deg)]
    body_r = render(fixture.body)
    seat_r = {a: render(fixture.placed("seat", a)) for a in angles}
    lid_r = {a: render(fixture.placed("lid", a)) for a in angles}
    lid_up = render(fixture.placed("lid", lid_open))
    rows, cols = body_r.shape

    def compose(*imgs):
        z = np.full((rows, cols), np.inf, dtype=np.float32)
        for im in imgs:
            part = im[:rows, :cols]
            z = np.minimum(z, np.where(part > 0, part, np.inf))
        z[~np.isfinite(z)] = 0.0
        return z

    hyps = {}
    for a in angles:
        hyps[("seat", a)] = compose(body_r, seat_r[a], lid_up)
        if a < lid_open:
            hyps[("lid", a)] = compose(body_r, seat_r[0.0], lid_r[a])
        if 0.0 < a < lid_open:
            hyps[("both", a)] = compose(body_r, seat_r[a], lid_r[a])
    base = hyps[("seat", 0.0)]
    differs = np.zeros((rows, cols), dtype=bool)
    for r in hyps.values():
        differs |= (r > 0) & (np.abs(r - base) > params.zone_m)
    dz_per_m = float(np.linalg.inv(body_from_cam)[2, 2])      # a body +z lift of 1 m moves every render by this in depth
    value = (hyps, differs, lid_open, dz_per_m)
    if len(_RENDERS) >= _RENDERS_MAX:
        _RENDERS.pop(next(iter(_RENDERS)))
    _RENDERS[key] = value
    return value + (False,)


def _measure(depth, intrinsics, camera_to_world, hands, params):
    """-> (the measured depth at 1/stride with the hands' pixels zeroed, pixels under the hands, fx, fy, cx, cy, w, h)."""
    get = intrinsics.get if isinstance(intrinsics, dict) else (lambda k: getattr(intrinsics, k))
    fx, fy, cx, cy = (float(get(k)) for k in ("fx", "fy", "cx", "cy"))
    d = np.asarray(depth, dtype=np.float32)
    h, w = d.shape
    s = int(params.stride)
    cam = np.asarray(camera_to_world, dtype=float)
    meas = d[0:h:s, 0:w:s].copy()
    meas[~np.isfinite(meas) | (meas <= 0.05) | (meas >= 3.0)] = 0.0
    hand_px = 0
    if hands:
        vv, uu = np.mgrid[0:h:s, 0:w:s]
        pts = np.stack([(uu - cx) * meas / fx, (vv - cy) * meas / fy, meas], axis=-1) @ cam[:3, :3].T + cam[:3, 3]
        hand = np.zeros(meas.shape, dtype=bool)
        for c in hands:
            hand |= np.linalg.norm(pts - np.asarray(c, dtype=float)[:3], axis=-1) <= params.hand_mask_m
        hand_px = int((hand & (meas > 0)).sum())
        meas[hand] = 0.0
    return meas, hand_px, fx, fy, cx, cy, w, h


def _scores(meas, hyps, differs, params):
    """-> (zone, {(family, deg): pixels explained - pixels the camera sees past}) over the deciding zone."""
    rows = min(differs.shape[0], meas.shape[0]); cols = min(differs.shape[1], meas.shape[1])
    meas = meas[:rows, :cols]
    zone = differs[:rows, :cols] & (meas > 0)
    scores = {}
    for key, r in hyps.items():
        r = r[:rows, :cols]
        ok = zone & (r > 0)
        scores[key] = int((ok & (np.abs(meas - r) < params.tol_m)).sum()) - int((ok & (meas > r + params.tol_m)).sum())
    return zone, scores


def _scores_at_heights(meas, hyps, differs, dz_per_m, offsets, params):
    """-> (zone, {offset: {(family, deg): score}}): the scores with the whole fixture raised by each offset (m), each
    render moved by offset * dz_per_m in depth (the pixels' sideways shift, < 1 px at 1/3 for 10 mm, is left out)."""
    rows = min(differs.shape[0], meas.shape[0]); cols = min(differs.shape[1], meas.shape[1])
    meas = meas[:rows, :cols]
    zone = differs[:rows, :cols] & (meas > 0)
    out = {o: {} for o in offsets}
    tol = params.tol_m
    for key, r in hyps.items():
        r = r[:rows, :cols]
        ok = zone & (r > 0)
        resid = np.sort((meas[ok] - r[ok]).astype(np.float64))     # measured minus rendered, per deciding pixel
        n = resid.size
        for o in offsets:
            shift = o * dz_per_m
            lo, hi = np.searchsorted(resid, [shift - tol, shift + tol])
            past = n - np.searchsorted(resid, shift + tol, side="right")
            out[o][key] = int(hi - lo) - int(past)
    return zone, out


def _strength(margin: float, params) -> float:
    return float(np.clip(0.5 + 0.5 * (abs(margin) - params.min_margin) / params.min_margin, 0.5, 1.0))


def fit_seat_state(depth, intrinsics, camera_to_world, fixture_pose, fixture: HingedFixture,
                   registration_m: float = 0.0, hands=(), params: SeatFitParams = DEFAULT_FIT) -> SeatFit:
    """The seat's state and hinge angle from this depth frame (metres), or no answer with the reason.

    `camera_to_world` and `fixture_pose` (body -> world) are in one world frame; `registration_m`
    raises the whole fixture along its body +z (the session's placement, as the gap check keeps it)."""
    t0 = time.time()
    out = SeatFit()
    if "seat" not in fixture.parts or "lid" not in fixture.parts:
        out.reason = "the fixture model has no hinged seat and lid"
        return out
    meas, out.hand_px, fx, fy, cx, cy, w, h = _measure(depth, intrinsics, camera_to_world, hands, params)
    s = int(params.stride)
    hyps, differs, lid_open, _dz, _cached = _hypotheses(camera_to_world, fixture_pose, fixture, registration_m,
                                                        fx, fy, cx, cy, w, h, params)
    zone, scores = _scores(meas, hyps, differs, params)
    out.zone_px = int(zone.sum())
    if out.zone_px < params.min_zone_px:
        out.reason = (f"only {out.zone_px} depth pixels where the seat's angle shows ({params.min_zone_px} needed at "
                      f"1/{s}; {out.hand_px} under the hands)")
        out.time_ms = (time.time() - t0) * 1000.0
        return out
    for (fam, a), v in scores.items():
        if fam not in out.best_by_family or v > out.best_by_family[fam][1]:
            out.best_by_family[fam] = (a, v)
    lifted = [(v, k) for k, v in scores.items() if k[0] in ("seat", "both") and k[1] >= params.min_deg]
    down = [(v, k) for k, v in scores.items() if k[0] == "lid" or (k[0] == "seat" and k[1] < params.min_deg)]
    up_v, up_k = max(lifted)
    dn_v, dn_k = max(down)
    out.lifted_score, out.down_score = int(up_v), int(dn_v)
    out.margin = float((up_v - dn_v) / max(abs(up_v), abs(dn_v), 1))
    best_v, best_k = max(up_v, dn_v), (up_k if up_v >= dn_v else dn_k)
    out.family = best_k[0]
    out.seat_deg = best_k[1] if best_k[0] in ("seat", "both") else 0.0
    out.lid_deg = (lid_open if best_k[0] == "seat" else best_k[1])
    detail = (f"seat {out.seat_deg:.0f} deg, lid {out.lid_deg:.0f} deg ({out.family}); lifted {up_v} vs down {dn_v} "
              f"over {out.zone_px} pixels at 1/{s}, margin {out.margin:+.2f}")
    strength = _strength(out.margin, params)
    if out.margin >= params.min_margin:
        out.state, out.confidence = "seat_lifted", strength
        out.reason = f"the seat is lifted: {detail}"
    elif out.margin <= -params.min_margin:
        out.state, out.confidence = "seat_closed", strength
        out.reason = f"the seat is down: {detail}"
    else:
        out.reason = f"lifted and down explain the frame alike: {detail}"
    out.time_ms = (time.time() - t0) * 1000.0
    return out


def _angles(key, lid_open):
    """(lid deg, seat deg) a hypothesis places."""
    fam, a = key
    return (lid_open, a) if fam == "seat" else (a, 0.0) if fam == "lid" else (a, a)


def _part_answer(scores, classify, part: str, params) -> PartAnswer:
    """The best hypothesis of each class `classify(key)` gives; the winner against the best of the other classes."""
    best = {}
    for key, v in scores.items():
        c = classify(key)
        if c not in best or v > best[c][0]:
            best[c] = (v, key)
    ranked = sorted(best.items(), key=lambda kv: -kv[1][0])
    classes = {c: [v, f"{k[0]} {k[1]:.0f}"] for c, (v, k) in ranked}
    (win, (wv, wk)) = ranked[0]
    if len(ranked) == 1:
        return PartAnswer(state=win, confidence=0.5, margin=1.0, reason=f"{win}: the only class the model can place", classes=classes)
    (run, (rv, _rk)) = ranked[1]
    margin = float((wv - rv) / max(abs(wv), abs(rv), 1))
    out = PartAnswer(margin=margin, classes=classes)
    detail = f"{win} {wv} vs {run} {rv}, margin {margin:+.2f}"
    if margin >= params.min_margin:
        out.state, out.confidence = win, _strength(margin, params)
        out.reason = detail
    else:
        out.reason = f"{win} and {run} explain the frame alike: {detail}"
    return out


def fit_fixture_state(depth, intrinsics, camera_to_world, fixture_pose, fixture: HingedFixture,
                      registration_m: float = 0.0, hands=(), params: SeatFitParams = DEFAULT_FIT) -> FixtureFit:
    """The lid's AND the seat's state from this depth frame, by the same render and compare as the seat's: every
    hypothesis places both parts, so one fit answers either question from the whole fixture in view, from any
    viewpoint, the hands left out. Lid: closed below `lid_min_deg`, open from `lid_open_min_deg`, lifted between.
    Seat: lifted from `min_deg` (with the lid open, or the seat and the lid together), else down. Each part is
    answered on its own margin; inside it, no answer. The renders are made once per camera pose (a check's frames)."""
    t0 = time.time()
    out = FixtureFit()
    if "seat" not in fixture.parts or "lid" not in fixture.parts:
        out.reason = "the fixture model has no hinged seat and lid"
        return out
    meas, out.hand_px, fx, fy, cx, cy, w, h = _measure(depth, intrinsics, camera_to_world, hands, params)
    s = int(params.stride)
    hyps, differs, lid_open, dz_per_m, out.cached = _hypotheses(camera_to_world, fixture_pose, fixture, registration_m,
                                                                fx, fy, cx, cy, w, h, params)
    n = int(round(params.height_search_m / params.height_step_m)) if params.height_step_m > 0 else 0
    offsets = [k * params.height_step_m for k in range(-n, n + 1)]
    zone, by_offset = _scores_at_heights(meas, hyps, differs, dz_per_m, offsets, params)
    # each hypothesis at its OWN best height: every class competes at its best, so the height found for one never
    # makes another lose (one height for all let a seat lifted under the lid borrow the ring's height, 7 Oct 13:15)
    scores, at = {}, {}
    for key in hyps:
        o = max(offsets, key=lambda o: (by_offset[o][key], -abs(o)))
        scores[key], at[key] = by_offset[o][key], o
    best_o = at[max(scores, key=scores.get)]
    out.height_offset_m = float(best_o)
    out.zone_px = int(zone.sum())
    if out.zone_px < params.min_zone_px:
        out.reason = (f"only {out.zone_px} depth pixels where the lid's or the seat's angle shows ({params.min_zone_px} "
                      f"needed at 1/{s}; {out.hand_px} under the hands)")
        out.lid.reason = out.seat.reason = out.reason
        out.time_ms = (time.time() - t0) * 1000.0
        return out
    best_k = max(scores, key=scores.get)
    out.family = best_k[0]
    out.lid_deg, out.seat_deg = _angles(best_k, lid_open)

    def lid_class(key):
        a = _angles(key, lid_open)[0]
        return "lid_closed" if a < params.lid_min_deg else "lid_open" if a >= params.lid_open_min_deg else "lid_lifted"

    def seat_class(key):
        return "seat_lifted" if key[0] in ("seat", "both") and key[1] >= params.min_deg else "seat_closed"

    # the lid in two steps: closed against not closed decides (ALIX's lift check accepts lifted and open alike, either
    # way round: lift_check.LIFTED_STATES), then the better of lifted and open names it — a lid lifted to 30 deg and a
    # standing lid behind a seat at 35 deg can look alike from the inspect pose (10 Oct 17:31)
    away = _part_answer(scores, lambda k: "lid_closed" if lid_class(k) == "lid_closed" else "lid_away", "lid", params)
    out.lid = away
    if away.state == "lid_away":
        which = _part_answer({k: v for k, v in scores.items() if lid_class(k) != "lid_closed"}, lid_class, "lid", params)
        name = max(which.classes, key=lambda c: which.classes[c][0])
        out.lid = PartAnswer(state=name, confidence=away.confidence, margin=away.margin, classes=dict(away.classes, **which.classes),
                             reason=f"not closed: {away.reason}; " + (f"{name}: {which.reason}" if which.state else
                                                                      f"lifted or open not told apart ({which.reason}), the better named"))
    out.seat = _part_answer(scores, seat_class, "seat", params)
    if params.seat_open_min_deg > 0.0 and out.seat.state == "seat_lifted":
        # the seat as the lid: lifted against down decides, then the better of lifted and open (standing) names it
        def seat_away(key):
            return "seat_open" if key[1] >= params.seat_open_min_deg else "seat_lifted"

        up = out.seat
        which = _part_answer({k: v for k, v in scores.items() if seat_class(k) == "seat_lifted"}, seat_away, "seat", params)
        name = max(which.classes, key=lambda c: which.classes[c][0])
        out.seat = PartAnswer(state=name, confidence=up.confidence, margin=up.margin, classes=dict(up.classes, **which.classes),
                              reason=f"not down: {up.reason}; " + (f"{name}: {which.reason}" if which.state else
                                                                   f"lifted or open not told apart ({which.reason}), the better named"))
    # THE SEAT UNDER A LIFTED LID IS NOT JUDGED (the owner, 2026-10-24): with the lid between closed and open the seat
    # is mostly hidden, and its ring (down) and the bowl's rim (came up with the lid) differ by ~2 cm there, inside the
    # model's own uncertainty: 24 Oct 15:53 the fit leaned "came along" at margin 0.27 with the seat down
    if lid_class(best_k) == "lid_lifted":
        out.seat_hidden = True
        out.seat = PartAnswer(margin=out.seat.margin, classes=out.seat.classes,
                              reason=f"not judged: the seat is under the lifted lid (lid {out.lid_deg:.0f} deg); "
                                     f"it leaned {out.seat.reason}")
    out.lid.deg, out.seat.deg = out.lid_deg, out.seat_deg
    where = (f"best: lid {out.lid_deg:.0f} deg, seat {out.seat_deg:.0f} deg ({out.family}) over {out.zone_px} pixels at 1/{s}, "
             f"the fixture {best_o * 1000:+.0f} mm from its registration")
    out.lid.reason = f"{out.lid.reason}; {where}"
    out.seat.reason = f"{out.seat.reason}; {where}"
    out.reason = where
    out.time_ms = (time.time() - t0) * 1000.0
    return out
