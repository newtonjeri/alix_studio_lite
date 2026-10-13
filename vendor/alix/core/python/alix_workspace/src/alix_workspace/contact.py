"""Contact relaxation: the press normal is a tolerance, not an equality.

The atlas was built on the assumption that a wipe holds the tool exactly normal
to the surface. That is what the generator authors, and it is what made the
straight-elbow shell such a hard wall: with the orientation pinned, the wrist
centre sits at a fixed offset from the contact point, so a waypoint whose wrist
centre falls outside the arm's ball is unreachable and nothing about the path
can change it.

Newton, 2026-08-26: the contact does **not** have to be normal. The surface
tolerates the tool rotating by up to ~30 degrees, and in some places it has to.

**Corrected 2026-08-27 — the tolerance is about the PAD's axes, at the LOCAL
surface, not about the reference frame's X and Z.** The first version of this
module (`TiltPolicy`) rotated the tool about base_link X and Z. Measured on
commode_c `front`, whose normal is 26 degrees off base_link X: a "tilt about X"
there is 0.3 degrees of actual press tilt and 5 degrees of SPIN about the pad
normal (which changes the brush width and nothing about contact), and the real
second contact axis — pitch — was never offered at all. `TiltPolicy` is kept
only so the atlas endpoint can still speak that vocabulary; `PadTiltPolicy` is
the law: roll about the wipe axis, pitch about the lateral axis, applied
``R_tool @ R_local`` about the contact point.

What makes a tilt legal is then a property of the pad AND of the surface under
it, judged twice:

* the contract's A.3 band rule, ``f = t / (D sin theta) >= 0.40`` against the
  node's LOCAL normal (theta is the misalignment, D the pad extent along the
  lifted direction, t the 20 mm compliance), with the convex-curvature term
  ``2 sqrt(2 t R) / D`` where the surface bends away; and
* the geometry itself (`PadContactProbe`): the tilted pad's edge points are
  measured against the real posed fixture skin, and the tilt is illegal if any
  point digs in beyond the compliance — which is what a CONCAVE surface, a
  neighbouring lip or a step under the pad do — and the contact fraction is the
  share of the pad within the compliance band, which is what an overhang or a
  free edge takes away.

On a flat surface the second rule binds first, at ``asin(2 t / D)`` = 30
degrees for the 80 mm pad — Newton's number; the contract's ceiling at the 0.40
gate is 38.7. Both are enforced.

That single relaxation moves the problem, because of where the wrist centre is.
It sits 341.8 mm from the contact point along the tool, so tilting the tool
swings the wrist centre along an arc of that radius.

**How much reach that actually buys, measured rather than asserted.** The arc
length is ``2 * 341.8 * sin(theta/2)`` = 177 mm at 30 degrees, but that is
travel, not radial gain: what matters is how much closer to the SHOULDER the
wrist centre gets, and that depends on the angle between the tool offset and the
shoulder-to-wrist ray. Swept over the 157 samples of ``toplid_right`` at a
+-30 degree allowance, the radial reduction is **50 to 114 mm, median 75 mm**,
and on the 16 samples that were beyond reach it is 58 to 88 mm against deficits
of 0.2 to 20.8 mm — three to four times what is needed.

It is not always that generous. The gain is largest when the press normal is
roughly PERPENDICULAR to the shoulder-to-wrist ray, which is the ordinary case
for a horizontal surface reached at arm's length, and it collapses toward second
order when the tool already points along that ray. A synthetic waypoint pushed
straight out along the ray gains only ~6 mm from the same 30 degrees. So the
policy is a search, not a formula, and a waypoint it cannot rescue is reported
as such rather than assumed away.

Measured on ``toplid_right``, whose five far waypoints the atlas proved to be
12-21 mm beyond the arm's 466 mm ball:

    point      over      tilt to reach        tilt to clear the 20 deg gate
    point11   +19.5 mm   x  +8.0  z -20.5     x +11.5  z -23.5
    point9    +15.8 mm   x  +7.5  z -17.0     x +10.5  z -20.5
    point7    +16.3 mm   x  +9.0  z -14.5     x +12.0  z -17.5
    point5    +12.5 mm   x  +8.0  z  -9.0     x +12.0  z -11.0
    point3    +21.1 mm   x +12.0  z  -9.0     x +15.0  z -11.0

Every one is inside the allowance, with room to spare. A config the reach
analysis condemned is recoverable by a rotation the surface already permits.

**Why this module is pure geometry.** Whether a tilt brings the wrist centre
back inside the ball is decided by one distance, exactly, with no IK and no
collision. So the search over tilts is a grid evaluation costing microseconds
per waypoint, and it is used to NARROW the tilt to a few candidates before the
expensive atlas is built at those candidates. It never asserts that a tilt is
feasible — only that it is not ruled out by reach. The atlas decides the rest.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

logger = logging.getLogger("alix_workspace.contact")

#: Weight on reach margin (mm) when two tilt profiles use the SAME tilt. Small
#: enough that the whole plausible margin range (0-120 mm) cannot outweigh one
#: step of the tilt grid (>= 2.5 deg), so it is strictly a tie-break and never
#: trades tilt for margin. See `TiltReport.smooth_profile`.
MARGIN_TIEBREAK = 0.01


def _rx(rad: float) -> np.ndarray:
    c, s = np.cos(rad), np.sin(rad)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def _rz(rad: float) -> np.ndarray:
    c, s = np.cos(rad), np.sin(rad)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


@dataclass
class TiltPolicy:
    """How far the contact may depart from the surface normal, and about what.

    ``max_x_deg`` / ``max_z_deg`` are limits about the REFERENCE frame's X and Z
    axes — the frame the task config authors its poses in — not about the tool's
    own axes. That matters: a tolerance the surface has is a property of the
    surface and the room, and expressing it in the tool frame would make it
    change meaning as the tool turns.

    ``ref_rotation`` is the reference frame's orientation in the analysis frame.
    For ALIX's ``alix_ref_frame`` it is the identity (the frame is a pure
    translation from ``base_link``, verified), but it is carried explicitly so
    that a fixture or venue whose reference frame is rotated does not silently
    tilt about the wrong axes.
    """

    max_x_deg: float = 0.0
    max_z_deg: float = 0.0
    step_deg: float = 2.5
    ref_rotation: np.ndarray = field(default_factory=lambda: np.eye(3))

    def __post_init__(self):
        self.ref_rotation = np.asarray(self.ref_rotation, dtype=float).reshape(3, 3)
        if self.step_deg <= 0:
            raise ValueError("step_deg must be positive")

    @property
    def active(self) -> bool:
        return self.max_x_deg > 0.0 or self.max_z_deg > 0.0

    def axis_grids(self) -> Tuple[np.ndarray, np.ndarray]:
        """The (x, z) tilt values to consider, always including exactly zero."""
        def one(limit):
            if limit <= 0.0:
                return np.zeros(1)
            n = int(np.floor(limit / self.step_deg))
            g = np.arange(-n, n + 1, dtype=float) * self.step_deg
            # The authored pose must always be a candidate, and it must be the
            # FIRST one, so ties resolve to "do not tilt at all".
            return g[np.argsort(np.abs(g), kind="stable")]
        return one(self.max_x_deg), one(self.max_z_deg)

    def rotation(self, x_deg: float, z_deg: float) -> np.ndarray:
        """The tilt as a rotation in the ANALYSIS frame, applied to the tool.

        Z after X, and both in reference-frame axes; conjugated into the
        analysis frame by ``ref_rotation`` so the pair means the same thing
        whatever the reference frame's own orientation is.
        """
        r_ref = _rz(np.radians(z_deg)) @ _rx(np.radians(x_deg))
        return self.ref_rotation @ r_ref @ self.ref_rotation.T

    def tilted_rotation(self, R0: np.ndarray, x_deg: float, z_deg: float) -> np.ndarray:
        """The tool rotation after this tilt — LEFT-multiplied (world axes)."""
        return self.rotation(x_deg, z_deg) @ np.asarray(R0, dtype=float)

    def apply(self, T: np.ndarray, x_deg: float, z_deg: float) -> np.ndarray:
        """Tilt a tool pose ABOUT ITS CONTACT POINT — position is preserved.

        The contact point is the one thing the wipe cannot give up; the
        orientation is the tolerance. Rotating about the origin instead would
        slide the tool off the surface it is supposed to be touching.
        """
        out = np.array(T, dtype=float, copy=True)
        out[:3, :3] = self.tilted_rotation(out[:3, :3], x_deg, z_deg)
        return out

    @property
    def axis_names(self) -> Tuple[str, str]:
        return ("ref X", "ref Z")


# ---------------------------------------------------------------------------
# The pad-local law
# ---------------------------------------------------------------------------

def _axis_index(letter: str) -> int:
    return {"x": 0, "y": 1, "z": 2}[str(letter).lower().lstrip("+-")]


@dataclass
class PadTiltPolicy:
    """Tilt about the PAD's own in-plane axes, through the contact point.

    ``press_axis`` names the tool axis that presses into the surface (signed,
    e.g. ``"+x"`` / ``"-x"`` / ``"z"``, as glove_frames.yaml declares it) and
    ``wipe_axis`` the tool axis the wipe travels along. The third axis is the
    lateral (row-step) direction. Then:

    * **roll** is a rotation about the WIPE axis — it lifts one lateral edge of
      the pad and presses the other;
    * **pitch** is a rotation about the LATERAL axis — it lifts the leading or
      trailing edge along the wipe.

    Both are expressed in the tool's frame and applied ``R_tool @ R_local``,
    so the same (roll, pitch) means the same physical thing at every node of
    every surface, which the reference-frame version could not claim. A spin
    about the press axis is deliberately NOT a parameter here: it does not
    change contact (it changes the brush width), and it is the atlas's own
    ``gammaDeg``.

    ``max_roll_deg`` / ``max_pitch_deg`` are POLICY limits; the contract's own
    ceiling is enforced separately by `PadContactProbe`, and a policy above it
    simply offers tilts the probe then refuses.
    """

    press_axis: str = "z"
    wipe_axis: str = "x"
    max_roll_deg: float = 30.0
    max_pitch_deg: float = 30.0
    step_deg: float = 5.0

    def __post_init__(self):
        if self.step_deg <= 0:
            raise ValueError("step_deg must be positive")
        p, w = _axis_index(self.press_axis), _axis_index(self.wipe_axis)
        if p == w:
            raise ValueError("press and wipe axes must differ")
        self._press_idx, self._wipe_idx = p, w
        self._lateral_idx = ({0, 1, 2} - {p, w}).pop()

    @property
    def active(self) -> bool:
        return self.max_roll_deg > 0.0 or self.max_pitch_deg > 0.0

    @property
    def axis_names(self) -> Tuple[str, str]:
        return ("roll about wipe", "pitch about lateral")

    def axis_grids(self) -> Tuple[np.ndarray, np.ndarray]:
        """(roll, pitch) candidates, zero first, ordered by magnitude."""
        def one(limit):
            if limit <= 0.0:
                return np.zeros(1)
            n = int(np.floor(limit / self.step_deg + 1e-9))
            g = np.arange(-n, n + 1, dtype=float) * self.step_deg
            return g[np.argsort(np.abs(g), kind="stable")]
        return one(self.max_roll_deg), one(self.max_pitch_deg)

    def rotation(self, roll_deg: float, pitch_deg: float) -> np.ndarray:
        """The tilt as a rotation in the TOOL frame (pitch after roll)."""
        from scipy.spatial.transform import Rotation
        e_w = np.zeros(3); e_w[self._wipe_idx] = 1.0
        e_l = np.zeros(3); e_l[self._lateral_idx] = 1.0
        R_roll = Rotation.from_rotvec(e_w * np.radians(roll_deg)).as_matrix()
        R_pitch = Rotation.from_rotvec(e_l * np.radians(pitch_deg)).as_matrix()
        return R_roll @ R_pitch

    def tilted_rotation(self, R0: np.ndarray, roll_deg: float, pitch_deg: float) -> np.ndarray:
        """The tool rotation after this tilt — RIGHT-multiplied (pad axes)."""
        return np.asarray(R0, dtype=float) @ self.rotation(roll_deg, pitch_deg)

    def apply(self, T: np.ndarray, roll_deg: float, pitch_deg: float) -> np.ndarray:
        """Tilt a tool pose about its contact point — position preserved."""
        out = np.array(T, dtype=float, copy=True)
        out[:3, :3] = self.tilted_rotation(out[:3, :3], roll_deg, pitch_deg)
        return out

    # -- pad axes of a given tool rotation ------------------------------------
    def press_direction(self, R: np.ndarray) -> np.ndarray:
        """Unit vector the pad presses ALONG (into the surface), world frame."""
        sign = -1.0 if str(self.press_axis).strip().startswith("-") else 1.0
        return np.asarray(R, dtype=float)[:, self._press_idx] * sign

    def wipe_direction(self, R: np.ndarray) -> np.ndarray:
        return np.asarray(R, dtype=float)[:, self._wipe_idx]

    def lateral_direction(self, R: np.ndarray) -> np.ndarray:
        return np.asarray(R, dtype=float)[:, self._lateral_idx]

    def misalignment_deg(self, R: np.ndarray, surface_normal: np.ndarray) -> float:
        """Angle between the press direction and the INWARD surface normal."""
        n = np.asarray(surface_normal, dtype=float)
        n = n / (np.linalg.norm(n) + 1e-12)
        c = float(np.dot(self.press_direction(R), -n))
        return float(np.degrees(np.arccos(np.clip(c, -1.0, 1.0))))


# ---------------------------------------------------------------------------
# The contract's band rule, kept as arithmetic with no frame assumptions
# ---------------------------------------------------------------------------

def contract_contact_fraction(theta_deg: float, pad_extent_m: float, thickness_m: float,
                              convex_radius_m: Optional[float] = None) -> float:
    """A.3: ``f = min(1, t / (D sin theta))``, min'd with the curvature term.

    The pad conforms up to its thickness ``t``, so the band still in contact
    along the lifted direction of extent ``D`` is ``t / sin theta``. On a
    CONVEX surface of radius ``R`` the flat pad's contact length is
    ``2 sqrt(2 t R)`` whatever the tilt, which binds below R ~ 40 mm. A
    concave surface only helps this fraction — its harm is dig-in, which
    `PadContactProbe` measures directly.
    """
    theta = abs(np.radians(float(theta_deg)))
    s = float(np.sin(theta))
    D = float(pad_extent_m)
    # A ZERO LIFTED EXTENT IS "NOTHING LIFTS", NOT A DIVISION.  `D` is the pad
    # projected onto the direction the press departs from the normal
    # (`PadContactProbe.evaluate`), and it goes to zero when that direction
    # falls on the pad's own press axis — an alignment only reachable at large
    # tilt, so while the policy held every direction to 30 deg nothing ever
    # divided by it.  MEASURED 2026-09-19: letting PITCH take the contract's
    # own 90 deg ceiling on commode_c bowl_inside_front reached it on the first
    # node and raised ZeroDivisionError out of the middle of the map build.
    # With no extent to lift there is no band to lose, so the fraction is 1.
    if D <= 1e-9:
        return 1.0
    f = 1.0 if s <= 1e-9 else min(1.0, float(thickness_m) / (D * s))
    if convex_radius_m is not None and convex_radius_m > 0:
        f = min(f, min(1.0, 2.0 * np.sqrt(2.0 * float(thickness_m) * float(convex_radius_m)) / D))
    return f


def contract_max_tilt_deg(pad_extent_m: float, thickness_m: float, gate: float) -> float:
    """The A.3 ceiling: ``asin(t / (D g))``.

    A zero extent or gate is NO LEVER, so the band rule never binds and the
    ceiling is 90 deg — the same limit the formula already returns once
    ``t / (D g)`` clips at 1, and the value it must give rather than dividing
    by zero (see `contract_contact_fraction`).
    """
    D, g = float(pad_extent_m), float(gate)
    if D <= 1e-9 or g <= 1e-9:
        return 90.0
    return float(np.degrees(np.arcsin(min(1.0, float(thickness_m) / (D * g)))))


# ---------------------------------------------------------------------------
# The surface skin — signed distance to an open surface, fast
# ---------------------------------------------------------------------------

class SurfaceSkin:
    """Signed distance to a triangle mesh's SKIN, along the local face normal.

    Why not `trimesh.proximity.signed_distance`: the commode parts are thin
    watertight SHELLS (body.stl: 40 184 faces, 2 litres of material in a
    600 x 370 x 400 mm box), so a pad corner 15 mm "into" the surface is
    through the wall and out the other side, where a volume-based sign calls it
    outside again. The quantity a contact law needs is height above the local
    skin — positive outside, negative dug in — and that is closest point on the
    surface plus the sign of the face normal there.

    Why not `proximity.closest_point`: 6 ms per point measured. A KD-tree over
    triangle centroids with an exact closest-point evaluation on the K nearest
    triangles is ~50 us per point and exact whenever the true closest triangle
    is among the K nearest by centroid — always the case within a few
    centimetres of a surface whose triangles are ~9 mm.
    """

    def __init__(self, vertices: np.ndarray, faces: np.ndarray, k: int = 24):
        from scipy.spatial import cKDTree
        V = np.asarray(vertices, dtype=float)
        F = np.asarray(faces, dtype=int)
        self.tri = V[F]                                     # (M, 3, 3)
        e1 = self.tri[:, 1] - self.tri[:, 0]
        e2 = self.tri[:, 2] - self.tri[:, 0]
        n = np.cross(e1, e2)
        norm = np.linalg.norm(n, axis=1)
        keep = norm > 1e-14
        self.tri = self.tri[keep]
        self.normals = n[keep] / norm[keep, None]
        self.centroids = self.tri.mean(axis=1)
        self.tree = cKDTree(self.centroids)
        self.k = int(k)

    @classmethod
    def from_meshes(cls, meshes: Sequence, k: int = 24) -> "SurfaceSkin":
        V, F, off = [], [], 0
        for m in meshes:
            v = np.asarray(m.vertices, dtype=float)
            f = np.asarray(m.faces, dtype=int)
            V.append(v)
            F.append(f + off)
            off += len(v)
        return cls(np.concatenate(V), np.concatenate(F), k=k)

    def signed_height(self, points: np.ndarray,
                      normal_hint: Optional[np.ndarray] = None,
                      min_agreement: float = 0.2) -> Tuple[np.ndarray, np.ndarray]:
        """(height, normal) per point: height > 0 outside the skin, < 0 dug in.

        ``normal_hint`` is the OUTWARD direction of the surface being probed.
        On a thin shell a point pushed through the wall is nearer the INNER
        skin than the outer one, and the inner skin's normal points the other
        way — so without the hint a 23 mm dig-in read as 19 mm (the wall's
        4 mm, measured on a synthetic plate) with a flipped sign. With it,
        candidate triangles whose normal disagrees with the hint are dropped
        before the closest one is chosen, so the height is always measured to
        the skin the pad is actually pressing. Falls back to every candidate
        where none agrees (a point far from any facing surface).

        Vectorised: one KD-tree query and one batched point-triangle distance
        for all points, ~30 us per point.
        """
        from trimesh.triangles import closest_point as _closest
        P = np.atleast_2d(np.asarray(points, dtype=float))
        N = len(P)
        k = int(min(self.k, len(self.centroids)))
        _d, idx = self.tree.query(P, k=k)
        idx = np.atleast_2d(idx).reshape(N, -1)
        cand = self.tri[idx]                                          # (N, k, 3, 3)
        cp = _closest(cand.reshape(-1, 3, 3),
                      np.repeat(P, idx.shape[1], axis=0)).reshape(N, -1, 3)
        d = np.linalg.norm(cp - P[:, None, :], axis=2)               # (N, k)
        if normal_hint is not None:
            hint = np.asarray(normal_hint, dtype=float)
            hint = hint / (np.linalg.norm(hint) + 1e-12)
            agree = (self.normals[idx] @ hint) >= float(min_agreement)  # (N, k)
            any_agree = agree.any(axis=1)
            d = np.where(agree | ~any_agree[:, None], d, np.inf)
        best = np.argmin(d, axis=1)
        rows = np.arange(N)
        nearest = cp[rows, best]
        nrm = self.normals[idx[rows, best]]
        sign = np.where(np.einsum("ij,ij->i", P - nearest, nrm) >= 0.0, 1.0, -1.0)
        return sign * d[rows, best], nrm


# ---------------------------------------------------------------------------
# The probe: is THIS tool pose a legal contact on THIS surface?
# ---------------------------------------------------------------------------

@dataclass
class PadGeometry:
    """The pad as the contact law sees it. Extents are along the WIPE and the
    LATERAL directions of `PadTiltPolicy`; ``thickness_m`` is the compliance
    depth ``t``.

    These are TOOL-AXIS extents, not motion: ``along_wipe_m`` is the pad's
    material extent along the axis the policy calls the wipe axis, and
    ``lateral_m`` its extent along the remaining in-plane axis. Both
    `sample_points_tool` and the lifted extent in `PadContactProbe.evaluate`
    read them that way.

    Which physical edge lies where WAS open (the docstring here used to tell
    callers to pass the larger extent for both, squaring the pad) and is now
    measured: the pad's long edge L lies on tool Y on every glove frame — see
    `PAD_LONG_AXIS_INDEX`. Build one with `pad_geometry_for` rather than by
    hand, so the mapping is made in a single place; never pass the same extent
    twice to be safe, because a square is not conservative in the direction
    that lifts the SHORT edge — it under-reports the pad's real tolerance
    there by a factor of L/W (2.3x on the palm, 4x on the finger pads).
    """

    along_wipe_m: float = 0.08
    lateral_m: float = 0.08
    thickness_m: float = 0.02

    def sample_points_tool(self, n_along: int = 5, n_lateral: int = 3,
                           wipe_idx: int = 0, lateral_idx: int = 1) -> np.ndarray:
        """Points on the pad face in the TOOL frame, edges included."""
        a = np.linspace(-0.5, 0.5, n_along) * self.along_wipe_m
        b = np.linspace(-0.5, 0.5, n_lateral) * self.lateral_m
        pts = np.zeros((n_along * n_lateral, 3))
        k = 0
        for x in a:
            for y in b:
                pts[k, wipe_idx] = x
                pts[k, lateral_idx] = y
                k += 1
        return pts


#: Which in-plane TOOL axis carries the pad's long edge L: tool Y, on every
#: glove frame and both hands.  Two independent sources agree — the Studio's
#: attachment law (web_editor/editor.js builds the pad box with L on Y for the
#: x-pressing and the z-pressing frame alike) and the measured box fit
#: (``tools/glove_pad_audit.py --free-orientation`` scores the L-on-Y placement
#: 0.559/0.571 against 0.388/0.388 for the 90-deg-rotated box on glove_frame_1,
#: and 0.728/0.736 against 0.489/0.484 on glove_frame_2; the two boxes have
#: equal area, so the score difference is backing material and nothing else).
PAD_LONG_AXIS_INDEX = 1


def wipe_axis_letter_for(press_axis: str) -> str:
    """Which TOOL axis the wipe travels along, given the pad's press axis.

    THE one home for a rule that had three shipped copies —
    `alix_taskgen.path_emission.wipe_axis_letter`, `server._tilt_policy` and
    `coverage._FOOTPRINT_AXES` — which agreed, and agreeing by coincidence
    three times is how the fourth copy disagrees.

    It is the in-plane axis that is NOT tool Y, so it carries the pad's SHORT
    edge (`PAD_LONG_AXIS_INDEX`): at zero wipe yaw the stroke travels along the
    pad's short edge and lays its long edge down as the swath. A y-press frame
    does not exist in glove_frames.yaml; it maps to x so the rule is total.
    """
    letter = str(press_axis).strip().lstrip("+-").lower() or "z"
    return {"z": "x", "x": "z", "y": "x"}.get(letter, "x")


def pad_axes_extents(pad: "PadGeometry", wipe_yaw_deg: float = 0.0):
    """``(along_travel_m, across_travel_m)`` for a pad spun by *wipe_yaw_deg*.

    The wipe yaw is the pad's spin about its own press axis — the free
    parameter of the contact pose (`path_emission.knot_poses_from` takes it,
    the start pose fixes it). At yaw 0 the tool's wipe axis lies on the travel
    direction, so the pad presents ``along_wipe_m`` along the travel and
    ``lateral_m`` across it; spun, each edge contributes to both, and the
    across-travel figure is the rectangle's SUPPORT WIDTH — which is what a
    swept band actually is:

        across = along_wipe . |sin psi| + lateral . |cos psi|
        along  = along_wipe . |cos psi| + lateral . |sin psi|

    So the swath is never asserted from a name (`footprint_m[0]` vs ``[1]``,
    "the edge along the wipe"); it is read off the pose the emitter builds.
    """
    import math as _math

    psi = _math.radians(float(wipe_yaw_deg))
    c, s = abs(_math.cos(psi)), abs(_math.sin(psi))
    a, b = float(pad.along_wipe_m), float(pad.lateral_m)
    return (a * c + b * s, a * s + b * c)


def pad_swath_m(pad: "PadGeometry", wipe_yaw_deg: float = 0.0) -> float:
    """The pad's swept width ACROSS the travel — what one stroke lays down,
    and therefore the only thing that sets a row pitch."""
    return float(pad_axes_extents(pad, wipe_yaw_deg)[1])


def pad_geometry_for(policy: "PadTiltPolicy", footprint_m, thickness_m: float) -> "PadGeometry":
    """The TRUE pad rectangle, laid on *policy*'s own axes.

    *footprint_m* is glove_frames.yaml's ``[W, L]`` — W the short edge, L the
    long one. `PAD_LONG_AXIS_INDEX` says L lies on tool Y, so whichever of the
    policy's wipe/lateral axes IS tool Y takes L and the other takes W. With
    the shipped press/wipe convention (wipe = the in-plane axis that is not Y)
    that puts W along the wipe and L across it, but nothing here assumes that:
    change either rule and this stays correct.
    """
    W, L = (float(footprint_m[0]), float(footprint_m[1]))
    along = L if policy._wipe_idx == PAD_LONG_AXIS_INDEX else W
    lateral = L if policy._lateral_idx == PAD_LONG_AXIS_INDEX else W
    return PadGeometry(along_wipe_m=along, lateral_m=lateral,
                       thickness_m=float(thickness_m))


def pad_geometry_for_axis(press_axis: str, footprint_m, thickness_m: float) -> "PadGeometry":
    """`pad_geometry_for` without a policy object — for callers that have a
    glove frame's press axis and nothing else (coverage helpers, the spacing
    rules). Same mapping, same single source for the wipe-axis rule."""
    return pad_geometry_for(
        PadTiltPolicy(press_axis=str(press_axis),
                      wipe_axis=wipe_axis_letter_for(press_axis),
                      max_roll_deg=0.0, max_pitch_deg=0.0),
        footprint_m, thickness_m)


@dataclass
class ContactVerdict:
    legal: bool
    reason: str
    misalignment_deg: float
    contract_fraction: float
    dig_in_mm: float            #: deepest pad point below the skin
    lift_mm: float              #: highest pad point above the skin
    support_fraction: float     #: share of pad points within +-t of the skin
    convex_radius_m: Optional[float] = None
    #: how far the pad had to RIDE OUT along the local normal before it was
    #: judged (0 unless the caller asked for it -- see `evaluate`). The caller
    #: must command the pose translated by this much, or the verdict is not
    #: about the pose it will send.
    standoff_m: float = 0.0

    def to_dict(self) -> dict:
        return {"legal": bool(self.legal), "reason": self.reason,
                "misalignmentDeg": round(self.misalignment_deg, 2),
                "contractFraction": round(self.contract_fraction, 3),
                "digInMm": round(self.dig_in_mm, 2), "liftMm": round(self.lift_mm, 2),
                "supportFraction": round(self.support_fraction, 3),
                "convexRadiusM": (None if self.convex_radius_m is None
                                  else round(self.convex_radius_m, 4)),
                "standoffM": round(self.standoff_m, 5)}


class PadContactProbe:
    """Judge a tool pose against the real surface under the pad.

    Two rules, both required (see the module docstring):

    1. the contract band rule at the contract gate, against the local normal;
    2. the geometry: no pad point dug in beyond ``t``, and at least ``gate`` of
       the pad within ``+-t`` of the skin.

    ``skin`` is the posed fixture surface (`SurfaceSkin`). ``local_normal`` may
    be supplied by the caller (the lattice's smoothed normal); otherwise the
    skin's normal under the pad centre is used.
    """

    def __init__(self, skin: SurfaceSkin, policy: PadTiltPolicy, pad: PadGeometry,
                 gate: float = 0.40, n_along: int = 5, n_lateral: int = 3):
        self.skin = skin
        self.policy = policy
        self.pad = pad
        self.gate = float(gate)
        self._pts_tool = pad.sample_points_tool(
            n_along, n_lateral, wipe_idx=policy._wipe_idx,
            lateral_idx=policy._lateral_idx)
        #: The worst-case A.3 ceiling — the LONG lever, the direction that
        #: binds first. Kept as the advertised scalar (the ladder clamps to it,
        #: figures plot it); the refusal in `evaluate` uses the extent that
        #: actually lifts at the pose being judged, which on a real rectangle
        #: is smaller than this in every direction but one.
        self.ceiling_deg = contract_max_tilt_deg(
            max(pad.along_wipe_m, pad.lateral_m), pad.thickness_m, self.gate)
        #: Per-direction ceilings. A ROLL about the wipe axis lifts the pad's
        #: LATERAL edge, so its lever is `lateral_m`; a PITCH about the lateral
        #: axis lifts the along-wipe edge, lever `along_wipe_m`. A single
        #: scalar cannot be both, which is what squaring the pad hid.
        self.roll_ceiling_deg = contract_max_tilt_deg(
            pad.lateral_m, pad.thickness_m, self.gate)
        self.pitch_ceiling_deg = contract_max_tilt_deg(
            pad.along_wipe_m, pad.thickness_m, self.gate)

    #: A dig-in is judged ONE step past the boundary so a surface that is not
    #: flat under the pad cannot leave the corrected pose sitting exactly on
    #: the threshold and failing by a rounding.
    STANDOFF_EPS_M = 0.0005

    def evaluate(self, T: np.ndarray, local_normal: Optional[np.ndarray] = None,
                 convex_radius_m: Optional[float] = None,
                 allow_standoff: bool = False) -> ContactVerdict:
        """Judge a tool pose. *allow_standoff* lets the pad RIDE OUT first.

        The pad face is sampled about the TOOL ORIGIN, so rotating it by theta
        buries its far edge by ``(D/2) sin theta`` whatever the surface under
        it does: on a flat plane the dig-in test therefore refuses every tilt
        past ``asin(2t/D)`` = 30 deg for the 80 mm lever, which is the same 30
        deg the policy cap spells out, charged twice. That is an artefact of
        WHERE THE PIVOT IS, not a property of the pad -- a real pad pressed at
        an angle rides on its deepest contact and lifts the rest away.

        Measured against the hardware-proven archive for `bowl_inside_front`
        (21 poses, misalignment 7 to 54 deg): the rigid test refuses 3 of them
        at 23.6 / 25.8 / 31.8 mm of dig-in, and the deepest is exactly
        ``40 sin 53.9 deg`` = 32.3 mm -- the artefact, to a tenth of a
        millimetre. Riding the pad out by ``dig - t`` clears all three with
        support 0.80 to 0.87 and contract fraction 0.53 to 0.96, both well
        over the 0.40 gate.

        With *allow_standoff* the pose is re-judged once, translated along the
        OUTWARD local normal by ``dig - t``, and the distance is reported as
        `ContactVerdict.standoff_m`. It is the CALLER'S JOB to command that
        translated pose: the verdict describes where the pad ends up, not
        where it was asked to go. Default False, so every existing caller
        keeps the rigid rule.
        """
        T = np.asarray(T, dtype=float)
        R, p = T[:3, :3], T[:3, 3]
        pts = p[None, :] + (R @ self._pts_tool.T).T
        if local_normal is None:
            # No caller normal: take the skin's own under the pad centre, but
            # oriented AGAINST the press so a shell's inner skin cannot be it.
            hc, nc = self.skin.signed_height(p[None, :],
                                             normal_hint=-self.policy.press_direction(R))
            local_normal = nc[0]
        n = np.asarray(local_normal, dtype=float)
        n = n / (np.linalg.norm(n) + 1e-12)
        h, n_face = self.skin.signed_height(pts, normal_hint=n)

        theta = self.policy.misalignment_deg(R, n)
        t = self.pad.thickness_m
        dig = float(max(0.0, -h.min()))
        standoff = 0.0
        if allow_standoff and dig > t:
            # ride the pad out to its deepest contact and judge it THERE. One
            # extra skin query, and only on a pose the rigid rule would refuse.
            standoff = dig - t + self.STANDOFF_EPS_M
            p = p + n * standoff
            pts = p[None, :] + (R @ self._pts_tool.T).T
            h, n_face = self.skin.signed_height(pts, normal_hint=n)
            dig = float(max(0.0, -h.min()))
        lift = float(max(0.0, h.max()))
        support = float(np.mean(np.abs(h) <= t + 1e-9))
        # D along the lifted direction: the pad extent projected onto the
        # direction in which the press departs from the normal.
        press = self.policy.press_direction(R)
        lift_dir = press - float(np.dot(press, -n)) * (-n)
        m = float(np.linalg.norm(lift_dir))
        if m > 1e-9:
            lift_dir /= m
            a = abs(float(np.dot(lift_dir, self.policy.wipe_direction(R))))
            b = abs(float(np.dot(lift_dir, self.policy.lateral_direction(R))))
            D = a * self.pad.along_wipe_m + b * self.pad.lateral_m
        else:
            D = max(self.pad.along_wipe_m, self.pad.lateral_m)
        f = contract_contact_fraction(theta, D, t, convex_radius_m)

        # The ceiling is read at the SAME extent the fraction is: the pad's
        # projection onto the direction that lifts. Judging it at
        # `self.ceiling_deg` (the long lever) refuses tilts about the pad's
        # long axis that the band rule allows -- the squared-pad defect,
        # which survived the constructor being handed a true rectangle.
        ceiling = contract_max_tilt_deg(D, t, self.gate)
        if theta > ceiling + 1e-6:
            return ContactVerdict(False, "contract-ceiling", theta, f, dig * 1000, lift * 1000,
                                  support, convex_radius_m, standoff)
        if f < self.gate:
            return ContactVerdict(False, "contract-fraction", theta, f, dig * 1000, lift * 1000,
                                  support, convex_radius_m, standoff)
        if dig > t + 1e-9:
            return ContactVerdict(False, "dig-in", theta, f, dig * 1000, lift * 1000,
                                  support, convex_radius_m, standoff)
        if support < self.gate:
            return ContactVerdict(False, "support", theta, f, dig * 1000, lift * 1000,
                                  support, convex_radius_m, standoff)
        return ContactVerdict(True, "", theta, f, dig * 1000, lift * 1000, support,
                              convex_radius_m, standoff)


@dataclass
class TiltReport:
    """What tilt each waypoint needs, and what tilt the whole stroke can share."""

    x_grid: np.ndarray                 #: (NX,) candidate tilts about ref X, deg
    z_grid: np.ndarray                 #: (NZ,) candidate tilts about ref Z, deg
    wrist_distance_mm: np.ndarray      #: (S, NX, NZ) — the whole search, kept
    reach_ok: np.ndarray               #: (S, NX, NZ) bool — inside the arm's ball
    gate_ok: np.ndarray                #: (S, NX, NZ) bool — clear of the straight elbow
    max_reach_mm: float
    gate_reach_mm: float
    gate_deg: float
    names: List[str] = field(default_factory=list)
    #: what the two grid axes ARE — ("roll about wipe", "pitch about lateral")
    #: for the pad-local policy, ("refX", "refZ") for the legacy one
    axis_names: Tuple[str, str] = ("refX", "refZ")

    # ------------------------------------------------------------------
    def per_waypoint_minimum(self, gate: bool = False) -> List[Optional[Tuple[float, float]]]:
        """Smallest tilt that rescues each waypoint on its own, or None."""
        ok = self.gate_ok if gate else self.reach_ok
        cost = np.abs(self.x_grid)[:, None] + np.abs(self.z_grid)[None, :]
        out = []
        for s in range(ok.shape[0]):
            m = ok[s]
            if not m.any():
                out.append(None)
                continue
            c = np.where(m, cost, np.inf)
            i, j = np.unravel_index(int(np.argmin(c)), c.shape)
            out.append((float(self.x_grid[i]), float(self.z_grid[j])))
        return out

    def uniform(self, gate: bool = False) -> Optional[Tuple[float, float]]:
        """The smallest single tilt that rescues EVERY waypoint, or None.

        A uniform tilt is worth finding first because it is the cheapest thing
        to author: one number on the region, not a per-point field. It exists
        only if the feasible tilt sets intersect across the whole stroke.
        """
        ok = self.gate_ok if gate else self.reach_ok
        both = ok.all(axis=0)
        if not both.any():
            return None
        cost = np.abs(self.x_grid)[:, None] + np.abs(self.z_grid)[None, :]
        c = np.where(both, cost, np.inf)
        i, j = np.unravel_index(int(np.argmin(c)), c.shape)
        return float(self.x_grid[i]), float(self.z_grid[j])

    def blocking_waypoints(self, gate: bool = False) -> List[int]:
        """Waypoints no tilt in the policy can rescue — the real hard cases."""
        ok = self.gate_ok if gate else self.reach_ok
        return [s for s in range(ok.shape[0]) if not ok[s].any()]

    def smooth_profile(self, max_step_deg: float = 5.0, gate: bool = False
                       ) -> Optional[np.ndarray]:
        """A per-waypoint tilt that varies smoothly along the stroke.

        Used when no uniform tilt exists. The same bottleneck argument as the
        swivel corridor: pick the path through the (x, z) grid whose worst
        waypoint uses the LEAST tilt, subject to the tilt moving no more than
        ``max_step_deg`` between consecutive waypoints — a surface tolerance
        that jumped 20 degrees between neighbouring contacts would not be a
        tolerance, it would be a discontinuity in the wipe.

        Returns all-zeros when the authored normal already works everywhere, and
        ``None`` when no continuous tilt profile makes every waypoint reachable.
        """
        ok = self.gate_ok if gate else self.reach_ok
        S, NX, NZ = ok.shape
        limit = self.gate_reach_mm if gate else self.max_reach_mm

        # The objective is to use the SMALLEST tilt that works — not the tilt
        # that maximises reach margin.
        #
        # That distinction was got wrong first and it matters. Swinging the
        # 341.8 mm tool offset back toward the shoulder always brings the wrist
        # centre closer, so a margin-maximising search tilts as hard as the
        # policy allows, and on a stroke that needed no tilt at all it returned
        # 15 to 23 degrees off the authored press normal. Pricing the tilt
        # against margin only moves the threshold: if tilting buys 75 mm of
        # bottleneck margin, any price small enough not to veto a NEEDED tilt is
        # too small to stop an unneeded one.
        #
        # So the bottleneck quantity is the tilt magnitude itself, negated (the
        # DP maximises a minimum, so this minimises the largest tilt used along
        # the path), with reach margin as a strict tie-break: MARGIN_TIEBREAK is
        # small enough that the whole 0-120 mm margin range cannot outweigh one
        # step of the tilt grid, so it only ever separates paths that use the
        # same tilt.
        cost = (np.abs(self.x_grid)[:, None] + np.abs(self.z_grid)[None, :])
        score = np.where(ok,
                         -cost[None, :, :]
                         + MARGIN_TIEBREAK * (limit - self.wrist_distance_mm),
                         -np.inf)
        if not np.isfinite(score[0]).any():
            return None
        best = score[0].reshape(-1).copy()
        flat_x = np.repeat(self.x_grid, NZ)
        flat_z = np.tile(self.z_grid, NX)
        reach_ok_step = (np.abs(flat_x[:, None] - flat_x[None, :]) <= max_step_deg + 1e-9) & \
                        (np.abs(flat_z[:, None] - flat_z[None, :]) <= max_step_deg + 1e-9)
        parent = np.full((S, NX * NZ), -1, dtype=np.int32)
        for s in range(1, S):
            cand = np.where(reach_ok_step, best[:, None], -np.inf)
            src = np.argmax(cand, axis=0)
            top = cand[src, np.arange(NX * NZ)]
            best = np.minimum(top, score[s].reshape(-1))
            parent[s] = src
            if not np.isfinite(best).any():
                return None
        end = int(np.argmax(best))
        if not np.isfinite(best[end]):
            return None
        path = np.zeros((S, 2))
        cur = end
        for s in range(S - 1, -1, -1):
            path[s] = (flat_x[cur], flat_z[cur])
            if s:
                cur = int(parent[s, cur])
        return path

    def to_dict(self) -> dict:
        uni = self.uniform()
        uni_gate = self.uniform(gate=True)
        per = self.per_waypoint_minimum()
        need = [s for s, p in enumerate(per) if p != (0.0, 0.0) and p is not None]
        return {
            "xGridDeg": [round(float(v), 2) for v in self.x_grid],
            "zGridDeg": [round(float(v), 2) for v in self.z_grid],
            "maxReachMm": round(self.max_reach_mm, 2),
            "gateReachMm": round(self.gate_reach_mm, 2),
            "gateDeg": self.gate_deg,
            "uniformForReach": uni,
            "uniformForGate": uni_gate,
            "waypointsNeedingTilt": need,
            "waypointsNoTiltCanFix": self.blocking_waypoints(),
            "perWaypointMinimum": [None if p is None else [round(p[0], 2), round(p[1], 2)]
                                   for p in per],
            "untiltedWristMm": [round(float(v), 2) for v in self.wrist_distance_mm[:, 0, 0]]
            if self.wrist_distance_mm.size else [],
            "axes": list(self.axis_names),
            # pad-local aliases of the two grids (the x/z names are the legacy
            # reference-frame reading and are kept for the Studio's readers)
            "rollGridDeg": [float(v) for v in self.x_grid],
            "pitchGridDeg": [float(v) for v in self.z_grid],
        }


def analyse_tilt(arm, poses: Sequence[np.ndarray], shoulder: np.ndarray,
                 policy, gate_deg: float = 20.0,
                 names: Optional[Sequence[str]] = None) -> TiltReport:
    """Evaluate every candidate tilt at every waypoint, by wrist-centre distance.

    Exact and cheap. The wrist centre is rigidly attached to the tool frame, so
    tilting the tool moves it to a point that is known in closed form, and
    whether the arm can put its wrist there is one distance against one radius.
    No IK, no collision — this narrows the field, and the atlas judges what
    survives.
    """
    poses = [np.asarray(T, dtype=float) for T in poses]
    S = len(poses)
    xs, zs = policy.axis_grids()
    v = arm._v_tip_wrist
    shoulder = np.asarray(shoulder, dtype=float)

    d = np.empty((S, len(xs), len(zs)), dtype=float)
    # Either policy: the wrist centre after the tilt is p + R_tilted @ v, and
    # `tilted_rotation` knows on which side of R0 its rotation goes.
    for s, T in enumerate(poses):
        p, R0 = T[:3, 3], T[:3, :3]
        for i, x in enumerate(xs):
            for j, z in enumerate(zs):
                d[s, i, j] = np.linalg.norm(p + policy.tilted_rotation(R0, x, z) @ v - shoulder)

    max_reach = arm.max_reach_m
    gate_reach = float(arm.distance_from_elbow(np.radians(gate_deg)))
    report = TiltReport(
        x_grid=xs, z_grid=zs, wrist_distance_mm=d * 1000.0,
        reach_ok=d <= max_reach + 1e-12, gate_ok=d <= gate_reach + 1e-12,
        max_reach_mm=max_reach * 1000.0, gate_reach_mm=gate_reach * 1000.0,
        gate_deg=float(gate_deg), names=list(names or []),
        axis_names=tuple(getattr(policy, "axis_names", ("refX", "refZ"))))
    logger.info("tilt search: %d waypoints x %d x %d tilts; %d need a tilt to reach, "
                "%d cannot be rescued at all; uniform=%s",
                S, len(xs), len(zs),
                len([p for p in report.per_waypoint_minimum() if p not in (None, (0.0, 0.0))]),
                len(report.blocking_waypoints()), report.uniform())
    return report
