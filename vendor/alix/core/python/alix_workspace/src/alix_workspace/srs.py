"""Shoulder-Revolute-Shoulder arm structure and the swivel (arm-angle) coordinate.

The RM75 is an **S-R-S** arm: joint axes 1,2,3 meet at one point, axes 5,6,7 meet
at another, joint 4 is alone between them, and there are no link offsets. Nothing
in this tree stated that as a fact you could compute against, so the redundancy —
the one-dimensional family of arm configurations behind every reachable pose —
had no representation on the Python side at all.

What that structure buys, and why this module exists:

* **The self-motion manifold is a rigid rotation.** Hold the tool pose and the
  pole; the shoulder and the wrist centre are then fixed points, the elbow orbits
  the line between them, and the whole arm is the seed configuration rotated
  about that line. So one numerical IK solve per pose yields the *entire* fibre
  in closed form — no per-swivel re-solve, no branch flips.

* **Both triples are exactly ZYZ.** Verified against the URDF over 400 random
  configurations: the shoulder rotation is ``Rz(q1)Ry(q2)Rz(q3)`` times a
  constant, and so is the wrist. Recovering joint values from a rotation is then
  a Euler decomposition with two sign branches each, which is why a pose has up
  to four configurations at one swivel angle and they are *disjoint* — a
  continuous motion cannot cross between them (Burdick 1989).

* **The straight-elbow singular set is a distance, not a posture.** The elbow is
  a planar two-link, so ``d^2 = a^2 + b^2 + 2ab cos q4`` where ``d`` is the
  shoulder-to-wrist distance. q4 therefore depends on nothing but ``d``: no
  choice of swivel, wipe yaw or wrist branch moves it. :func:`elbow_from_distance`
  and :func:`distance_from_elbow` are that relation, and they are what separates
  a singularity you can resolve from one that needs the fixture moved.

Accuracy: reconstruction through the exact ZYZ form now matches the URDF's own
forward kinematics to float precision (< 1 nm over the committed sweep). It used
to land ~5 um / 0.5 mdeg out, and that residual was entirely the URDF writing its
right angles as ``1.5708``, 3.3 urad short of pi/2; the xacro states ``${pi/2}``
since 2026-08-27. :meth:`SrsArm.branches_at` still accepts a ``polish`` argument
that runs damped-least-squares steps, which is now only useful if a future
description reintroduces rounding.

The swivel convention matches ``stroke/arm_kinematics.cpp::swivelAngle`` exactly
(same three link origins, same reference axis, same sign), so a swivel reported
here and one reported by the C++ stroke pipeline are the same number — provided
both measure against the same axis in the same frame. The reference axis is a
**model-frame** direction: it is chosen once per stroke (:meth:`SrsArm.swivel_ref_for`)
and passed explicitly, and a chain rooted at a link that is rotated relative to
the model root is refused unless that axis is given. Two replays of one fixed
188-degree track once read 270 and 230 degrees of swivel range because a script
built the arm on ``R_base_link`` and inherited its rotation; that construction no
longer succeeds by accident.
"""

from __future__ import annotations

import logging
import warnings
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np
from scipy.spatial.transform import Rotation

from alix_workspace.urdf_kinematics import Joint, UrdfModel

def _cross3(a, b, out):
    """3-vector cross product, written out, into *out*.

    `np.cross` is general — it accepts stacked inputs and routes every call
    through `moveaxis` and `normalize_axis_tuple` — and the Jacobian calls it
    once per joint per iteration: 614,726 calls and 16.5 s of an 84 s coverable
    stage, for nine multiplications each. Writing into *out* also avoids the
    temporary. Bit-identical to `np.cross`, asserted in the test.
    """
    a0, a1, a2 = a[0], a[1], a[2]
    b0, b1, b2 = b[0], b[1], b[2]
    out[0] = a1 * b2 - a2 * b1
    out[1] = a2 * b0 - a0 * b2
    out[2] = a0 * b1 - a1 * b0
    return out


logger = logging.getLogger("alix_workspace.srs")

#: Axis-intersection residual accepted when verifying the S-R-S property (m).
#: The measured residual on `alix.urdf` is ~1e-9; 0.5 mm is loose enough to
#: admit a differently-rounded URDF and tight enough that a genuinely offset
#: wrist (the common non-S-R-S 7-DOF layout) is refused.
SRS_TOLERANCE_M = 5.0e-4

#: |ref . a| above which the reference is too close to the shoulder-wrist line
#: for the swivel angle to be well conditioned. The same figure as
#: ``arm_kinematics.cpp``. The runtime's null-space drive differentiates psi, so
#: an axis that switched at this threshold mid-stroke would hand it a step; with
#: an explicit reference both sides refuse instead.
SWIVEL_REF_MAX_DOT = 0.95

#: Model-frame candidates for :meth:`SrsArm.swivel_ref_for`, in preference
#: order. Z first: the arms hang from vertical poles and every delivered stroke's
#: shoulder-wrist lines stay well away from vertical (max |Z . a| 0.776 over all
#: recorded plans and replays), so Z is the axis the runtime's numbers are in.
SWIVEL_REF_CANDIDATES = (
    np.array([0.0, 0.0, 1.0]),
    np.array([1.0, 0.0, 0.0]),
    np.array([0.0, 1.0, 0.0]),
)


class IllConditionedSwivelError(ValueError):
    """A fixed swivel reference passes within 18 degrees of THIS sample's
    shoulder-wrist line, where the projection that defines psi collapses.

    A ``ValueError`` subclass, so every caller that already refuses on one is
    unchanged; named so a caller that can degrade — the stroke gate turning an
    attempt into the `swivel-reference` class rather than aborting the run —
    can tell it from an argument it got wrong.
    """


#: STEREOGRAPHIC SWIVEL (Newton 2026-09-17, "both": the stereographic swivel angle with its
#: singular direction CHOSEN; Resource/reports/2026-09-17_newton_rulings_evening). A
#: conventional axis is singular on the whole LINE a = +-axis; the stereographic angle
#: (Elias & Wen 2024, eqs. 26-29) only on the HALF-LINE a = e_t. Its default placement puts
#: e_t behind the robot (-X) and reads like the conventional Z angle for a forward reach.
STEREO_DEFAULT_SINGULAR_DIRECTION = (-1.0, 0.0, 0.0)
STEREO_DEFAULT_REFERENCE_DIRECTION = (0.0, 0.0, 1.0)
#: The default placement is kept while every pose stays at least 45 deg from e_t
#: (e_t . a <= cos 45); nearer than that the frame starts to spin with the pose and
#: `stereographic_swivel_for` places e_t as far from the poses as it can instead.
STEREO_KEEP_DEFAULT_MAX_DOT = float(np.cos(np.radians(45.0)))
#: How far a written reference direction may be from normal to e_t (cosine): a plan
#: ROUNDS its directions, it does not bend them (config_loader / SwivelCoordinate).
STEREO_ORTHOGONALITY_TOL = 1e-3
#: Decimals a chosen placement is written with. The coordinate every psi is measured
#: in is the CANONICAL form of these rounded numbers, which is what the runtime
#: rebuilds from the file - so authoring and runtime agree to the bit.
STEREO_WRITE_DECIMALS = 6


@dataclass(frozen=True)
class StereographicSwivel:
    """The stereographic swivel coordinate: singular direction ``e_t``, reference
    direction ``e_r`` (both unit, MODEL frame, e_r normal to e_t).

    Build with :meth:`canonical`, which applies the runtime's own canonicalisation
    (``stroke::SwivelCoordinate::stereographic``): e_t normalised; e_r made exactly
    normal to it by Gram-Schmidt on the WRITTEN vector, then normalised. Pass an
    instance wherever a swivel ``ref`` is accepted (:meth:`SrsArm.swivel_from_points`,
    :meth:`SrsArm.fibre`, ...); psi is then measured from the zero direction
    ``((a - e_t) x e_r) x a`` and refused where ``e_t . a > SWIVEL_REF_MAX_DOT``.
    """

    e_t: Tuple[float, float, float] = STEREO_DEFAULT_SINGULAR_DIRECTION
    e_r: Tuple[float, float, float] = STEREO_DEFAULT_REFERENCE_DIRECTION
    #: the directions as they are WRITTEN to a plan (what `canonical` was given)
    written_t: Tuple[float, float, float] = STEREO_DEFAULT_SINGULAR_DIRECTION
    written_r: Tuple[float, float, float] = STEREO_DEFAULT_REFERENCE_DIRECTION

    @classmethod
    def canonical(cls, e_t: Sequence[float], e_r: Sequence[float]) -> "StereographicSwivel":
        t_in = np.asarray(e_t, dtype=float).reshape(3)
        r_in = np.asarray(e_r, dtype=float).reshape(3)
        nt, nr = float(np.linalg.norm(t_in)), float(np.linalg.norm(r_in))
        if not (nt > 1e-9) or not np.all(np.isfinite(t_in)):
            raise ValueError(f"stereographic swivel: the singular direction {t_in.tolist()} has no direction")
        if not (nr > 1e-9) or not np.all(np.isfinite(r_in)):
            raise ValueError(f"stereographic swivel: the reference direction {r_in.tolist()} has no direction")
        t = t_in / nt
        cosine = float(np.dot(t, r_in / nr))
        if abs(cosine) > STEREO_ORTHOGONALITY_TOL:
            raise ValueError(f"stereographic swivel: the reference direction is not normal to the singular "
                             f"direction (cosine {cosine:.6f}, tolerance {STEREO_ORTHOGONALITY_TOL:g})")
        r = r_in - float(np.dot(r_in, t)) * t
        r = r / float(np.linalg.norm(r))
        return cls(e_t=tuple(float(v) for v in t), e_r=tuple(float(v) for v in r),
                   written_t=tuple(float(v) for v in t_in), written_r=tuple(float(v) for v in r_in))

    @classmethod
    def default(cls) -> "StereographicSwivel":
        return cls.canonical(STEREO_DEFAULT_SINGULAR_DIRECTION, STEREO_DEFAULT_REFERENCE_DIRECTION)

    @classmethod
    def placed_at(cls, e_t: Sequence[float]) -> "StereographicSwivel":
        """The coordinate with singular direction *e_t*, rounded for writing, and its
        reference direction taken from model Z (model X when e_t is within 25 deg of
        Z), so psi's zero stays 'elbow up' wherever that is defined."""
        t = np.asarray(e_t, dtype=float).reshape(3)
        t = t / float(np.linalg.norm(t))
        up = np.array([0.0, 0.0, 1.0]) if abs(float(t[2])) < 0.9 else np.array([1.0, 0.0, 0.0])
        r = up - float(np.dot(up, t)) * t
        r = r / float(np.linalg.norm(r))
        k = STEREO_WRITE_DECIMALS
        return cls.canonical(np.round(t, k), np.round(r, k))

    @property
    def singular(self) -> np.ndarray:
        return np.asarray(self.e_t, dtype=float)

    @property
    def reference(self) -> np.ndarray:
        return np.asarray(self.e_r, dtype=float)

    def conditioning(self, a: np.ndarray) -> float:
        """e_t . a for the unit shoulder->wrist direction *a* (model frame)."""
        return float(np.dot(self.singular, np.asarray(a, dtype=float)))

    def plan_fields(self) -> Dict[str, object]:
        """The `swivel_plan` keys that declare this coordinate (config_loader.hpp)."""
        return {"convention": "stereographic",
                "singular_direction": [float(v) for v in self.written_t],
                "reference_direction": [float(v) for v in self.written_r]}

    def describe(self) -> str:
        return (f"stereographic e_t {np.round(self.singular, 6).tolist()} "
                f"e_r {np.round(self.reference, 6).tolist()}")


def swivel_coordinate_record(ref) -> Dict[str, object]:
    """A JSON-able record of a swivel ``ref``: a stereographic coordinate or a fixed axis."""
    if isinstance(ref, StereographicSwivel):
        return {"convention": "stereographic",
                "singularDirection": [round(float(v), 6) for v in ref.singular],
                "referenceDirection": [round(float(v), 6) for v in ref.reference]}
    if ref is None:
        return {"convention": "conventional", "axis": None}
    return {"convention": "conventional",
            "axis": [round(float(v), 6) for v in np.asarray(ref, dtype=float).reshape(3)]}


def swivel_conditioning(ref, a: np.ndarray) -> float:
    """The conditioning number of *ref* at unit direction *a* (both in one frame):
    ``e_t . a`` (stereographic) or ``|ref . a|`` (a fixed axis)."""
    a = np.asarray(a, dtype=float)
    if isinstance(ref, StereographicSwivel):
        return ref.conditioning(a)
    r = np.asarray(ref, dtype=float).reshape(3)
    return abs(float(np.dot(r / float(np.linalg.norm(r)), a)))


_legacy_ref_warned = False

_LEGACY_REF_MESSAGE = (
    "swivel reference ref=None (per-sample fallback to X above |ref.a| = 0.95) is "
    "deprecated and kept for one release so archived numbers reproduce; choose the "
    "axis once per stroke with SrsArm.swivel_ref_for and pass it explicitly")


def _warn_legacy_ref() -> None:
    """Once per process: ``ref=None`` still carries the per-sample fallback.

    Both channels on purpose. ``DeprecationWarning`` is what a test suite sees
    and what a release note is written against, but Python's default filters
    hide it outside ``__main__``, so a production caller left on the legacy
    path would be silent; the log line is what makes it visible in a server
    log. The switch itself is not an error — the archived numbers were
    measured with it — so this stays a warning.
    """
    global _legacy_ref_warned
    if _legacy_ref_warned:
        return
    _legacy_ref_warned = True
    logger.warning("%s", _LEGACY_REF_MESSAGE)
    warnings.warn(_LEGACY_REF_MESSAGE, DeprecationWarning, stacklevel=4)


def _skew(v: np.ndarray) -> np.ndarray:
    x, y, z = v
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])


def axis_line_distance(p1: np.ndarray, a1: np.ndarray,
                       p2: np.ndarray, a2: np.ndarray) -> float:
    """Shortest distance between two axis lines; 0 means they intersect."""
    c = np.cross(a1, a2)
    n = np.linalg.norm(c)
    if n < 1e-9:                       # parallel: distance between the lines
        d = p2 - p1
        return float(np.linalg.norm(d - (d @ a1) * a1))
    return float(abs((p2 - p1) @ c) / n)


def zyz_branches(M: np.ndarray) -> List[Tuple[float, float, float]]:
    """Both ZYZ Euler triples for a rotation matrix.

    ``Rotation.as_euler("ZYZ")`` returns only the branch with the middle angle
    in [0, pi]. The RM75's q2 and q6 both run negative, so the second branch
    ``(a +- pi, -b, c +- pi)`` is a real configuration, not a duplicate — and it
    is a *different* self-motion manifold, reachable only by re-solving, never
    by moving continuously. Returning one of the two silently halves the
    configurations the atlas can see.
    """
    a, b, c = Rotation.from_matrix(M).as_euler("ZYZ")
    a2 = a + np.pi if a < 0.0 else a - np.pi
    c2 = c + np.pi if c < 0.0 else c - np.pi
    return [(a, b, c), (a2, -b, c2)]


@dataclass
class SrsArm:
    """The S-R-S structure of one arm, and the swivel coordinate over it.

    Build with :meth:`from_model`; it discovers the joints and refuses a chain
    that is not S-R-S rather than returning wrong answers quietly.
    """

    model: UrdfModel
    base_link: str
    tip_link: str
    joints: List[Joint]                 #: the 7 revolute joints, shoulder to wrist
    pole_joint: Optional[str]           #: prismatic joint upstream of the arm, if any
    shoulder_link: str
    elbow_link: str
    wrist_link: str
    flange_link: str                    #: child of the last revolute joint

    upper_arm_m: float                  #: a — shoulder to elbow
    forearm_m: float                    #: b — elbow to wrist centre
    #: Configured swivel reference, a unit MODEL-frame axis. The value every
    #: production caller passes as ``ref``; model Z on the ``base_link`` chains.
    swivel_ref_axis: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, 1.0]))

    # constants captured at q = 0 (rotations only; translations vary with pole)
    _R_base: np.ndarray = field(default=None, repr=False)      # base -> j1's parent
    _R_a3: np.ndarray = field(default=None, repr=False)        # shoulder ZYZ offset
    _R_a57: np.ndarray = field(default=None, repr=False)       # wrist ZYZ offset
    _T_flange_tip: np.ndarray = field(default=None, repr=False)
    _v_tip_wrist: np.ndarray = field(default=None, repr=False)  # wrist centre in tip frame
    #: Rotation of ``base_link`` in the model frame; None when it is identity,
    #: so the production chain's arithmetic is untouched by the conversion.
    _R_root_base: Optional[np.ndarray] = field(default=None, repr=False)
    _residual_m: float = 0.0

    # ------------------------------------------------------------------
    @property
    def joint_names(self) -> List[str]:
        return [j.name for j in self.joints]

    @property
    def lower(self) -> np.ndarray:
        return np.array([j.lower for j in self.joints], dtype=float)

    @property
    def upper(self) -> np.ndarray:
        return np.array([j.upper for j in self.joints], dtype=float)

    @property
    def max_reach_m(self) -> float:
        """Largest shoulder-to-wrist-centre distance — the straight elbow."""
        return self.upper_arm_m + self.forearm_m

    # ------------------------------------------------------------------
    @classmethod
    def from_model(cls, model: UrdfModel, base_link: str, tip_link: str,
                   swivel_ref_axis: Optional[Sequence[float]] = None) -> "SrsArm":
        """Discover and verify the S-R-S structure of ``base_link -> tip_link``.

        ``swivel_ref_axis`` is the configured swivel reference, a MODEL-frame
        direction; it defaults to model Z. Every point this class computes is in
        ``base_link`` coordinates, so when ``base_link`` is rotated relative to
        the model root the default would silently mean "that link's Z" and the
        swivel would stop being the runtime's number: such a chain is refused
        unless the axis is passed explicitly, and an explicit axis is converted
        into the chain's coordinates so psi is the same number either way.

        Raises ``ValueError`` when the chain has other than 7 revolute joints,
        when the shoulder / wrist axes do not meet at a point, or when
        ``base_link`` is rotated and no axis was given.
        """
        chain = model.chain(base_link, tip_link)
        revolute = [j for j in chain if j.type in ("revolute", "continuous") and j.independent]
        if len(revolute) != 7:
            raise ValueError(
                f"{base_link} -> {tip_link} has {len(revolute)} revolute DOF, not 7 — "
                "the arm-angle parameterisation is defined for a 7R arm only")
        prismatic = [j for j in chain if j.type == "prismatic" and j.independent]
        pole = prismatic[0].name if prismatic else None
        if len(prismatic) > 1:
            raise ValueError(f"{len(prismatic)} prismatic joints in the chain; expected at most 1")

        home = model.link_world_transforms({})
        R_root_base = home[base_link][:3, :3]
        rotated = not np.allclose(R_root_base, np.eye(3), atol=1e-9)
        if swivel_ref_axis is None:
            if rotated:
                raise ValueError(
                    f"{base_link} is rotated relative to the model root "
                    f"{model.root_link()!r}:\n"
                    f"{np.array2string(R_root_base, precision=6, suppress_small=True)}\n"
                    "so the default swivel reference (model Z) would be read in that "
                    "link's frame and psi would not be the number the runtime reports. "
                    f"Root the chain at {model.root_link()!r}, or pass an explicit "
                    "model-frame swivel_ref_axis=")
            swivel_ref_axis = (0.0, 0.0, 1.0)
        ref = np.asarray(swivel_ref_axis, dtype=float).reshape(3)
        n_ref = float(np.linalg.norm(ref))
        if n_ref < 1e-9:
            raise ValueError("swivel_ref_axis has zero length")
        ref = ref / n_ref

        # Axis lines at q = 0, in model coordinates (only distances are taken).
        pts, axes = [], []
        for j in revolute:
            T = home[j.child]
            pts.append(T[:3, 3])
            axes.append(T[:3, :3] @ j.axis)

        residual = 0.0
        for group, label in (((0, 1, 2), "shoulder"), ((4, 5, 6), "wrist")):
            for i in group:
                for k in group:
                    if i < k:
                        d = axis_line_distance(pts[i], axes[i], pts[k], axes[k])
                        residual = max(residual, d)
                        if d > SRS_TOLERANCE_M:
                            raise ValueError(
                                f"{label} axes {i + 1} and {k + 1} miss by {d * 1e3:.2f} mm — "
                                f"{base_link} -> {tip_link} is not an S-R-S arm, so the "
                                "arm-angle parameterisation does not apply to it")

        shoulder_link = revolute[1].child      # axes 1,2 meet here
        elbow_link = revolute[3].child         # axes 3,4 meet here
        wrist_link = revolute[5].child         # axes 5,6 meet here
        flange_link = revolute[6].child

        p_s = home[shoulder_link][:3, 3]
        p_e = home[elbow_link][:3, 3]
        p_w = home[wrist_link][:3, 3]

        R_parent = home[revolute[0].parent][:3, :3]
        R_a3 = home[revolute[2].child][:3, :3]         # after q3, at q = 0
        R_a57 = home[elbow_link][:3, :3].T @ home[flange_link][:3, :3]
        T_flange_tip = np.linalg.inv(home[flange_link]) @ home[tip_link]
        T_tip = home[tip_link]
        v_tip_wrist = T_tip[:3, :3].T @ (p_w - T_tip[:3, 3])

        arm = cls(
            model=model, base_link=base_link, tip_link=tip_link, joints=revolute,
            pole_joint=pole, shoulder_link=shoulder_link, elbow_link=elbow_link,
            wrist_link=wrist_link, flange_link=flange_link,
            upper_arm_m=float(np.linalg.norm(p_e - p_s)),
            forearm_m=float(np.linalg.norm(p_w - p_e)),
            swivel_ref_axis=ref,
        )
        # chain_frames works in base_link coordinates, and so must the one
        # absolute rotation the fibre reconstruction keeps.
        arm._R_root_base = R_root_base.copy() if rotated else None
        arm._R_base = R_root_base.T @ R_parent if rotated else R_parent
        arm._R_a3 = R_parent.T @ R_a3               # so R_link3 = R_base @ ZYZ @ _R_a3
        arm._R_a57 = R_a57
        arm._T_flange_tip = T_flange_tip
        arm._v_tip_wrist = v_tip_wrist
        arm._residual_m = residual
        logger.info("S-R-S verified for %s -> %s: a=%.4f m, b=%.4f m, axis residual %.2e m",
                    base_link, tip_link, arm.upper_arm_m, arm.forearm_m, residual)
        return arm

    # ------------------------------------------------------------------
    def geometry(self) -> dict:
        """The numbers a placement decision is actually made on."""
        return {
            "upperArmM": round(self.upper_arm_m, 6),
            "forearmM": round(self.forearm_m, 6),
            "maxWristReachM": round(self.max_reach_m, 6),
            "toolOffsetM": round(float(np.linalg.norm(self._v_tip_wrist)), 6),
            "maxToolReachM": round(self.max_reach_m + float(np.linalg.norm(self._v_tip_wrist)), 6),
            "axisResidualM": self._residual_m,
            "jointNames": self.joint_names,
            "poleJoint": self.pole_joint,
            "limitsDeg": [[round(np.degrees(lo), 2), round(np.degrees(hi), 2)]
                          for lo, hi in zip(self.lower, self.upper)],
        }

    # ------------------------------------------------------------------
    # The straight-elbow relation — geometry, not posture
    # ------------------------------------------------------------------
    def elbow_from_distance(self, d_m) -> np.ndarray:
        """|q4| for a shoulder-to-wrist distance, in radians.

        NaN outside ``[|a-b|, a+b]``, where no configuration exists at all.
        """
        a, b = self.upper_arm_m, self.forearm_m
        d = np.asarray(d_m, dtype=float)
        c = (d * d - a * a - b * b) / (2.0 * a * b)
        out = np.arccos(np.clip(c, -1.0, 1.0))
        return np.where((d <= a + b + 1e-12) & (d >= abs(a - b) - 1e-12), out, np.nan)

    def distance_from_elbow(self, q4_rad) -> np.ndarray:
        """Shoulder-to-wrist distance at an elbow angle — the inverse relation.

        ``distance_from_elbow(radians(margin))`` is the radius of the shell in
        which the straight-elbow singularity lives: everything further out than
        it is inside the gate, and no redundancy resolution can leave it.
        """
        a, b = self.upper_arm_m, self.forearm_m
        return np.sqrt(a * a + b * b + 2.0 * a * b * np.cos(np.asarray(q4_rad, dtype=float)))

    def elbow_orbit_radius(self, q4_rad) -> np.ndarray:
        """Radius of the elbow's circle about the shoulder-wrist axis.

        ``r = a b sin(q4) / d``. This is how far the swivel coordinate actually
        moves the elbow, and it collapses to zero as the arm straightens — the
        redundancy runs out at the same place the reach does.
        """
        q4 = np.asarray(q4_rad, dtype=float)
        d = self.distance_from_elbow(q4)
        return self.upper_arm_m * self.forearm_m * np.abs(np.sin(q4)) / np.maximum(d, 1e-12)

    # ------------------------------------------------------------------
    # Forward kinematics restricted to the chain (fast; no whole-tree walk)
    # ------------------------------------------------------------------
    def _chain_cache(self):
        """Chain joints with their origin matrices precomputed.

        ``UrdfModel.chain`` walks the tree and ``origin_matrix`` rebuilds a
        rotation from Euler angles on every call; both are pure functions of the
        URDF, and the atlas calls this thousands of times per stroke.
        """
        cache = getattr(self, "_chain_cached", None)
        if cache is None:
            chain = self.model.chain(self.base_link, self.tip_link)
            cache = [(jt, jt.origin_matrix()) for jt in chain]
            object.__setattr__(self, "_chain_cached", cache)
        return cache

    def chain_frames(self, q: Sequence[float], pole: float = 0.0) -> Dict[str, np.ndarray]:
        """World transforms of the links the swivel maths needs.

        Deliberately not :meth:`UrdfModel.link_world_transforms`, which walks the
        whole robot (both arms, every finger) — the atlas calls this once per
        cell and the tree walk dominates.
        """
        vals = dict(zip(self.joint_names, np.asarray(q, dtype=float)))
        if self.pole_joint:
            vals[self.pole_joint] = float(pole)
        T = np.eye(4)
        out = {}
        for jt, origin in self._chain_cache():
            T = T @ origin
            if jt.movable:
                T = T @ jt.motion_matrix(float(vals.get(jt.name, 0.0)))
            out[jt.child] = T.copy()
        return out

    def tool_pose(self, q: Sequence[float], pole: float = 0.0) -> np.ndarray:
        return self.chain_frames(q, pole)[self.tip_link]

    # ------------------------------------------------------------------
    # The swivel coordinate
    # ------------------------------------------------------------------
    def _ref_in_base(self, ref: np.ndarray) -> np.ndarray:
        """A model-frame axis in the coordinates :meth:`chain_frames` works in."""
        if self._R_root_base is None:
            return ref
        return self._R_root_base.T @ ref

    def swivel_ref_for(self, directions: np.ndarray,
                       max_dot: float = SWIVEL_REF_MAX_DOT,
                       keep_configured: bool = False) -> np.ndarray:
        """Choose the swivel reference axis ONCE for a whole stroke.

        ``directions`` are the stroke's shoulder -> wrist-centre vectors, shape
        ``(N, 3)`` in ``base_link`` coordinates (any length; each is normalised).
        They need no IK: ``wrist_centre_of_pose(T) - shoulder`` for every pose,
        with the shoulder from ``chain_frames(zeros(7), pole)``. The candidates
        are model Z, X, Y in that order and the first one with the smallest
        ``max |ref . a|`` over the stroke wins, so Z — the runtime's axis — is
        kept whenever it is conditioned at least as well as the others.

        ``keep_configured=True`` returns :attr:`swivel_ref_axis` whenever that
        axis alone stays within ``max_dot`` on the stroke and only then falls
        back to the min-max choice. The swivel statistics a gate reads are NOT
        invariant to the reference (its projection turns with the shoulder-
        wrist line along the stroke): the delivered side_left ruckig_on_s
        track reads 32.9 deg range / 37.4 deg TV / 1 reversal on model Z and
        40.9 / 95.5 / 5 on model Y, although Y is the better-conditioned axis
        (max |ref . a| 0.58 vs 0.68). Anything compared against the runtime's
        number must therefore stay on the runtime's axis while that axis is
        usable — or emit the chosen axis as the config's ``swivel_plan.ref_axis``.

        Returns the chosen axis as a unit MODEL-frame vector, to be passed as
        ``ref`` to :meth:`swivel_from_points`, :meth:`swivel_angle` and
        :meth:`fibre`. Raises `IllConditionedSwivelError` when even the best
        candidate exceeds ``max_dot`` somewhere on the stroke: no fixed axis is
        well conditioned, and switching axes mid-stroke is exactly the
        discontinuity this exists to remove (the contingency is a stereographic
        SEW angle, not a per-sample fallback).
        """
        D = np.asarray(directions, dtype=float).reshape(-1, 3)
        n = np.linalg.norm(D, axis=1)
        D = D[n > 1e-9] / n[n > 1e-9, None]
        if len(D) == 0:
            raise ValueError("no shoulder->wrist directions to condition the swivel reference on")
        if keep_configured:
            configured = np.asarray(self.swivel_ref_axis, dtype=float)
            if float(np.max(np.abs(D @ self._ref_in_base(configured)))) <= max_dot:
                return configured.copy()
        best_worst, best = np.inf, None
        for cand in SWIVEL_REF_CANDIDATES:
            worst = float(np.max(np.abs(D @ self._ref_in_base(cand))))
            if worst < best_worst:
                best_worst, best = worst, cand
        if best_worst > max_dot:
            raise IllConditionedSwivelError(
                "no well-conditioned swivel reference for this stroke: the best of "
                f"model Z/X/Y is {best.tolist()} with max |ref.a| = {best_worst:.4f} > "
                f"{max_dot} over {len(D)} samples, so every fixed axis passes within "
                f"{np.degrees(np.arccos(min(best_worst, 1.0))):.1f} deg of the "
                "shoulder-wrist line somewhere on the path")
        return best.copy()

    def stereographic_swivel_for(self, directions: np.ndarray,
                                 keep_default_max_dot: float = STEREO_KEEP_DEFAULT_MAX_DOT,
                                 max_dot: float = SWIVEL_REF_MAX_DOT
                                 ) -> Tuple["StereographicSwivel", Dict[str, object]]:
        """THE STEREOGRAPHIC COORDINATE for a set of shoulder->wrist directions, and
        the record of how it was chosen.

        ``directions`` are ``(N, 3)`` in ``base_link`` coordinates (any length), as
        for :meth:`swivel_ref_for`. The default placement (e_t = -X) is kept while
        its worst ``e_t . a`` stays at or below ``keep_default_max_dot``; otherwise
        e_t is PLACED: among ~2000 near-uniform directions (then refined locally),
        the one with the fewest directions past ``max_dot`` and, among those, the
        largest angle to its nearest direction. It never raises for conditioning:
        a pose still past ``max_dot`` is refused by the fibre ALONE (the map drops
        that node), which is the most any fixed coordinate can do.

        MEASURED 2026-09-17 over 5429 realistic maps of the five fixtures: a free
        placement leaves 38 maps (0.7 %) with any pose past 0.95, the best of model
        Z/X/Y 926 (Resource/reports/2026-09-17_newton_rulings_evening).
        """
        D = np.asarray(directions, dtype=float).reshape(-1, 3)
        n = np.linalg.norm(D, axis=1)
        D = D[n > 1e-9] / n[n > 1e-9, None]
        if len(D) == 0:
            raise ValueError("no shoulder->wrist directions to condition the swivel coordinate on")
        default = StereographicSwivel.default()
        worst_default = float(np.max(D @ self._ref_in_base(default.singular)))
        record: Dict[str, object] = {"defaultWorstDot": round(worst_default, 4), "poses": int(len(D)),
                                     "keepDefaultMaxDot": round(float(keep_default_max_dot), 4)}
        if worst_default <= keep_default_max_dot:
            record.update(chosen="default", worstDot=round(worst_default, 4), posesPastLimit=0)
            return default, record

        def score(cands: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
            dots = D @ np.array([self._ref_in_base(c) for c in cands]).T      # (N, C)
            return (dots > max_dot).sum(axis=0), dots.max(axis=0)

        m = 2000
        i = np.arange(m) + 0.5
        phi = np.arccos(1.0 - 2.0 * i / m)
        theta = np.pi * (1.0 + 5.0 ** 0.5) * i
        cands = np.column_stack([np.cos(theta) * np.sin(phi), np.sin(theta) * np.sin(phi), np.cos(phi)])
        cands = np.vstack([cands, np.eye(3), -np.eye(3)])
        over, worst = score(cands)
        best = int(np.lexsort((worst, over))[0])
        # local refinement: a ring of directions within the grid's spacing
        rng = np.random.default_rng(20260917)
        for spread_deg in (4.0, 1.5, 0.5):
            local = cands[best] + np.radians(spread_deg) * rng.standard_normal((200, 3))
            local = local / np.linalg.norm(local, axis=1)[:, None]
            lo_over, lo_worst = score(local)
            k = int(np.lexsort((lo_worst, lo_over))[0])
            if (lo_over[k], lo_worst[k]) < (over[best], worst[best]):
                cands = np.vstack([cands, local[k]])
                over = np.append(over, lo_over[k])
                worst = np.append(worst, lo_worst[k])
                best = len(cands) - 1
        chosen = StereographicSwivel.placed_at(cands[best])
        dots = D @ self._ref_in_base(chosen.singular)
        record.update(chosen="placed", worstDot=round(float(dots.max()), 4),
                      posesPastLimit=int((dots > max_dot).sum()),
                      marginDeg=round(float(np.degrees(np.arccos(np.clip(dots.max(), -1.0, 1.0)))), 2))
        return chosen, record

    def _swivel_basis(self, a: np.ndarray, ref
                      ) -> Tuple[np.ndarray, np.ndarray]:
        """``(u, v)``: the in-plane frame psi is measured in, for unit axis ``a``.

        ``ref`` is a MODEL-frame axis or a :class:`StereographicSwivel`. Given, it is
        used as is and an ill-conditioned sample raises; ``None`` is the legacy
        per-sample behaviour (configured axis, X when |ref . a| > 0.95), deprecated.
        The stereographic zero is ``((a - e_t) x e_r) x a`` - the construction
        ``arm_kinematics.cpp::SwivelCoordinate::zeroDirection`` uses, held equal by
        the stereographic golden fixture.
        """
        if isinstance(ref, StereographicSwivel):
            e_t = self._ref_in_base(ref.singular)
            e_r = self._ref_in_base(ref.reference)
            c = float(e_t @ a)
            if c > SWIVEL_REF_MAX_DOT:
                raise IllConditionedSwivelError(
                    f"{ref.describe()} is ill-conditioned at this sample: e_t.a = {c:.4f} > "
                    f"{SWIVEL_REF_MAX_DOT} against the shoulder->wrist direction "
                    f"{np.round(a, 4).tolist()} (base_link coordinates). The coordinate is fixed "
                    "for the stroke and is never switched per sample")
            u = np.cross(np.cross(a - e_t, e_r), a)
            nu = float(np.linalg.norm(u))
            if nu < 1e-12:
                raise IllConditionedSwivelError(f"{ref.describe()} has no zero direction at this sample")
            u = u / nu
            v = np.cross(a, u)
            return u, v
        if ref is None:
            _warn_legacy_ref()
            r = self._ref_in_base(self.swivel_ref_axis)
            if abs(float(r @ a)) > SWIVEL_REF_MAX_DOT:
                r = self._ref_in_base(np.array([1.0, 0.0, 0.0]))
        else:
            r = np.asarray(ref, dtype=float).reshape(3)
            nr = float(np.linalg.norm(r))
            if nr < 1e-9:
                raise ValueError("swivel reference axis has zero length")
            r = self._ref_in_base(r / nr)
            dot = abs(float(r @ a))
            if dot > SWIVEL_REF_MAX_DOT:
                raise IllConditionedSwivelError(
                    f"swivel reference {np.asarray(ref, dtype=float).tolist()} is "
                    f"ill-conditioned at this sample: |ref.a| = {dot:.4f} > "
                    f"{SWIVEL_REF_MAX_DOT} against the shoulder->wrist direction "
                    f"{np.round(a, 4).tolist()} (base_link coordinates). The axis is "
                    "fixed for the stroke and is never switched per sample; choose it "
                    "with SrsArm.swivel_ref_for over the whole path")
        u = r - (r @ a) * a
        u = u / np.linalg.norm(u)
        v = np.cross(a, u)
        return u, v

    def swivel_from_points(self, p_s: np.ndarray, p_e: np.ndarray,
                           p_w: np.ndarray, ref: Optional[np.ndarray] = None) -> float:
        """Swivel angle from the three joint centres.

        Identical construction to ``arm_kinematics.cpp::swivelAngle``: the
        reference axis projected onto the plane normal to the shoulder-wrist
        axis, the elbow's in-plane angle measured from it with the same sign.

        ``ref`` is a MODEL-frame axis — the one :meth:`swivel_ref_for` chose for
        the stroke, or the arm's configured :attr:`swivel_ref_axis` — while
        ``p_s``, ``p_e``, ``p_w`` are in ``base_link`` coordinates. The two agree
        only because the chain's ``base_link`` is the model root, or because
        :meth:`from_model` was given the axis explicitly and converts it; a
        chain rooted at ``{L,R}_base_link`` without one is refused there. With
        ``ref`` given there is NO fallback: a sample with |ref . a| > 0.95
        raises rather than switching axes. ``ref=None`` keeps the old per-sample
        switch to X for one release (``DeprecationWarning``, once per process).
        """
        a = p_w - p_s
        na = np.linalg.norm(a)
        if na < 1e-9:
            return float("nan")
        a = a / na
        u, v = self._swivel_basis(a, ref)
        e = p_e - p_s
        e = e - (e @ a) * a
        return float(np.arctan2(e @ v, e @ u))

    def swivel_angle(self, q: Sequence[float], pole: float = 0.0,
                     ref: Optional[np.ndarray] = None) -> float:
        f = self.chain_frames(q, pole)
        return self.swivel_from_points(f[self.shoulder_link][:3, 3],
                                       f[self.elbow_link][:3, 3],
                                       f[self.wrist_link][:3, 3], ref)

    def wrist_centre_of_pose(self, T_tip: np.ndarray) -> np.ndarray:
        """Wrist centre implied by a tool pose.

        The wrist centre is rigidly attached to the tool frame (the three wrist
        rotations turn the tool *about* it), so this is exact and needs no IK.
        """
        T_tip = np.asarray(T_tip, dtype=float)
        return T_tip[:3, 3] + T_tip[:3, :3] @ self._v_tip_wrist

    # ------------------------------------------------------------------
    # The fibre: every configuration at one swivel angle
    # ------------------------------------------------------------------
    def fibre(self, T_tip: np.ndarray, seed_q: Sequence[float],
              pole: float = 0.0, ref: Optional[np.ndarray] = None
              ) -> Optional["SelfMotionFibre"]:
        """The whole self-motion family behind one tool pose.

        ``seed_q`` is any configuration that already reaches ``T_tip``; one
        numerical solve per pose is enough, because every other configuration on
        the fibre is that one rotated about the shoulder-wrist axis. Returns
        ``None`` when the seed is degenerate (shoulder and wrist coincident).
        ``ref`` is the stroke's swivel reference (see :meth:`swivel_from_points`);
        the fibre's ``psi`` is measured against it.
        """
        return SelfMotionFibre.build(self, np.asarray(T_tip, dtype=float),
                                     np.asarray(seed_q, dtype=float), float(pole), ref)

    def branches_at(self, T_tip: np.ndarray, psi: float, seed_q: Sequence[float],
                    pole: float = 0.0, respect_limits: bool = True,
                    polish: int = 0, ref: Optional[np.ndarray] = None) -> List[np.ndarray]:
        """Configurations holding ``T_tip`` at swivel ``psi``, from a seed solution.

        Scalar convenience over :meth:`fibre`. The elbow angle is a property of
        the pose, so it is carried over from the seed unchanged; only its
        **sign** distinguishes the two elbow manifolds, and reaching the other
        one needs a differently-seeded solve, not a swivel.

        Returns up to four configurations — the two shoulder and two wrist Euler
        branches — ordered by joint-limit margin, best first. They are disjoint:
        a continuous motion stays on one.
        """
        fib = self.fibre(T_tip, seed_q, pole, ref)
        if fib is None:
            return []
        Q = fib.at(np.array([float(psi)]))[0]           # (4, 7), NaN rows = no branch
        out = []
        for q in Q:
            if not np.all(np.isfinite(q)):
                if respect_limits:
                    continue
                q = np.nan_to_num(q)
            if polish:
                q = self._polish(q, T_tip, pole, iters=polish)
                if respect_limits and not self.within_limits(q):
                    continue
            out.append(q)
        out.sort(key=lambda qq: -self.limit_margin(qq))
        return out

    # ------------------------------------------------------------------
    def wrap_into_limits(self, q: np.ndarray) -> np.ndarray:
        """Fold each angle into its limit range by 2*pi where that is possible.

        Euler extraction returns values in (-pi, pi]; joints 1, 3, 5 reach +-177.6
        deg and joint 7 +-359.8, so a solution can be a full turn away from the
        representative the decomposition happened to pick. Broadcasts over any
        leading shape.
        """
        q = np.asarray(q, dtype=float).copy()
        lo, hi = self.lower, self.upper
        inside = (q >= lo) & (q <= hi)
        for k in (-1, 1, -2, 2):
            if inside.all():
                break
            cand = q + k * 2.0 * np.pi
            take = (~inside) & (cand >= lo) & (cand <= hi)
            q = np.where(take, cand, q)
            inside = inside | take
        return q

    _wrap_into_limits = wrap_into_limits          # retained: earlier private name

    def within_limits(self, q: Sequence[float]) -> bool:
        q = np.asarray(q, dtype=float)
        return bool(np.all(q >= self.lower - 1e-9) and np.all(q <= self.upper + 1e-9))

    def limit_margin(self, q: Sequence[float]) -> float:
        """Smallest distance from any joint to its nearest limit, in radians.

        Negative when a joint is outside its range, so it doubles as the
        violation depth.
        """
        q = np.asarray(q, dtype=float)
        return float(np.min(np.minimum(q - self.lower, self.upper - q)))

    # ------------------------------------------------------------------
    def _polish(self, q: np.ndarray, T_tip: np.ndarray, pole: float,
                iters: int = 2) -> np.ndarray:
        """Damped-least-squares steps onto the URDF's own forward kinematics.

        Not needed against the current description: the reconstruction matches
        FK to float precision. It absorbed the URDF's old `1.5708` rounding
        (~5 um) and would absorb any that returns.
        """
        T_tip = np.asarray(T_tip, dtype=float)
        for _ in range(int(iters)):
            f = self.chain_frames(q, pole)
            cur = f[self.tip_link]
            e_p = T_tip[:3, 3] - cur[:3, 3]
            e_r = Rotation.from_matrix(cur[:3, :3].T @ T_tip[:3, :3]).as_rotvec()
            e = np.concatenate([e_p, cur[:3, :3] @ e_r])
            if np.linalg.norm(e) < 1e-12:
                break
            J = self.jacobian(q, pole)
            dq = J.T @ np.linalg.solve(J @ J.T + 1e-8 * np.eye(6), e)
            q = q + dq
        return q

    def jacobian(self, q: Sequence[float], pole: float = 0.0) -> np.ndarray:
        """6x7 geometric Jacobian of the tool frame over the arm joints."""
        f = self.chain_frames(q, pole)
        p_e = f[self.tip_link][:3, 3]
        J = np.zeros((6, 7))
        for i, jt in enumerate(self.joints):
            T = f[jt.child]
            axis = T[:3, :3] @ jt.axis
            _cross3(axis, p_e - T[:3, 3], J[:3, i])
            J[3:, i] = axis
        return J

    # ------------------------------------------------------------------
    def fixed_normal_reach(self, press_normal: Sequence[float], surface_z: float,
                           pole: float = 0.0, y: float = 0.0,
                           elbow_margin_rad: float = 0.0,
                           shoulder: Optional[Sequence[float]] = None) -> Optional[float]:
        """Furthest forward the tool reaches on a plane at a FIXED press normal.

        A necessary geometric condition, and a far tighter one than the reach
        envelope: the envelope is a union over every tool orientation, but a wipe
        fixes the press axis to the surface. With the spin about that axis free,
        the wrist centre sweeps a small circle that has to meet the shoulder's
        wrist ball; this returns the largest ``x`` where it still does.

        Joint limits and every collision would tighten it further, so treat the
        answer as an upper bound, not a promise.
        """
        press = np.asarray(press_normal, dtype=float)
        press = press / np.linalg.norm(press)
        if shoulder is None:
            shoulder = self.chain_frames(np.zeros(7), pole)[self.shoulder_link][:3, 3]
        p_s = np.asarray(shoulder, dtype=float)

        v = self._v_tip_wrist
        along = float(v @ np.array([0.0, 0.0, 1.0]))     # tool-frame Z component
        # Decompose the tip->wrist offset about the press axis: the component
        # along it is fixed, the perpendicular part is what free spin sweeps.
        v_press = float(v @ press) * press
        perp = float(np.linalg.norm(v - v_press))
        # With the press axis pointing INTO a horizontal surface from above, the
        # wrist centre sits |along| above the contact point.
        z_w = surface_z + abs(along)

        radius = self.distance_from_elbow(elbow_margin_rad)
        dz = z_w - p_s[2]
        if abs(dz) > radius:
            return None
        rho = float(np.sqrt(radius * radius - dz * dz)) + perp
        dy = y - p_s[1]
        if abs(dy) > rho:
            return None
        return float(p_s[0] + np.sqrt(rho * rho - dy * dy))


# ---------------------------------------------------------------------------
# The fibre over one pose, evaluated in bulk
# ---------------------------------------------------------------------------
@dataclass
class SelfMotionFibre:
    """Every configuration behind one tool pose, as a function of swivel.

    Built once per pose (:meth:`SrsArm.fibre`) and then evaluated over a whole
    swivel grid at once — the atlas asks for 72 or more angles per pose, and
    everything except the swivel rotation itself is constant across them.

    The four branch slots are ``(shoulder branch, wrist branch)`` in the order
    ``(0,0), (0,1), (1,0), (1,1)``. Slot index is stable across swivel angles,
    which is what lets a corridor search require that a motion stays on one
    branch instead of teleporting between disjoint manifolds.
    """

    arm: "SrsArm"
    pole: float
    seed_q: np.ndarray
    psi0: float                     #: swivel angle of the seed
    axis: np.ndarray                #: unit shoulder -> wrist centre
    shoulder: np.ndarray
    wrist: np.ndarray
    elbow_seed: np.ndarray
    q4: float                       #: elbow angle — a property of the pose, not of psi
    seed_error_m: float = 0.0       #: |FK(seed) - requested target|, reported not absorbed
    #: Unit MODEL-frame swivel reference psi is measured against; None = legacy
    #: per-sample fallback (deprecated).
    ref: Optional[np.ndarray] = None
    _R3: np.ndarray = field(repr=False, default=None)
    _R4: np.ndarray = field(repr=False, default=None)
    _C: np.ndarray = field(repr=False, default=None)

    N_BRANCHES = 4

    @property
    def wrist_distance_m(self) -> float:
        """Shoulder to wrist centre — the only thing the elbow angle depends on."""
        return float(np.linalg.norm(self.wrist - self.shoulder))

    @property
    def elbow_orbit_radius_m(self) -> float:
        """How far the swivel actually moves the elbow at this pose."""
        return float(self.arm.elbow_orbit_radius(self.q4))

    def orbit(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """The elbow's circle, as ``(centre, A, B)`` with ``elbow(psi) = centre
        + cos(psi) A + sin(psi) B``.

        This is the swivel coordinate made physical, and it is what a viewer can
        actually be shown: psi is an abstract axis, but the elbow tracing a ring
        between the shoulder and the wrist is the mechanism itself. The wall
        does not block the contact point — it blocks an ARC of this ring.

        ``A`` and ``B`` are orthogonal and both of length ``a·sin(alpha)``, the
        orbit radius, which collapses to zero as the elbow straightens. The
        reference direction is the same one :meth:`SrsArm.swivel_from_points`
        measured :attr:`psi0` from (this fibre's :attr:`ref`), so a point at
        parameter psi on this ring is the elbow of the configuration this fibre
        returns at that same psi. Verified against forward kinematics to 3 um.
        """
        a, b = self.arm.upper_arm_m, self.arm.forearm_m
        d = self.wrist_distance_m
        if d < 1e-9:
            z = np.zeros(3)
            return self.shoulder.copy(), z, z.copy()
        cos_alpha = np.clip((d * d + a * a - b * b) / (2.0 * d * a), -1.0, 1.0)
        alpha = float(np.arccos(cos_alpha))
        centre = self.shoulder + (a * np.cos(alpha)) * self.axis
        radius = a * np.sin(alpha)

        u, v = self.arm._swivel_basis(self.axis, self.ref)
        return centre, radius * u, radius * v

    @classmethod
    def build(cls, arm: "SrsArm", T_tip: np.ndarray, seed_q: np.ndarray,
              pole: float, ref: Optional[np.ndarray] = None) -> Optional["SelfMotionFibre"]:
        """Anchor the fibre on the pose the seed ACTUALLY holds.

        A numerical seed converges to a tolerance — ``IKSolver.solve`` defaults
        to 2 mm — so the requested target and ``FK(seed)`` are not the same pose.
        Taking the shoulder from the seed and the wrist from the request mixes
        the two, and every configuration on the fibre then inherits the seed's
        residual as a pose error of its own. The fibre is built on ``FK(seed)``
        instead, which makes it exactly self-consistent, and the distance to the
        requested target is recorded as :attr:`seed_error_m` for the caller to
        judge rather than absorb.

        ``ref`` is the stroke's MODEL-frame swivel reference; see
        :meth:`SrsArm.swivel_from_points` for the frame requirement and what
        ``None`` still means.
        """
        f = arm.chain_frames(seed_q, pole)
        p_s = f[arm.shoulder_link][:3, 3]
        p_e = f[arm.elbow_link][:3, 3]
        p_w = f[arm.wrist_link][:3, 3]
        axis = p_w - p_s
        n = float(np.linalg.norm(axis))
        if n < 1e-9:
            return None
        axis = axis / n
        psi0 = arm.swivel_from_points(p_s, p_e, p_w, ref)      # validates ref
        if not np.isfinite(psi0):
            return None
        if ref is not None and not isinstance(ref, StereographicSwivel):
            ref = np.asarray(ref, dtype=float).reshape(3)
            ref = ref / np.linalg.norm(ref)
        T_held = f[arm.tip_link]
        err = float(np.linalg.norm(T_held[:3, 3] - np.asarray(T_tip, dtype=float)[:3, 3]))
        T_flange = f[arm.flange_link]
        return cls(
            arm=arm, pole=float(pole), seed_q=np.asarray(seed_q, dtype=float).copy(),
            psi0=float(psi0), axis=axis, shoulder=p_s.copy(), wrist=p_w.copy(),
            elbow_seed=p_e.copy(), q4=float(seed_q[3]), seed_error_m=err, ref=ref,
            _R3=f[arm.joints[2].child][:3, :3].copy(),
            _R4=f[arm.elbow_link][:3, :3].copy(),
            _C=T_flange[:3, :3] @ arm._R_a57.T,
        )

    # ------------------------------------------------------------------
    def at(self, psi: Sequence[float], respect_limits: bool = True) -> np.ndarray:
        """``(len(psi), 4, 7)`` joint values; NaN where that branch is out of limits.

        The pose is held exactly: only the arm plane turns. Position fidelity
        against the URDF's own FK is float precision (it was ~5 um until the
        description's `1.5708` right angles became `${pi/2}`) — see the module
        docstring.
        """
        psi = np.asarray(psi, dtype=float).reshape(-1)
        d_psi = np.arctan2(np.sin(psi - self.psi0), np.cos(psi - self.psi0))
        R_d = Rotation.from_rotvec(self.axis[None, :] * d_psi[:, None]).as_matrix()

        R3n = np.einsum("nij,jk->nik", R_d, self._R3)
        R4n = np.einsum("nij,jk->nik", R_d, self._R4)
        M = np.einsum("ij,njk,lk->nil", self.arm._R_base.T, R3n, self.arm._R_a3)
        N = np.einsum("nji,jk->nik", R4n, self._C)

        eul_s = Rotation.from_matrix(M).as_euler("ZYZ")
        eul_w = Rotation.from_matrix(N).as_euler("ZYZ")
        shoulder = _euler_branches(eul_s)      # (n, 2, 3)
        wrist = _euler_branches(eul_w)

        n = len(psi)
        Q = np.empty((n, self.N_BRANCHES, 7), dtype=float)
        for bi, (si, wi) in enumerate(((0, 0), (0, 1), (1, 0), (1, 1))):
            Q[:, bi, 0:3] = shoulder[:, si, :]
            Q[:, bi, 3] = self.q4
            Q[:, bi, 4:7] = wrist[:, wi, :]
        Q = self.arm.wrap_into_limits(Q)
        if respect_limits:
            bad = np.any((Q < self.arm.lower - 1e-9) | (Q > self.arm.upper + 1e-9), axis=-1)
            Q[bad] = np.nan
        return Q

    def limit_margins(self, Q: np.ndarray) -> np.ndarray:
        """Per-configuration distance to the nearest joint limit, in radians."""
        lo, hi = self.arm.lower, self.arm.upper
        return np.min(np.minimum(Q - lo, hi - Q), axis=-1)


def _euler_branches(eul: np.ndarray) -> np.ndarray:
    """``(n, 3)`` ZYZ triples -> ``(n, 2, 3)``, both branches. See :func:`zyz_branches`."""
    a, b, c = eul[:, 0], eul[:, 1], eul[:, 2]
    a2 = np.where(a < 0.0, a + np.pi, a - np.pi)
    c2 = np.where(c < 0.0, c + np.pi, c - np.pi)
    out = np.empty((len(eul), 2, 3), dtype=float)
    out[:, 0, :] = eul
    out[:, 1, 0], out[:, 1, 1], out[:, 1, 2] = a2, -b, c2
    return out
