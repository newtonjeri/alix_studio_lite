"""Seedless closed-form inverse kinematics for the S-R-S arm (plan/P15 step 2).

``solve(arm, T_tip, psi, pole, ref)`` returns the same ``(4, 7)`` block that
``arm.fibre(T_tip, seed_q, pole, ref).at([psi])[0]`` returns -- same branch
order, same psi convention, same NaN pattern -- but needs no seed and runs no
iteration.  Pure geometry and trigonometry.

WHY IT IS POSSIBLE
------------------
``SelfMotionFibre.at`` (srs.py:1054) needs exactly four things out of the seed:

    fib._R3   rotation of link 3 (after q3)        -> M(d) -> (q1, q2, q3)
    fib._R4   rotation of link 4 (after q4)        -> N(d) -> (q5, q6, q7)
    fib._C    R_flange . R_a57^T                   -> N(d)
    fib.q4    the elbow angle                      -> q4
    fib.psi0  the swivel angle of the seed         -> d = wrap(psi - psi0)

``_C`` is a function of the TOOL POSE alone (``R_flange = R_tip . R_ft^T`` with
``R_ft = arm._T_flange_tip[:3,:3]``), so it never needed a seed.  ``q4`` is a
function of the shoulder-to-wrist DISTANCE alone (srs.py:24-26) up to its sign.
And ``R_d . _R3`` / ``R_d . _R4`` are just the link-3 / link-4 rotations OF THE
CONFIGURATION AT psi -- which this module builds from the three joint centres
directly, so ``psi0`` and the rotation ``R_d`` never appear.

THE CONSTRUCTION
----------------
Measured off the URDF at ``q = 0`` (and re-verified on every call to
:func:`constants`, which REFUSES a chain that does not satisfy them):

    p_e = p_s + R3 . (a v)          v  = (0, -1, 0)  in the link-3 frame
    p_w = p_e + R4 . (b w)          w  = (0,  0, 1)  in the link-4 frame
    R4  = R3 . Rot(n3, q4) . A0     n3 = (0, 0, 1)   joint-4 axis, link-3 frame
    A0 . w = v                      i.e. the arm is STRAIGHT at q4 = 0

    n3 . v = 0   (the elbow axis is normal to the upper arm)

From the last three: with ``e = (p_e - p_s)/a`` and ``f = (p_w - p_e)/b``,

    f = Rot(n, gamma) e        n = R3 n3,   gamma = q4 + q40   (q40 = 0 here)

so ``|gamma| = arccos((d^2 - a^2 - b^2) / 2ab) = arm.elbow_from_distance(d)``
and ``n = sign(gamma) . unit(e x f)``: the ARM-PLANE NORMAL, whose sign is the
only thing the elbow branch is.  R3 is then pinned completely, because it maps
one orthonormal frame onto another:

    R3 = [e, n, e x n] . [v, n3, v x n3]^T

p_e itself comes from the elbow orbit (srs.py:979), whose parameter IS psi:

    p_e(psi) = p_s + a cos(alpha) . axis + a sin(alpha) . (cos psi u + sin psi v)

with ``(u, v) = arm._swivel_basis(axis, ref)`` -- the SAME function
``swivel_from_points`` measures psi with, so the convention (including the
``StereographicSwivel`` case and its ill-conditioning refusal) is inherited
rather than re-derived.  ``cos alpha = (d^2 + a^2 - b^2) / 2da``.

Both ``e`` and ``n`` rotate rigidly with p_e about the shoulder-wrist line, so
``R3(psi) = Rot(axis, psi - psi0) . R3(psi0)``: this construction and ``at()``
parameterise the same one-parameter family the same way, which is why the
agreement is exact rather than approximate.

EIGHT CONFIGURATIONS, NOT FOUR
------------------------------
``at()`` inherits the seed's elbow sign and therefore reports four branches.
The arm-plane normal can point either way, giving a SECOND R3
(``R3' = R3 . Rot(v, pi)``, elbow angle ``-gamma``) that holds the same pose at
the same psi with the same three joint centres.  It is not reachable from the
first by any swivel -- the sign of q4 cannot change without passing through the
straight-elbow singularity -- so it is a genuinely separate family and joint 4's
limits on the RM75 are symmetric (+-135 deg), which means it is usually legal.
:func:`solve` takes ``elbow_sign`` (default +1) and :func:`solve_all` returns
all eight rows.  This module does NOT merge them: reporting eight rows from a
function documented to return four would be the kind of quiet disagreement the
verification exists to catch.

MEASURED (2026-09-20, `verify.py` beside this file)
---------------------------------------------------
AGREEMENT.  90 real `toplid` right / R_glove_frame_2 poses -- recorded by
wrapping `ArmOracle.seed` for a whole `coverable` stage, so they are the tilt
grid's own poses, probe standoff included -- at 24 swivel angles each, against
the stroke's real stereographic coordinate: 2160 comparisons, max per-joint
disagreement with `at()` 1.9e-10 rad, 0 branch mismatches, 0 disagreements
about which branches are NaN.  At the seed's own psi the closed form hands
back the numerical seed itself, 89/89 (the 90th seed is outside the joint
limits, so no branch may return it), worst 1.0e-10 rad.

Plus 400 poses from random IN-LIMIT configurations across four swivel
conventions (model Z, model X, two stereographic placements) -- the real region
seeds only ever land on q4 > 0, so this is what exercises the negative elbow
family (199 / 201 split): max disagreement 5.1e-13 rad, 0 mismatches.

The 1.9e-10 on the real poses sits entirely in q1 and q3 (q2 is 6e-12): it is
the ZYZ azimuth pair losing digits as the shoulder nears gimbal, not a wrong
configuration -- the FK round trip on those same rows is 4.4e-13 m.

FK ROUND TRIP.  15282 returned configurations through `arm.tool_pose`, against
the COMMANDED pose: worst 4.4e-13 m and 4.7e-11 deg.

THE noSeed SET -- AND AN HONEST CORRECTION.  Over six `coverable` runs
(`bowl_inside_front` at the default budget and again with the whole tilt grid,
`bowl_inside_ring`, `seat_ring`, `side`, `toplid`): 4968 seed calls, 1087
returned None, 133 distinct poses.  ALL 133 have their wrist centre outside the
[0.046, 0.466] m shell, so NO configuration exists and the solver was right;
the closed form returns all-NaN for all 133, agreeing.  NOT ONE was a pose the
solver merely failed on.  `ArmOracle.seed` short-circuits on `in_ball` before
it spends any iterations, and on 400 poses built from random in-limit
configurations -- every one provably solvable -- it succeeded 400/400.  So this
module recovers no refused node on this fixture, and the premise that `noSeed`
is our solver failing did not reproduce.  What it does buy is a seed-free
0.36 ms per pose against 2.52 ms, exact arcs via `swivel_arcs` with no numerical
anchor, and the second elbow family: on all 90 toplid poses the family the seed
did NOT land on is also feasible, with a median 345 deg per branch of swivel
that the map cannot currently see.

GIMBAL
------
The sibling module ``alix_workspace.swivel_arcs`` has to guard against scipy's
``as_euler("ZYZ")`` abandoning the matrix-entry formulas inside |sin b| < 1e-7
(``swivel_arcs.GIMBAL_GUARD_RAD``).  This module does not need that guard and
deliberately does not carry one: it does not re-derive the Euler angles, it
calls ``Rotation.from_matrix(...).as_euler("ZYZ")`` and ``srs._euler_branches``
-- the very code ``at()`` calls -- on matrices it built itself.  Whatever scipy
does at gimbal, both sides do the same thing to the same matrix.
"""

from __future__ import annotations

from typing import Optional, Sequence, Tuple

import numpy as np
from scipy.spatial.transform import Rotation

from alix_workspace.srs import SelfMotionFibre, SrsArm, StereographicSwivel, _euler_branches

__all__ = ["solve", "solve_all", "fibre", "wrist_centre", "elbow_at", "constants",
           "shoulder_point", "frames_at", "BRANCH_PAIRS", "NotSrsError"]

#: (shoulder Euler branch, wrist Euler branch) per row, the order ``at()`` uses.
BRANCH_PAIRS = ((0, 0), (0, 1), (1, 0), (1, 1))

#: Tolerance on the structural facts :func:`constants` verifies (rad / m).
STRUCT_TOL = 1e-9

#: Orbit radius below which psi has no meaning (the elbow is on the
#: shoulder-wrist line: the straight-elbow singularity).  Metres.
ORBIT_DEGENERATE_M = 1e-12

#: Module-private caches, keyed by the arm's identity and holding a reference to
#: it so the key cannot be recycled under us.  These used to be attributes
#: written onto the caller's `SrsArm`; a shared arm object is handed to several
#: oracles at different poles in one process, and a module has no business
#: mutating a caller's object to memoise its own arithmetic.
_CONST_CACHE: dict = {}
_SHOULDER_CACHE: dict = {}

#: Same slack ``SelfMotionFibre.at`` applies to the joint-limit test.
LIMIT_TOL = 1e-9


class NotSrsError(ValueError):
    """The chain does not have the structure this construction is built on."""


# ---------------------------------------------------------------------------
# constants of the chain (rotation-only; the pole is a translation)
# ---------------------------------------------------------------------------
class _Const:
    __slots__ = ("B3", "A0", "q40", "R_ft", "R_base_T", "R_a3_T", "a", "b",
                 "lower", "upper")


def constants(arm: SrsArm) -> "_Const":
    """The chain constants the construction needs, verified, cached on *arm*.

    Raises :class:`NotSrsError` if any structural fact this module relies on
    fails -- the same posture ``SrsArm.from_model`` takes toward a chain that is
    not S-R-S.  Nothing here depends on the pole: the pole joint is prismatic,
    so it translates the shoulder and leaves every rotation alone (asserted).
    """
    cached = _CONST_CACHE.get(id(arm))
    if cached is not None and cached[0] is arm:
        return cached[1]

    Z = np.zeros(7)
    f0 = arm.chain_frames(Z, 0.0)
    R3h = f0[arm.joints[2].child][:3, :3]
    R4h = f0[arm.elbow_link][:3, :3]
    p_s0 = f0[arm.shoulder_link][:3, 3]
    p_e0 = f0[arm.elbow_link][:3, 3]
    p_w0 = f0[arm.wrist_link][:3, 3]
    a, b = float(arm.upper_arm_m), float(arm.forearm_m)

    v = R3h.T @ ((p_e0 - p_s0) / a)                 # upper arm, link-3 frame
    A0 = R3h.T @ R4h                                # link3 -> link4 at q4 = 0
    n3 = A0 @ np.asarray(arm.joints[3].axis, dtype=float)   # elbow axis, link-3
    n3 = n3 / float(np.linalg.norm(n3))
    w = R4h.T @ ((p_w0 - p_e0) / b)                 # forearm, link-4 frame
    f30 = A0 @ w                                    # forearm at q4 = 0, link-3

    if abs(float(v @ n3)) > STRUCT_TOL:
        raise NotSrsError(
            f"the joint-4 axis is not normal to the upper arm (cos = {v @ n3:.3e}); "
            "the elbow is not a planar two-link and this construction does not apply")
    # forearm direction at q4 = 0, as an angle about the elbow axis from v
    if abs(float(f30 @ n3)) > STRUCT_TOL:
        raise NotSrsError(
            f"the forearm is not normal to the joint-4 axis (cos = {f30 @ n3:.3e})")
    q40 = float(np.arctan2(float(np.cross(v, f30) @ n3), float(v @ f30)))

    # the pole must not rotate anything
    if arm.pole_joint:
        f1 = arm.chain_frames(Z, 0.137)
        for link in (arm.joints[2].child, arm.elbow_link, arm.tip_link):
            if not np.allclose(f0[link][:3, :3], f1[link][:3, :3], atol=1e-12):
                raise NotSrsError(f"the pole joint rotates {link}; it must only translate")

    B3 = np.column_stack([v, n3, np.cross(v, n3)])
    if not np.allclose(B3.T @ B3, np.eye(3), atol=1e-9) or float(np.linalg.det(B3)) < 0.0:
        raise NotSrsError("the link-3 reference frame is not right-handed orthonormal")

    c = _Const()
    c.B3 = B3
    c.A0 = A0
    c.q40 = q40
    c.R_ft = np.asarray(arm._T_flange_tip, dtype=float)[:3, :3]
    c.R_base_T = np.asarray(arm._R_base, dtype=float).T
    c.R_a3_T = np.asarray(arm._R_a3, dtype=float).T
    c.a, c.b = a, b
    c.lower, c.upper = arm.lower, arm.upper
    _CONST_CACHE[id(arm)] = (arm, c)
    return c


def shoulder_point(arm: SrsArm, pole: float = 0.0) -> np.ndarray:
    """Shoulder centre in ``base_link`` coordinates (a function of the pole only).

    Memoised per pole: the whole solve is a handful of 3x3 products, so an
    unmemoised chain walk would be most of its cost.
    """
    cache = _SHOULDER_CACHE.setdefault(id(arm), (arm, {}))[1]
    key = round(float(pole), 12)
    p = cache.get(key)
    if p is None:
        p = arm.chain_frames(np.zeros(7), float(pole))[arm.shoulder_link][:3, 3].copy()
        cache[key] = p
    return p


def wrist_centre(arm: SrsArm, T_tip: np.ndarray) -> np.ndarray:
    """``arm.wrist_centre_of_pose``, re-exported so callers need one import."""
    return arm.wrist_centre_of_pose(T_tip)


# ---------------------------------------------------------------------------
# the elbow orbit, in the ref convention psi is measured in
# ---------------------------------------------------------------------------
def _orbit(arm: SrsArm, p_s: np.ndarray, p_w: np.ndarray, ref
           ) -> Optional[Tuple[np.ndarray, np.ndarray, float, np.ndarray, np.ndarray, float]]:
    """``(axis, centre, radius, u, v, d)`` or None when no triangle closes.

    Identical arithmetic to ``SelfMotionFibre.orbit`` (srs.py:979), including
    the ``_swivel_basis`` call that fixes psi's zero -- and its refusal.
    """
    a, b = float(arm.upper_arm_m), float(arm.forearm_m)
    ax = p_w - p_s
    d = float(np.linalg.norm(ax))
    if d < 1e-9:
        return None
    axis = ax / d
    if d > a + b + 1e-12 or d < abs(a - b) - 1e-12:
        return None
    cos_alpha = float(np.clip((d * d + a * a - b * b) / (2.0 * d * a), -1.0, 1.0))
    sin_alpha = float(np.sqrt(max(0.0, 1.0 - cos_alpha * cos_alpha)))
    centre = p_s + (a * cos_alpha) * axis
    u, vv = arm._swivel_basis(axis, ref)          # raises on an ill-conditioned ref
    return axis, centre, a * sin_alpha, u, vv, d


def elbow_at(arm: SrsArm, T_tip: np.ndarray, psi: float, pole: float = 0.0,
             ref=None) -> Optional[np.ndarray]:
    """The elbow centre at swivel ``psi`` -- no IK, no seed."""
    p_s = shoulder_point(arm, pole)
    p_w = arm.wrist_centre_of_pose(T_tip)
    o = _orbit(arm, p_s, p_w, ref)
    if o is None:
        return None
    axis, centre, radius, u, v, d = o
    return centre + radius * (np.cos(psi) * u + np.sin(psi) * v)


# ---------------------------------------------------------------------------
# the solver
# ---------------------------------------------------------------------------
def _rot_about(n: np.ndarray, ang: float) -> np.ndarray:
    """Rodrigues, for a unit axis."""
    K = np.array([[0.0, -n[2], n[1]], [n[2], 0.0, -n[0]], [-n[1], n[0], 0.0]])
    return np.eye(3) + np.sin(ang) * K + (1.0 - np.cos(ang)) * (K @ K)


def frames_at(arm: SrsArm, T_tip: np.ndarray, psi: float, pole: float = 0.0,
              ref=None, elbow_sign: int = 1):
    """``(R3, R4, gamma, q4, p_s, p_e, p_w)`` at one swivel angle, or None.

    ``R3`` and ``R4`` are the rotations of link 3 and link 4 -- exactly what
    ``SelfMotionFibre`` caches as ``_R3`` / ``_R4`` (already turned by ``R_d``).
    ``gamma`` is the geometric elbow angle; ``q4 = gamma - q40``.
    """
    c = constants(arm)
    T_tip = np.asarray(T_tip, dtype=float)
    p_s = shoulder_point(arm, pole)
    p_w = arm.wrist_centre_of_pose(T_tip)
    o = _orbit(arm, p_s, p_w, ref)
    if o is None:
        return None
    axis, centre, radius, u, v, d = o
    if radius <= ORBIT_DEGENERATE_M:
        return None                      # straight elbow: psi carries no information
    if not np.isfinite(psi):
        return None                      # garbage in: refuse, never raise
    r_unit = np.cos(psi) * u + np.sin(psi) * v
    p_e = centre + radius * r_unit

    cos_alpha = float(np.clip((d * d + c.a * c.a - c.b * c.b) / (2.0 * d * c.a), -1.0, 1.0))
    sin_alpha = radius / c.a
    e = cos_alpha * axis + sin_alpha * r_unit            # exactly unit
    s = 1.0 if elbow_sign >= 0 else -1.0
    n = -s * np.cross(axis, r_unit)                      # exactly unit, normal to e
    # gamma BY ATAN2, not arccos.  The cosine form is sqrt-singular as the
    # elbow straightens: d -> a + b makes the argument -> 1, where arccos has
    # infinite slope, so a 1e-16 error in d becomes ~1e-8 in gamma and the
    # returned arm is straight at an arbitrary psi (measured: 6.9 nm of FK
    # error at the exact straight-elbow point).  The half-angle triangle gives
    # both components, so the angle comes from a well-conditioned atan2:
    #   |p_w - p_e| = b, |p_e - p_s| = a, and the angle at the elbow between
    #   (p_s - p_e) and (p_w - p_e) is pi - gamma.
    _es = p_s - p_e
    _ew = p_w - p_e
    _cos = float(np.dot(_es, _ew))
    _sin = float(np.linalg.norm(np.cross(_es, _ew)))
    gamma = s * float(np.pi - np.arctan2(_sin, _cos))

    Bw = np.column_stack([e, n, np.cross(e, n)])
    R3 = Bw @ c.B3.T
    q4 = gamma - c.q40
    R4 = _rot_about(n, q4) @ R3 @ c.A0
    return R3, R4, gamma, q4, p_s, p_e, p_w


def solve(arm: SrsArm, T_tip: np.ndarray, psi: float, pole: float = 0.0,
          ref=None, elbow_sign: int = 1) -> np.ndarray:
    """The four branch configurations holding ``T_tip`` at swivel ``psi``.

    ``(4, 7)``; a row is NaN where that branch violates a joint limit, and the
    whole block is NaN when the pose admits no configuration at all (the wrist
    centre is outside the ``[|a-b|, a+b]`` shell, or the elbow is straight so
    psi is undefined).  Row order is ``BRANCH_PAIRS``.

    ``elbow_sign`` selects the elbow family (the sign of q4); ``at()`` reports
    whichever one its seed happened to land on.  See the module docstring.
    """
    out = np.full((4, 7), np.nan, dtype=float)
    fr = frames_at(arm, T_tip, psi, pole, ref, elbow_sign)
    if fr is None:
        return out
    R3, R4, gamma, q4, p_s, p_e, p_w = fr
    c = constants(arm)

    R_flange = np.asarray(T_tip, dtype=float)[:3, :3] @ c.R_ft.T
    C = R_flange @ np.asarray(arm._R_a57, dtype=float).T

    M = (c.R_base_T @ R3 @ c.R_a3_T)[None, :, :]
    N = (R4.T @ C)[None, :, :]
    shoulder = _euler_branches(Rotation.from_matrix(M).as_euler("ZYZ"))   # (1,2,3)
    wrist = _euler_branches(Rotation.from_matrix(N).as_euler("ZYZ"))

    Q = np.empty((4, 7), dtype=float)
    for bi, (si, wi) in enumerate(BRANCH_PAIRS):
        Q[bi, 0:3] = shoulder[0, si, :]
        Q[bi, 3] = q4
        Q[bi, 4:7] = wrist[0, wi, :]
    Q = arm.wrap_into_limits(Q)
    bad = np.any((Q < c.lower - LIMIT_TOL) | (Q > c.upper + LIMIT_TOL), axis=-1)
    Q[bad] = np.nan
    return Q


def solve_all(arm: SrsArm, T_tip: np.ndarray, psi: float, pole: float = 0.0,
              ref=None) -> np.ndarray:
    """``(8, 7)``: rows 0-3 are ``elbow_sign=+1``, rows 4-7 ``elbow_sign=-1``."""
    return np.vstack([solve(arm, T_tip, psi, pole, ref, +1),
                      solve(arm, T_tip, psi, pole, ref, -1)])


# ---------------------------------------------------------------------------
# a real SelfMotionFibre, built without a seed
# ---------------------------------------------------------------------------
def fibre(arm: SrsArm, T_tip: np.ndarray, pole: float = 0.0, ref=None,
          elbow_sign: int = 1) -> Optional[SelfMotionFibre]:
    """``SrsArm.fibre`` without the seed: a genuine :class:`SelfMotionFibre`.

    Every field the class carries is filled from this module's construction, so
    the object is the real thing -- ``fib.at(psi)``, ``fib.orbit()``, and
    ``alix_workspace.swivel_arcs`` on top of it all work unchanged.  ``psi0`` is
    taken as 0 and the anchor configuration is the one at psi = 0, which is a
    free choice: the fibre is the same set whichever point anchors it.

    Returns None where no configuration exists (wrist centre outside the
    ``[|a-b|, a+b]`` shell) or where the elbow is straight, so psi is undefined.
    """
    fr = frames_at(arm, T_tip, 0.0, pole, ref, elbow_sign)
    if fr is None:
        return None
    R3, R4, gamma, q4, p_s, p_e, p_w = fr
    c = constants(arm)
    T_tip = np.asarray(T_tip, dtype=float)
    C = (T_tip[:3, :3] @ c.R_ft.T) @ np.asarray(arm._R_a57, dtype=float).T
    axis = p_w - p_s
    axis = axis / float(np.linalg.norm(axis))
    r = ref
    if r is not None and not isinstance(r, StereographicSwivel):
        r = np.asarray(r, dtype=float).reshape(3)
        r = r / float(np.linalg.norm(r))
    # `seed_q` is DATA on this object, not an input -- nothing here was seeded.
    # It must still be a configuration that holds the pose, because callers read
    # it as one.  psi = 0 need not be legal, so search the ring rather than
    # handing back a NaN row (measured: 13 of 60 fibres carried one).
    seed_q = None
    for _p in np.linspace(-np.pi, np.pi, 73)[:-1]:
        anchor = solve(arm, T_tip, float(_p), pole, ref, elbow_sign)
        for row in anchor:
            if np.isfinite(row).all():
                seed_q = row
                break
        if seed_q is not None:
            break
    if seed_q is None:
        return None                      # no legal configuration anywhere on the ring
    return SelfMotionFibre(
        arm=arm, pole=float(pole), seed_q=seed_q, psi0=0.0, axis=axis,
        shoulder=p_s.copy(), wrist=p_w.copy(), elbow_seed=p_e.copy(),
        q4=float(q4), seed_error_m=0.0, ref=r,
        _R3=R3.copy(), _R4=R4.copy(), _C=C)
