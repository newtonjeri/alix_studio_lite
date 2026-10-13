"""Closed-form feasible swivel arcs for an S-R-S arm under joint limits.

The exact set of arm angles at which a held pose is legal, as a union of ARCS
rather than a sampled mask.  This is `plan/P15` step 1, and it is what lets the
generator stop paying a damped-least-squares solve per candidate pose.

WHAT THIS REPLACES.  Stage 1 samples psi on a 5 degree grid
(`cspace_author.DEFAULT_SWIVEL_STEP_DEG`, 72 points) and recovers intervals by
run-length over the boolean mask, so an arc narrower than 5 degrees is invisible
and every boundary is known only to +-2.5 degrees.  Measured on the real
fixture, 104 fibres over `side` and `seat_ring`: of the 1431 arcs this module
finds, 57 are narrower than that grid step and the narrowest is 0.0687 degrees.
Those are not rounding -- they are corridors the map cannot currently see.

HOW IT WAS VERIFIED (2026-09-20).  Three independent implementations were
written from `srs.py` without sight of each other, cross-checked on a region
none of them chose, and attacked adversarially.  On every real pose all three
agree with `SelfMotionFibre.at` exactly -- ~115 million dense comparisons over
`toplid`, `side`, `seat_ring`, `bowl_inside_ring` and `bowl_inside_front`
(the curved bowl, normal spread 24.1 deg median / 89.5 deg max, the hardest
poses in the fixture), zero psi wrongly included and zero wrongly excluded.
The one place all three were WRONG is handled by `GIMBAL_GUARD_RAD` below.

The two error directions are not symmetric and are never reported merged: an
arc that is too LARGE is a correctness failure, because the map would certify a
pose the arm cannot hold; an arc that is too small is merely conservative.

Derived from ``alix_workspace.srs`` itself, not from a paper -- the reference is
Shimizu et al., IEEE T-RO 24(5):1131-1142 (2008), recorded in `docs/sources.md`
under 2026-09-20, but every formula below is checked against what `at()` does.

WHAT THE CODE ACTUALLY DOES
---------------------------
``SelfMotionFibre.at(psi)`` (srs.py:1054) computes, with ``d = wrap(psi - psi0)``
and ``R_d = Rot(axis, d)``::

    M(d) = R_base^T . (R_d . R3) . R_a3^T          # einsum "ij,njk,lk->nil"
    N(d) = (R_d . R4)^T . C                        # einsum "nji,jk->nik"

then ``eul_s = Rotation.from_matrix(M).as_euler("ZYZ")`` -> (q1, q2, q3) and
``eul_w = Rotation.from_matrix(N).as_euler("ZYZ")`` -> (q5, q6, q7), with
``q4 = fib.q4`` constant in psi.

scipy's ``as_euler("ZYZ")`` is INTRINSIC, i.e. ``M = Rz(a) Ry(b) Rz(c)``.
Writing that product out::

    M = [[ ca cb cc - sa sc,  -ca cb sc - sa cc,   ca sb],
         [ sa cb cc + ca sc,  -sa cb sc + ca cc,   sa sb],
         [          -sb cc,             sb sc,       cb]]

so, with scipy's proper-Euler range b in [0, pi] (verified numerically to
3e-14 over 2000 random triples):

    a = atan2(M[1,2], M[0,2])     TANGENT type
    b = acos (M[2,2])             COSINE  type
    c = atan2(M[2,1], -M[2,0])    TANGENT type

``_euler_branches`` (srs.py:1094) emits ``(a +- pi, -b, c +- pi)`` as branch 1.
Adding pi to an atan2 is a sign flip of BOTH arguments, so

    branch 0:  a = atan2( Y,  X)          branch 1:  a = atan2(-Y, -X)
    branch 0:  b = +acos(u)               branch 1:  b = -acos(u)

which reproduces ``a + pi if a < 0 else a - pi`` exactly, including the sign of
the zero at a == 0.  ``at()`` pairs them as ``((0,0),(0,1),(1,0),(1,1))``, so
branch index b maps to ``(shoulder = b // 2, wrist = b % 2)``.

WHY IT IS CLOSED FORM
---------------------
Rodrigues: ``R_d = (I + K^2) + sin(d) K - cos(d) K^2`` with ``K = skew(axis)``.
Both source matrices are therefore AFFINE in (sin d, cos d)::

    M(d) = Mc + sin(d) Ms + cos(d) Mk
    N(d) = Nc + sin(d) Ns + cos(d) Nk

    Mc = P (I+K^2) Q,  Ms = P K Q,      Mk = -P K^2 Q      P = R_base^T, Q = R3 R_a3^T
    Nc = R4^T (I+K^2) C, Ns = -R4^T K C, Nk = -R4^T K^2 C   (R_d^T flips only the K term)

Every joint-limit crossing is then a root of ``p sin d + q cos d + r = 0``:

  * cosine joints (q2, q6):  ``acos(u) = alpha``  <=>  ``u(d) = cos(alpha)``
  * tangent joints (q1,q3,q5,q7): ``atan2(Y,X) = alpha`` implies the collinearity
    ``Y(d) cos(alpha) - X(d) sin(alpha) = 0`` (a superset of the crossings --
    it also contains the antipode, which only over-splits and never misses)

Harmonic addition gives at most two roots each.  The branch sign flips both
arguments of the atan2 and negates the acos, and neither changes those
equations, so ONE breakpoint set serves all four branches.

THE THIRD FAMILY OF BREAKPOINTS (easy to miss)
----------------------------------------------
``atan2(Y, X)`` is discontinuous where ``(Y, X) = (0, 0)``, which for the
shoulder is ``M[0,2] = M[1,2] = 0``, i.e. ``sin b = 0``, i.e. ``|M[2,2]| = 1``:
Euler gimbal lock.  b = acos has a CUSP there (it behaves like |d - d0|) and a,
c generically JUMP BY PI across it, with no limit equation having a root.  So
``M[2,2] = +-1`` and ``N[2,2] = +-1`` are breakpoints in their own right.  They
are tangencies, hence numerically double roots, and are found with a tolerance.

WRAPPING
--------
``wrap_into_limits`` (srs.py:820) tries k in (0, -1, +1, -2, +2) turns before the
limit test, so the admissible set of the Euler REPRESENTATIVE theta in (-pi, pi]
is the circle set ``union_k ([lo, hi] - 2 pi k)``.  For the RM75 that matters:
joint 7 is +-360 deg = +-2pi, so its admissible set is the WHOLE circle and it
never constrains anything; joints 1/3/5 are +-178 deg, leaving a 4 deg forbidden
band straddling the atan2 branch cut that a naive implementation gets wrong.

METHOD
------
1. Exact affine coefficients from the fibre's own cached matrices.
2. Exact roots of every limit equation + every gimbal equation -> breakpoints.
3. The breakpoints cut the psi circle into cells with NO crossing inside, so the
   feasibility of a cell is decided by evaluating the closed form ONCE at the
   cell's midpoint.  That is a decision procedure over an exact cell complex,
   not a search: no grid, no step size, and the arc endpoints returned are the
   exact roots themselves, not sample positions.
4. Merge feasible cells, rotate d -> psi = wrap(d + psi0).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import numpy as np

TWO_PI = 2.0 * np.pi

#: Matches the tolerance ``SelfMotionFibre.at`` applies to the limit test.
LIMIT_TOL = 1e-9

#: How close the shoulder or wrist ZYZ MIDDLE ANGLE may come to 0 or pi before
#: this module stops trusting its own closed form and defers to `fib.at()`.
#:
#: WHY THIS GUARD EXISTS (measured 2026-09-20, three independent implementations
#: of this same closed form, all three wrong here). The arcs are exact because
#: every joint-limit crossing is a harmonic root -- but that argument assumes the
#: joint angles are the matrix-entry ZYZ formulas. At gimbal lock they are not:
#: scipy's `Rotation.as_euler("ZYZ")`, which `SelfMotionFibre.at` uses, stops
#: using those formulas inside |sin b| < 1e-7 and folds the first and third
#: angles together. So `at()` and the closed form describe DIFFERENT joint
#: triples there, the breakpoint set misses a real transition, and one cell's
#: midpoint then decides a whole cell wrongly -- in the DANGEROUS direction.
#:
#: Measured on the reproducing poses: with the elbow straight to ~1e-9 rad AND
#: the shoulder at the 1e-7 threshold, the three implementations certify 41.7,
#: 41.7 and 20.8 degrees of psi that `at()` refuses outright; on a shoulder
#: exactly singular, 134.8, 134.8 and 42.8 degrees. Either degeneracy ALONE is
#: harmless (measured 0.0009 deg, pure quantisation) -- it is the coincidence
#: that breaks it.
#:
#: 1e-5 rad is 100x scipy's own 1e-7 threshold, so the guard fires well before
#: the disagreement can begin. No pose on any commode_c region reaches it:
#: bowl_inside_front, bowl_inside_ring, seat_ring, side and toplid together give
#: 0 of 170 fibres flagged.
GIMBAL_GUARD_RAD = 1e-5

#: Step of the honest fallback when the guard fires, radians. Only ever used on
#: a fibre the closed form has disowned, so cost is irrelevant.
GUARD_FALLBACK_STEP_RAD = 1e-5

#: Turns ``SrsArm.wrap_into_limits`` is willing to add, in its own order.
WRAP_TURNS = (0, -1, 1, -2, 2)

#: Sentinel for "the whole circle".
FULL: object = "FULL"

_ARC_EPS = 1e-12


# ---------------------------------------------------------------------------
# circle-arc algebra
# ---------------------------------------------------------------------------
def wrap_pi(x):
    """Into [-pi, pi)."""
    return (np.asarray(x, dtype=float) + np.pi) % TWO_PI - np.pi


def _norm_arcs(arcs: Sequence[Tuple[float, float]], tol: float = _ARC_EPS):
    """Merge (start, end) pairs (end >= start) into canonical circle arcs.

    Returns ``FULL``, or a sorted list of disjoint ``(start, end)`` with
    ``start`` in [-pi, pi) and ``end`` in (start, start + 2 pi].
    """
    segs: List[List[float]] = []
    for a, b in arcs:
        if b - a <= 0.0:
            continue
        if b - a >= TWO_PI - _ARC_EPS:
            return FULL
        s = float(wrap_pi(a))
        e = s + (b - a)
        if e > np.pi:
            segs.append([s, np.pi])
            segs.append([-np.pi, e - TWO_PI])
        else:
            segs.append([s, e])
    if not segs:
        return []
    segs.sort()
    merged: List[List[float]] = [segs[0]]
    for s, e in segs[1:]:
        if s <= merged[-1][1] + tol:
            merged[-1][1] = max(merged[-1][1], e)
        else:
            merged.append([s, e])
    if len(merged) == 1 and merged[0][1] - merged[0][0] >= TWO_PI - 1e-9:
        return FULL
    if len(merged) > 1 and merged[0][0] <= -np.pi + tol and merged[-1][1] >= np.pi - tol:
        first = merged.pop(0)
        last = merged.pop(-1)
        merged.append([last[0], first[1] + TWO_PI])
        merged.sort()
        if len(merged) == 1 and merged[0][1] - merged[0][0] >= TWO_PI - 1e-9:
            return FULL
    return [(float(s), float(e)) for s, e in merged]


def _arc_endpoints(arcs) -> List[float]:
    """The wrapped boundary angles of a circle-arc set (empty when FULL)."""
    if arcs is FULL or not arcs:
        return []
    out: List[float] = []
    for s, e in arcs:
        out.append(float(wrap_pi(s)))
        out.append(float(wrap_pi(e)))
    return out


# ---------------------------------------------------------------------------
# joint limits -> admissible circle set for the Euler representative
# ---------------------------------------------------------------------------
def admissible_set(lo: float, hi: float, tol: float = LIMIT_TOL):
    """Circle set of Euler representatives that ``wrap_into_limits`` accepts.

    ``theta`` in (-pi, pi] passes iff ``theta + 2 pi k`` lands in [lo, hi] for
    some ``k`` the folding tries.
    """
    lo = float(lo) - tol
    hi = float(hi) + tol
    if hi - lo >= TWO_PI:
        return FULL
    arcs = []
    for k in WRAP_TURNS:
        a = max(lo - TWO_PI * k, -np.pi)
        b = min(hi - TWO_PI * k, np.pi)
        if b > a:
            arcs.append((a, b))
    return _norm_arcs(arcs)


def in_limits_scalar(theta: float, lo: float, hi: float, tol: float = LIMIT_TOL) -> bool:
    """Exactly the accept/reject ``wrap_into_limits`` + ``at``'s mask perform."""
    for k in WRAP_TURNS:
        v = theta + TWO_PI * k
        if lo - tol <= v <= hi + tol:
            return True
    return False


# ---------------------------------------------------------------------------
# affine coefficients
# ---------------------------------------------------------------------------
@dataclass
class FibreCoefficients:
    """``M(d) = Mc + sin d . Ms + cos d . Mk``; same for N.  Plus limits."""

    Mc: np.ndarray
    Ms: np.ndarray
    Mk: np.ndarray
    Nc: np.ndarray
    Ns: np.ndarray
    Nk: np.ndarray
    q4: float
    psi0: float
    lower: np.ndarray
    upper: np.ndarray
    axis: np.ndarray

    def M(self, d: float) -> np.ndarray:
        return self.Mc + np.sin(d) * self.Ms + np.cos(d) * self.Mk

    def N(self, d: float) -> np.ndarray:
        return self.Nc + np.sin(d) * self.Ns + np.cos(d) * self.Nk


def _skew(a: np.ndarray) -> np.ndarray:
    return np.array([[0.0, -a[2], a[1]],
                     [a[2], 0.0, -a[0]],
                     [-a[1], a[0], 0.0]])


def coefficients(fib, arm=None) -> FibreCoefficients:
    """Exact affine expansion of the two matrices ``at()`` decomposes."""
    arm = arm if arm is not None else fib.arm
    K = _skew(np.asarray(fib.axis, dtype=float))
    K2 = K @ K
    I3 = np.eye(3)

    P = np.asarray(arm._R_base, dtype=float).T
    Q = np.asarray(fib._R3, dtype=float) @ np.asarray(arm._R_a3, dtype=float).T
    Mc = P @ (I3 + K2) @ Q
    Ms = P @ K @ Q
    Mk = -(P @ K2 @ Q)

    R4T = np.asarray(fib._R4, dtype=float).T
    C = np.asarray(fib._C, dtype=float)
    Nc = R4T @ (I3 + K2) @ C
    Ns = -(R4T @ K @ C)
    Nk = -(R4T @ K2 @ C)

    return FibreCoefficients(
        Mc=Mc, Ms=Ms, Mk=Mk, Nc=Nc, Ns=Ns, Nk=Nk,
        q4=float(fib.q4), psi0=float(fib.psi0),
        lower=np.asarray(arm.lower, dtype=float),
        upper=np.asarray(arm.upper, dtype=float),
        axis=np.asarray(fib.axis, dtype=float),
    )


# ---------------------------------------------------------------------------
# closed-form joint angles (mirrors at(), branch for branch)
# ---------------------------------------------------------------------------
def _zyz_from(A: np.ndarray, flip: bool) -> Tuple[float, float, float]:
    """(a, b, c) of ``A = Rz(a) Ry(b) Rz(c)``; ``flip`` selects Euler branch 1."""
    if flip:
        a = float(np.arctan2(-A[1, 2], -A[0, 2]))
        b = -float(np.arccos(min(1.0, max(-1.0, A[2, 2]))))
        c = float(np.arctan2(-A[2, 1], A[2, 0]))
    else:
        a = float(np.arctan2(A[1, 2], A[0, 2]))
        b = float(np.arccos(min(1.0, max(-1.0, A[2, 2]))))
        c = float(np.arctan2(A[2, 1], -A[2, 0]))
    return a, b, c


def joint_angles(coef: FibreCoefficients, d: float, branch: int) -> np.ndarray:
    """The 7 Euler representatives at ``d``, before limit folding."""
    si, wi = branch // 2, branch % 2
    q1, q2, q3 = _zyz_from(coef.M(d), bool(si))
    q5, q6, q7 = _zyz_from(coef.N(d), bool(wi))
    return np.array([q1, q2, q3, coef.q4, q5, q6, q7], dtype=float)


def _feasible_at(coef: FibreCoefficients, d: float, branch: int) -> bool:
    q = joint_angles(coef, d, branch)
    for j in range(7):
        if not in_limits_scalar(q[j], coef.lower[j], coef.upper[j]):
            return False
    return True


# ---------------------------------------------------------------------------
# roots of p sin d + q cos d + r = 0
# ---------------------------------------------------------------------------
def harmonic_roots(p: float, q: float, r: float,
                   tangency_tol: float = 1e-9) -> List[float]:
    """Every d in [-pi, pi) solving ``p sin d + q cos d + r = 0``.

    ``tangency_tol`` admits the double root of a grazing equation (used for the
    gimbal equations ``M[2,2] = +-1``, which touch rather than cross).
    """
    R = float(np.hypot(p, q))
    if R <= 1e-15:
        return []
    s = -float(r) / R
    if s > 1.0:
        if s - 1.0 > tangency_tol:
            return []
        s = 1.0
    elif s < -1.0:
        if -1.0 - s > tangency_tol:
            return []
        s = -1.0
    phi = float(np.arctan2(q, p))          # p sin d + q cos d = R sin(d + phi)
    t = float(np.arcsin(s))
    return [float(wrap_pi(t - phi)), float(wrap_pi(np.pi - t - phi))]


def _tangent_breaks(Yc, Xc, alphas) -> List[float]:
    """Roots of ``Y(d) cos a - X(d) sin a = 0`` for each boundary angle a."""
    out: List[float] = []
    for a in alphas:
        ca, sa = np.cos(a), np.sin(a)
        p = Yc[0] * ca - Xc[0] * sa
        q = Yc[1] * ca - Xc[1] * sa
        r = Yc[2] * ca - Xc[2] * sa
        out.extend(harmonic_roots(p, q, r))
    return out


def _cosine_breaks(uc, alphas) -> List[float]:
    """Roots of ``u(d) = cos(a)`` for each boundary angle a."""
    out: List[float] = []
    for a in alphas:
        out.extend(harmonic_roots(uc[0], uc[1], uc[2] - float(np.cos(a))))
    return out


def _entry(Mc, Ms, Mk, i, j) -> Tuple[float, float, float]:
    """(p, q, r) for ``A[i, j](d) = p sin d + q cos d + r``."""
    return (float(Ms[i, j]), float(Mk[i, j]), float(Mc[i, j]))


def breakpoints(coef: FibreCoefficients) -> np.ndarray:
    """Every d at which any joint of ANY branch can change limit status.

    Branch-independent: the branch flips both atan2 arguments and negates the
    acos, neither of which changes the equations below.
    """
    A1 = admissible_set(coef.lower[0], coef.upper[0])
    A2 = admissible_set(coef.lower[1], coef.upper[1])
    A3 = admissible_set(coef.lower[2], coef.upper[2])
    A5 = admissible_set(coef.lower[4], coef.upper[4])
    A6 = admissible_set(coef.lower[5], coef.upper[5])
    A7 = admissible_set(coef.lower[6], coef.upper[6])

    cuts: List[float] = []
    # shoulder, from M
    cuts += _tangent_breaks(_entry(coef.Mc, coef.Ms, coef.Mk, 1, 2),
                            _entry(coef.Mc, coef.Ms, coef.Mk, 0, 2),
                            _arc_endpoints(A1))
    cuts += _cosine_breaks(_entry(coef.Mc, coef.Ms, coef.Mk, 2, 2),
                           _arc_endpoints(A2))
    m20 = _entry(coef.Mc, coef.Ms, coef.Mk, 2, 0)
    cuts += _tangent_breaks(_entry(coef.Mc, coef.Ms, coef.Mk, 2, 1),
                            (-m20[0], -m20[1], -m20[2]),
                            _arc_endpoints(A3))
    # wrist, from N
    cuts += _tangent_breaks(_entry(coef.Nc, coef.Ns, coef.Nk, 1, 2),
                            _entry(coef.Nc, coef.Ns, coef.Nk, 0, 2),
                            _arc_endpoints(A5))
    cuts += _cosine_breaks(_entry(coef.Nc, coef.Ns, coef.Nk, 2, 2),
                           _arc_endpoints(A6))
    n20 = _entry(coef.Nc, coef.Ns, coef.Nk, 2, 0)
    cuts += _tangent_breaks(_entry(coef.Nc, coef.Ns, coef.Nk, 2, 1),
                            (-n20[0], -n20[1], -n20[2]),
                            _arc_endpoints(A7))
    # gimbal: |A[2,2]| = 1 makes both atan2 arguments vanish and a, c jump by pi
    for u in (_entry(coef.Mc, coef.Ms, coef.Mk, 2, 2),
              _entry(coef.Nc, coef.Ns, coef.Nk, 2, 2)):
        cuts += harmonic_roots(u[0], u[1], u[2] - 1.0)
        cuts += harmonic_roots(u[0], u[1], u[2] + 1.0)

    if not cuts:
        return np.zeros(0, dtype=float)
    b = np.sort(np.asarray(cuts, dtype=float))
    keep = np.concatenate([[True], np.diff(b) > 1e-12])
    b = b[keep]
    if len(b) > 1 and (b[0] + TWO_PI) - b[-1] <= 1e-12:
        b = b[:-1]
    return b


# ---------------------------------------------------------------------------
# the answer
# ---------------------------------------------------------------------------
def gimbal_risk(coef: "FibreCoefficients") -> dict:
    """Does either ZYZ middle angle come within `GIMBAL_GUARD_RAD` of 0 or pi?

    Exact, and two scalar tests: the middle angle is ``b(d) = acos(u(d))`` with
    ``u(d) = uc + us sin d + uk cos d``, so over the whole circle u ranges over
    ``[uc - R, uc + R]`` with ``R = hypot(us, uk)``.  b therefore reaches its
    minimum at ``acos(min(1, uc + R))`` and its maximum at
    ``acos(max(-1, uc - R))`` -- no sampling, no roots.
    """
    lim = float(np.cos(GIMBAL_GUARD_RAD))
    out = {"risk": False, "shoulder": False, "wrist": False}
    for name, (Mc, Ms, Mk) in (("shoulder", (coef.Mc, coef.Ms, coef.Mk)),
                               ("wrist", (coef.Nc, coef.Ns, coef.Nk))):
        uc, us, uk = float(Mc[2, 2]), float(Ms[2, 2]), float(Mk[2, 2])
        R = float(np.hypot(us, uk))
        hit = (uc + R >= lim) or (uc - R <= -lim)
        out[name] = bool(hit)
        out["risk"] = out["risk"] or bool(hit)
    return out


def _arcs_from_truth(fib, branch: int, step: float = GUARD_FALLBACK_STEP_RAD) -> np.ndarray:
    """Arcs in d, read off `fib.at()` itself. The guard's honest fallback.

    Conservative by construction: a boundary is placed at the last sample that
    `at()` accepted, so the returned set is a subset of the true one by at most
    one step. Never larger, which is the direction that matters.
    """
    d = np.arange(-np.pi, np.pi, step)
    psi = wrap_pi(d + float(fib.psi0))
    Q = fib.at(psi, respect_limits=True)
    ok = np.isfinite(Q[:, branch, :]).all(axis=-1)
    if not ok.any():
        return np.zeros((0, 2), dtype=float)
    if ok.all():
        return np.array([[-np.pi, np.pi]], dtype=float)
    n = len(ok)
    start = int(np.argmax(ok & ~np.roll(ok, 1)))
    order = (np.arange(n) + start) % n
    arcs, k = [], 0
    while k < n:
        if not ok[order[k]]:
            k += 1
            continue
        j = k
        while j + 1 < n and ok[order[j + 1]]:
            j += 1
        arcs.append((float(d[order[k]]), float(d[order[k]]) + ((j - k) * step)))
        k = j + 1
    merged = _norm_arcs(arcs)
    if merged is FULL:
        return np.array([[-np.pi, np.pi]], dtype=float)
    return np.array(merged, dtype=float).reshape(-1, 2)


def feasible_arcs_d(fib, arm=None, branch: int = 0,
                    coef: Optional[FibreCoefficients] = None,
                    cuts: Optional[np.ndarray] = None) -> np.ndarray:
    """Feasible arcs in the fibre's own ``d = wrap(psi - psi0)`` coordinate."""
    coef = coef if coef is not None else coefficients(fib, arm)
    if not in_limits_scalar(coef.q4, coef.lower[3], coef.upper[3]):
        return np.zeros((0, 2), dtype=float)
    # THE GUARD. Near a ZYZ gimbal this module's joint formulas and the ones
    # `at()` actually uses are different functions, so the closed form has no
    # claim there and says so rather than guessing (see `GIMBAL_GUARD_RAD`).
    if gimbal_risk(coef)["risk"]:
        return _arcs_from_truth(fib, branch)
    cuts = breakpoints(coef) if cuts is None else cuts

    if cuts.size == 0:
        return (np.array([[-np.pi, np.pi]], dtype=float)
                if _feasible_at(coef, 0.0, branch) else np.zeros((0, 2), dtype=float))

    n = len(cuts)
    if n == 1:
        return (np.array([[cuts[0], cuts[0] + TWO_PI]], dtype=float)
                if _feasible_at(coef, float(cuts[0] + np.pi), branch)
                else np.zeros((0, 2), dtype=float))

    ends = np.concatenate([cuts[1:], [cuts[0] + TWO_PI]])
    mids = 0.5 * (cuts + ends)
    ok = np.array([_feasible_at(coef, float(m), branch) for m in mids], dtype=bool)

    if ok.all():
        return np.array([[-np.pi, np.pi]], dtype=float)
    if not ok.any():
        return np.zeros((0, 2), dtype=float)

    # rotate so cell 0 starts a run
    start = int(np.argmax(ok & ~np.roll(ok, 1)))
    order = (np.arange(n) + start) % n
    arcs: List[Tuple[float, float]] = []
    k = 0
    while k < n:
        if not ok[order[k]]:
            k += 1
            continue
        lo = cuts[order[k]]
        j = k
        while j + 1 < n and ok[order[j + 1]]:
            j += 1
        hi_i = (order[j] + 1) % n
        hi = cuts[hi_i]
        span = (hi - lo) % TWO_PI
        if span <= 1e-15:
            span = TWO_PI
        arcs.append((float(lo), float(lo + span)))
        k = j + 1
    merged = _norm_arcs(arcs)
    if merged is FULL:
        return np.array([[-np.pi, np.pi]], dtype=float)
    return np.array(merged, dtype=float).reshape(-1, 2)


def feasible_arcs(fib, arm=None, branch: int = 0,
                  coef: Optional[FibreCoefficients] = None,
                  cuts: Optional[np.ndarray] = None) -> np.ndarray:
    """``(K, 2)`` closed, wrapped psi intervals where branch ``branch`` is legal.

    Each row is ``(lo, hi)`` in radians, both in [-pi, pi); ``lo > hi`` means the
    arc crosses -pi.  A single row ``(-pi, pi)`` means the whole circle; an empty
    ``(0, 2)`` array means the branch is infeasible at every swivel angle.

    No sampling: the endpoints are exact roots of the limit equations.
    """
    arcs_d = feasible_arcs_d(fib, arm, branch, coef=coef, cuts=cuts)
    if arcs_d.shape[0] == 0:
        return arcs_d
    if arcs_d.shape[0] == 1 and arcs_d[0, 1] - arcs_d[0, 0] >= TWO_PI - 1e-9:
        return np.array([[-np.pi, np.pi]], dtype=float)
    psi0 = coef.psi0 if coef is not None else float(fib.psi0)
    out = wrap_pi(arcs_d + psi0)
    return np.asarray(out, dtype=float).reshape(-1, 2)


def feasible_arcs_all(fib, arm=None) -> List[np.ndarray]:
    """``feasible_arcs`` for branches 0..3, sharing one coefficient expansion."""
    coef = coefficients(fib, arm)
    cuts = breakpoints(coef)
    return [feasible_arcs(fib, arm, b, coef=coef, cuts=cuts) for b in range(4)]


# ---------------------------------------------------------------------------
# extras, useful for diagnosis
# ---------------------------------------------------------------------------
def per_joint_arcs_d(fib, arm=None, branch: int = 0,
                     coef: Optional[FibreCoefficients] = None) -> List[np.ndarray]:
    """Per-joint feasible d-arcs (joint 4 is all-or-nothing)."""
    coef = coef if coef is not None else coefficients(fib, arm)
    cuts = breakpoints(coef)
    out: List[np.ndarray] = []
    for j in range(7):
        if j == 3:
            ok_all = in_limits_scalar(coef.q4, coef.lower[3], coef.upper[3])
            out.append(np.array([[-np.pi, np.pi]]) if ok_all else np.zeros((0, 2)))
            continue
        if cuts.size == 0:
            q = joint_angles(coef, 0.0, branch)
            ok_all = in_limits_scalar(q[j], coef.lower[j], coef.upper[j])
            out.append(np.array([[-np.pi, np.pi]]) if ok_all else np.zeros((0, 2)))
            continue
        ends = np.concatenate([cuts[1:], [cuts[0] + TWO_PI]])
        mids = 0.5 * (cuts + ends)
        arcs = []
        for i, m in enumerate(mids):
            q = joint_angles(coef, float(m), branch)
            if in_limits_scalar(q[j], coef.lower[j], coef.upper[j]):
                arcs.append((float(cuts[i]), float(ends[i])))
        merged = _norm_arcs(arcs)
        if merged is FULL:
            out.append(np.array([[-np.pi, np.pi]]))
        else:
            out.append(np.array(merged, dtype=float).reshape(-1, 2))
    return out


def arcs_contain(arcs: np.ndarray, psi) -> np.ndarray:
    """Membership test for the wrapped ``(K, 2)`` arc format."""
    psi = wrap_pi(np.asarray(psi, dtype=float))
    out = np.zeros(np.shape(psi), dtype=bool)
    arcs = np.asarray(arcs, dtype=float).reshape(-1, 2)
    for lo, hi in arcs:
        if hi - lo >= TWO_PI - 1e-9 or (lo <= -np.pi + 1e-12 and hi >= np.pi - 1e-12):
            return np.ones(np.shape(psi), dtype=bool)
        if lo <= hi:
            out |= (psi >= lo) & (psi <= hi)
        else:
            out |= (psi >= lo) | (psi <= hi)
    return out


def arc_distance(arcs: np.ndarray, psi: float) -> float:
    """Angular distance from ``psi`` to the nearest arc BOUNDARY (radians)."""
    arcs = np.asarray(arcs, dtype=float).reshape(-1, 2)
    if arcs.size == 0:
        return float("inf")
    e = arcs.reshape(-1)
    d = np.abs(wrap_pi(e - float(psi)))
    return float(np.min(d))


def total_measure(arcs: np.ndarray) -> float:
    """Total feasible swivel measure in radians."""
    arcs = np.asarray(arcs, dtype=float).reshape(-1, 2)
    if arcs.size == 0:
        return 0.0
    tot = 0.0
    for lo, hi in arcs:
        if lo <= -np.pi + 1e-12 and hi >= np.pi - 1e-12:
            return TWO_PI
        tot += (hi - lo) % TWO_PI
    return float(tot)
