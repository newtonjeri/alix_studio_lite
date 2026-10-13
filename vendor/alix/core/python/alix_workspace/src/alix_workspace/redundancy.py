"""The redundancy atlas: what the arm can do behind a stroke, not just at a point.

`reachability` and `manipulability` both reduce over configurations — the
envelope records that *some* joint solution puts the tool in a voxel, and the
dexterity map records the *best* one. In free space that is a fair summary. In an
800 mm cubicle it is not, because the configuration that reaches the point is
usually the one that puts the elbow through a wall, and neither map has a
coordinate in which that could be expressed.

This module analyses the coordinate they reduced over. The wipe constrains a
contact point and a press normal — five of six pose freedoms — and the arm has
eight, so the pre-image of one contact is a three-parameter family: the elbow
swivel `psi`, the pole height `h`, and the spin `gamma` about the press axis.
Fixing `h` and `gamma` (both constant within a stroke) leaves one image per
stroke: **swivel against arc length**, every cell carrying whether a
configuration exists there and, if not, what stopped it.

The question the image answers is *connectivity*, not point reachability. A
stroke is feasible when a continuous curve crosses it from the first waypoint to
the last without entering a blocked region — and, because the Euler branches are
disjoint manifolds (Burdick 1989), without teleporting between them.
:func:`corridor` is that search, as an exact bottleneck dynamic program over
`(swivel, branch)` rather than a scan over candidate shapes.

**Deliberately geometric.** A cell is blocked by joint limits, by the vendor
singular sets, or by collision — nothing else. No velocity, acceleration, jerk or
scaling value is read, from a config or from anywhere else, so an atlas stays
valid when a task is re-timed or re-authored and can be compared across tasks
that were authored with different parameters.

Collision, when asked for, is delegated to
:class:`alix_taskgen.body_collision.BodyCollisionWorld` — the same world the
generation gate uses, with the same ACM, the same allowance for the links that
are *meant* to touch the surface, and the same walls. A second implementation
would drift from it, and the atlas would then disagree with the gate about what
is blocked.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Sequence, Tuple

import numpy as np

from alix_workspace.srs import SelfMotionFibre, SrsArm

logger = logging.getLogger("alix_workspace.redundancy")

# ---------------------------------------------------------------------------
# Why a cell is not available. Ordered by cost: a cell reports the first thing
# that ruled it out, and the cheap tests run first so collision is only asked
# where a configuration actually exists.
# ---------------------------------------------------------------------------
FREE = 0
OUT_OF_REACH = 1   #: wrist centre outside the arm's ball — PROVEN, not a solver verdict
NO_SEED = 2        #: inside the ball, but the numerical seed did not converge
NO_BRANCH = 3      #: this Euler branch does not exist here
LIMIT = 4          #: a joint is outside its URDF range
SINGULAR = 5       #: inside the vendor singular set at the gate margin
SELF_COLLISION = 6
ENVIRONMENT = 7    #: the cubicle walls
FIXTURE = 8        #: the commode, beyond what the contact allowance permits
PARTNER = 9        #: the OTHER arm — never excusable, never ACM-disabled

#: The swivel ramp slopes the RUNTIME can take, degrees per metre — one of
#: these per section, and nothing between them. `alix_taskgen.stroke_gate.
#: SLOPES_DEG_PER_M` is the authority (it is what `choose_ramps` searches);
#: this repeats it because alix_workspace may not import alix_taskgen, and a
#: caller that has the real one should pass it (`swivel_slopes_deg_per_m`).
RUNTIME_SWIVEL_SLOPES_DEG_PER_M = (-100.0, -60.0, -30.0, 0.0, 30.0, 60.0, 100.0)

CODE_LABELS = {
    FREE: "free",
    OUT_OF_REACH: "beyond the wrist ball",
    NO_SEED: "IK did not converge",
    NO_BRANCH: "branch does not exist",
    LIMIT: "joint limit",
    SINGULAR: "singular set",
    SELF_COLLISION: "self-collision",
    ENVIRONMENT: "wall",
    FIXTURE: "fixture",
    PARTNER: "partner arm",
}

#: Substrings that classify a blocking pair reported by the body-collision
#: world. Bin, UGV, poles and covers are the ENVIRONMENT from the arm's point
#: of view; the partner arm is its own class because nothing may excuse it.
_ENV_TOKENS = ("washroom", "wall", "floor_", "ceiling", "ugv", "bin_", "pole",
               "sliding_plate", "cover", "glove_wearing")
_FIXTURE_TOKENS = ("commode", "lid", "seat", "bowl", "cistern", "fixture")


def classify_pair(pair: str, side: Optional[str] = None) -> int:
    """Map a ``"A<->B"`` blocking pair onto a block code.

    With ``side`` ("right"/"left") the robot-internal pairs are told apart: a
    link of the OTHER arm or hand is PARTNER, two links of the active arm are
    SELF_COLLISION. Without it, every robot-internal pair was reported as
    self-collision — including the partner arm, which is the one the pairing
    measurements found most often.
    """
    low = pair.lower()
    if any(t in low for t in _FIXTURE_TOKENS):
        return FIXTURE
    if any(t in low for t in _ENV_TOKENS):
        return ENVIRONMENT
    if side in ("right", "left"):
        other = ("l_", "ilh") if side == "right" else ("r_", "irh")
        links = [x.strip().lower() for x in pair.split("<->")]
        if any(x.startswith(other) for x in links):
            return PARTNER
    return SELF_COLLISION


# ---------------------------------------------------------------------------
# Vendor singular sets, vectorised.
# ---------------------------------------------------------------------------
def singular_margin_deg(q_deg: np.ndarray) -> np.ndarray:
    """Distance to the NEAREST vendor singular set, over any leading shape.

    The elementwise twin of :func:`alix_taskgen.singularity.margin_deg`, which is
    scalar-only and would be called once per cell. The formulas are copied from
    it deliberately rather than re-derived, and ``test_redundancy`` asserts the
    two agree on random poses — if the vendor law changes there, this fails.
    """
    q = np.asarray(q_deg, dtype=float)
    q2, q3, q4, q5, q6 = q[..., 1], q[..., 2], q[..., 3], q[..., 4], q[..., 5]

    def at(v, target):
        return np.abs(np.abs(v) - target)

    t1 = np.maximum(at(q2, 0.0), at(q6, 0.0))
    t2 = at(q4, 0.0)
    t3 = np.maximum(at(q2, 0.0), at(q3, 90.0))
    t4 = np.maximum(at(q6, 0.0), at(q5, 90.0))
    return np.minimum(np.minimum(t1, t2), np.minimum(t3, t4))


# ---------------------------------------------------------------------------
@dataclass
class AtlasLayer:
    """One ``(pole, wipe yaw)`` slice: swivel against arc length along a stroke.

    ``code``, ``margin_deg`` and ``q`` are indexed ``[waypoint, swivel, branch]``.
    Branch index is stable across the whole layer, which is what lets
    :func:`corridor` forbid a jump between disjoint manifolds.
    """

    psi_deg: np.ndarray                  #: (P,) swivel grid
    code: np.ndarray                     #: (S, P, 4) uint8
    margin_deg: np.ndarray               #: (S, P, 4) float32 — min(limit, singular)
    limit_margin_deg: np.ndarray         #: (S, P, 4) float32
    singular_margin_deg: np.ndarray      #: (S, P, 4) float32
    q: np.ndarray                        #: (S, P, 4, 7) float32, radians; NaN where absent
    pole_m: float
    gamma_deg: float
    gate_deg: float
    max_reach_mm: float                  #: the arm's own wrist-ball radius
    seed_ok: np.ndarray                  #: (S,) bool
    wrist_distance_mm: np.ndarray        #: (S,) shoulder to wrist centre
    elbow_orbit_mm: np.ndarray           #: (S,) how far swivel moves the elbow here
    q4_deg: np.ndarray                   #: (S,) elbow angle — a property of the pose
    seed_error_mm: np.ndarray = field(   #: (S,) |FK(seed) - requested pose|
        default_factory=lambda: np.zeros(0))
    # The elbow's circle per waypoint: elbow(psi) = centre + cos(psi) A + sin(psi) B.
    # The swivel axis made physical — what a 3-D view can actually show, since a
    # wall blocks an ARC of this ring rather than the contact point.
    orbit_centre: np.ndarray = field(default_factory=lambda: np.zeros((0, 3)))
    orbit_a: np.ndarray = field(default_factory=lambda: np.zeros((0, 3)))
    orbit_b: np.ndarray = field(default_factory=lambda: np.zeros((0, 3)))
    shoulder: np.ndarray = field(default_factory=lambda: np.zeros(3))
    blocking_pairs: Dict[str, int] = field(default_factory=dict)
    collision_checked: int = 0
    names: List[str] = field(default_factory=list)

    @property
    def shape(self) -> Tuple[int, int, int]:
        return self.code.shape

    def free_mask(self) -> np.ndarray:
        return self.code == FREE

    def summary(self) -> dict:
        """Counts by block code, and the coordinates a decision hangs on."""
        total = int(self.code.size)
        counts = {CODE_LABELS[c]: int((self.code == c).sum())
                  for c in sorted(CODE_LABELS) if (self.code == c).any()}
        free = self.free_mask()
        per_wp = free.any(axis=(1, 2))
        dead = [i for i, ok in enumerate(per_wp) if not ok]
        # The straight-elbow set is a distance, so it is worth reporting as one:
        # a waypoint inside the shell cannot be rescued by any swivel.
        return {
            "poleM": round(self.pole_m, 4),
            "gammaDeg": round(self.gamma_deg, 2),
            "gateDeg": self.gate_deg,
            "waypoints": int(self.code.shape[0]),
            "swivelSamples": int(self.code.shape[1]),
            "branches": int(self.code.shape[2]),
            "freeCells": int(free.sum()),
            "freeFraction": round(float(free.sum()) / total, 4) if total else 0.0,
            "byCode": counts,
            "deadWaypoints": dead,
            "maxReachMm": round(self.max_reach_mm, 2),
            "overshootMm": [round(float(v - self.max_reach_mm), 2)
                            for v in self.wrist_distance_mm
                            if np.isfinite(v) and v > self.max_reach_mm],
            "collisionChecked": self.collision_checked,
            "blockingPairs": dict(sorted(self.blocking_pairs.items(),
                                         key=lambda kv: -kv[1])[:20]),
            "wristDistanceMm": [round(float(v), 2) for v in self.wrist_distance_mm],
            "elbowOrbitMm": [round(float(v), 2) for v in self.elbow_orbit_mm],
            "q4Deg": [round(float(v), 2) for v in self.q4_deg],
            # The fibre holds FK(seed) exactly; this is how far that is from the
            # pose that was asked for, so a loose seed can never masquerade as a
            # tight atlas. Anything above ~0.05 mm means the seed solver's
            # tolerance, not the reconstruction, is the limiting error.
            "seedErrorMm": (round(float(np.nanmax(self.seed_error_mm)), 4)
                            if self.seed_error_mm.size and
                            np.isfinite(self.seed_error_mm).any() else None),
        }


# ---------------------------------------------------------------------------
CollisionScreen = Callable[[np.ndarray], List[str]]
"""``q_deg7 -> blocking pair strings``. Empty means the pose is clear."""


def compute_layer(arm: SrsArm, poses: Sequence[np.ndarray],
                  *,
                  psi_deg: Optional[Sequence[float]] = None,
                  pole_m: float = 0.0,
                  gamma_deg: float = 0.0,
                  gate_deg: float = 20.0,
                  seeds: Optional[Sequence[Optional[np.ndarray]]] = None,
                  seed_solver=None,
                  collision: Optional[CollisionScreen] = None,
                  collision_budget: int = 20000,
                  names: Optional[Sequence[str]] = None,
                  progress: Optional[Callable[[int, int], None]] = None,
                  swivel_ref: Optional[np.ndarray] = None
                  ) -> AtlasLayer:
    """Build one swivel-against-arc-length image for a stroke.

    ``poses`` are 4x4 tool targets in ``base_link``, already carrying whatever
    wipe yaw this layer represents (``gamma_deg`` is recorded, not applied).
    ``seeds`` supplies one configuration per pose that already reaches it; where
    it is ``None``, ``seed_solver(T, q_init)`` is called — pass an
    :class:`alix_workspace.ik.IKSolver`-backed callable.

    ``swivel_ref`` is the MODEL-frame axis the layer's ``psi`` is measured
    against, fixed for the whole stroke; it defaults to the arm's configured
    axis (model Z on the production chains — the runtime's coordinate). A pose
    on which that axis is ill-conditioned raises rather than switching axes,
    which would put a step in the very coordinate the corridor steps through;
    :meth:`SrsArm.swivel_ref_for` picks a better fixed axis when one exists.

    Tests run cheapest first, so ``collision`` is only invoked on cells that have
    a configuration inside limits and clear of the singular sets. That ordering
    is the whole reason the atlas is affordable: on a typical stroke it removes
    most cells before the millisecond-scale collision query is ever reached.
    """
    poses = [np.asarray(T, dtype=float) for T in poses]
    S = len(poses)
    if S == 0:
        raise ValueError("no poses supplied")
    from alix_workspace.srs import StereographicSwivel
    ref = (arm.swivel_ref_axis if swivel_ref is None
           else swivel_ref if isinstance(swivel_ref, StereographicSwivel)
           else np.asarray(swivel_ref, dtype=float))
    psi = np.radians(np.asarray(
        psi_deg if psi_deg is not None else np.arange(-180.0, 180.0, 5.0), dtype=float))
    P = len(psi)
    B = SelfMotionFibre.N_BRANCHES

    code = np.full((S, P, B), NO_SEED, dtype=np.uint8)
    q_all = np.full((S, P, B, 7), np.nan, dtype=np.float32)
    lim_deg = np.zeros((S, P, B), dtype=np.float32)
    sing_deg = np.zeros((S, P, B), dtype=np.float32)
    seed_ok = np.zeros(S, dtype=bool)
    wrist_mm = np.full(S, np.nan)
    orbit_mm = np.full(S, np.nan)
    q4_deg = np.full(S, np.nan)
    seed_err_mm = np.full(S, np.nan)
    orbit_c = np.full((S, 3), np.nan)
    orbit_a = np.full((S, 3), np.nan)
    orbit_b = np.full((S, 3), np.nan)

    # The shoulder is fixed once the pole is chosen, and the wrist centre is
    # fixed by the pose alone (the wrist rotations turn the tool ABOUT it), so
    # "is this waypoint reachable at all" is one distance against one radius —
    # exact, instant, and independent of every solver. Asking it first means a
    # genuinely out-of-reach waypoint is never reported as "IK failed", which
    # reads as a tuning problem and is not one.
    shoulder = arm.chain_frames(np.zeros(7), pole_m)[arm.shoulder_link][:3, 3]
    reach = arm.max_reach_m

    prev_q = None
    for si, T in enumerate(poses):
        d = float(np.linalg.norm(arm.wrist_centre_of_pose(T) - shoulder))
        wrist_mm[si] = d * 1000.0
        if d > reach:
            code[si] = OUT_OF_REACH
            continue
        seed = None if seeds is None else seeds[si]
        if seed is None and seed_solver is not None:
            seed = seed_solver(T, prev_q)
        if seed is None:
            continue
        seed = np.asarray(seed, dtype=float)
        fib = arm.fibre(T, seed, pole_m, ref)
        if fib is None:
            continue
        seed_ok[si] = True
        prev_q = seed
        wrist_mm[si] = fib.wrist_distance_m * 1000.0
        orbit_mm[si] = fib.elbow_orbit_radius_m * 1000.0
        q4_deg[si] = np.degrees(fib.q4)
        seed_err_mm[si] = fib.seed_error_m * 1000.0
        orbit_c[si], orbit_a[si], orbit_b[si] = fib.orbit()

        Q = fib.at(psi, respect_limits=False)                 # (P, B, 7)
        exists = np.all(np.isfinite(Q), axis=-1)
        inside = np.all((Q >= arm.lower - 1e-9) & (Q <= arm.upper + 1e-9), axis=-1)
        lm = np.degrees(np.min(np.minimum(Q - arm.lower, arm.upper - Q), axis=-1))
        sm = singular_margin_deg(np.degrees(Q))

        c = np.full((P, B), LIMIT, dtype=np.uint8)
        c[~exists] = NO_BRANCH
        ok = exists & inside
        c[ok] = FREE
        c[ok & (sm < gate_deg)] = SINGULAR

        q_all[si] = Q.astype(np.float32)
        lim_deg[si] = np.where(exists, lm, 0.0).astype(np.float32)
        sing_deg[si] = np.where(exists, sm, 0.0).astype(np.float32)
        code[si] = c
        if progress is not None:
            progress(si + 1, S)

    margin = np.minimum(lim_deg, sing_deg).astype(np.float32)
    margin[code != FREE] = -1.0

    layer = AtlasLayer(
        psi_deg=np.degrees(psi), code=code, margin_deg=margin,
        limit_margin_deg=lim_deg, singular_margin_deg=sing_deg, q=q_all,
        pole_m=float(pole_m), gamma_deg=float(gamma_deg), gate_deg=float(gate_deg),
        max_reach_mm=arm.max_reach_m * 1000.0, seed_ok=seed_ok,
        orbit_centre=orbit_c, orbit_a=orbit_a, orbit_b=orbit_b,
        shoulder=shoulder, wrist_distance_mm=wrist_mm, elbow_orbit_mm=orbit_mm,
        q4_deg=q4_deg, seed_error_mm=seed_err_mm, names=list(names or []))

    if collision is not None:
        _apply_collision(layer, collision, budget=collision_budget)
    return layer


def _apply_collision(layer: AtlasLayer, collision: CollisionScreen,
                     budget: int = 20000) -> None:
    """Collision-screen the surviving cells, cheapest-first and budgeted.

    Cells are visited in descending kinematic margin so that, if the budget runs
    out, what got checked is the part of the corridor a search would actually
    want to use. Unchecked cells keep their kinematic verdict and the count is
    reported — a truncated sweep must never read as a clean one.
    """
    idx = np.argwhere(layer.code == FREE)
    if not len(idx):
        return
    order = np.argsort(-layer.margin_deg[idx[:, 0], idx[:, 1], idx[:, 2]])
    idx = idx[order][:budget]
    checked = 0
    for s, p, b in idx:
        q = layer.q[s, p, b]
        if not np.all(np.isfinite(q)):
            continue
        pairs = collision(np.degrees(q.astype(float)))
        checked += 1
        if pairs:
            worst = min(classify_pair(x) for x in pairs)
            layer.code[s, p, b] = worst
            layer.margin_deg[s, p, b] = -1.0
            for x in pairs:
                layer.blocking_pairs[x] = layer.blocking_pairs.get(x, 0) + 1
    layer.collision_checked = checked
    logger.info("collision-screened %d cells; %d distinct blocking pairs",
                checked, len(layer.blocking_pairs))


# ---------------------------------------------------------------------------
# Corridor search
# ---------------------------------------------------------------------------
@dataclass
class Corridor:
    """A continuous choice of swivel across a whole stroke."""

    found: bool
    psi_deg: np.ndarray = field(default_factory=lambda: np.zeros(0))
    branch: np.ndarray = field(default_factory=lambda: np.zeros(0, dtype=int))
    q: np.ndarray = field(default_factory=lambda: np.zeros((0, 7)))
    min_margin_deg: float = -1.0
    blocked_at: int = -1                 #: first waypoint with no reachable cell
    blocked_reason: str = ""
    #: WHICH failure, machine-readable, because the three need different fixes.
    #: "dead"   — the ring at that waypoint has no free swivel at all: a
    #:            standoff / pole / other-arm decision, and no path edit helps.
    #: "step"   — free swivel exists on both sides but no continuous step reaches
    #:            it: a sampling or continuity-limit decision.
    #: "travel" — continuous paths exist, but none keeps the swivel travel of
    #:            every SECTION within `swivel_travel_cap_deg_per_m`: the
    #:            runtime's one ramp per section could not carry it — a section
    #:            boundary (a reversal), the wipe yaw or the pole changes it.
    #: "ramp"   — a corridor within the travel budget exists, but its SHAPE is
    #:            not one the runtime can follow: no continuous chain of one
    #:            straight ramp per section stays with it. Travel is a rate and
    #:            says nothing about shape (a section that goes out and back
    #:            spends little and no line follows it), and the runtime's ramp
    #:            carries on from where the LAST one ended, so an error the
    #:            corridor does not share accumulates across sections.
    #: A caller that only reads `blocked_reason` gets prose it has to parse, and
    #: the failures read almost alike in prose.
    blocked_kind: str = ""
    required_joint_step_deg: Optional[float] = None
    max_swivel_step_deg: float = 0.0
    swivel_range_deg: float = 0.0
    # ---- swivel TRAVEL: what the runtime's per-section ramp has to carry -----
    #: total variation of the unwrapped swivel over the whole corridor, degrees
    swivel_travel_deg: float = 0.0
    #: per section: ``{"section", "s0", "s1", "lengthM", "travelDeg",
    #: "degPerM", "budgetDeg", "subPitch", "withinCap"}`` — the section rule is
    #: the caller's (a reversal turn); one section when none was given.
    #: ``budgetDeg`` is ``cap x length``, what the cap allows over THAT
    #: length, and is the number ``withinCap`` judges, so the report and the
    #: search agree; ``degPerM`` stays the raw rate. ``subPitch`` marks a
    #: section whose budget is under one grid cell — there the swivel may not
    #: turn at all, and a reader looking at a refusal should know that the
    #: grid and not the cap is what refused.
    swivel_travel_sections: List[dict] = field(default_factory=list)
    swivel_travel_max_deg_per_m: Optional[float] = None
    swivel_travel_cap_deg_per_m: Optional[float] = None
    #: for a `travel` refusal: the least travel any margin-optimal corridor
    #: needs on the worst section, degrees per metre
    required_swivel_travel_deg_per_m: Optional[float] = None
    # ---- swivel SHAPE: can one ramp per section actually follow this? -------
    #: True when a continuous piecewise-linear swivel reference — one slope
    #: per section out of the runtime's set — stays within
    #: `swivel_ramp_band_deg` of this corridor all the way; None when no cap
    #: (and so no ramp rule) was asked for.
    swivel_ramp_pass: Optional[bool] = None
    #: the slopes such a reference takes, one per section, degrees per metre
    swivel_ramp_slopes_deg_per_m: List[float] = field(default_factory=list)
    #: how far the reference may sit from the corridor: half a grid cell, the
    #: rounding and nothing else — in `ramp` mode the runtime pins psi to the
    #: ramp (`stroke_gate.track(..., band=0)`), so the corridor's cell has to
    #: be the ramp's own nearest grid representative
    swivel_ramp_band_deg: Optional[float] = None
    #: for a `ramp` refusal: the band that WOULD have carried it, or None when
    #: no band up to 180 deg does (the shape, not the tolerance, is the defect)
    required_swivel_ramp_band_deg: Optional[float] = None
    n_sections: int = 0

    def to_dict(self) -> dict:
        cap = self.swivel_travel_cap_deg_per_m
        worst = self.swivel_travel_max_deg_per_m
        # The verdict is the per-section budget the SEARCH enforced, not the
        # worst RATE: a rate is undefined on a section of no arc length (which
        # may still not turn) and it is a rounded number, so judging on it
        # could return `found` with `pass` False — a corridor whose own gate
        # row said it had failed.
        verdicts = [r["withinCap"] for r in self.swivel_travel_sections
                    if r.get("withinCap") is not None]
        return {
            "found": self.found,
            "psiDeg": [round(float(v), 2) for v in self.psi_deg],
            "branch": [int(v) for v in self.branch],
            "minMarginDeg": round(float(self.min_margin_deg), 2),
            "blockedAt": int(self.blocked_at),
            "blockedReason": self.blocked_reason,
            "blockedKind": self.blocked_kind,
            "requiredJointStepDeg": (None if self.required_joint_step_deg is None
                                     else round(float(self.required_joint_step_deg), 1)),
            "maxSwivelStepDeg": round(float(self.max_swivel_step_deg), 2),
            "swivelRangeDeg": round(float(self.swivel_range_deg), 2),
            "swivelTravelDeg": round(float(self.swivel_travel_deg), 2),
            "swivelTravelMaxDegPerM": None if worst is None else round(float(worst), 2),
            "swivelTravelCapDegPerM": None if cap is None else float(cap),
            "swivelTravelSections": [dict(sec) for sec in self.swivel_travel_sections],
            "swivelTravelPass": None if not verdicts else bool(all(verdicts)),
            "swivelTravelSubPitchSections": [int(r["section"])
                                             for r in self.swivel_travel_sections
                                             if r.get("subPitch")],
            "requiredSwivelTravelDegPerM": (
                None if self.required_swivel_travel_deg_per_m is None
                else round(float(self.required_swivel_travel_deg_per_m), 1)),
            "swivelRampPass": self.swivel_ramp_pass,
            "swivelRampSlopesDegPerM": [float(v) for v in self.swivel_ramp_slopes_deg_per_m],
            "swivelRampBandDeg": (None if self.swivel_ramp_band_deg is None
                                  else round(float(self.swivel_ramp_band_deg), 3)),
            "requiredSwivelRampBandDeg": (
                None if self.required_swivel_ramp_band_deg is None
                else round(float(self.required_swivel_ramp_band_deg), 2)),
            "nSections": int(self.n_sections),
        }


#: What a cell outside the band costs, in the travel DP's own units (grid steps
#: of swivel), when it is one full band-width out. 1.0 says "being a band-width
#: out of position is worth one grid step of swivel travel".
BAND_COST_WEIGHT = 1.0
#: What a cell with no lateral position at all costs. Large so it is never
#: preferred, FINITE so it cannot poison the arithmetic. NaN must not reach the
#: DP: 0.0 would read as perfectly in band for a sample that has no arm solution.
BAND_COST_NAN = 1.0e6


def band_cost(values: np.ndarray, near: float, far: float, *,
              weight: float = BAND_COST_WEIGHT, within: bool = False,
              branches: Optional[int] = None) -> np.ndarray:
    """Cost of sitting outside ``[near, far]`` — 0 inside, rising linearly out.

    THE SHAPE IS A POLICY CHOICE and the two candidates are indistinguishable
    from the DP's output, so it is named here rather than left implicit.

    ``within=False`` (default) is **R2 as written**: the rule states a BAND —
    ``R_Link4.y in [-0.30, -0.08]`` — not a target, so every position inside it
    satisfies the rule equally and costs nothing. This reproduces the mask's
    intent as a preference, which is the whole reason the band enters as a cost:
    a mask REMOVES cells, and on regions passing by a thin margin that shows up
    as gate failures rather than as a fix.

    ``within=True`` additionally prefers the OUTBOARD end inside the band, on the
    argument that further from the partner is better for cross-arm clearance.
    That is a policy Newton has not stated, it is off by default, and it should
    be turned on against a measurement rather than against the argument — the
    honest cost of leaving it off is that once every in-band cell is free the
    travel term decides, and it may park the elbow at the inboard edge when far
    more was available.

    ORTHOGONAL TO THE PAD STORY, and this is worth stating because a reviewer will
    ask. The swivel is the self-motion family behind ONE tool pose: every
    configuration on the fibre reaches the same tip frame. Biasing psi therefore
    moves the ELBOW and leaves the hand exactly where it was — it cannot change
    the tilt, cannot change which of the two palm pads carries contact (they sit
    90 deg apart on one hand and roll into each other as the tilt varies;
    `alix_taskgen.pad_transition`), and cannot move a node into the no-contact
    tilt hole. The pad allowance is `contact_link_groups[ik_frame]` x
    `contact_surface_objects[surface]` — hand links against the FIXTURE — and
    never excuses a cross-arm pair (`PARTNER`, above: never ACM-disabled).

    NaN in ``values`` (a sample with no cell) becomes `BAND_COST_NAN`, never 0.0.
    With ``branches`` the ``(S, P)`` input is broadcast to the ``(S, P, B)`` the
    DP takes.
    """
    v = np.asarray(values, dtype=float)
    span = max(float(far) - float(near), 1e-9)
    out = np.where(v < near, near - v, np.where(v > far, v - far, 0.0))
    cost = weight * out / span
    if within:
        # The in-band slope is ADDED to the out-of-band penalty, over the whole
        # range, via the position clipped into the band: continuous at both
        # edges and falling monotonically all the way to the outboard edge.
        # FIXED 2026-09-21: it used to REPLACE the in-band zero with
        # `(far - v) / span`, which is 1.0 at the inboard edge while the
        # out-of-band term just outside it is ~0 -- so an elbow 1 cm INBOARD of
        # the band (0.41) cost less than one inside it at 0.12 (0.82), and the
        # corridor was paid to stay inboard. Measured on row 3A: `within=True`
        # reproduced the band-OFF corridor exactly. `within=False` is untouched.
        cost = cost + weight * (float(far) - np.clip(v, near, far)) / span
    cost = np.where(np.isfinite(v), cost, BAND_COST_NAN)
    if branches:
        cost = np.repeat(cost[:, :, None], int(branches), axis=2)
    return np.ascontiguousarray(cost, dtype=float)


def _corridor_dp(score: np.ndarray, q: np.ndarray, jstep: np.ndarray, offsets,
                 allow_branch_change: bool, *, mode: str, floor: float = -np.inf,
                 K: int = 1, bins_of_step: Optional[np.ndarray] = None,
                 boundary: Optional[np.ndarray] = None,
                 cell_cost: Optional[np.ndarray] = None):
    """One dynamic program over ``(swivel, branch[, travel used in this section])``.

    ``mode="bottleneck"`` maximises the path's WORST cell score (the corridor a
    stroke is authored down the middle of); ``mode="travel"`` minimises the
    path's total swivel travel, in grid steps, over the cells scoring at least
    ``floor``. Both walk the same transitions: a swivel offset from ``offsets``
    and a per-joint step within ``jstep[s-1]`` (branch changes only when
    allowed). With ``K > 1`` the state also carries how many grid steps of
    swivel travel the path has spent in the CURRENT section: a step may not
    push that past ``bins_of_step[s-1]`` (the budget of the section the step
    belongs to) and it resets to zero on arriving at a sample where
    ``boundary`` is set — the runtime's ramp starts afresh there.

    Returns ``(path_p, path_b, value, blocked_at)``; ``path_p`` is None when
    no state survives, ``blocked_at`` the first sample none reaches.
    """
    S, P, B = score.shape
    NEG = -np.inf
    track = K > 1
    eye = np.eye(B, dtype=bool)
    kk = np.arange(K)

    def allowed_at(s):
        ok = np.isfinite(score[s])
        if mode == "travel":
            ok &= score[s] >= floor
        return ok

    # A per-cell PREFERENCE, honoured in travel mode only. Travel mode already
    # walks nothing but cells scoring >= `floor`, so a cost added here re-ranks
    # corridors that ALL pass and can never make a passing stroke refuse. That is
    # the whole reason the elbow band enters as a cost rather than as a mask: a
    # mask removes cells, and on regions that pass by a thin margin removing cells
    # shows up as gate failures instead of as a fix.
    if cell_cost is None or mode != "travel":
        cell_cost = np.zeros_like(score)
    ok0 = allowed_at(0)
    best = np.full((P, B, K), NEG)
    best[:, :, 0] = np.where(ok0, score[0] if mode == "bottleneck" else -cell_cost[0], NEG)
    if not np.isfinite(best).any():
        return None, None, None, 0
    # one packed parent index per state: src_p * (B * K) + src_b * K + src_k
    parents = np.full((S, P, B, K), -1, dtype=np.int32)
    BK = B * K

    for s in range(1, S):
        allowed = allowed_at(s)
        if not allowed.any():
            return None, None, None, s
        cand = np.full((P, B, K), NEG)
        cand_par = np.full((P, B, K), -1, dtype=np.int32)
        for o in offsets:
            a = abs(int(o)) if track else 0
            if a >= K:
                continue
            prev = np.roll(best, o, axis=0)                   # (P, B, K), source b
            prev_q = np.roll(q[s - 1], o, axis=0)             # (P, B, 7)
            src_p = np.roll(np.arange(P), o)
            d = np.abs(q[s][:, None, :, :] - prev_q[:, :, None, :]).max(axis=-1)
            feasible = d <= jstep[s - 1]                      # (P, Bsrc, Btgt)
            if not allow_branch_change:
                feasible &= eye[None, :, :]
            if a:
                shifted = np.full((P, B, K), NEG)
                shifted[..., a:] = prev[..., :K - a]
            else:
                shifted = prev
            if track and bins_of_step is not None:
                lim = int(bins_of_step[s - 1])
                if lim + 1 < K:
                    shifted = shifted.copy()
                    shifted[..., lim + 1:] = NEG
            if mode == "travel":
                shifted = shifted - abs(int(o))
            vals = np.where(feasible[:, :, :, None], shifted[:, :, None, :], NEG)  # (P,Bsrc,Btgt,K)
            src = np.argmax(vals, axis=1)                                          # (P, Btgt, K)
            top = np.take_along_axis(vals, src[:, None, :, :], axis=1)[:, 0, :, :]
            better = top > cand
            par = src_p[:, None, None] * BK + src * K + (kk - a)[None, None, :]
            cand = np.where(better, top, cand)
            cand_par = np.where(better, par, cand_par)
        if mode == "bottleneck":
            best = np.minimum(cand, np.where(allowed, score[s], NEG)[..., None])
        else:
            best = np.where(allowed[..., None], cand - cell_cost[s][..., None], NEG)
        if track and boundary is not None and boundary[s]:
            # the ramp restarts here: the section's travel is spent, whatever
            # it was, and only the best state per (swivel, branch) carries on
            kb = np.argmax(best, axis=2)
            top = np.take_along_axis(best, kb[..., None], axis=2)[..., 0]
            par0 = np.take_along_axis(cand_par, kb[..., None], axis=2)[..., 0]
            best = np.full((P, B, K), NEG)
            best[:, :, 0] = top
            cand_par = np.full((P, B, K), -1, dtype=np.int32)
            cand_par[:, :, 0] = par0
        parents[s] = cand_par
        if not np.isfinite(best).any():
            return None, None, None, s

    flat = int(np.argmax(best))
    p, rem = divmod(flat, BK)
    b, k = divmod(rem, K)
    if not np.isfinite(best[p, b, k]):
        return None, None, None, S - 1
    value = float(best[p, b, k])
    path_p = np.zeros(S, dtype=int)
    path_b = np.zeros(S, dtype=int)
    path_p[-1], path_b[-1] = p, b
    for s in range(S - 1, 0, -1):
        packed = int(parents[s, p, b, k])
        p, rem = divmod(packed, BK)
        b, k = divmod(rem, K)
        path_p[s - 1], path_b[s - 1] = p, b
    return path_p, path_b, value, -1


def _section_bounds(sections: np.ndarray, S: int) -> List[Tuple[int, int, int]]:
    """``[(section id, first sample, last sample)]`` in walking order.

    A section's last sample is the next section's first: they share the step
    that crosses the boundary, which is charged to the earlier section — the
    same rule the DP's ``bins_of_step`` uses.
    """
    ids = [int(v) for v in dict.fromkeys(int(v) for v in sections)]
    out = []
    for j in ids:
        members = np.flatnonzero(sections == j)
        out.append((j, int(members[0]), min(int(members[-1]) + 1, S - 1)))
    return out


def _section_budget_deg(cap: Optional[float], length: Optional[float],
                        pitch: float) -> Optional[float]:
    """How much swivel travel one section may spend, degrees, or None.

    ``cap x length``, which is what the runtime's one ramp carries over that
    arc, and nothing more. An earlier version floored it at one grid pitch so
    that a section too short for the grid to express the cap could still turn
    a single cell; that floor grows without bound as the length falls (3.3x
    the cap over 30 mm) and it accumulates — eight such sections in a row
    drifted 40 deg where the ramps carry 24 — so a section whose budget is
    under one cell now buys nothing and the DP holds the swivel across it.
    Such a section is still reported ``subPitch``: there the grid, not the
    cap, is what refuses, and the caller reading a `travel` failure on a 30 mm
    section should know that before it reaches for the cap.
    """
    if cap is None or length is None:
        return None
    return max(0.0, float(cap) * float(length))


def _unwrap_deg(psi_path: np.ndarray) -> np.ndarray:
    """The corridor's swivel as a continuous angle: the grid wraps at +-180
    and a corridor may legitimately cross it, so every reading of psi as a
    QUANTITY (range, travel, the distance to a ramp) is taken off this."""
    d = np.diff(psi_path)
    d = (d + 180.0) % 360.0 - 180.0
    return np.concatenate([[psi_path[0]], psi_path[0] + np.cumsum(d)])


def _ramp_fit(psi_unwrapped: np.ndarray, bounds: Sequence[Tuple[int, int, int]],
              path_s: np.ndarray, slopes: Sequence[float], band: float):
    """Can the runtime's swivel reference follow this corridor? Exactly.

    The runtime resolves psi as ONE straight ramp per section, its slope from
    a fixed set, starting where the PREVIOUS section's ramp ended — the swivel
    of the joint state it arrives with (`stroke_gate._ramp_search`), not the
    corridor's psi at that sample. So the question a per-section travel budget
    cannot answer is this one: is there a continuous piecewise-linear
    reference, one slope per section, that stays within *band* of the corridor
    the whole way? Travel bounds a rate and says nothing about shape, and an
    error the reference and the corridor do not share accumulates across
    sections until the ramp is somewhere else entirely.

    Exact, not sampled: with ``v(s) = psi_ref(s) - psi(s)``, a section's
    constraint ``|v(s0) + c_k(s)| <= band`` for every sample is one interval
    of admissible ``v(s0)`` per slope, and the exit value is a translate of
    it. Propagating the union of those intervals decides feasibility for every
    reference at once; intervals another already contains are dropped (exact),
    and the list is capped by keeping the widest (a drop can only refuse
    more, never admit more).

    Returns ``(ok, one slope per section, index of the first section that no
    slope and no reference can carry)``.
    """
    state = [(0.0, 0.0, -1, 0.0)]        # (lo, hi, parent, slope); v(start) = 0
    history: List[list] = []
    for jj, (_j, s0, s1) in enumerate(bounds):
        if s1 <= s0:                     # a section with no step of its own
            nxt = [(lo, hi, i, 0.0) for i, (lo, hi, _p, _k) in enumerate(state)]
        else:
            ds = path_s[s0 + 1:s1 + 1] - path_s[s0]
            dpsi = psi_unwrapped[s0 + 1:s1 + 1] - psi_unwrapped[s0]
            nxt = []
            for k in slopes:
                c = float(k) * ds - dpsi
                lo_req, hi_req = -band - float(c.min()), band - float(c.max())
                if lo_req > hi_req:
                    continue
                shift = float(c[-1])
                for i, (lo, hi, _p, _k) in enumerate(state):
                    a, b = max(lo, lo_req), min(hi, hi_req)
                    if a <= b + 1e-12:
                        nxt.append((a + shift, b + shift, i, float(k)))
        if not nxt:
            return False, [], jj
        state = _prune_intervals(nxt)
        history.append(state)
    picks, i = [], 0
    for row in reversed(history):
        picks.append(row[i][3])
        i = row[i][2]
    return True, list(reversed(picks)), -1


def _prune_intervals(states: List[tuple], cap: int = 64) -> List[tuple]:
    """Drop every interval another already contains, then keep the widest."""
    kept: List[tuple] = []
    for st in sorted(states, key=lambda t: (t[0], -t[1])):
        if any(k[0] <= st[0] + 1e-12 and st[1] <= k[1] + 1e-12 for k in kept):
            continue
        kept.append(st)
    if len(kept) > cap:
        kept = sorted(kept, key=lambda t: t[0] - t[1])[:cap]
    return kept


def _required_ramp_band_deg(psi_unwrapped: np.ndarray, bounds, path_s, slopes,
                            band: float, ceiling: float = 180.0) -> Optional[float]:
    """The smallest band that would have carried this corridor, or None."""
    if not _ramp_fit(psi_unwrapped, bounds, path_s, slopes, ceiling)[0]:
        return None
    lo, hi = float(band), float(ceiling)
    for _ in range(24):
        mid = 0.5 * (lo + hi)
        if _ramp_fit(psi_unwrapped, bounds, path_s, slopes, mid)[0]:
            hi = mid
        else:
            lo = mid
    return hi


def _travel_report(psi_path: np.ndarray, bounds: Sequence[Tuple[int, int, int]],
                   path_s: Optional[np.ndarray], cap: Optional[float],
                   pitch: float):
    """Per-section swivel travel of a corridor: ``(sections, total, worst)``."""
    d = np.diff(psi_path)
    d = (d + 180.0) % 360.0 - 180.0
    rows = []
    worst = None
    for j, s0, s1 in bounds:
        travel = float(sum(abs(float(d[s - 1])) for s in range(s0 + 1, s1 + 1)))
        length = (None if path_s is None
                  else float(path_s[s1] - path_s[s0]))
        per_m = (None if not length or length <= 1e-9 else travel / length)
        if per_m is not None:
            worst = per_m if worst is None else max(worst, per_m)
        budget = _section_budget_deg(cap, length, pitch)
        rows.append({"section": j, "s0": s0, "s1": s1,
                     "lengthM": None if length is None else round(length, 4),
                     "travelDeg": round(travel, 2),
                     "degPerM": None if per_m is None else round(per_m, 1),
                     "budgetDeg": None if budget is None else round(budget, 2),
                     "subPitch": (None if budget is None
                                  else bool(float(cap) * float(length) < pitch)),
                     "withinCap": (None if budget is None
                                   else bool(travel <= budget + 1e-9))})
    return rows, float(np.abs(d).sum()) if len(d) else 0.0, worst


def _rounded_ramp_search(score: np.ndarray, q: np.ndarray, jstep: np.ndarray,
                         step_idx: int, path_s: np.ndarray, slopes: Sequence[float],
                         pitch: float, allow_branch_change: bool):
    """The widest corridor that IS a rounded ramp: ONE slope for the whole
    stroke, the anchor any cell of the grid.

    Every member of this family satisfies the runtime's rule by construction —
    psi is the nearest grid cell to a straight reference at every waypoint —
    so it is where a corridor whose shape no ramp can follow is looked for
    second. One slope and not one per section: the per-section freedom is the
    main DP's business, and this is a fallback that has to be cheap and
    certain rather than complete.

    Returns ``(path_p, path_b, margin)``, or ``(None, None, None)``.
    """
    S, P, B = score.shape
    eye = np.eye(B, dtype=bool)
    arc = path_s - path_s[0]
    rows = np.arange(S)[None, :]
    best = (None, None, None)
    for k in sorted(slopes, key=lambda v: (abs(v), v)):
        off = np.rint(float(k) * arc / pitch).astype(int)
        if S > 1 and int(np.abs(np.diff(off)).max()) > step_idx:
            continue                       # this slope is not a continuous step
        idx = (np.arange(P)[:, None] + off[None, :]) % P          # (P, S)
        sc = score[rows, idx]                                     # (P, S, B)
        qq = q[rows, idx]                                         # (P, S, B, 7)
        val = sc[:, 0, :].copy()
        parent = np.zeros((S, P, B), dtype=np.int32)
        for s in range(1, S):
            d = np.abs(qq[:, s][:, None, :, :] - qq[:, s - 1][:, :, None, :]).max(axis=-1)
            feasible = d <= jstep[s - 1]                          # (P, Bsrc, Btgt)
            if not allow_branch_change:
                feasible &= eye[None, :, :]
            vals = np.where(feasible, val[:, :, None], -np.inf)
            srcb = np.argmax(vals, axis=1)                         # (P, Btgt)
            top = np.take_along_axis(vals, srcb[:, None, :], axis=1)[:, 0, :]
            val = np.minimum(top, sc[:, s, :])
            parent[s] = srcb
        flat = int(np.argmax(val))
        p0, b = divmod(flat, B)
        margin = float(val[p0, b])
        if not np.isfinite(margin) or (best[2] is not None and margin <= best[2]):
            continue
        path_b = np.zeros(S, dtype=int)
        path_b[-1] = b
        for s in range(S - 1, 0, -1):
            b = int(parent[s, p0, b])
            path_b[s - 1] = b
        best = (idx[p0].copy(), path_b, margin)
    return best


def corridor(layer: AtlasLayer, *, max_swivel_step_deg: float = 15.0,
             max_joint_step_deg: float = 10.0,
             step_scale: Optional[Sequence[float]] = None,
             allow_branch_change: bool = False,
             swivel_travel_cap_deg_per_m: Optional[float] = None,
             swivel_slopes_deg_per_m: Optional[Sequence[float]] = None,
             swivel_ramp_band_deg: Optional[float] = None,
             path_s: Optional[Sequence[float]] = None,
             sections: Optional[Sequence[int]] = None,
             minimise_travel: bool = True,
             cell_cost: Optional[np.ndarray] = None,
             _diagnose: bool = True) -> Corridor:
    """Widest continuous swivel corridor across the stroke, or where it dies.

    An exact bottleneck dynamic program over ``(swivel, branch)``: it returns the
    path whose *worst* cell margin is as large as possible, which is the corridor
    a stroke should be authored down the middle of. Two constraints define
    "continuous":

    * ``max_swivel_step_deg`` — how far the swivel may move between consecutive
      waypoints. The swivel grid wraps, so a corridor may legitimately cross
      +-180 degrees.
    * ``max_joint_step_deg`` — how far any single joint may move. This is what
      forbids a jump between the disjoint Euler branches, and it catches the case
      the swivel step alone misses: two configurations at nearly the same swivel
      angle can still be a wrist flip apart.

    ``step_scale`` (length ``S-1``) multiplies the joint-step limit per step. A
    joint-step limit is a limit per unit of PATH, not per index, and a stroke's
    waypoints are not equally spaced — a boustrophedon's row change hops one row
    pitch where its along-row steps are a fraction of that. Holding one constant
    across both is twice as strict at the hop, and that asymmetry alone has
    failed strokes whose free swivel ring was 200 cells wide on either side of
    the break. Pass ``gap_s / nominal_gap`` and the limit follows the geometry.

    Neither is a velocity limit and neither comes from a task config; they are
    the discretisation's own continuity requirement, and both are reported back
    so a result is never read without them.

    **Swivel travel.** The runtime resolves the swivel as ONE linear ramp per
    section (a section ends at a reversal turn) whose slope it takes from a
    fixed set no steeper than 100 deg/m, so a corridor that wanders — the
    bottleneck objective is flat over a wide ring and its arbitrary tie-break
    wandered 302 deg on side_left — is a witness for a motion the runtime
    cannot make. Two things answer that here:

    * among the corridors that share the best worst-cell margin, the one with
      the least total swivel travel is returned (``minimise_travel``; an exact
      second pass, so the margin is unchanged);
    * ``swivel_travel_cap_deg_per_m`` bounds the travel of every section to
      ``cap x section length`` — no pitch of slack, so a section whose budget
      is under one grid cell may not turn at all (it is reported ``subPitch``:
      there the grid, not the cap, is what refuses). ``path_s`` (cumulative
      arc length per waypoint, metres) and ``sections`` (a section id per
      waypoint, constant within a section) supply the lengths; one section
      when none is given.
      The cap is enforced inside the search — a lower-margin corridor that
      keeps within it is preferred to a wider one that does not — and a stroke
      no corridor keeps within it is the ``travel`` failure, which names the
      section and the least travel any margin-optimal corridor needs there.

    A travel budget is a bound on a RATE and says nothing about SHAPE, so a
    corridor within it can still be one no ramp follows: a section that goes
    out and back spends little, and the next section's ramp carries on from
    where the last one ENDED rather than from the corridor. So the returned
    corridor is finally certified against the runtime's own rule — one
    straight ramp per section, its slope from ``swivel_slopes_deg_per_m``
    (default `RUNTIME_SWIVEL_SLOPES_DEG_PER_M`, filtered by the cap),
    continuous across sections, within ``swivel_ramp_band_deg`` (default half
    a grid cell, which is the rounding and nothing else) of the corridor at
    every waypoint. A corridor that fails it is retried inside a family that
    satisfies the rule by construction — the best rounding of a straight
    reference, over every anchor and every single slope
    (`_rounded_ramp_search`) — and refused as ``ramp`` when that finds none
    either. So the retry is a fallback and not a second search of the whole
    space: a shape needing DIFFERENT slopes per section that the main DP did
    not happen to find is refused, which errs toward the ladder rather than
    toward a certificate that does not hold. Both the cap and the certificate
    are only asked for when a cap is given: the cap is what says the stroke
    will be run in the ramp mode.

    Per-section travel is reported on every found corridor whether or not a
    cap was asked for, so the number the cap would judge is always visible.

    ``cell_cost`` (a PREFERENCE, e.g. `band_cost`) enters the least-travel pass
    only, so it can never refuse or lower the margin. KNOWN LIMITATION: it is
    honoured only while that pass's path survives the ramp check. When the
    travel optimum's SHAPE fails it, the fallback `_rounded_ramp_search` picks
    the widest-margin rounded ramp and does NOT see ``cell_cost``, so the
    preference is DROPPED SILENTLY and the corridor is the unbiased one.
    MEASURED 2026-09-21, row 7 right (seat_ring_right): `band_cost(within=True)`
    drove the travel pass to chase the band's outboard edge sample by sample
    (value -55.6), that shape failed the ramp rule, and the returned corridor
    was the no-preference one (psi -120, elbow 1.2 mm) -- bit-identical to
    passing no cost. The plain band (`within=False`) is costless inside the
    band, settles on a constant swivel and survives. Before relying on a
    preference, compare against ``cell_cost=None``; before turning
    `within=True` on, the fallback has to honour the cost (Newton's call).
    """
    S, P, B = layer.code.shape
    score = np.where(layer.code == FREE, layer.margin_deg, -np.inf).astype(float)
    q = np.nan_to_num(layer.q.astype(float), nan=1e6)

    step_idx = P if P < 2 else max(1, int(round(
        max_swivel_step_deg / (360.0 / P))))
    offsets = range(-step_idx, step_idx + 1)
    scale = (np.ones(max(0, S - 1)) if step_scale is None
             else np.asarray(step_scale, dtype=float).reshape(-1))
    if len(scale) != max(0, S - 1):
        raise ValueError(f"step_scale must have {S - 1} entries, got {len(scale)}")
    jstep = np.radians(max_joint_step_deg) * scale
    pitch = 360.0 / P if P else 360.0

    sec = (np.zeros(S, dtype=int) if sections is None
           else np.asarray(sections, dtype=int).reshape(-1))
    if len(sec) != S:
        raise ValueError(f"sections must have {S} entries, got {len(sec)}")
    ps = None if path_s is None else np.asarray(path_s, dtype=float).reshape(-1)
    if ps is not None and len(ps) != S:
        raise ValueError(f"path_s must have {S} entries, got {len(ps)}")
    cap = None if swivel_travel_cap_deg_per_m is None else float(swivel_travel_cap_deg_per_m)
    if cap is not None and ps is None:
        raise ValueError("a swivel travel cap needs path_s (arc length per waypoint)")
    boundary = np.zeros(S, dtype=bool)
    if S > 1:
        boundary[1:] = sec[1:] != sec[:-1]
    bounds = _section_bounds(sec, S)
    band = (0.5 * pitch if swivel_ramp_band_deg is None
            else float(swivel_ramp_band_deg))
    slope_set = (RUNTIME_SWIVEL_SLOPES_DEG_PER_M if swivel_slopes_deg_per_m is None
                 else tuple(float(v) for v in swivel_slopes_deg_per_m))
    # a slope the cap forbids is not a slope the runtime may take here; 0 is
    # always available (the `hold` the ramp mode falls back to)
    slopes = (() if cap is None else
              tuple(sorted({0.0} | {float(k) for k in slope_set
                                    if abs(float(k)) <= cap + 1e-9})))

    def refused(kind, at, reason, **extra):
        return Corridor(found=False, blocked_at=at, blocked_kind=kind,
                        blocked_reason=reason, max_swivel_step_deg=max_swivel_step_deg,
                        swivel_travel_cap_deg_per_m=cap, n_sections=len(set(sec.tolist())),
                        swivel_ramp_band_deg=None if cap is None else band, **extra)

    def dead_at(s):
        return refused("dead", s, _dead_reason(layer, s))

    def build(path_p, path_b, min_margin):
        psi_path = layer.psi_deg[path_p]
        q_path = layer.q[np.arange(S), path_p, path_b].astype(float)
        d = np.diff(psi_path)
        d = (d + 180.0) % 360.0 - 180.0
        unwrapped = _unwrap_deg(psi_path)
        rows, total, worst = _travel_report(psi_path, bounds, ps, cap, pitch)
        fit_ok, picks, _bad = (None, [], -1) if cap is None else _ramp_fit(
            unwrapped, bounds, ps, slopes, band)
        return Corridor(
            found=True, psi_deg=psi_path, branch=path_b, q=q_path,
            min_margin_deg=float(min_margin),
            max_swivel_step_deg=float(np.abs(d).max()) if len(d) else 0.0,
            swivel_range_deg=float(unwrapped.max() - unwrapped.min()),
            swivel_travel_deg=total, swivel_travel_sections=rows,
            swivel_travel_max_deg_per_m=worst, swivel_travel_cap_deg_per_m=cap,
            swivel_ramp_pass=fit_ok, swivel_ramp_slopes_deg_per_m=picks,
            swivel_ramp_band_deg=None if cap is None else band,
            n_sections=len(rows))

    def search(K=1, bins_of_step=None):
        """The bottleneck pass, then (optionally) the least-travel pass over
        the cells that keep its margin. ``(path_p, path_b, margin, blocked_at)``."""
        pp, pb, val, at = _corridor_dp(score, q, jstep, offsets, allow_branch_change,
                                       mode="bottleneck", K=K, bins_of_step=bins_of_step,
                                       boundary=boundary)
        if pp is None:
            return None, None, None, at
        if minimise_travel:
            tp, tb, _tv, _at = _corridor_dp(score, q, jstep, offsets, allow_branch_change,
                                            mode="travel", floor=val - 1e-9, K=K,
                                            bins_of_step=bins_of_step, boundary=boundary,
                                            cell_cost=cell_cost)
            if tp is not None:
                pp, pb = tp, tb
        return pp, pb, val, -1

    # ---- 1. the plain search: the verdict `step` / `dead` is decided here -----
    if not np.isfinite(score[0]).any():
        return dead_at(0)
    path_p, path_b, margin, at = search()
    if path_p is None:
        if not np.isfinite(score[at]).any():
            return dead_at(at)
        reason = ("no continuous step reaches this waypoint "
                  f"(swivel step <= {max_swivel_step_deg:g} deg, "
                  f"joint step <= {max_joint_step_deg:g} deg)")
        # A step-constrained failure is the one a caller can act on, so say
        # what it would actually have taken. Bisect the joint-step limit
        # rather than reading it off an unconstrained run: the unconstrained
        # optimum is free to jump between branches for a better margin, and
        # the size of that jump is not evidence that a continuous path needs
        # one. (It read as such in an earlier version and was wrong — the
        # stroke it declared unsalvageable was fine at a finer sampling.)
        need = None
        if _diagnose:
            need = _required_joint_step(layer, max_swivel_step_deg,
                                        max_joint_step_deg, allow_branch_change,
                                        step_scale=scale)
            if need is not None:
                reason += (f". A corridor exists once the joint-step limit reaches "
                           f"about {need:.0f} deg. That limit scales with how finely "
                           "the stroke is sampled, so resampling more finely is the "
                           "other way to meet it")
            else:
                reason += (". No joint-step limit rescues it: the free cells do not "
                           "span the stroke as one connected region")
        return refused("step", at, reason, required_joint_step_deg=need)
    cor = build(path_p, path_b, margin)
    if cap is None:
        return cor

    # ---- 2. the least-travel margin-optimal corridor breaks the cap: search
    # with the per-section budget IN the state, accepting a smaller margin ----
    if any(r["withinCap"] is False for r in cor.swivel_travel_sections):
        bins = {}
        for j, s0, s1 in bounds:
            budget = _section_budget_deg(cap, float(ps[s1] - ps[s0]), pitch)
            bins[j] = 0 if budget is None else int(np.floor(budget / pitch + 1e-9))
        bins_of_step = (np.array([bins[int(sec[s - 1])] for s in range(1, S)], dtype=int)
                        if S > 1 else np.zeros(0, dtype=int))
        # K = 1 would turn the tracking OFF, and every budget here is zero
        # exactly when the grid may not turn at all — the case to enforce
        K = max(2, int(max(bins.values())) + 1 if bins else 1)
        witness = cor
        path_p, path_b, margin, at = search(K=K, bins_of_step=bins_of_step)
        if path_p is None:
            rows = witness.swivel_travel_sections
            # the section to NAME is one that failed, by how far it overran —
            # a section of no arc length has no rate at all and would sort
            # last on one, which is how a zero-length overrun named its neighbour
            pool = ([r for r in rows if r["withinCap"] is False]
                    or [r for r in rows if r["degPerM"] is not None])
            worst = max(pool, key=lambda r: (r["travelDeg"] - (r["budgetDeg"] or 0.0),
                                             r["degPerM"] or 0.0), default=None)
            needs = ("" if worst is None or worst["degPerM"] is None else
                     f" needs {worst['degPerM']:.0f} deg/m")
            where = ("" if worst is None else
                     f": section {worst['section']} (samples {worst['s0']}-{worst['s1']}, "
                     f"{worst['lengthM']:.3f} m){needs} spends {worst['travelDeg']:g} deg of "
                     f"the {worst['budgetDeg']:g} its length allows, on the least-travel "
                     "corridor with the best margin"
                     + (" — a section that short may not turn at all, the grid "
                        "refuses before the cap does" if worst["subPitch"] else ""))
            return refused("travel", at,
                           f"no corridor keeps the swivel travel of every section within "
                           f"{cap:g} deg/m{where}",
                           required_swivel_travel_deg_per_m=(None if worst is None
                                                             else worst["degPerM"]),
                           swivel_travel_deg=witness.swivel_travel_deg,
                           swivel_travel_sections=witness.swivel_travel_sections,
                           swivel_travel_max_deg_per_m=witness.swivel_travel_max_deg_per_m)
        cor = build(path_p, path_b, margin)
    if cor.swivel_ramp_pass:
        return cor

    # ---- 3. within the budget, but no chain of ramps follows its SHAPE:
    # look inside the family that satisfies the rule by construction --------
    witness = cor
    fam_p, fam_b, fam_margin = _rounded_ramp_search(score, q, jstep, step_idx, ps,
                                                    slopes, pitch, allow_branch_change)
    if fam_p is not None:
        family = build(fam_p, fam_b, fam_margin)
        if family.swivel_ramp_pass:
            return family
    uw = _unwrap_deg(witness.psi_deg)
    _ok, _picks, bad = _ramp_fit(uw, bounds, ps, slopes, band)
    need = _required_ramp_band_deg(uw, bounds, ps, slopes, band)
    _j, s0, s1 = bounds[max(0, bad)]
    where = (f"section {_j} (samples {s0}-{s1}, {float(ps[s1] - ps[s0]):.3f} m)"
             if bad >= 0 else "the corridor as a whole")
    why = ("no band up to 180 deg carries its shape — psi turns back on itself "
           "inside a section, where a ramp may only go one way"
           if need is None else
           f"a band of {need:.1f} deg would carry it, where the grid gives {band:g}")
    return refused("ramp", int(s0),
                   f"no chain of one straight swivel ramp per section (slopes "
                   f"{', '.join(f'{k:g}' for k in slopes)} deg/m, continuous across "
                   f"sections) stays within {band:g} deg of a corridor here: {where} "
                   f"leaves the band first, and {why}",
                   required_swivel_ramp_band_deg=need,
                   swivel_travel_deg=witness.swivel_travel_deg,
                   swivel_travel_sections=witness.swivel_travel_sections,
                   swivel_travel_max_deg_per_m=witness.swivel_travel_max_deg_per_m,
                   swivel_ramp_pass=False)


def _required_joint_step(layer: AtlasLayer, swivel_step_deg: float,
                         from_deg: float, allow_branch_change: bool,
                         step_scale: Optional[Sequence[float]] = None) -> Optional[float]:
    """Smallest joint-step limit at which a corridor appears, or None.

    Bisection over the limit, each probe an ordinary (undiagnosed) corridor
    search. Monotone by construction — a corridor feasible at one limit is
    feasible at every larger one — so the bisection is exact to its tolerance.
    """
    hi = 360.0
    if not corridor(layer, max_swivel_step_deg=swivel_step_deg,
                    max_joint_step_deg=hi, step_scale=step_scale,
                    allow_branch_change=allow_branch_change,
                    minimise_travel=False, _diagnose=False).found:
        return None
    lo = float(from_deg)
    for _ in range(7):
        mid = 0.5 * (lo + hi)
        ok = corridor(layer, max_swivel_step_deg=swivel_step_deg,
                      max_joint_step_deg=mid, step_scale=step_scale,
                      allow_branch_change=allow_branch_change,
                      minimise_travel=False, _diagnose=False).found
        if ok:
            hi = mid
        else:
            lo = mid
    return hi


def _dead_reason(layer: AtlasLayer, s: int) -> str:
    """Why a waypoint has no free cell at all — the actionable half of a failure."""
    col = layer.code[s]
    if (col == OUT_OF_REACH).all():
        over = layer.wrist_distance_mm[s] - layer.max_reach_mm
        return (f"wrist centre {layer.wrist_distance_mm[s]:.1f} mm from the shoulder, "
                f"{over:.1f} mm beyond the {layer.max_reach_mm:.1f} mm the arm has — "
                "unreachable at every swivel, wipe yaw and wrist branch. Only "
                "standoff, pole height or the other arm changes this.")
    if (col == NO_SEED).all():
        return ("inside the wrist ball, but the numerical seed did not converge here — "
                "a solver result, not a proof")
    counts = {CODE_LABELS[c]: int((col == c).sum())
              for c in sorted(CODE_LABELS) if c != FREE and (col == c).any()}
    lead = max(counts, key=counts.get) if counts else "unknown"
    extra = ""
    if lead == CODE_LABELS[SINGULAR] and np.isfinite(layer.wrist_distance_mm[s]):
        extra = (f"; wrist centre {layer.wrist_distance_mm[s]:.1f} mm from the shoulder "
                 f"(elbow {layer.q4_deg[s]:.1f} deg) — a straight-elbow block is a "
                 "distance, so no swivel can clear it")
    return f"every swivel blocked, mostly by {lead} ({counts}){extra}"
