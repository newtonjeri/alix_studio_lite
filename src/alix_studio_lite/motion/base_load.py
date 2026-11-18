"""The load a moving arm puts on the UGV: the force and the moment its mount must supply to
move the arm's mass, from the robot model's own link masses and inertias (ALIX's URDF).

What is computed, for one arm following a joint trajectory with its pole and hand held still:

- the arm's linear momentum p = sum(m_i v_i) and its angular momentum about the UGV's base frame
  origin, L = sum(m_i r_i x v_i + I_i w_i), over the seven moving segments (each arm link with
  everything fixed to it; the hand and the camera ride on the last one);
- the load is their rate of change, F = dp/dt and M = dL/dt. The UGV feels the opposite, so the
  magnitudes are what matter: |F| in newtons, |M| in newton metres.

Gravity is left out on purpose: the arm's weight shifting over the UGV depends on the path only,
so no timing changes it. Acceleration sets the size of this load, jerk how fast it changes.

Along a fixed path q(x) the load is linear in the path acceleration and the squared path speed,
load = A(x) xdd + B(x) xd^2 (``path_coeffs``), which is what lets an optimiser hold it as a limit.

These are model figures: the URDF's masses are the arm maker's (6.06 kg for links 1 to 7, 0.62 kg
below the wrist), and nothing here has been measured on the real UGV.

What the load is held against (``ugv_stand``; the user, 18 Nov: "take the weight of the UGV with the
mass set in the URDF"): the UGV's own links weigh 98.3 kg in the URDF (its body's 90 kg, the one
figure set by hand, plus the suspensions, wheels and lower cover) and stand on four wheels 275 mm
apart front to back and 221 mm side to side (ALIX's support polygon, the wheel links' origins:
Resource/reports/2026-09-06_inertia_revision). That weight holds a pair of wheels down against a
moment of up to weight x the centre of mass's distance to that pair's line; the shortest distance
gives the smallest such moment (105.8 N m), and half of it (``UGV_MOMENT_SHARE``; the user, 18 Nov:
"say half") is the moment limit when a region sets none: 52.9 N m. The base frame's origin is in the
middle of the wheels, 25 mm above the floor (the wheel links' origins less ALIX's 85 mm wheel radius),
so the moment about it carries all but 25 mm of a sideways force's lever; an up-and-down force adds
its own times the distance to the wheel line. The limit leaves both out (``tip_shares`` reads the
moment about each wheel line with them, for a check). Sliding the UGV would
need the wheels' grip on the floor, which neither the URDF nor ALIX records, so the weight gives no
force limit of its own. The rest of the robot's weight and where the arms hang are left out, as
gravity is above: this is the dynamic load against the UGV's own weight.
"""
from __future__ import annotations

import xml.etree.ElementTree as ET
from dataclasses import dataclass
from functools import lru_cache

import numpy as np

from .. import vendor_paths
from .limits import side_prefix

POLE_M = 0.150                     # the pole height used when a file does not say (ALIX's half_length)
G = 9.81                           # m/s^2
UGV_LINKS = "UGV_"                 # the UGV's own links in the URDF: its body, lower cover, suspensions and wheels
UGV_MOMENT_SHARE = 0.5             # of the moment the UGV's weight holds: the limit when a region sets none (the user, 18 Nov)
WHEEL_RADIUS_M = 0.085             # ALIX's LAYERS.md and ARCHITECTURE.md (its wheel mesh reads 84.5 mm): the URDF states no radius


@dataclass(frozen=True)
class Segment:
    """One rigid moving part: an arm joint's child link and everything fixed to it."""
    link: str                      # the frame the part is described in
    mass: float                    # kg
    com: np.ndarray                # (3,) centre of mass in that frame, m
    inertia: np.ndarray            # (3, 3) about the centre of mass, in that frame's axes, kg m^2


def _rpy(r: float, p: float, y: float) -> np.ndarray:
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def _inertials(urdf_path) -> dict:
    """{link: (mass, centre of mass (3,), inertia about it in the link's axes (3, 3))}."""
    out = {}
    for link in ET.parse(str(urdf_path)).getroot().findall("link"):
        ine = link.find("inertial")
        if ine is None or ine.find("mass") is None:
            continue
        m = float(ine.find("mass").get("value"))
        if m <= 0:
            continue
        org = ine.find("origin")
        xyz = np.array([float(v) for v in (org.get("xyz") if org is not None and org.get("xyz") else "0 0 0").split()])
        rpy = [float(v) for v in (org.get("rpy") if org is not None and org.get("rpy") else "0 0 0").split()]
        t = ine.find("inertia")
        g = (lambda k: float(t.get(k, 0.0))) if t is not None else (lambda k: 0.0)
        i_own = np.array([[g("ixx"), g("ixy"), g("ixz")], [g("ixy"), g("iyy"), g("iyz")], [g("ixz"), g("iyz"), g("izz")]])
        r = _rpy(*rpy)
        out[link.get("name")] = (m, xyz, r @ i_own @ r.T)
    return out


def _lump(parts: list) -> tuple:
    """One body from several: (mass, centre of mass, inertia about it), all in the same frame."""
    m = sum(p[0] for p in parts)
    com = sum(p[0] * p[1] for p in parts) / m
    ine = np.zeros((3, 3))
    for pm, pc, pi in parts:
        d = pc - com
        ine += pi + pm * (float(d @ d) * np.eye(3) - np.outer(d, d))
    return m, com, ine


class ArmLoad:
    """One arm's moving mass and the load its motion puts on the UGV."""

    def __init__(self, side: str, pole_m: float = POLE_M, urdf_path=None):
        vendor_paths.activate()
        from alix_workspace.urdf_kinematics import UrdfModel

        self.side = side
        self.pole_m = float(pole_m)
        pre = side_prefix(side)
        path = urdf_path or vendor_paths.URDF
        self.model = UrdfModel.from_file(str(path))
        root = self.model.root_link()
        inert = _inertials(path)
        zero = self.model.link_world_transforms({})
        by_parent: dict = {}
        for jt in self.model.joints_by_child.values():
            by_parent.setdefault(jt.parent, []).append(jt)
        child_of = {jt.name: jt.child for jt in self.model.joints_by_child.values()}
        refs = [child_of[f"{pre}_joint{k}"] for k in range(1, 8)]
        self.segments: list[Segment] = []
        self.chains = []
        for k, ref in enumerate(refs):
            stop = refs[k + 1] if k < 6 else None
            links, stack = [], [ref]
            while stack:                                   # this joint's links, up to the next arm joint
                cur = stack.pop()
                links.append(cur)
                for jt in by_parent.get(cur, []):
                    if jt.child != stop:
                        stack.append(jt.child)
            t_ref_inv = np.linalg.inv(zero[ref])
            parts = []
            for name in links:
                if name not in inert:
                    continue
                m, c, i = inert[name]
                t = t_ref_inv @ zero[name]
                r = t[:3, :3]
                parts.append((m, r @ c + t[:3, 3], r @ i @ r.T))
            m, com, ine = _lump(parts)
            self.segments.append(Segment(ref, m, com, ine))
            self.chains.append(self.model.chain(root, ref))
        self.mass = float(sum(s.mass for s in self.segments))

    # ── where the moving mass is ────────────────────────────────────────────────────────────
    def frames(self, q_deg: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """(centres of mass (7, N, 3), segment rotations (7, N, 3, 3)) in the UGV's base frame."""
        q = np.radians(np.atleast_2d(np.asarray(q_deg, dtype=float)))
        n = q.shape[0]
        pole = np.full((n, 1), self.pole_m)
        com = np.empty((7, n, 3))
        rot = np.empty((7, n, 3, 3))
        for k, (seg, chain) in enumerate(zip(self.segments, self.chains)):
            t = self.model.fk_batch(chain, np.hstack([pole, q[:, :k + 1]]))
            rot[k] = t[:, :3, :3]
            com[k] = np.einsum("nij,j->ni", rot[k], seg.com) + t[:, :3, 3]
        return com, rot

    def _momentum(self, q_deg: np.ndarray, x) -> np.ndarray:
        """(N, 6) linear and angular momentum per unit rate of ``x`` (x = time: the momentum itself)."""
        com, rot = self.frames(q_deg)
        dc = np.gradient(com, x, axis=1)
        w_skew = np.gradient(rot, x, axis=1) @ np.swapaxes(rot, -1, -2)
        w = np.stack([w_skew[..., 2, 1], w_skew[..., 0, 2], w_skew[..., 1, 0]], axis=-1)
        out = np.zeros((com.shape[1], 6))
        for k, seg in enumerate(self.segments):
            i_world = rot[k] @ seg.inertia @ np.swapaxes(rot[k], -1, -2)
            out[:, :3] += seg.mass * dc[k]
            out[:, 3:] += seg.mass * np.cross(com[k], dc[k]) + np.einsum("nij,nj->ni", i_world, w[k])
        return out

    # ── the load ────────────────────────────────────────────────────────────────────────────
    def wrench(self, q_deg: np.ndarray, dt: float) -> tuple[np.ndarray, np.ndarray]:
        """(force (N, 3) in newtons, moment (N, 3) in newton metres about the UGV base frame's origin)
        for a trajectory sampled every ``dt`` seconds. Give it a band-limited trajectory: integer
        millidegrees differentiated twice are noise."""
        q = np.atleast_2d(np.asarray(q_deg, dtype=float))
        if q.shape[0] < 5:
            return np.zeros((q.shape[0], 3)), np.zeros((q.shape[0], 3))
        rate = np.gradient(self._momentum(q, float(dt)), float(dt), axis=0)
        return rate[:, :3], rate[:, 3:]

    def path_coeffs(self, q_deg: np.ndarray, x: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """(A, B), each (K, 6), for the path's knots ``q_deg`` at the path coordinate ``x``:
        [force, moment] = A xdd + B xd^2 at every knot."""
        x = np.asarray(x, dtype=float)
        a = self._momentum(q_deg, x)
        return a, np.gradient(a, x, axis=0)


@lru_cache(maxsize=8)
def arm_load(side: str, pole_m: float = POLE_M) -> ArmLoad:
    return ArmLoad(side, pole_m)


def magnitudes(q_deg: np.ndarray, dt: float, side: str, pole_m: float = POLE_M) -> tuple[np.ndarray, np.ndarray]:
    """(|force| (N,), |moment| (N,)) for a trajectory sampled every ``dt``."""
    f, m = arm_load(side, round(float(pole_m), 4)).wrench(q_deg, dt)
    return np.linalg.norm(f, axis=1), np.linalg.norm(m, axis=1)


@dataclass(frozen=True)
class UgvStand:
    """The UGV's weight on its wheels, from the URDF's own masses and wheel places."""
    mass_kg: float                 # the UGV's links together
    body_kg: float                 # of which UGV_body: the one figure set by hand
    weight_n: float
    com: tuple                     # its centre of mass in the base frame (x, y, z), m
    wheels: tuple                  # the wheel links' origins (x, y), m: the corners of what it stands on
    levers_m: dict                 # {"front" | "rear" | "left" | "right": the centre of mass's distance to that wheel line}
    lever_m: float                 # the shortest of them
    tip_moment_nm: float           # weight x lever_m: beyond it the weight alone no longer holds that wheel pair down
    floor_z: float = 0.0           # the floor's height in the base frame, m: the wheel links' origins less the wheel radius

    def view(self) -> dict:
        return dict(mass_kg=round(self.mass_kg, 1), body_kg=round(self.body_kg, 1), weight_n=round(self.weight_n, 0),
                    lever_mm=round(self.lever_m * 1000.0, 0), tip_moment_nm=round(self.tip_moment_nm, 1),
                    share=UGV_MOMENT_SHARE, limit_nm=weight_moment_limit_nm(),
                    levers_mm={k: round(v * 1000.0, 0) for k, v in self.levers_m.items()})


@lru_cache(maxsize=2)
def ugv_stand(urdf_path=None) -> UgvStand:
    """The UGV's weight and what it stands on, read from the URDF (all joints at zero)."""
    vendor_paths.activate()
    from alix_workspace.urdf_kinematics import UrdfModel

    path = urdf_path or vendor_paths.URDF
    zero = UrdfModel.from_file(str(path)).link_world_transforms({})
    inert = _inertials(path)
    mine = {name: v for name, v in inert.items() if name.startswith(UGV_LINKS)}
    wheels = sorted(name for name in mine if name.endswith("_wheel"))
    if len(wheels) < 3:
        raise ValueError(f"the URDF sets {len(wheels)} UGV wheel(s): no stand to take the UGV's weight on")
    mass = float(sum(m for m, _c, _i in mine.values()))
    com = sum(m * (zero[name][:3, :3] @ c + zero[name][:3, 3]) for name, (m, c, _i) in mine.items()) / mass
    xy = np.array([zero[name][:2, 3] for name in wheels])
    mid = xy.mean(axis=0)
    ring = xy[np.argsort(np.arctan2(xy[:, 1] - mid[1], xy[:, 0] - mid[0]))]      # the corners, going round
    levers = {}
    for a, b in zip(ring, np.roll(ring, -1, axis=0)):
        along = (b - a) / np.linalg.norm(b - a)
        out = np.array([along[1], -along[0]])                                    # the line's normal, away from the middle
        side = (a + b) / 2.0 - mid
        name = ("front" if side[0] > 0 else "rear") if abs(side[0]) > abs(side[1]) else ("left" if side[1] > 0 else "right")
        levers[name] = float((a - com[:2]) @ out)
    lever = min(levers.values())
    if lever <= 0:
        raise ValueError("the URDF's UGV centre of mass is not over its wheels")
    weight = mass * G
    floor = float(min(zero[name][2, 3] for name in wheels)) - WHEEL_RADIUS_M
    return UgvStand(mass, float(mine.get("UGV_body", (0.0,))[0]), weight, tuple(float(v) for v in com),
                    tuple((float(x), float(y)) for x, y in ring), levers, lever, weight * lever, floor)


def weight_moment_limit_nm() -> float:
    """The moment limit the UGV's own weight gives: half of what it holds (ugv_stand, UGV_MOMENT_SHARE). It is the
    limit when a region sets none."""
    return round(UGV_MOMENT_SHARE * ugv_stand().tip_moment_nm, 1)


def tip_shares(force: np.ndarray, moment: np.ndarray, stand: UgvStand | None = None) -> dict:
    """{"front" | "rear" | "left" | "right": (N,) the load's moment about that wheel line, as a share of what the
    UGV's weight holds about it}, for a force (N, 3) and a moment (N, 3) about the base frame's origin (``wrench``).
    The wheel lines are taken on the floor (``floor_z``, a little under the base frame's origin). Both signs are
    counted for every line (one tips over it, the other presses it down), so a share is never read low; a turning
    moment about the upright axis tips nothing and is left out."""
    st = stand or ugv_stand()
    f, m = np.asarray(force, dtype=float), np.asarray(moment, dtype=float)
    xs, ys = [x for x, _y in st.wheels], [y for _x, y in st.wheels]
    g = st.floor_z
    # the moment about a point P of the line is M - P x F: for a line along x through (0, d, g) its x part is
    # Mx - d Fz + g Fy, for a line along y through (a, 0, g) its y part is My + a Fz - g Fx
    along_x = lambda d: m[:, 0] - d * f[:, 2] + g * f[:, 1]                    # noqa: E731
    along_y = lambda a: m[:, 1] + a * f[:, 2] - g * f[:, 0]                    # noqa: E731
    lines = dict(left=along_x(max(ys)), right=along_x(min(ys)), front=along_y(max(xs)), rear=along_y(min(xs)))
    return {name: np.abs(v) / (st.weight_n * st.levers_m[name]) for name, v in lines.items()}
