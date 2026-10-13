"""
dual_arm_plan_collision.py
==========================
Detect inter-arm (left-vs-right) collisions when TWO single-arm plans are executed
SIMULTANEOUSLY, using FCL (python-fcl via trimesh) — no ROS, no MoveIt.

Input: two plan JSON files (as saved by the cleaning/mechanism tasks under
``plans/`` and ``Resource/plans/``), one for the left arm and one for the right.
Both are assumed to start at t=0 and run concurrently. Each plan's per-stage
``sub_trajectories`` are concatenated into a continuous joint timeline (each joint
carries its last value forward); the two timelines are sampled on a common time
grid; at each sample the FULL robot is FK-posed (``UrdfModel.link_world_transforms``)
and the LEFT link set is collision-checked against the RIGHT link set.

Only INTER-arm collisions are reported. Same-arm adjacency never arises (the two
sets are disjoint), and any pair listed in the SRDF ``<disable_collisions>`` tags
is ignored (so statically-close torso/base pairs don't false-positive). Links
without collision geometry are skipped — same as MoveIt.

This is a visualization/diagnostic aid: decimated collision meshes, no link
padding and a hand-rolled ACM mean results need not exactly match MoveIt.
"""

from __future__ import annotations

import bisect
import functools
import json
import logging
import math
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np

logger = logging.getLogger("alix_workspace.dual_arm_plan_collision")

# Default robot description (source tree, portable — no install/share dependency).
_DEFAULT_URDF = Path(__file__).resolve().parent / "urdf" / "alix.urdf"


# ---------------------------------------------------------------------------
# Plan parsing
# ---------------------------------------------------------------------------
def load_plan_timeline(plan_path: str) -> Tuple[List[float], List[Dict[str, float]]]:
    """Concatenate a plan's per-stage sub_trajectories into one timeline.

    Returns ``(times, rows)`` where ``times[i]`` is the absolute time (s) from the
    plan start and ``rows[i]`` is the cumulative joint-value map at that instant
    (every joint seen so far holds its most recent value).
    """
    data = json.loads(Path(plan_path).read_text())
    all_subs = data.get("sub_trajectories", [])
    subs = [st for st in all_subs if st.get("joint_names") and st.get("waypoints")]
    dropped = len(all_subs) - len(subs)
    declared = data.get("num_sub_trajectories")
    if dropped or (declared is not None and int(declared) != len(all_subs)):
        # Previously silent: stages without joint_names/waypoints were skipped
        # and num_sub_trajectories routinely over-counts (e.g. 26 declared vs
        # 22 present) — surface it so "checked the whole plan" is honest.
        logger.warning(
            "%s: using %d/%d sub_trajectories (%d without joints/waypoints"
            "%s)", Path(plan_path).name, len(subs), len(all_subs), dropped,
            f"; header declares {declared}" if declared is not None else "")

    times: List[float] = []
    rows: List[Dict[str, float]] = []
    running: Dict[str, float] = {}
    t_offset = 0.0

    for st in subs:
        names = st["joint_names"]
        wps = st["waypoints"]
        stage_end = 0.0
        for wp in wps:
            rel_t = float(wp.get("time_from_start", 0.0))
            pos = wp.get("positions", [])
            for i, n in enumerate(names):
                if i < len(pos):
                    running[n] = float(pos[i])
            times.append(t_offset + rel_t)
            rows.append(dict(running))
            stage_end = max(stage_end, rel_t)
        t_offset += stage_end  # next stage starts where this one ended

    return times, rows


def _sample_at(times: List[float], rows: List[Dict[str, float]], t: float) -> Dict[str, float]:
    """Linearly interpolate the joint map at time ``t`` (clamped to the ends)."""
    if not times:
        return {}
    if t <= times[0]:
        return dict(rows[0])
    if t >= times[-1]:
        return dict(rows[-1])
    j = bisect.bisect_right(times, t)
    t0, t1 = times[j - 1], times[j]
    r0, r1 = rows[j - 1], rows[j]
    a = (t - t0) / (t1 - t0) if t1 > t0 else 0.0
    out = dict(r0)
    for k, v in r1.items():
        base = r0.get(k, v)
        out[k] = base + a * (v - base)
    return out


# ---------------------------------------------------------------------------
# SRDF disabled pairs
# ---------------------------------------------------------------------------
def load_disabled_pairs(srdf_path: str) -> set:
    """Parse ``<disable_collisions link1= link2=>`` into a set of frozenset pairs."""
    disabled = set()
    try:
        root = ET.parse(srdf_path).getroot()
        for dc in root.iter("disable_collisions"):
            l1, l2 = dc.get("link1"), dc.get("link2")
            if l1 and l2:
                disabled.add(frozenset((l1, l2)))
    except Exception as exc:  # noqa: BLE001
        logger.warning("Could not parse disable_collisions from %s: %s", srdf_path, exc)
    return disabled


# ---------------------------------------------------------------------------
# Per-arm collision manager (meshes added once; only transforms update per frame)
# ---------------------------------------------------------------------------
class _ArmCollision:
    """Wraps a trimesh CollisionManager for one arm's links (transform-cached)."""

    def __init__(self, rcm, link_filter, boxes=None):
        import trimesh
        self.mgr = trimesh.collision.CollisionManager()
        self.entries = []  # (obj_name, link, origin 4x4)
        for link, geoms in rcm.links.items():
            if not link_filter(link):
                continue
            for gi, geom in enumerate(geoms):
                mesh = rcm._load_mesh(geom)
                if mesh is None:
                    continue
                name = f"{link}#{gi}"
                self.mgr.add_object(name, mesh)
                self.entries.append((name, link, np.asarray(geom.origin, dtype=float)))
        # A box rides its root link exactly like a mesh rides its own: registering
        # it with (root, T_box) means `pose()` needs no special case, and because
        # the hand is rigid after joint 7 the box is valid at every arm
        # configuration without refitting.  `boxes`: [(name, root_link, T_box, extents)].
        for name, root, T_box, extents in (boxes or ()):
            self.mgr.add_object(name, trimesh.creation.box(extents=extents))
            self.entries.append((name, root, np.asarray(T_box, dtype=float)))

    def pose(self, transforms: Dict[str, np.ndarray]) -> None:
        for name, link, origin in self.entries:
            T = transforms.get(link)
            if T is not None:
                self.mgr.set_transform(name, T @ origin)

    @property
    def link_count(self) -> int:
        return len({link for _, link, _ in self.entries})


# Whitelist: only arm structural links + the bare palm (no base, rubbers, fingers).
# Perf (2026-08-01): the old prefix filter (L_|ILH / R_|IRH) pulled in ~42 links
# per arm — every finger segment, rubber pad, tip, and hand base — making the
# per-sample collision check ~5,500 link pairs.  This slim set (~13/arm) drops
# it to ~500 pairs/sample while keeping the collision pairs that matter.
_ARM_LINKS_LEFT  = frozenset({"L_CameraHolderLink", "L_ConnectorLink"} |
                              {f"L_Link{i}" for i in range(1, 8)})
_ARM_LINKS_RIGHT = frozenset({"R_CameraHolderLink", "R_ConnectorLink", "camera_link"} |
                              {f"R_Link{i}" for i in range(1, 8)})
_PALM = {"left": frozenset({"ILH_plam"}), "right": frozenset({"IRH_plam"})}
_WHITELIST = {"left": _ARM_LINKS_LEFT | _PALM["left"],
              "right": _ARM_LINKS_RIGHT | _PALM["right"]}


def _arm_filter(side: str):
    """Return a predicate that accepts only arm links + palm for *side*."""
    ok = _WHITELIST[side]
    return lambda link: link in ok


# ---------------------------------------------------------------------------
# Hand envelopes — one box in place of 31 finger meshes
# ---------------------------------------------------------------------------
#: How much bigger than the fitted bounding box each envelope is, per DIMENSION
#: (length, width and height each grow by this — half of it on every face).
#: Newton, 2026-09-17: "a union of cuboids fitting the hands at full open and
#: grasped, a little bit bigger than the mesh bounding box would be the best and
#: cheapest collision checking solution, bigger by almost .5cm in all dimensions"
#: — "at the end of the day you do not want to move too close to the fingers, or
#: the hand of the other arm".  So the growth is a CLEARANCE INTENT, not slop:
#: a box that reads closer than the meshes is reporting what he asked for.
#:
#: `alix_taskgen.body_collision` fits the same box for the IDLE PARTNER hand and
#: carries its own copy of this constant; that copy should delegate here (this
#: module is the lower layer — body_collision already imports from it).  Until it
#: does, the two must not be allowed to drift: `test_hand_envelope_parity`.
HAND_ENVELOPE_GROWTH_M = 0.005

_HAND_PREFIX = {"left": "ILH_", "right": "IRH_"}


#: Everything rigid after joint 7. The root is `CameraHolderLink`, NOT `Link7`:
#: joint 7 is `{L,R}_joint7`, whose CHILD is `CameraHolderLink`; `Link7` hangs off
#: it by a fixed joint, as do the camera and (through `ConnectorLink`) the whole
#: hand. Rooting at `Link7` — the first thing this code did — silently leaves the
#: camera holder and `camera_link` OUTSIDE the group and boxes one link too few.
DISTAL_ROOT = {"left": "L_CameraHolderLink", "right": "R_CameraHolderLink"}


def _fit_envelope(model, rcm, links, root, joints):
    """`(root, T_root_box, extents)` — the AABB of *links* in *root*'s frame at
    *joints*, grown by `HAND_ENVELOPE_GROWTH_M` per dimension. ``None`` if empty."""
    tf = model.link_world_transforms(dict(joints))
    have = [l for l in links if l in tf and l in rcm.links]
    if not have or root not in tf:
        return None
    to_root = np.linalg.inv(tf[root])
    pts = []
    for link in have:
        for geom in rcm.links[link]:
            mesh = rcm._load_mesh(geom)
            if mesh is None or not len(mesh.vertices):
                continue
            M = to_root @ tf[link] @ np.asarray(geom.origin, dtype=float)
            V = np.asarray(mesh.vertices, dtype=float)
            pts.append(V @ M[:3, :3].T + M[:3, 3])
    if not pts:
        return None
    P = np.vstack(pts)
    half = HAND_ENVELOPE_GROWTH_M / 2.0
    lo, hi = P.min(axis=0) - half, P.max(axis=0) + half
    T_box = np.eye(4)
    T_box[:3, 3] = (lo + hi) / 2.0
    return root, T_box, hi - lo


def distal_envelope(model, rcm, srdf, side: str, state: str,
                    urdf_path: Optional[str] = None):
    """One box for EVERY link after joint 7 — flange, connector, camera holder,
    camera and the whole hand — in `L_Link7`/`R_Link7`'s frame.

    Rigid by construction: nothing between joint 7 and the fingertips moves once
    the hand state is fixed, so this box is valid at every arm configuration.

    It is strictly more conservative than `hand_envelope` and it costs the
    verdict its resolution: a contact reports `distal_envelope`, not
    `camera_link` or a named finger. Measure before preferring it —
    `Resource/reports/2026-09-18_hand_envelopes/`.
    """
    root = DISTAL_ROOT[side]
    links = _side_descendants(str(urdf_path or _DEFAULT_URDF), root)
    try:
        joints = srdf.states_in_group(f"inspire_hand_{side}").get(state) or {}
    except Exception:  # noqa: BLE001
        return None
    if not joints:
        return None
    return _fit_envelope(model, rcm, links, root, joints)


def distal_envelope_box(model, rcm, srdf, side: str, state: str,
                        urdf_path: Optional[str] = None):
    """`distal_envelope` packaged for `_ArmCollision(boxes=...)`."""
    got = distal_envelope(model, rcm, srdf, side, state, urdf_path)
    if got is None:
        return None
    root, T_box, extents = got
    return (f"{_HAND_PREFIX[side]}distal_envelope#{state}", root, T_box, extents)


def hand_envelope_over(rcm, side: str, transforms_list, urdf_path: Optional[str] = None):
    """`(root, T_root_box, extents)` covering the hand at EVERY pose in
    *transforms_list* — the union of its per-pose AABBs in the hand's own frame.

    THIS IS THE ONE TO USE AS A SCREEN. A box fitted at a single state bounds the
    meshes only at THAT state, and a cleaning plan opens and closes the hand
    (0.0 -> 1.3 rad on the drivers, with the distal joints following by
    `<mimic>`). A single-state box would therefore fail to contain a more-open
    hand, and a screen that does not contain what it screens can MISS a contact.
    Taking the union over the poses actually evaluated removes that failure mode
    by construction: whatever the hand does at a node, the box holds it.

    The union is over the poses given, so it is sound for exactly those poses —
    pass the same node set the grid will evaluate, not a sample of it.
    """
    prefix = _HAND_PREFIX[side]
    links = [l for l in rcm.links if l.startswith(prefix)]
    root = f"{prefix}base_link"
    verts = {}
    for link in links:
        for gi, geom in enumerate(rcm.links[link]):
            mesh = rcm._load_mesh(geom)
            if mesh is None or not len(mesh.vertices):
                continue
            verts[(link, gi)] = (np.asarray(mesh.vertices, dtype=float),
                                 np.asarray(geom.origin, dtype=float))
    if not verts:
        return None
    lo = np.full(3, np.inf)
    hi = np.full(3, -np.inf)
    seen = 0
    for T in transforms_list:
        if root not in T:
            continue
        to_root = np.linalg.inv(T[root])
        seen += 1
        for (link, _gi), (V, origin) in verts.items():
            Tl = T.get(link)
            if Tl is None:
                continue
            M = to_root @ Tl @ origin
            P = V @ M[:3, :3].T + M[:3, 3]
            lo = np.minimum(lo, P.min(axis=0))
            hi = np.maximum(hi, P.max(axis=0))
    if not seen or not np.isfinite(lo).all():
        return None
    half = HAND_ENVELOPE_GROWTH_M / 2.0
    lo, hi = lo - half, hi + half
    T_box = np.eye(4)
    T_box[:3, 3] = (lo + hi) / 2.0
    return root, T_box, hi - lo


def hand_envelope(model, rcm, srdf, side: str, state: str):
    """``(root_link, T_root_box, extents)`` — the whole hand's bounding box at
    *state*, in the hand's own base frame, grown by `HAND_ENVELOPE_GROWTH_M`.

    Axis-aligned in the hand root's frame, so it rides the arm with no extra
    machinery: registered with the root link and `T_root_box` as its origin, it
    poses exactly like any mesh entry.

    ONE STATE OR SEVERAL, and the difference is the point.  A hand nobody
    commands may be in any of its states, so it needs the UNION of their boxes —
    that is the idle-partner case `body_collision` handles.  A hand that is
    WIPING has its state fixed by the config (through `ik_frame` / the glove
    frame), so it takes the single box for that state, which is far tighter: the
    union of an open hand and a grasped one contains all the air between spread
    fingers.  This function answers one state; the caller decides how many.

    Returns ``None`` when the state is unknown or the hand's links do not
    resolve to exactly one root — never a guess.
    """
    prefix = _HAND_PREFIX[side]
    try:
        joints = srdf.states_in_group(f"inspire_hand_{side}").get(state) or {}
    except Exception:  # noqa: BLE001
        return None
    if not joints:
        return None
    tf = model.link_world_transforms(dict(joints))
    by_child = getattr(model, "joints_by_child", {}) or {}
    hand = [l for l in rcm.links if l.startswith(prefix) and l in tf]
    roots = [l for l in hand
             if not str(getattr(by_child.get(l), "parent", "")).startswith(prefix)]
    if not hand or len(roots) != 1:
        return None
    root = roots[0]
    to_root = np.linalg.inv(tf[root])
    pts = []
    for link in hand:
        for geom in rcm.links[link]:
            mesh = rcm._load_mesh(geom)
            if mesh is None or not len(mesh.vertices):
                continue
            M = to_root @ tf[link] @ np.asarray(geom.origin, dtype=float)
            V = np.asarray(mesh.vertices, dtype=float)
            pts.append(V @ M[:3, :3].T + M[:3, 3])
    if not pts:
        return None
    P = np.vstack(pts)
    half = HAND_ENVELOPE_GROWTH_M / 2.0
    lo, hi = P.min(axis=0) - half, P.max(axis=0) + half
    T_box = np.eye(4)
    T_box[:3, 3] = (lo + hi) / 2.0
    return root, T_box, hi - lo


# ---------------------------------------------------------------------------
# Main entry point
# ---------------------------------------------------------------------------
def _build_home(rcm, srdf_path: Optional[str],
                left_hand_state: Optional[str] = None,
                right_hand_state: Optional[str] = None) -> Dict[str, float]:
    """Seed the joints the arm plans DON'T carry, so FK matches the real running
    scene: initial positions + bin LOWERED (rail extended + dropped) + pole
    full_length. Single source of truth shared by the collision sweep and the
    plain frame generator (compute_plan_frames).

    THE HAND IS ONE OF THOSE JOINTS, AND ITS DEFAULT IS WRONG FOR A WIPE.
    `initial_positions.yaml` leaves all 24 hand joints at 0.0 = FULLY OPEN, and a
    plan carries no hand joints at all (`load_plan_timeline` on every recorded
    plan), so without a state named here the sweep measures a pose nothing ever
    commanded — with spread fingers reaching further than any real wiping hand.

    Measured, 2026-09-18, row 10 (`seat_ring_bottom` R||L, the two recorded
    `ruckig_pro_only` plans, aligned, dt 0.02):

        fully open (the default)  50.77 mm, closest pair thumb_4 <-> thumb_4
        `grasp` (a hand on a pad) 77.38 mm, closest pair L_Link3 <-> R_Link4

    Both the margin AND which links are closest change. Pass the state the task
    actually holds; leave it None only to reproduce an older number on purpose.
    `Resource/reports/2026-09-18_hand_envelopes/` §4.

    THE POLE IS THE SAME TRAP, AND IT BITES A CORRIDOR RATHER THAN A PLAN.
    This seeds `dual_poles full_length` = **0.300 m**, which is harmless for a
    plan sweep — every recorded plan carries its own `<SIDE>_sliding_plate_joint`
    (0.075 / 0.150, matching its config) and overrides this on `q.update(sample)`.
    A corridor from `rd.corridor` carries `joint1..7` and NO pole, so a caller
    that poses `cor.q` on top of this home leaves the arm at 0.300 while its IK
    was solved at whatever its config declares. Verified by FK against the
    requested cleaning points, every sample: `sliding_plate = oracle.pole_m`
    lands **0.00 mm** from target, `= 0.300` misses by **exactly 225.00 mm**
    (`top_right`, 0.075) and **exactly 150.00 mm** (`bottomlid_left`, 0.150).
    It is a pure vertical translation, so nothing throws and nothing looks wrong
    — and on a DUAL-ARM sweep it changes the very gap being measured whenever the
    two sides declare different heights. Set the pole explicitly for any
    GENERATED side. Note also that the live harness seeds both plates at 0.15
    (`body_collision` documents this), so 0.300 is not the runtime's world
    either. 2026-09-18: this silently invalidated six analysis scripts.
    """
    from alix_workspace.srdf_states import SrdfStates, load_initial_positions
    try:
        home = load_initial_positions(rcm.index)
    except Exception:  # noqa: BLE001
        home = {}
    if srdf_path:
        try:
            srdf = SrdfStates.from_file(srdf_path)
            home.update(srdf.group_state("bin_rail", "BIN_LOWERED"))
            home.update(srdf.group_state("dual_poles", "full_length"))
        except Exception:  # noqa: BLE001
            logger.warning("Could not seed bin/pole state from SRDF %s", srdf_path)
        for side, state in (("left", left_hand_state), ("right", right_hand_state)):
            if not state:
                continue
            try:
                srdf = SrdfStates.from_file(srdf_path)
                joints = srdf.states_in_group(f"inspire_hand_{side}").get(state)
            except Exception:  # noqa: BLE001
                joints = None
            if not joints:
                raise ValueError(
                    f"no SRDF state {state!r} in group inspire_hand_{side} — the hand "
                    f"pose decides the verdict, so an unknown state is refused rather "
                    f"than silently left fully open")
            home.update(joints)
    return home


def _overlay_link_names(rcm, urdf_path: str) -> List[str]:
    """Sorted union of collision + visual link names — every link the web overlay
    poses, so visual-only parts that ride the arms (camera, finger pads) animate
    too instead of floating at home. Only names are read (no mesh load)."""
    from alix_workspace.collision import RobotCollisionModel
    overlay = set(rcm.links.keys())
    try:
        overlay |= set(RobotCollisionModel.from_file(urdf_path, tag="visual").links.keys())
    except Exception:  # noqa: BLE001
        logger.warning("Could not read visual link names; posing collision links only")
    return sorted(overlay)


def _concat_timelines(plans: List[str], start: float = 0.0
                      ) -> Tuple[List[float], List[Dict[str, float]]]:
    """Concatenate a SEQUENCE of plans into one timeline (plan k+1 begins where
    plan k ended), offset by ``start``. Joints carry forward across plan
    boundaries. Models one arm's chain (or a single plan when the list has one)."""
    times: List[float] = []
    rows: List[Dict[str, float]] = []
    running: Dict[str, float] = {}
    offset = float(start)
    for p in plans:
        tt, rr = load_plan_timeline(p)
        for j, tv in enumerate(tt):
            running.update(rr[j])
            times.append(offset + tv)
            rows.append(dict(running))
        offset += (tt[-1] if tt else 0.0)
    return times, rows


def compute_plan_frames(
    plans: Optional[List[str]] = None,
    tracks: Optional[List[dict]] = None,
    urdf_path: Optional[str] = None,
    srdf_path: Optional[str] = None,
    dt: float = 0.1,
    initial: Optional[Dict[str, float]] = None,
    extra_links: Optional[List[str]] = None,
) -> dict:
    """FK-pose the robot along one or more concurrent tracks and return per-sample
    link transforms for the web overlay — WITHOUT the inter-arm collision sweep.
    The SAME frame shape the DAC playback consumes, so the overlay renders it
    unchanged. Covers every schedule step type:

      • single / compound : one track, one plan (compound carries both arms).
      • paired            : two tracks, one plan each, both starting at t=0.
      • chained           : two tracks, each a plan SEQUENCE (per-arm chain).
      • serial            : two tracks with the second offset by ``start``.

    Inputs (one of):
      ``plans``  — flat list of plan paths, each treated as its own concurrent
                   track starting at t=0 (single / compound / paired).
      ``tracks`` — list of ``{"plans": [path, ...], "start": float}`` for chains /
                   staggered starts.

    ``initial`` overlays the home pose with carried-over joint values (the state
    the robot was left in by the PREVIOUS schedule steps), so joints these plans
    don't drive hold their pose instead of snapping back to home/ready.

    ``extra_links`` adds geometry-less URDF links (e.g. the glove/tool IK frames)
    to every frame, so callers can track tool-frame positions along the plan.

    Returns ``dt``, ``num_samples``, ``sample_times``, ``frames``
    (``{sample_index: {link: flat 4x4}}``), ``mesh_links`` and ``final_joints``
    (the joint values these plans end at — feed it into the next step's
    ``initial`` to chain steps seamlessly)."""
    from alix_workspace.urdf_kinematics import UrdfModel
    from alix_workspace.collision import RobotCollisionModel
    from alix_workspace.srdf_states import find_srdf

    if tracks is None:
        plist = [p for p in (plans or []) if p]
        if not plist:
            raise ValueError("compute_plan_frames: no plan given")
        tracks = [{"plans": [p], "start": 0.0} for p in plist]

    urdf_path = str(urdf_path or _DEFAULT_URDF)
    model = UrdfModel.from_file(urdf_path)
    rcm = RobotCollisionModel.from_file(urdf_path, tag="collision")
    srdf_path = srdf_path or find_srdf(str(Path(urdf_path).parent))
    base = _build_home(rcm, srdf_path)
    if initial:
        base.update({k: float(v) for k, v in initial.items()})
    mesh_links = _overlay_link_names(rcm, urdf_path)
    pose_links = list(mesh_links)
    for ln in (extra_links or []):
        if ln not in pose_links:
            pose_links.append(ln)

    track_tl = [
        _concat_timelines([p for p in (tr.get("plans") or []) if p], tr.get("start", 0.0))
        for tr in tracks
    ]
    total = max((t[-1] if t else 0.0) for t, _ in track_tl) if track_tl else 0.0
    n_samples = int(round(total / dt)) + 1 if total > 0 else 1

    sample_times: List[float] = []
    frames: Dict[str, Dict[str, list]] = {}
    for i in range(n_samples):
        t = i * dt
        q = dict(base)
        for times, rows in track_tl:
            if times:
                q.update(_sample_at(times, rows, t))
        sample_times.append(round(t, 4))
        T = model.link_world_transforms(q)
        frames[str(i)] = {
            ln: [round(float(x), 6) for x in T[ln].reshape(-1)]
            for ln in pose_links if ln in T
        }

    final_joints: Dict[str, float] = {}
    for _times, rows in track_tl:
        if rows:
            final_joints.update(rows[-1])

    return {
        "tracks": [{"plans": [str(p) for p in (tr.get("plans") or [])],
                    "start": float(tr.get("start", 0.0))} for tr in tracks],
        "urdf": urdf_path,
        "dt": dt,
        "num_samples": n_samples,
        "sample_times": sample_times,
        "frames": frames,
        "mesh_links": mesh_links,
        "final_joints": {k: round(float(v), 6) for k, v in final_joints.items()},
    }


def compute_dual_arm_plan_collisions(
    left_plan: str,
    right_plan: str,
    urdf_path: Optional[str] = None,
    srdf_path: Optional[str] = None,
    dt: float = 0.1,
    window: int = 100,
    return_frames: bool = False,
    include_self: bool = True,
    include_body: bool = True,
    extra_managers: Optional[Dict[str, object]] = None,
    padding_m: float = 0.0,
    left_hand_state: Optional[str] = None,
    right_hand_state: Optional[str] = None,
) -> dict:
    """Run the FULL collision sweep for two simultaneously-executed plans.

    Checked per sample (SRDF-disabled pairs filtered everywhere):
      • LEFT arm links vs RIGHT arm links (the original inter-arm check),
      • each arm INTERNALLY (self-collision within one arm/hand),
      • each arm vs the SLIM body set — bin_link + glove_wearing_unit_arm
        only (2026-07-29: the full non-arm link set made the paired sweep
        multi-second for no signal; the bin is LOWERED into the workspace by
        _build_home and the lever is what the palms work against),
      • each arm vs every ``extra_managers`` entry (e.g. fixture meshes at
        their scene pose, passed by the studio server) — reported as
        ``<manager_key>`` in the pair.

    ``padding_m`` > 0 additionally flags near-misses closer than the padding
    (approximating MoveIt's default link padding) using distance queries.

    Returns a dict with the joint timeline (for FK playback) and the list of
    colliding samples (index, time, link pairs), plus ±``window`` slices around
    each collision cluster. When ``return_frames`` is set, also returns
    ``frames`` — per-window-sample link transforms (link -> flat 4x4) ready to
    pose the web robot overlay — and ``mesh_links`` (the posable link set).
    """
    from alix_workspace.urdf_kinematics import UrdfModel
    from alix_workspace.collision import RobotCollisionModel
    from alix_workspace.srdf_states import find_srdf

    urdf_path = str(urdf_path or _DEFAULT_URDF)
    model = UrdfModel.from_file(urdf_path)
    rcm = RobotCollisionModel.from_file(urdf_path, tag="collision")

    srdf_path = srdf_path or find_srdf(str(Path(urdf_path).parent))
    disabled = load_disabled_pairs(srdf_path) if srdf_path else set()

    # Seed non-plan joints (bin LOWERED + pole full_length + initial positions).
    # Shared with compute_plan_frames so the running scene is defined in ONE place.
    home = _build_home(rcm, srdf_path, left_hand_state, right_hand_state)

    # Build per-arm collision managers once. The "body" set is deliberately
    # SLIM (decision 2026-07-29): checking each arm against EVERY non-arm link
    # (UGV shell, wheels, pole bases…) made the paired-task sweep in the
    # studio's Collisions tab multi-second per pair for no signal — the arms
    # physically reach only the BIN and the GLOVE LEVER during paired
    # cleaning/carry work, so those two are the whole meaningful body set.
    kBodyCheckLinks = ("bin_link", "glove_wearing_unit_arm")
    left_cm = _ArmCollision(rcm, _arm_filter("left"))
    right_cm = _ArmCollision(rcm, _arm_filter("right"))
    body_cm = (_ArmCollision(rcm, lambda link: link in kBodyCheckLinks)
               if include_body else None)

    # Plan timelines + common time grid (both execute from t=0 concurrently).
    lt, lr = load_plan_timeline(left_plan)
    rt, rr = load_plan_timeline(right_plan)
    total = max(lt[-1] if lt else 0.0, rt[-1] if rt else 0.0)
    n_samples = int(round(total / dt)) + 1 if total > 0 else 1

    joint_timeline: List[Dict[str, float]] = []
    sample_times: List[float] = []
    collisions: List[dict] = []

    for i in range(n_samples):
        t = i * dt
        q = dict(home)
        if lt:
            q.update(_sample_at(lt, lr, t))
        if rt:
            q.update(_sample_at(rt, rr, t))
        joint_timeline.append(q)
        sample_times.append(round(t, 4))

        transforms = model.link_world_transforms(q)
        left_cm.pose(transforms)
        right_cm.pose(transforms)
        if body_cm is not None:
            body_cm.pose(transforms)

        pairs = []

        def _filter_names(names, tag_b=None):
            for a, b in names:
                la = a.split("#", 1)[0]
                lb = tag_b if tag_b is not None else b.split("#", 1)[0]
                if la == lb or frozenset((la, lb)) in disabled:
                    continue
                pairs.append(sorted((la, lb)))

        hit, names = left_cm.mgr.in_collision_other(right_cm.mgr, return_names=True)
        if hit:
            _filter_names(names)
        if include_self:
            for cm in (left_cm, right_cm):
                s_hit, s_names = cm.mgr.in_collision_internal(return_names=True)
                if s_hit:
                    _filter_names(s_names)
        if body_cm is not None:
            for cm in (left_cm, right_cm):
                b_hit, b_names = cm.mgr.in_collision_other(body_cm.mgr, return_names=True)
                if b_hit:
                    _filter_names(b_names)
        for key, mgr in (extra_managers or {}).items():
            for cm in (left_cm, right_cm):
                e_hit, e_names = cm.mgr.in_collision_other(mgr, return_names=True)
                if e_hit:
                    _filter_names(((a, key) for a, _ in e_names), tag_b=key)
        if padding_m > 0:
            d = left_cm.mgr.min_distance_other(right_cm.mgr)
            if 0.0 < d < padding_m:
                pairs.append(["left_arm", f"right_arm (within {padding_m*1000:.0f} mm padding)"])
        # Dedup pairs
        uniq = sorted({tuple(p) for p in pairs})
        if uniq:
            collisions.append({"index": i, "time": round(t, 4),
                               "pairs": [list(p) for p in uniq]})

    # ±window slices around each collision (clustered so adjacent hits share one).
    windows = []
    for c in collisions:
        lo = max(0, c["index"] - window)
        hi = min(n_samples - 1, c["index"] + window)
        if windows and lo <= windows[-1]["end"]:
            windows[-1]["end"] = max(windows[-1]["end"], hi)
        else:
            windows.append({"center": c["index"], "start": lo, "end": hi})

    # Per-sample link transforms (link -> flat 4x4), ready to pose the web robot
    # overlay. Computed for the WHOLE timeline (not just collision windows) so the
    # full motion can always be played back/scrubbed, collisions or not.
    frames: Dict[str, Dict[str, list]] = {}
    mesh_links: List[str] = []
    if return_frames:
        # Pose every link the web overlay renders = visual ∪ collision links, so
        # visual-only parts that ride the arms (camera, finger pads) animate too.
        mesh_links = _overlay_link_names(rcm, urdf_path)
        for i in range(n_samples):
            T = model.link_world_transforms(joint_timeline[i])
            frames[str(i)] = {
                ln: [round(float(x), 6) for x in T[ln].reshape(-1)]
                for ln in mesh_links if ln in T
            }

    return {
        "left_plan": str(left_plan),
        "right_plan": str(right_plan),
        "urdf": urdf_path,
        "dt": dt,
        "num_samples": n_samples,
        "left_links": left_cm.link_count,
        "right_links": right_cm.link_count,
        "body_links": body_cm.link_count if body_cm is not None else 0,
        "checks": (["left-vs-right"]
                   + (["self-left", "self-right"] if include_self else [])
                   + (["left-vs-body", "right-vs-body"] if body_cm is not None else [])
                   + [f"arms-vs-{k}" for k in (extra_managers or {})]),
        "disabled_pairs": len(disabled),
        "sample_times": sample_times,
        "joint_timeline": joint_timeline,   # list of {joint: value} per sample (for FK playback)
        "collisions": collisions,           # [{index, time, pairs:[[a,b],...]}]
        "windows": windows,                 # [{center, start, end}]
        "frames": frames,                   # {sample_index(str): {link: flat 4x4}} (window samples)
        "mesh_links": mesh_links,           # posable links (those with collision meshes)
    }


# ---------------------------------------------------------------------------
# Cross-arm separation (the R6 measurement)
# ---------------------------------------------------------------------------
@functools.lru_cache(maxsize=8)
def _side_descendants(urdf_path: str, root: str) -> frozenset:
    """Every link rigidly or jointly descended from one arm's base.

    NOT a name-prefix test, and the difference is not academic: `camera_link` is
    fixed to `R_CameraHolderLink` — it is the right WRIST camera — so a
    `R_`/`IRH_` prefix filter drops a real, protruding right-arm link. It was
    found by the slim Studio set reporting a contact the prefix-built "full" set
    could not see (2026-08-20).
    """
    tree = ET.parse(urdf_path)
    kids: Dict[str, List[str]] = {}
    for j in tree.getroot().findall("joint"):
        kids.setdefault(j.find("parent").get("link"), []).append(
            j.find("child").get("link"))
    out, stack = set(), [root]
    while stack:
        n = stack.pop()
        if n in out:
            continue
        out.add(n)
        stack.extend(kids.get(n, []))
    return frozenset(out)


# The MOUNT: the pole assembly each arm base is bolted to. It is an ANCESTOR of
# the arm root, so it is excluded from every arm set here and measured
# separately as `structural_floor_mm` — left and right mounts sit a fixed
# distance apart, and left in would make "cross-arm separation" a constant that
# says nothing about the arms.
_MOUNT = {"L_Pole", "L_Sliding_Plate", "R_Pole", "R_Sliding_Plate"}

_ARM_ROOT = {"left": "L_base_link", "right": "R_base_link"}


def _full_arm_filter(side: str, urdf_path: Optional[str] = None):
    """EVERY collision link on one arm — structure, links, hand, fingers, camera.

    `_arm_filter` keeps a deliberately slim 13-link set because the paired sweep
    in Studio's Collisions tab had to be fast. For a SEPARATION measurement that
    set is the wrong one: it drops every finger segment, pad, tip and rubber, and
    the fingers are what arrive first. Newton, 2026-08-20: *"you have to use all
    the arm links to know which are closest"*.
    """
    links = _side_descendants(str(urdf_path or _DEFAULT_URDF), _ARM_ROOT[side])
    return lambda link: link in links


def _boxed_arm_filter(side: str, urdf_path: Optional[str] = None):
    """`full`, minus the hand — whose 27 collision links the caller replaces with
    ONE envelope box (`hand_envelope_box`).

    The flange, connector and camera holder stay as MESHES deliberately. Rolling
    them into the box would take the cross-side pair count only 132 -> 64 while
    destroying the thing the pair matrix is for: `camera_link` is the named
    contact in five of the measured pairs and R3 ("point the right wrist camera
    outward") is a fix that exists only because the sweep could name it. A
    verdict that says `distal_box` instead of `camera_link` costs more than the
    2x buys.
    """
    prefix = _HAND_PREFIX[side]
    links = {l for l in _side_descendants(str(urdf_path or _DEFAULT_URDF), _ARM_ROOT[side])
             if not l.startswith(prefix)}
    return lambda link: link in links


def hand_envelope_box(model, rcm, srdf, side: str, state: str):
    """`hand_envelope` packaged for `_ArmCollision(boxes=...)`, or ``None``.

    The name keeps the hand's own `ILH_`/`IRH_` prefix so the cross-arm pair
    filter still treats a contact on it as cross-arm — never excusable by a
    config declaration — and so a reported pair still reads as a hand.
    """
    got = hand_envelope(model, rcm, srdf, side, state)
    if got is None:
        return None
    root, T_box, extents = got
    return (f"{_HAND_PREFIX[side]}hand_envelope#{state}", root, T_box, extents)


def _distal_boxed_arm_filter(side: str, urdf_path: Optional[str] = None):
    """`full`, minus EVERY link after joint 7 — the caller replaces that whole
    group with one `distal_envelope_box`."""
    urdf = str(urdf_path or _DEFAULT_URDF)
    distal = _side_descendants(urdf, DISTAL_ROOT[side])
    links = {l for l in _side_descendants(urdf, _ARM_ROOT[side]) if l not in distal}
    return lambda link: link in links


LINK_SETS = ("slim", "hands", "full", "arms", "boxed", "distal")


def _hand_filter(side: str, urdf_path: Optional[str] = None):
    """ONLY the hand: the 31 links under `ILH_`/`IRH_` on that side.

    Added 2026-09-09. `full` (37x38 = 1406 link pairs per query) re-measures the
    arms every time, and the arm-to-arm minimum is usually already known from a
    `slim` pass — so asking "do the HANDS come closer than the arms do?" should
    not pay for the arms again. hands-vs-hands is 31x31 = 961 pairs, 0.68x full.

    The saving is real but modest, and that is the honest point: the hand MESHES
    are simplified, but there are 31 hand links per side against 10-11 arm links,
    and cost scales with link COUNT, not mesh complexity. A single bounding
    sphere per hand was tried first and is far cheaper (one distance per sample
    pair, 23 s for row 10) but far too loose to certify: the hand is elongated,
    its enclosing radius is 143 mm, and 2x143 mm of slack swamps a 78.62 mm arm
    gap — it returned -88.6 mm, which bounds nothing useful."""
    pfx = "ILH" if side == "left" else "IRH"
    links = {n for n in _side_descendants(str(urdf_path or _DEFAULT_URDF),
                                          _ARM_ROOT[side]) if n.startswith(pfx)}
    return lambda link: link in links


def _make_filter(link_set: str, side: str, urdf_path: str):
    if link_set == "slim":
        return _arm_filter(side)
    if link_set == "hands":
        return _hand_filter(side, urdf_path)
    if link_set in ("full", "arms"):
        return _full_arm_filter(side, urdf_path)
    if link_set == "boxed":
        return _boxed_arm_filter(side, urdf_path)
    if link_set == "distal":
        return _distal_boxed_arm_filter(side, urdf_path)
    raise ValueError(f"link_set must be one of {LINK_SETS}")


def _link_spheres(rcm, link_filter):
    """Per link: local sphere (centre, radius) covering its collision mesh.

    Used only to LOWER-BOUND the arm-to-arm distance, so it must never
    under-estimate a radius. Taken from the mesh vertices directly.
    """
    import numpy as _np
    out = {}
    for link, geoms in rcm.links.items():
        if not link_filter(link):
            continue
        for gi, geom in enumerate(geoms):
            mesh = rcm._load_mesh(geom)
            if mesh is None:
                continue
            v = _np.asarray(mesh.vertices, dtype=float)
            c = v.mean(axis=0)
            r = float(_np.linalg.norm(v - c, axis=1).max())
            out[f"{link}#{gi}"] = (link, _np.asarray(geom.origin, dtype=float), c, r)
    return out


def link_sphere_sets(rcm, link_filter, bins: int = 4):
    """``{object_name: [(centre, radius), ...]}`` — several spheres per collision
    geom instead of one, so the lower bound they give is much tighter.

    WHY, in one number: an arm link is long and thin, the worst case for a single
    sphere. `R_Link2` needs r = 103.4 mm to cover a body only 42.2 mm thick, and a
    PAIR's bound loses twice that slack — about 75 mm against a 50 mm cap. That is
    why ~43 mesh pairs per cell look like candidates when one is the answer.

    BINNED BY TRIANGLE, NOT BY VERTEX, and the distinction is a correctness one:
    if vertices were split along the axis, a triangle straddling two bins would
    have its interior inside neither sphere, the union would stop covering the
    mesh SURFACE, and the "lower bound" would stop bounding. Each triangle is
    assigned whole, and its bin's sphere covers all three of its vertices, so
    every surface point lies in some sphere by construction.

    This never changes a reported distance: it only decides which mesh pairs are
    worth an exact `fcl.distance`. FCL still runs on the meshes.
    """
    import numpy as _np
    out = {}
    for link, geoms in rcm.links.items():
        if not link_filter(link):
            continue
        for gi, geom in enumerate(geoms):
            mesh = rcm._load_mesh(geom)
            if mesh is None or not len(mesh.vertices):
                continue
            M = _np.asarray(geom.origin, dtype=float)
            V = _np.asarray(mesh.vertices, dtype=float) @ M[:3, :3].T + M[:3, 3]
            name = f"{link}#{gi}"
            whole = (V.mean(axis=0), float(_np.linalg.norm(V - V.mean(axis=0), axis=1).max()))
            faces = getattr(mesh, "faces", None)
            if bins <= 1 or faces is None or not len(faces):
                out[name] = [whole]
                continue
            F = _np.asarray(faces)
            Q = V - V.mean(axis=0)
            axis = _np.linalg.svd(Q, full_matrices=False)[2][0]
            t = V @ axis
            tc = t[F].mean(axis=1)                       # one key per TRIANGLE
            edges = _np.linspace(tc.min(), tc.max(), bins + 1)
            edges[-1] += 1e-9
            idx = _np.clip(_np.searchsorted(edges, tc, side="right") - 1, 0, bins - 1)
            subs = []
            for b in range(bins):
                tri = F[idx == b]
                if not len(tri):
                    continue
                P = V[_np.unique(tri)]                   # every vertex of every triangle in the bin
                c = P.mean(axis=0)
                subs.append((c, float(_np.linalg.norm(P - c, axis=1).max())))
            out[name] = subs or [whole]
    return out


def posed_sphere_sets(sphere_sets, names, transforms, origins):
    """Flatten `link_sphere_sets` for one posed arm, in ``names`` order.

    Returns ``(centres, radii, starts)`` where ``starts[k]`` is where object k's
    sub-spheres begin — contiguous, so a per-object minimum is one
    ``np.minimum.reduceat`` rather than a scatter.
    """
    cs, rs, starts = [], [], []
    for name in names:
        starts.append(len(cs))
        T = transforms.get(origins[name])
        for c, r in sphere_sets.get(name, ()):
            if T is None:
                cs.append(np.zeros(3)); rs.append(0.0)
                continue
            cs.append(T[:3, :3] @ c + T[:3, 3]); rs.append(r)
        if len(cs) == starts[-1]:          # object contributed nothing
            cs.append(np.zeros(3)); rs.append(0.0)
    return np.asarray(cs), np.asarray(rs), np.asarray(starts)


def _posed_spheres(spheres, transforms):
    """Per-LINK world spheres for a posed arm: (centres Nx3, radii N).

    PER LINK, not one sphere for the whole arm. A single arm-sized sphere has a
    ~500 mm radius, so `||cL-cR|| - rL - rR` is negative on essentially every
    sample pair and prunes nothing — measured, after the first version of this
    prefilter left the O(n^2) sweep on course for ~7 hours. Per-link spheres are
    tight enough to prune the great majority, and the bound stays exact.
    """
    import numpy as _np
    pts, rads = [], []
    for _name, (link, origin, c, r) in spheres.items():
        T = transforms.get(link)
        if T is None:
            continue
        M = T @ origin
        pts.append(M[:3, :3] @ c + M[:3, 3])
        rads.append(r)
    if not pts:
        return None, None
    return _np.asarray(pts), _np.asarray(rads)


@functools.lru_cache(maxsize=4)
def _separation_context(urdf_path: str, srdf_path: Optional[str], link_set: str,
                        left_hand_state: Optional[str] = None,
                        right_hand_state: Optional[str] = None):
    """Model + per-arm collision managers + link spheres, built ONCE.

    Rebuilding these per call cost ~10 s each — the whole cost of a short sweep —
    because `_ArmCollision` loads and decimates every collision mesh on the side.
    The managers carry mutable transforms, so this is safe only for the
    single-threaded use here; a threaded caller must build its own (and note the
    known segfault when one FCL manager is shared across threads).
    """
    from alix_workspace.urdf_kinematics import UrdfModel
    from alix_workspace.collision import RobotCollisionModel

    model = UrdfModel.from_file(urdf_path)
    rcm = RobotCollisionModel.from_file(urdf_path, tag="collision")
    # The hand states pose the MESHES as well as fitting any box, so the two
    # always describe the same hand. Mismatch them and a box stops bounding the
    # meshes it replaces — how the row 10 "violation" of 2026-09-18 happened.
    home = _build_home(rcm, srdf_path, left_hand_state, right_hand_state)

    def make(side):
        return _make_filter(link_set, side, urdf_path)

    # `boxed` swaps each hand's 27 collision links for ONE envelope box. Because a
    # box CONTAINS the meshes it replaces, the separation it reports is a sound
    # LOWER BOUND on the mesh separation — never optimistic. So a boxed reading
    # above a clearance threshold CERTIFIES the pair at that threshold with no
    # mesh query, for any threshold, and only a reading below it has to fall
    # through to `full`. That is what makes screening the whole pair matrix
    # affordable (132 cross-side pairs instead of 1406) without weakening a
    # verdict. The bound's slack is the box's over-reach: median 23-31 mm per
    # hand, worst 80 mm (scratchpad measurement, 2026-09-18), which is why the
    # thin pairs fall through and the far ones do not.
    boxes = {"left": (), "right": ()}
    spheres = {"left": _link_spheres(rcm, make("left")),
               "right": _link_spheres(rcm, make("right"))}
    if link_set in ("boxed", "distal"):
        from alix_workspace.srdf_states import SrdfStates
        srdf = SrdfStates.from_file(srdf_path) if srdf_path else None
        want = {"left": left_hand_state, "right": right_hand_state}
        build = (hand_envelope_box if link_set == "boxed" else
                 lambda m, r, s, side, st: distal_envelope_box(m, r, s, side, st, urdf_path))
        for side in ("left", "right"):
            state = want[side]
            got = (build(model, rcm, srdf, side, state)
                   if (srdf is not None and state) else None)
            if got is None:
                raise ValueError(
                    f"link_set='boxed' needs a commanded hand state for {side}: the hand "
                    f"holds ONE pose for a cleaning action, so the box is that pose's box. "
                    f"Got {state!r}.")
            boxes[side] = (got,)
            # The sphere prefilter prunes pairs it can prove cannot be closest, so
            # it must see the box too — otherwise it would prune the very pair the
            # box exists to report, and the "bound" would stop bounding. Radius is
            # the box's half-diagonal, which covers it exactly.
            name, root, T_box, extents = got
            e = np.asarray(extents, dtype=float)
            spheres[side][name] = (root, np.asarray(T_box, dtype=float),
                                   np.zeros(3), float(np.linalg.norm(e) / 2.0))

    return {
        "model": model, "home": home,
        "left_cm": _ArmCollision(rcm, make("left"), boxes["left"]),
        "right_cm": _ArmCollision(rcm, make("right"), boxes["right"]),
        "left_sp": spheres["left"],
        "right_sp": spheres["right"],
        "mount_l": _ArmCollision(rcm, lambda k: k in _MOUNT and k.startswith("L_")),
        "mount_r": _ArmCollision(rcm, lambda k: k in _MOUNT and k.startswith("R_")),
    }


def cross_arm_separation(
    left_plan: str,
    right_plan: str,
    urdf_path: Optional[str] = None,
    srdf_path: Optional[str] = None,
    dt: float = 0.05,
    offset_invariant: bool = True,
    offset_window_s: Optional[float] = None,
    link_set: str = "full",
    report_pairs: int = 8,
    progress: Optional[callable] = None,
    left_hand_state: Optional[str] = None,
    right_hand_state: Optional[str] = None,
) -> dict:
    """Minimum LEFT-arm-to-RIGHT-arm distance, and WHICH links get closest.

    `compute_dual_arm_plan_collisions` answers "did they touch?"; a dispatcher
    deciding whether to run two arms at once needs "how close, and where?".

    TWO NUMBERS, and the difference matters:

    * `aligned_min_m` — both plans started at t=0 on one grid. This is what a
      PAIRED row (`11R,11L`, `10R,3L`) actually executes: free-running, each arm
      on its own arrivals.
    * `min_m` (`offset_invariant=True`) — the minimum over pairings of a left
      sample with a right sample, i.e. over relative time offsets. A LOCKED
      barrier or a CHAINED gate re-phases the arms by construction, and two
      free-running arms DRIFT, so the t=0 alignment is not the alignment they
      run.

    `offset_window_s` bounds that search to |offset| <= window. **Use it.**
    Unbounded, the answer on this fixture is 0.00 mm for every concurrent pair
    measured (2026-08-20): both arms reach over one small commode and their
    upper-arm links (`L_Link3` / `R_Link4`) sweep through each other at SOME
    phasing. A criterion that fails everything cannot discriminate. Bounded to a
    realistic drift it becomes a usable gate — and one an executor can actually
    hold, by keeping the two arms within `window` of their nominal phasing.

    `link_set="full"` uses all 39 collision links per side. **All 1521
    cross-side pairs are live** — `alix.srdf` disables none of them — so the
    manager-to-manager query needs no pair filtering and the answer is exact
    for this mesh set. `link_set="slim"` reproduces the 13-link Studio set,
    which is an UPPER bound: adding links can only lower the minimum.

    The offset-invariant sweep is O(n^2) and the full link set is ~1521 pairs
    per query, so it is prefiltered with a per-sample bounding sphere per arm.
    The prefilter is a true lower bound (||cL-cR|| - rL - rR), so it prunes only
    pairs that CANNOT beat the running best — it never changes the answer.

    Inherits the module's caveats: decimated collision meshes, no MoveIt link
    padding. It is a bound to reason with, not MoveIt's own check.
    """
    import time as _time

    import numpy as _np

    from alix_workspace.srdf_states import find_srdf

    if link_set not in LINK_SETS:
        raise ValueError(f"link_set must be one of {LINK_SETS}")
    urdf_path = str(urdf_path or _DEFAULT_URDF)
    srdf_path = srdf_path or find_srdf(str(Path(urdf_path).parent))
    ctx = _separation_context(urdf_path, srdf_path, link_set,
                              left_hand_state, right_hand_state)
    model, home = ctx["model"], ctx["home"]
    left_cm, right_cm = ctx["left_cm"], ctx["right_cm"]
    mount_l, mount_r = ctx["mount_l"], ctx["mount_r"]
    left_sp, right_sp = ctx["left_sp"], ctx["right_sp"]

    lt, lr = load_plan_timeline(left_plan)
    rt, rr = load_plan_timeline(right_plan)
    total = max(lt[-1] if lt else 0.0, rt[-1] if rt else 0.0)
    n = int(round(total / dt)) + 1 if total > 0 else 1

    t0 = _time.perf_counter()

    def transforms_for(times, rows):
        out = []
        for i in range(n):
            q = dict(home)
            if times:
                q.update(_sample_at(times, rows, i * dt))
            out.append(model.link_world_transforms(q))
        return out

    left_T, right_T = transforms_for(lt, lr), transforms_for(rt, rr)

    # The mount separation, so a reader can tell a real arm approach from the
    # fixed distance between the two pole assemblies.
    mount_l.pose(left_T[0])
    mount_r.pose(right_T[0])
    md, mnames = mount_l.mgr.min_distance_other(mount_r.mgr, return_names=True)
    structural = {"mm": float(md) * 1000.0,
                  "pair": [n.split("#", 1)[0] for n in mnames]}
    left_S = [_posed_spheres(left_sp, T) for T in left_T]
    right_S = [_posed_spheres(right_sp, T) for T in right_T]
    right_C = _np.asarray([c for c, _ in right_S])          # (n, kR, 3)
    right_R = _np.asarray([r for _, r in right_S])          # (n, kR)

    _LEFT_LINKS = _side_descendants(urdf_path, _ARM_ROOT["left"]) | {
        "L_Pole", "L_Sliding_Plate"}

    def query(i: int, j: int):
        left_cm.pose(left_T[i])
        right_cm.pose(right_T[j])
        d, names = left_cm.mgr.min_distance_other(right_cm.mgr, return_names=True)
        a, b = (names[0].split("#", 1)[0], names[1].split("#", 1)[0])
        # CLAMP AT ZERO. Below zero the backend reports a penetration depth, and
        # two facts make that unusable as a ranking: the sphere prefilter bounds
        # DISTANCE, so it stops being sound the moment the running best goes
        # negative; and depth for the closest-distance pair is not the deepest
        # penetration anyway. Symptom that found it (2026-08-20): the minimum
        # over a +/-1.0 s drift window read DEEPER than over +/-2.0 s, which is
        # impossible when the smaller window is contained in the larger.
        # Contact is a binary verdict; past it there is nothing left to rank.
        return max(0.0, float(d)), (a, b) if a in _LEFT_LINKS else (b, a)

    # aligned (t=0) sweep — every sample, no prefilter needed
    aligned, aligned_at, aligned_pair = math.inf, None, None
    contacts: List[dict] = []
    pair_hits: Dict[tuple, int] = {}
    per_sample = []
    for i in range(n):
        d, pair = query(i, i)
        if d <= 0.0:
            contacts.append({"left_s": round(i * dt, 4), "right_s": round(i * dt, 4),
                             "pair": list(pair)})
        pair_hits[pair] = pair_hits.get(pair, 0) + 1
        per_sample.append({"t": round(i * dt, 4), "mm": d * 1000.0, "pair": pair})
        if d < aligned:
            aligned, aligned_at, aligned_pair = d, {"left_s": round(i * dt, 4),
                                                    "right_s": round(i * dt, 4)}, pair

    worst, worst_at, worst_pair = aligned, aligned_at, aligned_pair
    pruned = queried = 0
    win = None if offset_window_s is None else int(round(offset_window_s / dt))
    if offset_invariant:
        for i in range(n):
            cL, rL = left_S[i]
            if progress and i % 60 == 0:
                progress(i, n, worst * 1000.0)
            # One vectorised lower bound against every CANDIDATE right sample:
            # (kL,1,1,3) vs (1,m,kR,3) -> (kL,m,kR), min over the link axes.
            # With a drift window the candidate slice is O(win), not O(n) — that
            # is what turns the windowed sweep from quadratic into linear. At
            # dt=0.02 with a +/-30 ms window it is the difference between 2.5M
            # distances per row and 5k.
            lo = 0 if win is None else max(0, i - win)
            hi = n if win is None else min(n, i + win + 1)
            pruned += n - (hi - lo)
            dist = _np.linalg.norm(cL[:, None, None, :] - right_C[None, lo:hi, :, :],
                                   axis=3)
            lower = (dist - rL[:, None, None] - right_R[None, lo:hi, :]).min(axis=(0, 2))
            order = _np.argsort(lower)          # closest candidates first, so the
            for rank, j in enumerate(order):    # running best tightens fastest
                if lower[j] >= worst:
                    pruned += (hi - lo) - rank  # sorted: every later j is too
                    break
                queried += 1
                jj = int(j) + lo
                d, pair = query(i, jj)
                if d < worst:
                    worst, worst_at, worst_pair = d, {"left_s": round(i * dt, 4),
                                                      "right_s": round(jj * dt, 4)}, pair
                if d <= 0.0:
                    contacts.append({"left_s": round(i * dt, 4),
                                     "right_s": round(jj * dt, 4),
                                     "pair": list(pair)})
                    break          # contact is binary; nothing left to rank
            if worst <= 0.0:
                break

    top = sorted(pair_hits.items(), key=lambda kv: -kv[1])[:report_pairs]
    closest = sorted(per_sample, key=lambda r: r["mm"])[:report_pairs]
    return {
        "collides": bool(worst <= 0.0),
        "contacts": contacts[:report_pairs],
        "aligned_collides": bool(aligned <= 0.0),
        "min_m": float(worst), "min_mm": float(worst) * 1000.0,
        "min_pair": worst_pair, "at": worst_at,
        "aligned_min_m": float(aligned), "aligned_min_mm": float(aligned) * 1000.0,
        "aligned_pair": aligned_pair, "aligned_at": aligned_at,
        "link_set": link_set,
        "structural_floor_mm": structural["mm"],
        "structural_pair": structural["pair"],
        "links_per_side": {"left": left_cm.link_count, "right": right_cm.link_count},
        "offset_invariant": bool(offset_invariant),
        "offset_window_s": offset_window_s,
        "closest_pair_frequency": [(list(p), c) for p, c in top],
        "closest_samples": closest,
        "samples": n, "dt": dt, "duration_s": round(total, 4),
        "prefilter_pruned": pruned, "prefilter_queried": queried,
        "left_plan": str(left_plan), "right_plan": str(right_plan),
        "elapsed_s": round(_time.perf_counter() - t0, 2),
    }


def _main() -> int:
    import argparse
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(message)s")
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("left_plan")
    ap.add_argument("right_plan")
    ap.add_argument("--urdf", default=None)
    ap.add_argument("--srdf", default=None)
    ap.add_argument("--dt", type=float, default=0.1)
    ap.add_argument("--window", type=int, default=100)
    args = ap.parse_args()

    res = compute_dual_arm_plan_collisions(
        args.left_plan, args.right_plan, args.urdf, args.srdf, args.dt, args.window)

    print(f"\nURDF: {res['urdf']}")
    print(f"Left links: {res['left_links']}   Right links: {res['right_links']}   "
          f"SRDF disabled pairs: {res['disabled_pairs']}")
    print(f"Samples: {res['num_samples']} @ dt={res['dt']}s  "
          f"(total {round((res['num_samples']-1)*res['dt'],2)}s)")
    print(f"Colliding samples: {len(res['collisions'])}   "
          f"collision windows: {len(res['windows'])}")
    for c in res["collisions"][:20]:
        pairs = ", ".join(f"{a}<->{b}" for a, b in c["pairs"])
        print(f"  t={c['time']:.2f}s (sample {c['index']}): {pairs}")
    if len(res["collisions"]) > 20:
        print(f"  ... and {len(res['collisions']) - 20} more")
    return 0


if __name__ == "__main__":
    raise SystemExit(_main())
