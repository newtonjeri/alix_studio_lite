"""A recording's collision check (feedback v5, slide 5; the owner, 16 Oct: "If the robot arm collides
with a wall or encounters an obstacle during playback ... tell at what frame the collision occurs,
there is already a slim collision checker implemented in ALIX Studio, include the walls (the created
walls in the setup stage)").

Built from ALIX's own collision code, vendored as it is (alix_workspace): the robot's collision meshes
from its URDF (``RobotCollisionModel``), one arm's every collision link with its hand
(``_ArmCollision`` with ``_full_arm_filter``, the set ALIX uses where the fingers matter), FCL through
trimesh, and the SRDF's disabled pairs between the two arms. Every 5 ms line of the recording is
checked, at the pole height and hand pose it was taught with, against:

- the room's four walls as Prepare measured them (``placement.scene``: back, left, right, and the
  front wall that the 3D draws as a line on the floor), or ALIX's venue cubicle when the room has
  not been measured;
- the commode in the region's lid and seat state, where Lite's 3D puts it (``fixtures.spec``, moved
  by the task's UGV placement);
- the other arm, standing.

Cleaning touches the commode, so the hand on the commode is CONTACT, not a collision: it is
reported only when it goes deeper than ALIX's pad compliance (20 mm, ``body_collision``). Any other
link of the arm on the commode, anything on a wall, and the two arms touching are collisions. The
glove, the pad and the brush are not modelled (ALIX has them as data only), so a real contact comes
some 20 mm before the model's. Nothing is blocked by the result: it is reported, with the first
line that collides.

FCL is not safe across threads, so the check runs in the worker process (``workers.pool()``).
"""
from __future__ import annotations

import time
from functools import lru_cache
from pathlib import Path

import numpy as np

HAND_PREFIX = {"left": "ILH_", "right": "IRH_"}
CONTACT_DEPTH_MM = 20.0           # ALIX's pad compliance (alix_taskgen.body_collision.PAD_COMPLIANCE_MM)
WALL_T_M, WALL_H_M = 0.10, 2.0    # the front wall as a box like the others (placement.WALL_*)
MAX_EVENTS = 50


@lru_cache(maxsize=None)
def _rcm():
    from alix_workspace.collision import RobotCollisionModel

    from .. import vendor_paths

    return RobotCollisionModel.from_file(str(vendor_paths.URDF), "collision")


@lru_cache(maxsize=None)
def _arm(side: str, part: str = "all"):
    """One arm's collision links: ``all``, its ``hand`` (every link of the hand), or the rest of the arm
    (``structure``): the arm and the hand are tested apart, as the hand's touch is the cleaning."""
    from alix_workspace.dual_arm_plan_collision import _ArmCollision, _full_arm_filter

    from .. import vendor_paths

    full = _full_arm_filter(side, str(vendor_paths.URDF))
    hand = HAND_PREFIX[side]
    keep = {"all": full, "hand": lambda ln: full(ln) and ln.startswith(hand),
            "structure": lambda ln: full(ln) and not ln.startswith(hand)}[part]
    return _ArmCollision(_rcm(), keep)


@lru_cache(maxsize=None)
def _disabled() -> frozenset:
    from alix_workspace.dual_arm_plan_collision import load_disabled_pairs

    from .. import vendor_paths

    return frozenset(load_disabled_pairs(str(vendor_paths.SRDF)))


@lru_cache(maxsize=None)
def _commode(fid: str, cubicle_mm: int, phase: str) -> tuple:
    """The fixture's meshes in a lid/seat state, posed in the robot base frame at ALIX's calibrated
    standoff, as Lite's 3D shows them: ((name, mesh, 4x4), ...)."""
    import trimesh

    from . import fixtures

    out = []
    for part in fixtures.spec(fid, cubicle_mm)["parts"]:
        url = (part.get("state_meshes") or {}).get(phase) or part.get("mesh")
        path = fixtures.mesh_path(fid, url.rsplit("/", 1)[-1]) if url else None
        if path is None:
            continue
        mesh = trimesh.load(str(path), force="mesh")
        scale = np.asarray(part.get("scale") or [1, 1, 1], dtype=float)
        if not np.allclose(scale, 1.0):
            mesh.apply_scale(scale)
        out.append((f"commode {part['name']}", mesh, np.asarray(part["matrices"][phase], dtype=float).reshape(4, 4)))
    return tuple(out)


def _objects(scene: dict, fid: str, cubicle_mm: int, phase: str) -> tuple[list, str]:
    """Each wall and each part of the commode as its own collision manager, [(kind, name, manager)],
    and what the walls were made from."""
    import trimesh

    def one(name, mesh, T):
        m = trimesh.collision.CollisionManager()
        m.add_object(name, mesh, transform=T)
        return m

    out = []
    walls = scene.get("walls")
    if walls:
        made = "the room measured in Prepare"
        for w in walls:
            T = np.eye(4)
            T[:3, 3] = w["center"]
            out.append(("wall", f"{w['name']} wall", one(w["name"], trimesh.creation.box(extents=w["size"]), T)))
        if scene.get("front_x") is not None and scene.get("room_y"):
            y0, y1 = scene["room_y"]
            T = np.eye(4)
            T[:3, 3] = [scene["front_x"] - WALL_T_M / 2, (y0 + y1) / 2, WALL_H_M / 2]
            out.append(("wall", "front wall", one("front", trimesh.creation.box(extents=[WALL_T_M, (y1 - y0) + 2 * WALL_T_M, WALL_H_M]), T)))
    else:
        from alix_workspace.plan_scene_validation import load_wall_manager

        from .. import vendor_paths

        made = f"ALIX's {cubicle_mm} mm cubicle (the room is not measured in Prepare)"
        out.append(("wall", "cubicle wall", load_wall_manager(vendor_paths.cubicle_yaml(cubicle_mm))[0]))
    shift = np.eye(4)
    shift[:2, 3] = (scene.get("offset_m") or [0.0, 0.0])[:2]
    for name, mesh, M in _commode(fid, cubicle_mm, phase):
        out.append(("commode", name, one(name, mesh, shift @ M)))
    return out, made


def _pairs(arm, other, disabled=frozenset()) -> list[tuple[str, str, float]]:
    """The links of ``arm`` in ``other``'s objects, with the deepest penetration of each pair (mm)."""
    _, _, data = arm.mgr.in_collision_other(other, return_names=True, return_data=True)
    mine = {name for name, _, _ in arm.entries}
    out = {}
    for c in data:
        names = list(c.names)
        a, b = (names + names)[:2]
        a, b = (a, b) if a in mine else (b, a)
        a, b = a.split("#", 1)[0], b.split("#", 1)[0]
        if frozenset((a, b)) in disabled:
            continue
        out[(a, b)] = max(out.get((a, b), 0.0), 1000.0 * float(getattr(c, "depth", 0.0) or 0.0))
    return [(a, b, d) for (a, b), d in out.items()]


def check(md_path: str, side: str, stage: dict, home: dict, scene: dict, fid: str, phase: str = "lid_closed",
          cubicle_mm: int = 1400) -> dict:
    """Every line of a recording (``md_path``) against the walls, the commode and the other arm,
    standing. ``stage``: {pole_mm, hand}, as it was taught; ``home``: the standing pose's joint values;
    ``scene``: ``placement.scene`` for the recording's room and UGV placement. Lines are 1-based, as
    the plots number their rows."""
    from alix_studio_lite import vendor_paths

    vendor_paths.activate()
    from ..motion import track
    from . import robot_model, staging

    t0 = time.monotonic()
    md = track.read_md(Path(md_path))
    model = robot_model.fk_model()
    s, other = staging.SIDE_PREFIX[side], ("right" if side == "left" else "left")
    values = dict(home)
    values.update(staging.joint_values(side, stage.get("pole_mm"), stage.get("hand")))
    structure, hand, everything = _arm(side, "structure"), _arm(side, "hand"), _arm(side, "all")
    partner = _arm(other)
    partner.pose(model.link_world_transforms(dict(home)))
    objects, made = _objects(scene, fid, int(cubicle_mm), phase)
    disabled = _disabled()
    joints = [f"{s}_joint{i}" for i in range(1, 8)]
    q = np.radians(np.asarray(md, dtype=float) / 1000.0)
    contact = dict(lines=0, first_line=None, deepest_mm=0.0)
    hits = []                                  # (line, frozenset of (what, object))
    for i in range(q.shape[0]):
        values.update(zip(joints, q[i].tolist()))
        tf = model.link_world_transforms(values)
        structure.pose(tf)
        hand.pose(tf)
        found = set()
        for kind, name, mgr in objects:          # yes / no first: the details only where an event starts
            if kind == "wall":
                if structure.mgr.in_collision_other(mgr) or hand.mgr.in_collision_other(mgr):
                    found.add(("wall", name))
                continue
            if structure.mgr.in_collision_other(mgr):
                found.add(("commode", name))
            if hand.mgr.in_collision_other(mgr):  # cleaning touches the commode: deeper than the pad's 20 mm is reported
                depth = max([d for _, _, d in _pairs(hand, mgr)] or [0.0])
                contact["lines"] += 1
                contact["first_line"] = contact["first_line"] or i + 1
                contact["deepest_mm"] = max(contact["deepest_mm"], depth)
                if depth > CONTACT_DEPTH_MM:
                    found.add(("hand deep in the commode", name))
        if structure.mgr.in_collision_other(partner.mgr) or hand.mgr.in_collision_other(partner.mgr):
            everything.pose(tf)
            if _pairs(everything, partner.mgr, disabled):
                found.add(("other arm", f"the {other.upper()} arm"))
        if found:
            hits.append((i + 1, frozenset(found)))
    events = []
    for line, found in hits:
        if events and events[-1]["last_line"] == line - 1 and events[-1]["_found"] == found:
            events[-1]["last_line"] = line
            continue
        events.append(dict(first_line=line, last_line=line, _found=found, what=sorted({w for w, _ in found})))
    by_name = {name: mgr for _, name, mgr in objects}
    for e in events[:MAX_EVENTS]:              # which links, how deep: at the event's first line
        values.update(zip(joints, q[e["first_line"] - 1].tolist()))
        tf = model.link_world_transforms(values)
        everything.pose(tf)
        pairs = []
        for what, name in sorted(e.pop("_found")):
            mgr = partner.mgr if what == "other arm" else by_name[name]
            for link, obj, depth in _pairs(everything, mgr, disabled if what == "other arm" else frozenset()):
                if what == "commode" and link.startswith(HAND_PREFIX[side]):
                    continue                     # the hand's touch there is contact, said apart
                pairs.append(dict(what=what, link=link, object=name if what != "other arm" else f"{other.upper()} arm {obj}",
                                  depth_mm=round(depth, 1)))
        e["pairs"] = pairs
    for e in events[MAX_EVENTS:]:
        e.pop("_found", None)
    return dict(ok=not events, lines=int(q.shape[0]), first=events[0] if events else None, events=events[:MAX_EVENTS],
                more=max(0, len(events) - MAX_EVENTS), colliding_lines=len(hits),
                contact=dict(contact, deepest_mm=round(contact["deepest_mm"], 1)),
                checked=dict(walls=made, commode=phase, other_arm=f"the {other.upper()} arm, standing",
                             links=everything.link_count, contact_depth_mm=CONTACT_DEPTH_MM),
                seconds=round(time.monotonic() - t0, 2))
