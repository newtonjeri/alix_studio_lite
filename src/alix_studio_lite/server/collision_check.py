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
  by the task's UGV placement; its lid and seat open only as far as the wall behind lets them);
- the other arm, standing;
- the robot's own bin, lowered as it stands for cleaning (the owner, 19 Oct: "include the bin body, the
  bin handles and the bin covers in the collision checking matrices"): the bin, its rail and carriage,
  its two handles and its two lid-opening covers (``BIN_LINKS``), as ALIX's own paired sweep checks each
  arm against the bin (``dual_arm_plan_collision``: "the bin is LOWERED into the workspace").

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
#: the bin as the URDF has it: the rail on the UGV, the carriage on the rail, the bin on the carriage, and on the bin
#: its two handles and its two lid-opening covers (all with collision meshes)
BIN_LINKS = frozenset({"bin_rail_link", "bin_carriage_link", "bin_link", "bin_handle_left", "bin_handle_right",
                       "lid_opening_cover1", "lid_opening_cover2"})
CONTACT_DEPTH_MM = 20.0           # ALIX's pad compliance (alix_taskgen.body_collision.PAD_COMPLIANCE_MM)
LINK_EVERY = 4                    # the colliding links read again every 4th line (20 ms, the 3D's own step) in one collision
DEPTH_EVERY_S = 0.1               # while the hand stays on the commode its depth is measured every 0.1 s (FCL gathers
                                  # thousands of contact points for it), at once when a contact starts
WALL_T_M, WALL_H_M = 0.10, 2.0    # the front wall as a box like the others (placement.WALL_*)
MAX_EVENTS = 50
EXACT_MOMENTS = 12                # a clearance stretch's moments measured on the real meshes (clearance_states)


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
def _bin():
    """The bin's collision links (BIN_LINKS), posed where the standing pose puts them (the bin lowered, as
    ALIX's sweep seeds it from the SRDF): ``pose(tf)`` once per check, then every arm is tested against it."""
    from alix_workspace.dual_arm_plan_collision import _ArmCollision

    return _ArmCollision(_rcm(), lambda ln: ln in BIN_LINKS)


@lru_cache(maxsize=None)
def _disabled() -> frozenset:
    from alix_workspace.dual_arm_plan_collision import load_disabled_pairs

    from .. import vendor_paths

    return frozenset(load_disabled_pairs(str(vendor_paths.SRDF)))


@lru_cache(maxsize=None)
def _commode(fid: str, cubicle_mm: int, phase: str, back_mm: float | None = None) -> tuple:
    """The fixture's meshes in a lid/seat state, posed in the robot base frame at ALIX's calibrated
    standoff, as Lite's 3D shows them: ((name, mesh, 4x4), ...). ``back_mm``: the gap to the wall behind,
    which the lid and the seat open no further than (``fixtures.spec``)."""
    import trimesh

    from . import fixtures

    out = []
    for part in fixtures.spec(fid, cubicle_mm, back_mm)["parts"]:
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
    back = scene.get("back_mm")
    for name, mesh, M in _commode(fid, cubicle_mm, phase, None if back is None else round(float(back), 1)):
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


def _names(arm, other, disabled=frozenset()) -> set:
    """The link names of ``arm`` touching ``other``'s objects, with the other's (a link of the other
    arm, or a wall's or the commode's name): names only, no contact data, so cheap enough for every
    colliding line (the 3D turns those links red at that moment)."""
    hit, pairs = arm.mgr.in_collision_other(other, return_names=True)
    out = set()
    if not hit:
        return out
    mine = {name for name, _, _ in arm.entries}
    for a, b in pairs:
        a, b = (a, b) if a in mine else (b, a)
        a, b = a.split("#", 1)[0], b.split("#", 1)[0]
        if frozenset((a, b)) not in disabled:
            out.add((a, b))
    return out


def _spans(hits: list) -> list:
    """[(index, links)] of the colliding moments as runs [first, last, sorted links], one per run of
    consecutive moments with the same links."""
    out = []
    for i, links in hits:
        if out and out[-1][1] == i - 1 and out[-1][2] == links:
            out[-1][1] = i
        else:
            out.append([i, i, links])
    return [[a, b, sorted(links)] for a, b, links in out]


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
    partner, bin_ = _arm(other), _bin()
    standing = model.link_world_transforms(dict(home))
    partner.pose(standing)
    bin_.pose(standing)                        # lowered, where it stands for cleaning
    objects, made = _objects(scene, fid, int(cubicle_mm), phase)
    disabled = _disabled()
    joints = [f"{s}_joint{i}" for i in range(1, 8)]
    q = np.radians(np.asarray(md, dtype=float) / 1000.0)
    contact = dict(lines=0, first_line=None, deepest_mm=0.0)
    hits = []                                  # (line, frozenset of (what, object))
    lit = []                                   # (line, frozenset of the robot's links colliding then)
    named = (None, -LINK_EVERY, frozenset())   # (what collided, at which line its links were read, the links)
    depth_at = {}                              # commode part -> (last line in contact, line measured, depth)
    DEPTH_LINES = max(1, int(round(DEPTH_EVERY_S / 0.005)))
    for i in range(q.shape[0]):
        values.update(zip(joints, q[i].tolist()))
        tf = model.link_world_transforms(values)
        structure.pose(tf)
        hand.pose(tf)
        found, hit_with = set(), []              # yes / no first: the names only where something collides
        for kind, name, mgr in objects:
            if kind == "wall":
                if structure.mgr.in_collision_other(mgr) or hand.mgr.in_collision_other(mgr):
                    found.add(("wall", name))
                    hit_with += [(structure, mgr), (hand, mgr)]
                continue
            if structure.mgr.in_collision_other(mgr):
                found.add(("commode", name))
                hit_with.append((structure, mgr))
            if hand.mgr.in_collision_other(mgr):  # cleaning touches the commode: deeper than the pad's 20 mm is reported
                was = depth_at.get(name)
                if was is None or was[0] != i - 1 or i - was[1] >= DEPTH_LINES:
                    depth = max([d for _, _, d in _pairs(hand, mgr)] or [0.0])
                    depth_at[name] = (i, i, depth)
                else:
                    depth = was[2]
                    depth_at[name] = (i, was[1], depth)
                contact["lines"] += 1
                contact["first_line"] = contact["first_line"] or i + 1
                contact["deepest_mm"] = max(contact["deepest_mm"], depth)
                if depth > CONTACT_DEPTH_MM:
                    found.add(("hand deep in the commode", name))
                    hit_with.append((hand, mgr))
        touching = set()
        if structure.mgr.in_collision_other(partner.mgr) or hand.mgr.in_collision_other(partner.mgr):
            everything.pose(tf)
            touching = _names(everything, partner.mgr, disabled)
            if touching:
                found.add(("other arm", f"the {other.upper()} arm"))
        if structure.mgr.in_collision_other(bin_.mgr) or hand.mgr.in_collision_other(bin_.mgr):
            everything.pose(tf)
            on_bin = _names(everything, bin_.mgr, disabled)
            if on_bin:
                found.add(("bin", "the bin"))
                touching = touching | on_bin
        if found:
            found = frozenset(found)
            if named[0] == found and i - named[1] < LINK_EVERY:
                links = named[2]                 # the same collision a moment later: its links as read then
            else:
                links = set()
                for arm, mgr in hit_with:
                    links |= {a for a, _ in _names(arm, mgr)}
                links = frozenset(links | {a for a, _ in touching} | {b for _, b in touching})
                named = (found, i, links)
            hits.append((i + 1, found))
            lit.append((i + 1, links))
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
            mgr = partner.mgr if what == "other arm" else bin_.mgr if what == "bin" else by_name[name]
            for link, obj, depth in _pairs(everything, mgr, disabled if what in ("other arm", "bin") else frozenset()):
                if what == "commode" and link.startswith(HAND_PREFIX[side]):
                    continue                     # the hand's touch there is contact, said apart
                pairs.append(dict(what=what, link=link, depth_mm=round(depth, 1),
                                  object=f"{other.upper()} arm {obj}" if what == "other arm" else obj if what == "bin" else name))
        e["pairs"] = pairs
    for e in events[MAX_EVENTS:]:
        e.pop("_found", None)
    return dict(ok=not events, lines=int(q.shape[0]), first=events[0] if events else None, events=events[:MAX_EVENTS],
                more=max(0, len(events) - MAX_EVENTS), colliding_lines=len(hits), spans=_spans(lit), phase=phase,
                contact=dict(contact, deepest_mm=round(contact["deepest_mm"], 1)),
                checked=dict(walls=made, commode=phase, other_arm=f"the {other.upper()} arm, standing",
                             bin="the bin, lowered: its body, rail and carriage, handles and covers",
                             links=everything.link_count, contact_depth_mm=CONTACT_DEPTH_MM),
                seconds=round(time.monotonic() - t0, 2))


def check_states(times, arms: dict, home: dict, scene: dict, fid: str, phases, cubicle_mm: int = 1400,
                 progress=None) -> dict:
    """Merge + Sequence S2's check (feedback v5, slide 9: "Simulation Mode ... with the collision
    check", the two arms against each other too): both arms through a run on one clock. ``times``: the
    samples (s); ``arms``: {side: {pole: [n] mm, hand: [n][6] counts, q: [n][7] deg}} for the arms that
    run (an arm not given stands at ``home``); ``phases``: the commode's lid/seat state at each sample.
    Each running arm against the walls and the commode as ``check`` does (the hand on the commode is
    contact, deeper than 20 mm reported), and the two arms against each other (the SRDF's disabled
    pairs aside). Events say their first and last sample's time."""
    from alix_studio_lite import vendor_paths

    vendor_paths.activate()
    from . import robot_model, staging

    t0 = time.monotonic()
    model = robot_model.fk_model()
    times = [float(t) for t in times]
    n = len(times)
    sides = [s for s in ("left", "right") if s in arms]
    data = {s: dict(pole=np.asarray(arms[s]["pole"], dtype=float), hand=np.asarray(arms[s]["hand"], dtype=float),
                    q=np.radians(np.asarray(arms[s]["q"], dtype=float)),
                    known=np.asarray(arms[s].get("known") or [True] * n, dtype=bool)) for s in sides}
    joints = {s: [f"{staging.SIDE_PREFIX[s]}_joint{i}" for i in range(1, 8)] for s in ("left", "right")}
    parts = {s: (_arm(s, "structure"), _arm(s, "hand")) for s in sides}
    whole = {s: _arm(s) for s in ("left", "right")}
    bin_ = _bin()
    bin_.pose(model.link_world_transforms(dict(home)))     # lowered, where it stands for cleaning
    disabled = _disabled()
    objects_by_phase, made = {}, None
    for ph in sorted(set(phases)):
        objects_by_phase[ph], made = _objects(scene, fid, int(cubicle_mm), ph)
    contact = {s: dict(samples=0, first_t=None, deepest_mm=0.0) for s in sides}

    def values_at(i):
        v = dict(home)
        for s in sides:
            d = data[s]
            v.update(staging.joint_values(s, d["pole"][i], [int(round(c)) for c in d["hand"][i]]))
            v.update(zip(joints[s], d["q"][i].tolist()))
        return v

    hits, lit, last_key, last_found, last_links = [], [], None, frozenset(), frozenset()
    depth_at = {}                              # (arm, commode part) -> (last sample in contact, sample measured, depth)
    for i in range(n):
        key = (phases[i], tuple(bool(data[s]["known"][i]) for s in sides),
               tuple(np.round(np.concatenate([np.concatenate([[data[s]["pole"][i]], data[s]["hand"][i], data[s]["q"][i]])
                                              for s in sides]), 6)))
        if key == last_key:                    # nothing moved since the sample before: the same answer
            found, links = last_found, last_links
        else:
            tf = model.link_world_transforms(values_at(i))
            found, links = set(), set()
            for s in ("left", "right"):
                whole[s].pose(tf)
            for s in sides:
                if not data[s]["known"][i]:                # where the arm is is not known (a step not drawn)
                    continue
                structure, hand = parts[s]
                structure.pose(tf)
                hand.pose(tf)
                for kind, name, mgr in objects_by_phase[phases[i]]:
                    if kind == "wall":
                        if structure.mgr.in_collision_other(mgr) or hand.mgr.in_collision_other(mgr):
                            found.add((s, "wall", name))
                            links |= {a for a, _ in _names(whole[s], mgr)}
                        continue
                    if structure.mgr.in_collision_other(mgr):
                        found.add((s, "commode", name))
                        links |= {a for a, _ in _names(structure, mgr)}
                    if hand.mgr.in_collision_other(mgr):
                        was = depth_at.get((s, name))
                        if was is None or was[0] != i - 1 or times[i] - times[was[1]] >= DEPTH_EVERY_S - 1e-9:
                            depth = max([d for _, _, d in _pairs(hand, mgr)] or [0.0])
                            depth_at[(s, name)] = (i, i, depth)
                        else:
                            depth = was[2]
                            depth_at[(s, name)] = (i, was[1], depth)
                        c = contact[s]
                        c["samples"] += 1
                        c["first_t"] = c["first_t"] if c["first_t"] is not None else times[i]
                        c["deepest_mm"] = max(c["deepest_mm"], depth)
                        if depth > CONTACT_DEPTH_MM:
                            found.add((s, "hand deep in the commode", name))
                            links |= {a for a, _ in _names(hand, mgr)}
                if structure.mgr.in_collision_other(bin_.mgr) or hand.mgr.in_collision_other(bin_.mgr):
                    on_bin = _names(whole[s], bin_.mgr, disabled)
                    if on_bin:
                        found.add((s, "bin", "the bin"))
                        links |= {a for a, _ in on_bin}
            both_known = all(data[s]["known"][i] for s in sides)
            if both_known and whole["left"].mgr.in_collision_other(whole["right"].mgr):
                touching = _names(whole["left"], whole["right"].mgr, disabled)
                if touching:
                    found.add(("both", "the two arms", "LEFT and RIGHT arms"))
                    links |= {a for a, _ in touching} | {b for _, b in touching}
            found, links = frozenset(found), frozenset(links)
            last_key, last_found, last_links = key, found, links
        if found:
            hits.append((i, found))
            lit.append((i, links))
        if progress is not None and i % 500 == 0:
            progress(i, n)
    events = []
    for i, found in hits:
        if events and events[-1]["_last"] == i - 1 and events[-1]["_found"] == found:
            events[-1]["_last"] = i
            continue
        events.append(dict(_first=i, _last=i, _found=found))
    out = []
    for e in events[:MAX_EVENTS]:              # which links, how deep: at the event's first sample
        tf = model.link_world_transforms(values_at(e["_first"]))
        for s in ("left", "right"):
            whole[s].pose(tf)
        by_name = {name: mgr for _, name, mgr in objects_by_phase[phases[e["_first"]]]}
        pairs = []
        for s, what, name in sorted(e["_found"]):
            if what == "the two arms":
                for link, obj, depth in _pairs(whole["left"], whole["right"].mgr, disabled):
                    pairs.append(dict(side="both", what=what, link=link, object=f"RIGHT arm {obj}", depth_mm=round(depth, 1)))
                continue
            if what == "bin":
                for link, obj, depth in _pairs(whole[s], bin_.mgr, disabled):
                    pairs.append(dict(side=s, what=what, link=link, object=obj, depth_mm=round(depth, 1)))
                continue
            for link, obj, depth in _pairs(whole[s], by_name[name]):
                if what == "commode" and link.startswith(HAND_PREFIX[s]):
                    continue                     # the hand's touch there is contact, said apart
                pairs.append(dict(side=s, what=what, link=link, object=name, depth_mm=round(depth, 1)))
        out.append(dict(first_t=round(times[e["_first"]], 3), last_t=round(times[e["_last"]], 3),
                        sides=sorted({s for s, _, _ in e["_found"]}), what=sorted({w for _, w, _ in e["_found"]}), pairs=pairs))
    spans = [[round(times[a], 3), round(times[b], 3), links] for a, b, links in _spans(lit)]
    return dict(ok=not events, samples=n, first=out[0] if out else None, events=out, more=max(0, len(events) - MAX_EVENTS),
                colliding_samples=len(hits), spans=spans, dt=round(times[1] - times[0], 4) if n > 1 else 0.0,
                contact={s: dict(c, deepest_mm=round(c["deepest_mm"], 1)) for s, c in contact.items()},
                checked=dict(walls=made, arms=sides, two_arms=True, contact_depth_mm=CONTACT_DEPTH_MM,
                             bin="the bin, lowered: its body, rail and carriage, handles and covers"),
                seconds=round(time.monotonic() - t0, 2))


# ── the two arms apart at every timing (S2: the owner, 22 Oct)

@lru_cache(maxsize=None)
def _hulls(side: str):
    """One arm's collision links (the set ``_arm`` checks) as convex hulls, with each hull's reach from its link's
    origin. A hull holds its mesh, so the gap between two hulls is never more than the real one; FCL measures it
    some 15 times faster (1 ms a moment against 18), which lets S2 check the arms at many timings."""
    import trimesh
    from alix_workspace.dual_arm_plan_collision import _full_arm_filter

    from .. import vendor_paths

    rcm, keep = _rcm(), _full_arm_filter(side, str(vendor_paths.URDF))
    mgr, entries, reach = trimesh.collision.CollisionManager(), [], {}
    for link, geoms in rcm.links.items():
        if not keep(link):
            continue
        for gi, geom in enumerate(geoms):
            mesh = rcm._load_mesh(geom)
            if mesh is None:
                continue
            hull, origin = mesh.convex_hull, np.asarray(geom.origin, dtype=float)
            name = f"{link}#{gi}"
            mgr.add_object(name, hull)
            entries.append((name, link, origin))
            pts = np.asarray(hull.vertices, dtype=float) @ origin[:3, :3].T + origin[:3, 3]
            reach[link] = max(reach.get(link, 0.0), float(np.max(np.linalg.norm(pts, axis=1))) if len(pts) else 0.0)
    links = sorted(reach)
    return mgr, entries, links, np.asarray([reach[ln] for ln in links])


def clearance_states(scenarios: list[dict], home: dict, clearance_mm: float, progress=None) -> list[dict]:
    """The two arms against each other with ``clearance_mm`` between them, in each of ``scenarios`` (each ``times``
    and ``arms``: both arms' pole mm, hand counts, joints deg and ``known`` at those times, as check_states takes
    them). A moment is skipped when no link of one arm can come that close to the other (their origins' distance less
    their hulls' reach); otherwise the hulls' least gap is measured. Each scenario's stretches closer than the
    clearance: their first and last time, the hulls' least gap (mm; below 0 when they overlap) and the least gap between
    the arms' real meshes over the stretch (``exact_mm``, at up to EXACT_MOMENTS of its moments, the hulls' closest
    among them), with the two links then. A stretch only the hulls make that close (the meshes never are) is left out."""
    from alix_studio_lite import vendor_paths

    vendor_paths.activate()
    from . import robot_model, staging

    model = robot_model.fk_model()
    joints = {s: [f"{staging.SIDE_PREFIX[s]}_joint{i}" for i in range(1, 8)] for s in ("left", "right")}
    hulls = {s: _hulls(s) for s in ("left", "right")}
    exact = {s: _arm(s) for s in ("left", "right")}
    limit = float(clearance_mm) / 1000.0
    tf_of: dict = {}                       # (side, pose) -> that arm's link transforms (each arm's own; the poses repeat)
    gap_of: dict = {}                      # (left pose, right pose) -> (gap m, names)
    exact_of: dict = {}                    # (left pose, right pose) -> (the meshes' gap mm, names)

    def pose_key(arm, i):
        return (round(float(arm["pole"][i]), 1), tuple(int(round(float(c))) for c in arm["hand"][i]),
                tuple(round(float(v), 3) for v in arm["q"][i]))

    def transforms(side, key):
        tf = tf_of.get((side, key))
        if tf is None:
            pole, hand, q = key
            v = dict(home)
            v.update(staging.joint_values(side, pole, list(hand)))
            v.update(zip(joints[side], np.radians(np.asarray(q, dtype=float)).tolist()))
            full = model.link_world_transforms(v)
            names = hulls[side][2]
            tf = ({ln: full[ln] for _, ln, _ in hulls[side][1] if ln in full},
                  np.asarray([full[ln][:3, 3] for ln in names]))
            if len(tf_of) > 400000:
                tf_of.clear()
            tf_of[(side, key)] = tf
        return tf

    def gap(kl, kr):
        got = gap_of.get((kl, kr))
        if got is None:
            (tl, ol), (tr, orr) = transforms("left", kl), transforms("right", kr)
            ra, rb = hulls["left"][3], hulls["right"][3]
            d = np.linalg.norm(ol[:, None, :] - orr[None, :, :], axis=2) - ra[:, None] - rb[None, :]
            if float(d.min()) > limit:                                 # nothing of one arm can be that close to the other
                got = (float(d.min()), None)
            else:
                for s, tf in (("left", tl), ("right", tr)):
                    mgr, entries = hulls[s][0], hulls[s][1]
                    for name, link, origin in entries:
                        T = tf.get(link)
                        if T is not None:
                            mgr.set_transform(name, T @ origin)
                if hulls["left"][0].in_collision_other(hulls["right"][0]):
                    dist, names = 0.0, None
                    _, data = hulls["left"][0].in_collision_other(hulls["right"][0], return_data=True)
                    if data:
                        deepest = max(data, key=lambda c: (float(getattr(c, "depth", 0.0) or 0.0), tuple(sorted(c.names))))
                        dist, names = -float(getattr(deepest, "depth", 0.0) or 0.0), tuple(deepest.names)
                    got = (dist, names)
                else:
                    dist, names = hulls["left"][0].min_distance_other(hulls["right"][0], return_names=True)
                    got = (float(dist), tuple(names))
            if len(gap_of) > 400000:
                gap_of.clear()
            gap_of[(kl, kr)] = got
        return got

    out, total, done = [], sum(len(sc["times"]) for sc in scenarios), 0
    for sc in scenarios:
        times, arms = [float(t) for t in sc["times"]], sc["arms"]
        L, R = arms["left"], arms["right"]
        events, cur = [], None
        for i, t in enumerate(times):
            done += 1
            if progress is not None and done % 2000 == 0:
                progress(done, total)
            if not (bool(L["known"][i]) and bool(R["known"][i])):
                cur = None
                continue
            kl, kr = pose_key(L, i), pose_key(R, i)
            d, names = gap(kl, kr)
            if d < limit:
                if cur is None:
                    cur = dict(first_t=t, last_t=t, min_mm=d * 1000.0, names=names, at=i, keys=(kl, kr), moments=[])
                    events.append(cur)
                else:
                    cur["last_t"] = t
                    if d * 1000.0 < cur["min_mm"]:
                        cur.update(min_mm=d * 1000.0, names=names, at=i, keys=(kl, kr))
                cur["moments"].append((t, kl, kr))
            else:
                cur = None
        kept = []
        for e in events:                                    # the real meshes' gap over the stretch, said exactly
            moments = e.pop("moments")
            pick = moments if len(moments) <= EXACT_MOMENTS else \
                [moments[int(k)] for k in np.linspace(0, len(moments) - 1, EXACT_MOMENTS - 1)]
            best = None
            for t, kl, kr in [(None, *e.pop("keys"))] + pick:  # the hulls' closest moment first
                got = exact_of.get((kl, kr))
                if got is None:
                    exact["left"].pose(transforms("left", kl)[0])
                    exact["right"].pose(transforms("right", kr)[0])
                    if exact["left"].mgr.in_collision_other(exact["right"].mgr):
                        _, data = exact["left"].mgr.in_collision_other(exact["right"].mgr, return_data=True)
                        deepest = max(data, key=lambda c: (float(getattr(c, "depth", 0.0) or 0.0), tuple(sorted(c.names))),
                                      default=None)                # the deepest contact (a set's first is any one)
                        got = (0.0, tuple(deepest.names) if deepest is not None else ())
                    else:
                        dist, names = exact["left"].mgr.min_distance_other(exact["right"].mgr, return_names=True)
                        got = (round(1000.0 * float(dist), 1), tuple(names))
                    exact_of[(kl, kr)] = got
                if best is None or got[0] < best[0]:
                    best = got
                if best[0] <= 0.0:
                    break
            if best[0] >= float(clearance_mm):              # only the hulls come that close
                continue
            e["exact_mm"] = best[0]
            names = best[1] or e["names"] or ()
            e.pop("names", None)
            e["links"] = sorted({n.split("#", 1)[0] for n in names})
            e["min_mm"] = round(e["min_mm"], 1)
            e["first_t"], e["last_t"] = round(e["first_t"], 3), round(e["last_t"], 3)
            kept.append(e)
        out.append(dict(events=kept[:MAX_EVENTS], more=max(0, len(kept) - MAX_EVENTS), samples=len(times)))
    return out
