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
