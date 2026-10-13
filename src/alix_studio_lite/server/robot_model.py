"""The robot for the 3D view, built the way ALIX Studio builds its robot overlay.

Meshes, as Studio's ``do_robot_geometry(set=visual)``: the pre-baked overlay for the links it holds
(``assets/robot/visual_overlay_baked.npz``, made from the vendored URDF by ALIX's own bake tool:
``tools/bake_robot_overlay.py``), and the URDF's visual meshes at full resolution
(``local_meshes(max_faces=0, split_materials=True)``) for any link the bake lacks. A baked link
whose bounding box is more than 2 mm from the current mesh's is named in the build notes: the bake
is older than the mesh and should be re-baked. Each mesh is in its LINK frame with its own colour.

Posing, as Studio and the Lite plan's one-FK rule: the server computes every link's base-frame
transform with ALIX's FK (``UrdfModel.link_world_transforms``); the page applies one 4x4 per link.

The payload is cached as JSON in ``<data>/cache``, keyed by the URDF's sha256 and the files the
build reads, so later starts read one file. Units are SI: radians and metres.
"""
from __future__ import annotations

import base64
import hashlib
import json
import math
import os
import threading
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

from .. import vendor_paths

VERSION = 3
CACHE_NAME = f"robot_model_v{VERSION}.json"
#: Lite's bake of the vendored URDF, by ALIX's tools/meshes/bake_visual_overlay.py (tools/bake_robot_overlay.py)
BAKE = vendor_paths.ASSETS / "robot/visual_overlay_baked.npz"
#: a baked link whose bounding box is further than this from the current mesh's is reported as stale
STALE_M = 0.002
HOME_SOURCE = ("initial_positions.yaml of the *_description packages (arms, poles, UGV, hands), then the "
               "SRDF states bin_rail/BIN_LOWERED and dual_poles/full_length: ALIX Studio's robot overlay pose")
#: frames one track request may ask for (view3d.js asks for at most 1500)
MAX_TRACK_FRAMES = 5000
_MOVABLE = ("revolute", "continuous", "prismatic")


def build_robot_model(cache_dir) -> dict:
    """The payload for ``GET /api/lite/robot-model``: from the cache when it is current, else built
    and cached."""
    vendor_paths.activate()
    cache = Path(cache_dir) / CACHE_NAME
    key = cache_key()
    try:
        payload = json.loads(cache.read_text())
        if payload.get("key") == key:
            return payload
    except (OSError, ValueError):
        pass
    payload = _build(key)
    cache.parent.mkdir(parents=True, exist_ok=True)
    tmp = cache.with_suffix(".tmp")
    tmp.write_text(json.dumps(payload, separators=(",", ":")))
    os.replace(tmp, cache)                       # a reader never sees half a file
    return payload


def cache_key() -> str:
    """sha256 of the URDF, plus the size and mtime of every other file the build reads (meshes and
    the .mtl beside each OBJ, the bake, the SRDF, the initial positions). Cheap (~20 ms)."""
    from alix_workspace.package_index import PackageIndex

    urdf = vendor_paths.URDF
    index = PackageIndex.discover(str(urdf))
    files = {vendor_paths.SRDF, BAKE}
    for mesh in ET.parse(urdf).getroot().iter("mesh"):
        p = index.resolve_uri(mesh.get("filename", ""))
        if p is not None:
            files |= {Path(p), Path(p).with_suffix(".mtl")}         # an .mtl gives an OBJ its colours
    files |= {Path(d) / "config/initial_positions.yaml" for d in index.name_to_dir.values()}
    stamps = []
    for p in sorted(files):
        try:
            st = p.stat()
            stamps.append(f"{p}:{st.st_size}:{st.st_mtime_ns}")
        except OSError:
            stamps.append(f"{p}:missing")
    h = hashlib.sha256(f"v{VERSION} stale={STALE_M}\n".encode())
    h.update(urdf_sha256().encode())
    h.update("\n".join(stamps).encode())
    return h.hexdigest()


def urdf_sha256() -> str:
    return hashlib.sha256(vendor_paths.URDF.read_bytes()).hexdigest()


# ───────────────────────────────────────────────────────────── FK (shared with the routes)

_FK = None
_FK_LOCK = threading.Lock()


def fk_model():
    """ALIX's URDF model, parsed once: the one FK that poses the page's robot."""
    global _FK
    with _FK_LOCK:
        if _FK is None:
            vendor_paths.activate()
            from alix_workspace.urdf_kinematics import UrdfModel

            _FK = UrdfModel.from_file(str(vendor_paths.URDF))
        return _FK


def pose(values: dict, links) -> dict:
    """Base-frame transforms of ``links`` (16 numbers, row-major) for joint ``values`` in radians and
    metres. Joints not given are 0 and mimic joints follow their source, as in UrdfModel."""
    tf = fk_model().link_world_transforms(values)
    return {ln: tf[ln].reshape(-1).tolist() for ln in links if ln in tf}


def moving_links(side: str, links) -> list[str]:
    """The links of ``links`` that an arm's joints move: everything past joint1, hand included."""
    model = fk_model()
    s = {"left": "L", "right": "R"}[side]
    start = next(j.child for j in model.joints_by_child.values() if j.name == f"{s}_joint1")
    below, stack = {start}, [start]
    while stack:
        parent = stack.pop()
        for j in model.joints_by_child.values():
            if j.parent == parent:
                below.add(j.child)
                stack.append(j.child)
    return [ln for ln in links if ln in below]


def track_frames(side: str, joints_deg, every: int, base: dict, links) -> dict:
    """Frames of an arm track for the links it moves: every ``every``-th line and the last one.
    ``joints_deg`` is [7][N] in degrees (a recording's joint1..7 are the URDF's as they are);
    ``base`` holds the other joints in radians and metres. Frames are float32, base64, shaped
    [frame][link][16] row-major."""
    q = np.asarray(joints_deg, dtype=float)
    n = q.shape[1]
    lines = list(range(0, n, every))
    if lines[-1] != n - 1:
        lines.append(n - 1)
    s = {"left": "L", "right": "R"}[side]
    names = [f"{s}_joint{i}" for i in range(1, 8)]
    model = fk_model()
    out = np.empty((len(lines), len(links), 16), dtype="<f4")
    values = dict(base)
    for f, i in enumerate(lines):
        values.update(zip(names, np.radians(q[:, i]).tolist()))
        tf = model.link_world_transforms(values)
        out[f] = [tf[ln].reshape(-1) for ln in links]
    return dict(side=side, every=every, lines=lines, links=list(links),
                frames=base64.b64encode(out.tobytes()).decode("ascii"))


# ───────────────────────────────────────────────────────────── build

def _build(key: str) -> dict:
    """ALIX Studio's do_robot_geometry(set=visual) (alix_taskgen/server.py:1918-1998), plus a stale-bake warning."""
    from alix_workspace.collision import RobotCollisionModel

    t0 = time.time()
    model = fk_model()
    vcm = RobotCollisionModel.from_file(str(vendor_paths.URDF), tag="visual")
    # every link once at full resolution (Studio's _OVERLAY_MAX_FACES = 0): the fall-through for links
    # the bake lacks, and the yardstick for the stale-bake warning
    live = vcm.local_meshes(max_faces=0, split_materials=True)
    baked = [m for m in _load_baked_visual(BAKE) or [] if m["link"] in vcm.links]
    stale = _stale_links(baked, live)
    from_bake = {m["link"] for m in baked}
    entries = ([dict(m, source="bake") for m in baked]
               + [dict(m, source="live") for m in live if m["link"] not in from_bake])

    links = {name: [] for name in _all_links()}
    meshes, count = {}, {"bake": [0, 0], "live": [0, 0]}          # source -> [triangles, vertices]
    for m in entries:
        mid = f"{m['link']}#{len(links[m['link']])}"
        links[m["link"]].append(mid)
        meshes[mid] = _mesh_payload(m)
        count[m["source"]][0] += len(m["faces"])
        count[m["source"]][1] += len(m["vertices"])

    # Studio's standing pose: the simulation start, bin lowered, full poles (server.py:1986-1995)
    home = _home(model, vcm.index)
    tf = model.link_world_transforms(home)
    home_pose = {ln: tf[ln].reshape(-1).round(6).tolist() for ln, ids in links.items() if ids and ln in tf}
    fallback = sorted({m["link"] for m in entries if m["source"] == "live"})
    notes = []
    if fallback:
        notes.append(f"Not in {BAKE.name}, so served at full resolution: " + ", ".join(fallback))
    if stale:
        notes.append("The bake is older than these meshes (bounding boxes apart): "
                     + ", ".join(f"{ln} ({mm:.0f} mm)" for ln, mm in stale.items())
                     + ". Re-bake with tools/bake_robot_overlay.py.")
    missing_mtl = sorted({p.name for p in _obj_files(vcm, vcm.links) if not p.with_suffix(".mtl").is_file()})
    if missing_mtl:
        notes.append("OBJ meshes whose .mtl is not in vendor/, so they have trimesh's default grey "
                     "(in a bake made from them too): " + ", ".join(missing_mtl))
    return dict(
        version=VERSION, key=key, urdf_sha256=urdf_sha256(), root=model.root_link(),
        joints=[_joint_view(j) for j in model.joints_by_child.values()],
        links=links, meshes=meshes, homePose=home_pose, home=home, home_source=HOME_SOURCE,
        groups=_groups(model),
        stats=dict(meshes=len(meshes), triangles=count["bake"][0] + count["live"][0],
                   vertices=count["bake"][1] + count["live"][1],
                   bake_file=str(BAKE.relative_to(vendor_paths.LITE_ROOT) if BAKE.is_relative_to(vendor_paths.LITE_ROOT) else BAKE),
                   bake=dict(triangles=count["bake"][0], vertices=count["bake"][1], links=len(from_bake)),
                   live=dict(triangles=count["live"][0], vertices=count["live"][1], links=fallback),
                   stale={ln: round(mm, 1) for ln, mm in stale.items()}, build_s=round(time.time() - t0, 2)),
        notes=notes)


# ALIX Studio's _load_baked_visual (alix_taskgen/server.py:1885-1905), keeping the bake's float32 /
# uint32 arrays for Lite's binary transport instead of rounded JSON lists.
def _load_baked_visual(path):
    """Baked payload entries ``[{link, vertices, faces, color?}]``, or None."""
    try:
        z = np.load(path)                        # the meta is a plain string: no pickle needed
        meta = json.loads(str(z["meta"]))
        out = []
        for i, m in enumerate(meta):
            e = {"link": m["link"], "vertices": z[f"V{i}"].astype("<f4"), "faces": z[f"F{i}"].astype("<u4")}
            if m.get("color") is not None:
                e["color"] = m["color"]
            out.append(e)
        return out
    except Exception as exc:  # noqa: BLE001 — a bad bake must not kill the overlay
        print(f"[robot-model] ignoring unreadable bake {path}: {exc}", flush=True)
        return None


# ALIX Studio's _mesh_payload (alix_taskgen/server.py:1908-1915): one entry for the wire, with
# base64 float32 positions and uint32 indices instead of rounded JSON lists.
def _mesh_payload(m) -> dict:
    """Serialize one mesh entry for the wire (vertices in the link frame + colour)."""
    return {"positions": base64.b64encode(np.ascontiguousarray(m["vertices"], dtype="<f4").tobytes()).decode("ascii"),
            "indices": base64.b64encode(np.ascontiguousarray(m["faces"], dtype="<u4").tobytes()).decode("ascii"),
            "color": m.get("color"), "source": m["source"]}


def _stale_links(baked: list[dict], live: list[dict]) -> dict:
    """Baked links whose bounding box is more than STALE_M from the live mesh's: link -> mm."""
    def boxes(entries):
        out = {}
        for m in entries:
            v = np.asarray(m["vertices"], dtype=float)
            lo, hi = v.min(0), v.max(0)
            if m["link"] in out:
                lo, hi = np.minimum(lo, out[m["link"]][0]), np.maximum(hi, out[m["link"]][1])
            out[m["link"]] = (lo, hi)
        return out
    now, then = boxes(live), boxes(baked)
    stale = {}
    for link, (lo, hi) in then.items():
        if link not in now:
            continue
        gap = max(np.abs(lo - now[link][0]).max(), np.abs(hi - now[link][1]).max())
        if gap > STALE_M:
            stale[link] = float(gap) * 1000
    return stale


def _obj_files(vcm, links) -> list[Path]:
    out = []
    for link in links:
        for g in vcm.links.get(link, []):
            p = vcm.index.resolve_uri(g.filename) if g.filename else None
            if p is not None and Path(p).suffix.lower() == ".obj":
                out.append(Path(p))
    return out


# ── joints, home, groups

def _all_links() -> list[str]:
    return [link.get("name") for link in ET.parse(vendor_paths.URDF).getroot().findall("link")]


def _joint_view(j) -> dict:
    limited = j.type in ("revolute", "prismatic")           # a continuous joint has no limits
    return dict(name=j.name, type=j.type, parent=j.parent, child=j.child,
                xyz=[float(x) for x in j.xyz], rpy=[float(x) for x in j.rpy], axis=[float(x) for x in j.axis],
                lower=float(j.lower) if limited else None, upper=float(j.upper) if limited else None,
                mimic=dict(joint=j.mimic[0], multiplier=j.mimic[1], offset=j.mimic[2]) if j.mimic else None)


def _home(model, index) -> dict:
    """ALIX Studio's standing pose: the simulation start, with the bin lowered and full poles."""
    from alix_workspace.srdf_states import SrdfStates, load_initial_positions

    values = load_initial_positions(index)
    srdf = SrdfStates.from_file(str(vendor_paths.SRDF))
    values.update(srdf.group_state("bin_rail", "BIN_LOWERED"))
    values.update(srdf.group_state("dual_poles", "full_length"))
    return {j.name: float(values.get(j.name, 0.0)) for j in model.joints_by_child.values() if j.independent}


def _groups(model) -> list[dict]:
    """Joint lists for the panel: arms, pole lifts, hands (every joint past an arm's last joint),
    then the rest. Mimic joints are left out: they follow their source joint."""
    all_joints = list(model.joints_by_child.values())
    joints = [j for j in all_joints if j.independent]
    by_name = {j.name: j for j in all_joints}

    def below(link: str) -> set:
        out, stack = set(), [link]
        while stack:
            parent = stack.pop()
            for j in all_joints:
                if j.parent == parent:
                    out.add(j.name)
                    stack.append(j.child)
        return out

    groups = []
    for side, s in (("left", "L"), ("right", "R")):
        groups.append(dict(key=f"{side}_arm", label=f"{side.upper()} arm", arm=side,
                           joints=[f"{s}_joint{i}" for i in range(1, 8)]))
    groups.append(dict(key="poles", label="Poles", arm=None,
                       joints=[j.name for j in joints if j.type == "prismatic" and j.parent.endswith("Pole")]))
    for side, s in (("left", "L"), ("right", "R")):
        last = by_name.get(f"{s}_joint7")
        hand = below(last.child) if last else set()
        groups.append(dict(key=f"{side}_hand", label=f"{side.upper()} hand", arm=side,
                           joints=[j.name for j in joints if j.name in hand]))
    used = {n for g in groups for n in g["joints"]}
    groups.append(dict(key="others", label="Bin, glove arm and UGV", arm=None,
                       joints=[j.name for j in joints if j.name not in used]))
    return groups


def si_values(raw, names) -> dict:
    """``{joint: value}`` from a request, as floats; raises ValueError naming unknown joints."""
    if not isinstance(raw, dict):
        raise ValueError("joints must be an object of joint values in radians and metres.")
    out = {str(k): float(v) for k, v in raw.items()}
    unknown = sorted(set(out) - set(names))
    if unknown:
        raise ValueError("Unknown joints: " + ", ".join(unknown))
    bad = [k for k, v in out.items() if not math.isfinite(v)]
    if bad:
        raise ValueError("Not a number: " + ", ".join(bad))
    return out
