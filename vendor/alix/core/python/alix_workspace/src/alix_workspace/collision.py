"""Robot-link vs fixture collision marking (ROS/MoveIt-free).

Given an IK joint configuration, this poses the robot's *collision* link meshes
(read straight from the URDF) at their world transforms and finds which faces of
a fixture mesh (the commode/urinal/etc. STL) the robot intrudes on — so the
viewer can paint those faces (and a ring of neighbours) red.

No ROS, no MoveIt. Geometry is handled with ``trimesh``; if ``python-fcl`` is
installed (via ``trimesh.collision``) it is used to gate genuine interpenetration
precisely, otherwise a robust surface-distance test is used as a fallback.

Mesh URIs of the form ``package://<pkg>/<rel>`` are resolved against a colcon
install/share layout (``<ws>/install/<pkg>/share/<pkg>/<rel>``), auto-detected by
walking up from the URDF, or against explicit ``mesh_roots``.
"""

from __future__ import annotations

import logging
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Sequence

import numpy as np
from scipy.spatial.transform import Rotation

from alix_workspace.package_index import PackageIndex
from alix_workspace.urdf_kinematics import UrdfModel

logger = logging.getLogger("alix_workspace.collision")


@dataclass
class _MeshGeom:
    filename: str
    scale: np.ndarray
    origin: np.ndarray   # 4x4 collision <origin> within the link frame
    color: Optional[np.ndarray] = None   # URDF <material><color> rgba (0-1), or None
    primitive: Optional[dict] = None     # box/cylinder/sphere spec instead of a file


@dataclass
class RobotCollisionModel:
    """Per-link collision-mesh metadata parsed from a URDF, with package://
    resolution and a small load cache."""

    urdf_path: str
    links: Dict[str, List[_MeshGeom]] = field(default_factory=dict)
    tag: str = "collision"   # URDF element to read meshes from: "collision" | "visual"
    index: Optional[PackageIndex] = None
    _cache: Dict[str, object] = field(default_factory=dict)

    @classmethod
    def from_file(cls, urdf_path: str, tag: str = "collision",
                  index: Optional[PackageIndex] = None) -> "RobotCollisionModel":
        # Resolve package:// meshes from the SOURCE tree (portable across
        # machines, no dependency on a built install/share layout).
        self = cls(urdf_path=str(urdf_path), tag=tag,
                   index=index or PackageIndex.discover(urdf_path))
        self._parse_links()
        return self

    # ------------------------------------------------------------------
    def _parse_links(self) -> None:
        root = ET.parse(self.urdf_path).getroot()
        # Robot-level named materials, so a visual that references one by name
        # (``<material name="aluminum"/>``) still resolves to a colour.
        global_colors: Dict[str, np.ndarray] = {}
        for mat in root.findall("material"):
            col = mat.find("color")
            if mat.get("name") and col is not None and col.get("rgba"):
                global_colors[mat.get("name")] = _floats(col.get("rgba"), [1, 1, 1, 1])
        for link in root.findall("link"):
            name = link.get("name", "")
            geoms: List[_MeshGeom] = []
            for col in link.findall(self.tag):
                geom_el = col.find("geometry")
                if geom_el is None:
                    continue
                origin_el = col.find("origin")
                xyz = _floats(origin_el.get("xyz") if origin_el is not None else None, [0, 0, 0])
                rpy = _floats(origin_el.get("rpy") if origin_el is not None else None, [0, 0, 0])
                O = np.eye(4)
                O[:3, :3] = Rotation.from_euler("xyz", rpy).as_matrix()
                O[:3, 3] = xyz
                color = _visual_color(col, global_colors)
                mesh = geom_el.find("mesh")
                if mesh is not None:
                    geoms.append(_MeshGeom(
                        filename=mesh.get("filename", ""),
                        scale=_floats(mesh.get("scale"), [1, 1, 1]),
                        origin=O, color=color,
                    ))
                    continue
                # PRIMITIVE geometry (box / cylinder / sphere) — e.g. the Blender
                # cylinder on the connector links and the sliding-plate box+cylinder
                # (which have no mesh at all, so were previously missing entirely).
                prim = _parse_primitive(geom_el)
                if prim is not None:
                    geoms.append(_MeshGeom(filename="", scale=np.ones(3),
                                           origin=O, color=color, primitive=prim))
            if geoms:
                self.links[name] = geoms

    # ------------------------------------------------------------------
    def _resolve(self, uri: str) -> Optional[Path]:
        return self.index.resolve_uri(uri) if self.index else None

    def _load_mesh(self, geom: _MeshGeom):
        import trimesh
        if geom.primitive is not None:
            key = "prim:" + repr(sorted(geom.primitive.items()))
            if key in self._cache:
                return self._cache[key]
            mesh = _make_primitive(geom.primitive)
            self._cache[key] = mesh
            return mesh
        key = f"{geom.filename}|{geom.scale.tolist()}"
        if key in self._cache:
            return self._cache[key]
        path = self._resolve(geom.filename)
        mesh = None
        if path and path.is_file():
            try:
                loaded = trimesh.load(str(path), force="mesh", process=False)
                if isinstance(loaded, trimesh.Trimesh) and len(loaded.faces):
                    mesh = loaded
                    if not np.allclose(geom.scale, 1.0):
                        mesh = mesh.copy()
                        mesh.apply_scale(geom.scale)
            except Exception as exc:  # pragma: no cover - bad/again mesh files
                logger.warning("Failed to load collision mesh %s: %s", path, exc)
        self._cache[key] = mesh
        return mesh

    def _load_visual_submeshes(self, geom: _MeshGeom):
        """Per-material submeshes of a DAE/OBJ visual, each with its own colour:
        ``[(mesh_in_geometry_frame, rgb_0_1_or_None, name), ...]``.

        RViz parity: a mesh file that carries its own materials (COLLADA, OBJ
        with an MTL) is displayed with those materials, one colour per material
        group; the URDF ``<material>`` only covers meshes that have no colour of
        their own (STL). Loading WITHOUT ``force="mesh"`` is the whole trick —
        ``force="mesh"`` concatenates the scene into one Trimesh, and the
        per-material split is gone before we ever see it (that collapse is what
        made the Studio robot monochrome). Returns ``None`` for formats without
        embedded materials (STL, primitives) and on load failure, so the caller
        falls back to the single-mesh path."""
        import trimesh
        path = self._resolve(geom.filename)
        if path is None or not path.is_file() or path.suffix.lower() not in (".dae", ".obj"):
            return None
        key = f"submeshes|{geom.filename}|{geom.scale.tolist()}"
        if key in self._cache:
            return self._cache[key]
        S = np.eye(4)
        S[:3, :3] *= geom.scale
        subs = []
        try:
            loaded = trimesh.load(str(path), process=False)
            if isinstance(loaded, trimesh.Scene):
                for node in loaded.graph.nodes_geometry:
                    T_node, gname = loaded.graph[node]
                    m = loaded.geometry[gname]
                    if not isinstance(m, trimesh.Trimesh) or not len(m.faces):
                        continue
                    mm = m.copy()
                    mm.apply_transform(S @ np.asarray(T_node, dtype=float))
                    subs.append((mm, _mesh_file_color(m), str(gname)))
            elif isinstance(loaded, trimesh.Trimesh) and len(loaded.faces):
                mm = loaded.copy()
                mm.apply_transform(S)
                subs.append((mm, _mesh_file_color(loaded), path.stem))
        except Exception as exc:  # pragma: no cover - bad/odd mesh files
            logger.warning("Failed to load visual submeshes %s: %s", path, exc)
            subs = []
        result = subs or None
        self._cache[key] = result
        return result

    # ------------------------------------------------------------------
    def local_meshes(self, link_filter=None, max_faces: int = 800,
                     split_materials: bool = False) -> List[dict]:
        """Each mesh in its LINK frame (the geometry's <origin> baked in), so a
        client can render it by applying only the link's world transform. Returns
        ``[{"link": str, "vertices": Nx3, "faces": Mx3}, ...]``. Cheap to send
        once; the per-config link transforms come separately.

        Meshes are decimated to ~``max_faces`` (these robot meshes are millions
        of faces total) for a lightweight overlay, if ``fast_simplification`` is
        available; otherwise sent as-is.

        ``split_materials=True`` emits one entry per material submesh of every
        DAE/OBJ mesh, coloured by that material's diffuse (RViz precedence: the
        file's own materials beat the URDF ``<material>``; STL and primitives
        keep the URDF colour). Entries still carry the owning ``link``, so a
        client that groups by link renders this without any change."""
        out = []
        for link, geoms in self.links.items():
            if link_filter is not None and not link_filter(link):
                continue
            for geom in geoms:
                if split_materials and geom.primitive is None:
                    subs = self._load_visual_submeshes(geom)
                    if subs:
                        urdf_rgb = geom.color[:3] if geom.color is not None else None
                        for sub, rgb, _name in subs:
                            m = sub.copy()
                            m.apply_transform(geom.origin)   # into the link frame
                            try:
                                m.merge_vertices()
                            except Exception:  # noqa: BLE001
                                pass
                            v, f = _decimate(np.asarray(m.vertices, dtype=float),
                                             np.asarray(m.faces, dtype=np.int64),
                                             max_faces)
                            use = rgb if rgb is not None else urdf_rgb
                            color = ([round(float(x), 4) for x in use[:3]]
                                     if use is not None else None)
                            out.append({"link": link, "vertices": v, "faces": f,
                                        "color": color})
                        continue
                mesh = self._load_mesh(geom)
                if mesh is None:
                    continue
                m = mesh.copy()
                m.apply_transform(geom.origin)   # into the link frame
                # WELD duplicate vertices first. STL stores a triangle soup (each
                # face has its own copy of each vertex); without welding there is
                # no shared topology, so any decimation tears the mesh into holes
                # (and the client can't share vertices for smooth shading). This is
                # THE fix for the "holes" — area is preserved after welding.
                try:
                    m.merge_vertices()
                except Exception:  # noqa: BLE001 — keep going with the unwelded mesh
                    pass
                v, f = _decimate(np.asarray(m.vertices, dtype=float),
                                 np.asarray(m.faces, dtype=np.int64), max_faces)
                # Colour: URDF <material><color> wins (it is the intended look and
                # the only colour STL has); else the mesh file's own material/vertex
                # colour (DAE/OBJ); else None -> client default.
                rgb = geom.color[:3] if geom.color is not None else _mesh_file_color(mesh)
                color = [round(float(x), 4) for x in rgb[:3]] if rgb is not None else None
                out.append({"link": link, "vertices": v, "faces": f, "color": color})
        return out

    def posed_meshes(self, model: UrdfModel, joint_values: Dict[str, float],
                     base_transform: Optional[np.ndarray] = None,
                     link_filter=None) -> List[object]:
        """Return the robot collision meshes posed in the world (base) frame for
        the given joint configuration. ``link_filter(link_name)->bool`` selects
        which links to include (default: all links that have collision meshes)."""
        transforms = model.link_world_transforms(joint_values, base_transform)
        out = []
        for link, geoms in self.links.items():
            if link_filter is not None and not link_filter(link):
                continue
            T_link = transforms.get(link)
            if T_link is None:
                continue
            for geom in geoms:
                mesh = self._load_mesh(geom)
                if mesh is None:
                    continue
                m = mesh.copy()
                m.apply_transform(T_link @ geom.origin)
                out.append(m)
        return out


def _floats(text, default) -> np.ndarray:
    if not text:
        return np.array(default, dtype=float)
    return np.array([float(x) for x in str(text).split()], dtype=float)


def _parse_primitive(geom_el) -> Optional[dict]:
    """A box/cylinder/sphere URDF primitive as a spec dict, or ``None``."""
    box = geom_el.find("box")
    if box is not None and box.get("size"):
        return {"type": "box", "size": tuple(_floats(box.get("size"), [0.01, 0.01, 0.01]).tolist())}
    cyl = geom_el.find("cylinder")
    if cyl is not None:
        return {"type": "cylinder", "radius": float(cyl.get("radius", 0.01)),
                "length": float(cyl.get("length", 0.01))}
    sph = geom_el.find("sphere")
    if sph is not None:
        return {"type": "sphere", "radius": float(sph.get("radius", 0.01))}
    return None


def _make_primitive(spec: dict):
    """Build a trimesh for a primitive spec (already welded; URDF axis = Z)."""
    import trimesh
    t = spec.get("type")
    try:
        if t == "box":
            return trimesh.creation.box(extents=np.asarray(spec["size"], dtype=float))
        if t == "cylinder":
            return trimesh.creation.cylinder(radius=spec["radius"], height=spec["length"], sections=32)
        if t == "sphere":
            return trimesh.creation.icosphere(radius=spec["radius"])
    except Exception as exc:  # noqa: BLE001
        logger.warning("Failed to build primitive %s: %s", spec, exc)
    return None


def _visual_color(visual_el, global_colors: Dict[str, np.ndarray]) -> Optional[np.ndarray]:
    """RGBA (0-1) from a URDF ``<visual>`` element: an inline ``<material><color>``
    or, failing that, a robot-level named material. ``None`` if no colour."""
    mat = visual_el.find("material")
    if mat is None:
        return None
    col = mat.find("color")
    if col is not None and col.get("rgba"):
        return _floats(col.get("rgba"), [1, 1, 1, 1])
    name = mat.get("name")
    if name and name in global_colors:
        return global_colors[name]
    return None


def _mesh_file_color(mesh) -> Optional[np.ndarray]:
    """A single representative RGB (0-1) from a loaded mesh's own material/vertex
    colours (DAE/OBJ carry these; STL does not). ``None`` if unavailable."""
    vis = getattr(mesh, "visual", None)
    if vis is None:
        return None
    try:
        mat = getattr(vis, "material", None)
        if mat is not None:
            for attr in ("main_color", "baseColorFactor", "diffuse"):
                c = getattr(mat, attr, None)
                if c is not None and len(c) >= 3:
                    return np.asarray(c[:3], dtype=float) / 255.0
    except Exception:  # noqa: BLE001
        pass
    try:
        fc = getattr(vis, "face_colors", None)
        if fc is not None and len(fc):
            return np.asarray(fc, dtype=float)[:, :3].mean(axis=0) / 255.0
    except Exception:  # noqa: BLE001
        pass
    return None


def _decimate(vertices: np.ndarray, faces: np.ndarray, max_faces: int):
    """Reduce a mesh toward ``max_faces`` if it is heavier and a decimator is
    available; otherwise return it unchanged. Returns (vertices, faces)."""
    if max_faces <= 0 or len(faces) <= max_faces:
        return vertices, faces
    try:
        import fast_simplification
        reduction = max(0.0, min(0.999, 1.0 - max_faces / len(faces)))
        v, f = fast_simplification.simplify(
            vertices.astype(np.float32), faces.astype(np.int32), target_reduction=reduction)
        return np.asarray(v, dtype=float), np.asarray(f, dtype=np.int64)
    except Exception:
        return vertices, faces


# ----------------------------------------------------------------------
# Persistent FCL collision pair (fast per-pose checks)
# ----------------------------------------------------------------------
class PosedRobotCollision:
    """Persistent robot-vs-fixture collision checker.

    Building an FCL BVH for a mesh is the expensive part of a collision query
    (seconds for detailed robot/fixture meshes). This class builds each robot
    link's BVH ONCE (meshes in link frame, geometry origin baked separately)
    and re-poses them per query with ``set_transform`` — no BVH rebuild — so a
    per-pose robot-vs-fixture check drops from seconds to milliseconds.

    Pair it with a cached fixture manager from :func:`build_fixture_manager`.
    """

    def __init__(self, rcm: "RobotCollisionModel", model: UrdfModel, link_filter=None):
        import trimesh
        self._model = model
        self._mgr = trimesh.collision.CollisionManager()
        self._entries = []   # (object_name, link_name, geom origin 4x4)
        for link, geoms in rcm.links.items():
            if link_filter is not None and not link_filter(link):
                continue
            for gi, geom in enumerate(geoms):
                mesh = rcm._load_mesh(geom)
                if mesh is None or not len(mesh.faces):
                    continue
                name = f"{link}#{gi}"
                self._mgr.add_object(name, mesh, np.eye(4))
                self._entries.append((name, link, geom.origin))

    def set_pose(self, joint_values: Dict[str, float],
                 base_transform: Optional[np.ndarray] = None) -> None:
        transforms = self._model.link_world_transforms(joint_values, base_transform)
        for name, link, origin in self._entries:
            T = transforms.get(link)
            if T is not None:
                self._mgr.set_transform(name, T @ origin)

    def in_collision(self, fixture_manager) -> bool:
        return bool(self._mgr.in_collision_other(fixture_manager))

    def contact_count(self, fixture_manager) -> int:
        """Number of FCL contact pairs against the fixture (0 = no collision).
        A cheap collision-intensity proxy that avoids the expensive per-face
        distance pass."""
        hit, data = self._mgr.in_collision_other(fixture_manager, return_data=True)
        return len(data) if hit else 0


def build_fixture_manager(fixture_mesh):
    """One-object CollisionManager for a static fixture mesh (BVH built once —
    cache and reuse it across queries against :class:`PosedRobotCollision`)."""
    import trimesh
    mgr = trimesh.collision.CollisionManager()
    mgr.add_object("fixture", fixture_mesh)
    return mgr


# ----------------------------------------------------------------------
# Fixture face marking
# ----------------------------------------------------------------------
#: centroids per `trimesh.proximity.closest_point` call in `fixture_contact_faces`: bounds the call's transient
#: memory (+0.41 GB at 500 against +2.83 GB unbatched on the measured 8803-centroid case) for ~6 % more time
CONTACT_QUERY_BATCH = 500


def fixture_contact_faces(robot_meshes: Sequence[object], fixture_mesh,
                          contact_tol_m: float = 0.005, ring: int = 1,
                          require_fcl_collision: bool = True) -> List[int]:
    """Indices of fixture faces in contact with any robot mesh, plus ``ring``
    rings of topological neighbours.

    A face is "in contact" if its centroid is within ``contact_tol_m`` of (or
    inside) the robot surface. When ``python-fcl`` is available and
    ``require_fcl_collision`` is set, faces are only returned if fcl confirms a
    genuine mesh interpenetration first (kills near-miss false positives)."""
    import trimesh

    if not robot_meshes or fixture_mesh is None or not len(fixture_mesh.faces):
        return []

    robot = trimesh.util.concatenate(robot_meshes)
    if robot is None or not len(robot.faces):
        return []

    # AABB pre-filter: only fixture faces whose centroid sits inside the robot's
    # bounding box (padded by tol) can possibly be in contact — this avoids the
    # expensive surface-distance query over the whole fixture mesh.
    centroids = fixture_mesh.triangles_center
    lo, hi = robot.bounds
    pad = max(contact_tol_m, 1e-6)
    near_box = np.all((centroids >= lo - pad) & (centroids <= hi + pad), axis=1)
    cand_idx = np.nonzero(near_box)[0]
    if not len(cand_idx):
        return []

    # Optional precise gate: only mark faces if there is a real collision.
    if require_fcl_collision and not _fcl_in_collision(robot, fixture_mesh):
        return []

    # Distance from each candidate face centroid to the robot surface -- asked in
    # BATCHES of CONTACT_QUERY_BATCH centroids. trimesh's closest_point holds every
    # (centroid, nearby robot triangle) pair of one call in memory at once, so its
    # peak grows with the candidate count: MEASURED 2026-10-01 (followups T2, the
    # Studio's /api/ik face marking replayed from a captured editor request, 8803
    # candidate faces of 40184 against a 36544-face robot) +2.83 GB unbatched,
    # +0.69 GB at 1000, +0.41 GB at 500, +0.21 GB at 250, in 3.70 / 3.78 / 3.91 /
    # 4.16 s, the marked faces identical every time (each centroid's distance is
    # its own) -- the transient that, on top of a ~2 GB session, OOM-killed a 6G
    # Studio during a collision Validate (2026-10-01 Studio check).
    cand_centroids = centroids[cand_idx]
    try:
        dist = np.concatenate([
            trimesh.proximity.closest_point(robot, cand_centroids[i:i + CONTACT_QUERY_BATCH])[1]
            for i in range(0, len(cand_centroids), CONTACT_QUERY_BATCH)])
    except Exception:
        # proximity needs a watertight-ish surface; fall back to vertex KDTree.
        from scipy.spatial import cKDTree
        dist = cKDTree(robot.vertices).query(cand_centroids)[0]
    contact = set(cand_idx[np.nonzero(dist <= contact_tol_m)[0]].tolist())
    if not contact:
        return []

    return sorted(_expand_rings(fixture_mesh, contact, ring))


def _expand_rings(mesh, faces: set, ring: int) -> set:
    if ring <= 0:
        return faces
    # adjacency: face -> neighbouring faces (share an edge)
    adj: Dict[int, List[int]] = {}
    for a, b in mesh.face_adjacency:
        adj.setdefault(int(a), []).append(int(b))
        adj.setdefault(int(b), []).append(int(a))
    frontier = set(faces)
    result = set(faces)
    for _ in range(ring):
        nxt = set()
        for f in frontier:
            nxt.update(adj.get(f, ()))
        nxt -= result
        result |= nxt
        frontier = nxt
        if not frontier:
            break
    return result


def _fcl_in_collision(robot, fixture) -> bool:
    """True if robot/fixture interpenetrate (python-fcl via trimesh).

    python-fcl being unavailable used to silently return True, marking faces
    as colliding on the distance test alone — a hidden environment problem
    masquerading as a collision result. Raise instead."""
    import trimesh
    if not getattr(trimesh.collision, "fcl", None):
        raise RuntimeError(
            "python-fcl is required for collision checking but is not "
            "importable — install python-fcl (results would otherwise be "
            "silently wrong).")
    mgr = trimesh.collision.CollisionManager()
    mgr.add_object("robot", robot)
    return bool(mgr.in_collision_single(fixture))
