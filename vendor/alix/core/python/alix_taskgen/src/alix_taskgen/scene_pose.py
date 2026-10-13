"""Resolve component STL poses from fixture scene YAML files."""

from pathlib import Path

import numpy as np
import yaml
from scipy.spatial.transform import Rotation

from alix_taskgen.fixture_parts import load_part_registry
from alix_taskgen.transforms import Transform
from alix_taskgen.transforms import pose_rpy_to_transform
from alix_taskgen.paths import strip_package_uri, tasks_pkg_root


def load_scene(scene_path):
    with open(scene_path, "r", encoding="utf-8") as stream:
        return yaml.safe_load(stream)


# UGV-front to fixture-front distance the calibrated scene.yaml pose was
# measured at. Tasks may override per-config (task_parameters.ugv_standoff_m);
# the difference shifts the fixture along base_link +X (the approach axis —
# physically the UGV repositions, so in robot-fixed frames the fixture moves).
from alix_taskgen.constants import DEFAULT_UGV_STANDOFF_M  # single source (F73)


def default_ugv_standoff(scene_path):
    """The standoff the fixture's calibrated pose was measured at.

    Declared as ``fixture.pose.default_ugv_standoff_m`` in scene.yaml; falls
    back to the fleet-wide 0.25 m convention when absent.
    """
    scene = load_scene(scene_path) or {}
    pose = (scene.get("fixture", {}) or {}).get("pose", {}) or {}
    value = pose.get("default_ugv_standoff_m")
    return float(value) if value is not None else DEFAULT_UGV_STANDOFF_M


# Standoff bounds live in constants.py — the single source of truth (F73).
# The FLOOR is Newton's fleet rule: never closer than 250 mm to the UGV front.
from alix_taskgen.constants import UGV_STANDOFF_MIN_M, UGV_STANDOFF_MAX_M
UGV_STANDOFF_RANGE_M = (UGV_STANDOFF_MIN_M, UGV_STANDOFF_MAX_M)


def validate_ugv_standoff(value):
    """Sanity-check that a standoff is metres and within the fleet bounds.

    Above the max is a unit slip (380 for 0.38) — rejected with the hint.
    BELOW THE 250 mm FLOOR is a data error but not a generation-stopper: the
    value clamps to the floor with a loud warning, so a stray declaration
    (commode_b once shipped a region at 0.03) surfaces instead of either
    silently violating the fleet rule or killing a whole fixture sweep."""
    v = float(value)
    lo, hi = UGV_STANDOFF_RANGE_M
    if lo <= v <= hi:
        return v
    if v > hi:
        raise ValueError(f"ugv_standoff must be METRES within [{lo}, {hi}], got {value}"
                         f" — did you mean {v / 1000:.3f} m?")
    import warnings
    warnings.warn(f"ugv_standoff {value} m is below the {lo} m fleet floor "
                  f"(UGV front to fixture front tip) — clamped to {lo}")
    return lo


def standoff_delta(scene_path, ugv_standoff_m):
    """base_link X shift for a task standoff (0.0 when unset / at default).

    The shift is along base_link +X unconditionally BY DESIGN — it mirrors the
    C++ runtime (articulated_scene_builder.cpp does ``cpos_vec[0] +=
    standoff_delta``), so studio and robot always agree. For a yawed fixture
    base_link X is no longer the approach axis; both sides share that
    limitation, so we warn instead of silently diverging from the runtime.
    """
    if ugv_standoff_m is None:
        return 0.0
    delta = validate_ugv_standoff(ugv_standoff_m) - default_ugv_standoff(scene_path)
    if abs(delta) > 1e-9:
        try:
            fixture = (load_scene(scene_path) or {}).get("fixture", {}) or {}
            rpy = (fixture.get("pose", {}) or {}).get("orientation_rpy_deg", [0.0, 0.0, 0.0])
            if abs(float(rpy[2])) > 1.0:
                import warnings
                warnings.warn(
                    f"UGV standoff shift is applied along base_link X (matching the "
                    f"C++ runtime), but this fixture has yaw={rpy[2]}° — the shift "
                    f"is not along the fixture's approach axis.", stacklevel=2)
        except Exception:
            pass
    return delta


def load_washroom_rotation(scene_path):
    """Load washroom structure orientation from the sibling venue washroom_scene.yaml.

    Walks up from the fixture scene.yaml two directories to reach the config/
    folder, then looks for config/scene/washroom_scene.yaml.  Returns a
    ``Rotation`` matching the washroom_structure orientation, or ``None`` if
    the venue file does not exist or carries no orientation.

    This mirrors the ``washroom_quat`` applied in C++
    ``ArticulatedSceneBuilder::calculateAbsolutePose``.
    """
    venue_path = Path(scene_path).parents[2] / "scene" / "washroom_scene.yaml"
    if not venue_path.is_file():
        return None
    try:
        with open(venue_path, encoding="utf-8") as f:
            venue = yaml.safe_load(f)
    except Exception:
        return None
    rpy_deg = (
        venue.get("washroom_scene", {})
             .get("washroom_structure", {})
             .get("pose", {})
             .get("orientation_rpy_deg")
    )
    if rpy_deg is None:
        return None
    return Rotation.from_euler("xyz", rpy_deg, degrees=True)


def _fixture_root_transform(scene_path, washroom_rotation=None, standoff_delta_m=0.0):
    """The fixture's root placement in base_link (pose + standoff + washroom)."""
    scene = load_scene(scene_path)
    fixture = scene.get("fixture", {})
    fixture_pose = fixture.get("pose", {})
    position = np.asarray(fixture_pose.get("position", [0.0, 0.0, 0.0]), dtype=float)
    # Per-task UGV standoff: the fixture shifts along base_link +X by the
    # difference from the calibrated default (see standoff_delta()).
    position = position + np.array([float(standoff_delta_m), 0.0, 0.0])
    rpy = list(fixture_pose.get("orientation_rpy_deg", [0.0, 0.0, 0.0]))
    root = pose_rpy_to_transform(position.tolist() + rpy)

    # Compose washroom orientation: mirrors C++ calculateAbsolutePose which does
    # final_quat = washroom_quat * object_quat. VERIFIED against the C++ chain
    # (articulated_scene_builder.cpp): for a base_link-relative calibrated pose
    # the runtime first maps the position through toWashroomRelative (the exact
    # inverse rotation) and then back through calculateAbsolutePose, so the NET
    # position is unchanged and only the orientation composes — exactly this.
    if washroom_rotation is not None:
        root = Transform(root.translation, washroom_rotation * root.rotation)
    return root


def _registry_part(scene_path, component, fixture_state=None):
    """Resolve a part by name; 'body'/'default' alias to the primary part."""
    registry = load_part_registry(scene_path)
    if not registry:
        return registry, None
    part = registry.part(component)
    if part is None and registry.is_combined:
        # Combined fixture: `lid` is not a part, it is a STATE of the whole
        # commode. Resolve to the pre-baked mesh for the state this region needs.
        part = registry.state_part(fixture_state) or registry.primary_part
    if part is None and component in ("body", "default"):
        part = registry.primary_part
    return registry, part


def resolve_component_pose(scene_path, component, articulation_state=None, joint_angle_deg=None,
                           washroom_rotation=None, standoff_delta_m=0.0):
    root = _fixture_root_transform(scene_path, washroom_rotation, standoff_delta_m)
    registry = load_part_registry(scene_path)
    if not registry:
        # Degenerate scene with no declared parts: everything sits at the root.
        return root
    part = registry.part(component)
    if part is None:
        if registry.is_combined:
            # Every pre-baked state mesh is authored in the fixture frame, so
            # it sits at the root whichever state was asked for. Nothing to
            # articulate — that is what "combined" means.
            return root
        if component in ("body", "default"):
            # Legacy alias: the static fixture root, even when no part block
            # carries that name (pre-baked-mesh fixtures, minimal scenes).
            return root
        raise ValueError(
            f"Unknown fixture part '{component}'. Parts declared in scene.yaml: "
            f"{', '.join(registry.names)}")
    if part.static:
        return root

    # Parent chain: the joint origin lives in the parent part's frame. Parents
    # resolve at their default (closed/static) state — fixtures articulating a
    # part on another moving part need explicit per-part states (future work).
    if part.parent and part.parent != (registry.primary_part.name if registry.primary_part else None):
        parent_transform = resolve_component_pose(
            scene_path, part.parent, washroom_rotation=washroom_rotation,
            standoff_delta_m=standoff_delta_m)
    else:
        parent_transform = root

    joint = part.joint or {}
    origin = np.asarray(joint.get("origin", [0.0, 0.0, 0.0]), dtype=float)
    origin_rpy = joint.get("origin_rpy_deg", [0.0, 0.0, 0.0])
    if joint_angle_deg is not None:
        angle = float(joint_angle_deg)
    else:
        angle = part.joint_angle_deg(articulation_state)

    hinge_world = parent_transform.apply_points([origin])[0]
    hinge_rotation = parent_transform.rotation * Rotation.from_euler("xyz", origin_rpy, degrees=True)
    if abs(angle) < 1e-12:
        return Transform(hinge_world, hinge_rotation)

    axis = np.asarray(joint.get("axis", [0.0, 1.0, 0.0]), dtype=float)
    axis_world = hinge_rotation.apply(axis)
    axis_norm = np.linalg.norm(axis_world)
    if axis_norm < 1e-12:
        raise ValueError("Joint axis cannot be zero")
    joint_rotation = Rotation.from_rotvec(np.deg2rad(angle) * axis_world / axis_norm)
    return Transform(hinge_world, joint_rotation * hinge_rotation)


def resolve_mesh_path(scene_path, component, repo_root, fixture_state=None):
    """Mesh for *component*; for a combined fixture, for *fixture_state*."""
    _, part = _registry_part(scene_path, component, fixture_state=fixture_state)
    mesh_uri = part.mesh_uri if part else None
    if not mesh_uri:
        return None
    relative = strip_package_uri(mesh_uri)
    if relative is not None:
        return tasks_pkg_root(repo_root) / relative
    return Path(mesh_uri)


def fixture_components(scene_path):
    """Part names for a fixture (viewer/paint), from the part registry."""
    return list(load_part_registry(scene_path).names)


def fixture_sagittal_plane(scene_path, washroom_rotation=None, standoff_delta_m=0.0):
    """The fixture's left/right symmetry plane in base_link.

    ``{"origin": [x,y,z], "normal": [nx,ny,nz]}`` — the plane through the
    fixture root spanned by its local X-Z axes. Arm-side geometry (RL_paired
    splitting, right<->left mirroring) reflects/divides about THIS plane
    instead of assuming base_link Y=0, so off-center or yawed fixtures get the
    correct division. The normal is oriented toward base_link +Y (the robot's
    left) so "left of the plane" keeps its robot-relative meaning regardless
    of fixture yaw (e.g. commode_c's washroom yaw=180 flips the fixture's own
    +Y to base_link −Y). For every centered, unrotated fixture this degrades
    exactly to the historical base_link Y=0 plane.
    """
    root = _fixture_root_transform(scene_path, washroom_rotation, standoff_delta_m)
    # The plane normal is the fixture's LATERAL axis in base_link. That is
    # local +Y for the fleet's yaw-0/180 fixtures — but a fixture whose mesh
    # carries fore-aft on local Y (washtub_a, yaw 90 in scene.yaml) maps local
    # Y to base_link ∓X, which turned the "sagittal" split FORE-AFT: the basin
    # divided front-half/back-half and its front lip fell entirely into one
    # arm's half (F75). Pick whichever local axis actually lies lateral —
    # degrades exactly to local Y for every yaw-0/180 fixture.
    candidates = [np.asarray(root.rotation.apply(axis), dtype=float)
                  for axis in ([1.0, 0.0, 0.0], [0.0, 1.0, 0.0])]
    normal = max(candidates, key=lambda v: abs(float(v[1])))
    normal = normal / np.linalg.norm(normal)
    if normal[1] < 0:
        normal = -normal
    return {"origin": [float(v) for v in root.translation],
            "normal": [float(v) for v in normal]}


def resolve_mesh_scale(scene_path, component):
    """Per-part mesh scale from scene.yaml (default unit).

    Previously ``scale:`` was parsed by the C++ scene builder but SILENTLY
    ignored here — a fixture authored in millimetres (commode_a, scale 0.001)
    rendered 1000x too large in the studio. Apply with
    ``mesh.apply_scale(resolve_mesh_scale(...))`` right after loading, BEFORE
    any placement transform.
    """
    _, part = _registry_part(scene_path, component)
    return tuple(part.scale) if part is not None else (1.0, 1.0, 1.0)


def _repair_inverted_bodies(mesh):
    """Invert connected bodies whose winding is consistent but INWARD.

    commode_a's open-state STLs ship shells wound inside-out (lid_open: 10,996
    of 58,938 faces in negative-volume bodies; lid_seat_open NET negative) —
    the viewer rebuilds normals from winding, so those shells shade black, and
    the oriented-normals generation regime would press from the wrong side.
    Only bodies that are winding-CONSISTENT with negative signed volume are
    flipped: for them the sign is meaningful and the repair is exact; bodies
    with inconsistent winding are left for the upstream watertight
    preprocessing (trimesh's own fix_normals declines them too).
    """
    try:
        bodies = mesh.split(only_watertight=False)
    except Exception:  # noqa: BLE001 — unsplittable: leave as loaded
        return mesh
    # Degenerate slivers report volume ~0 with an arbitrary sign — flipping
    # them churns bytes for nothing (and trimesh warns divide-by-zero on
    # their centroids). 1e-9 m^3 = 1 mm^3.
    _MIN_VOLUME_M3 = 1e-9
    if len(bodies) <= 1:
        if mesh.is_winding_consistent and mesh.volume < -_MIN_VOLUME_M3:
            mesh.invert()
        return mesh
    flipped = False
    for body in bodies:
        if body.is_winding_consistent and body.volume < -_MIN_VOLUME_M3:
            body.invert()
            flipped = True
    if not flipped:
        return mesh
    import trimesh
    return trimesh.util.concatenate(bodies)


def load_component_mesh(scene_path, component, repo_root, fixture_state=None):
    """Load a part's mesh with its scene.yaml scale applied (local frame).

    ``fixture_state`` matters for COMBINED fixtures (commode_a): a part name
    like ``lid`` is a STATE of the whole fixture there, and resolving without
    it silently returns the closed-state STL for every region.

    Returns None when the part has no mesh on disk.
    """
    import trimesh
    mesh_path = resolve_mesh_path(scene_path, component, repo_root,
                                  fixture_state=fixture_state)
    if not mesh_path or not Path(mesh_path).is_file():
        return None
    mesh = trimesh.load(mesh_path, force="mesh")
    scale = resolve_mesh_scale(scene_path, component)
    if any(abs(s - 1.0) > 1e-12 for s in scale):
        mesh.apply_scale(scale)
    return _repair_inverted_bodies(mesh)