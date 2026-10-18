"""A soiled fixture for Gazebo: the same field, written as static meshes.

Newton, 2026-10-05: "how do we launch a scene with the waste and stains loaded
as well" — and the worlds are consumed by Gazebo AND Isaac. Isaac gets the
live field (`alix_sim --soil`). Gazebo has no Python in the loop, so it gets
the field FROZEN at launch: the same plan, the same seed, the same placement
rule (`soil.place_from_plan`), written as OBJ meshes with per-cell colour
bands and an SDF model that the launch file spawns at the fixture's pose. The
marks do not respond to the pads there; the truth file beside the model says
what was placed, so a perception run against Gazebo can still be scored.

Two models come out:

    <out>/soil/model.sdf          marks + fixture waste, in the fixture BODY frame
    <out>/floor_waste/model.sdf   floor waste, in the robot's parked (base_link) frame
    <out>/soil_truth.json         what was placed, where, how much

The body frame is the frame `body.stl` is modelled in, which is also the
frame the Gazebo commode SDF's `body_link` uses (its visual has an identity
pose), so spawning the soil model with the commode's own `-x -y -z -R -P -Y`
lands every mark on the porcelain it was placed on. Lid and seat marks are
composed through the hinge the way `scene.yaml` declares it, at the CLOSED
angle, because that is where the Gazebo model spawns its joints.

Pure numpy + yaml. The STL is not loaded here (no snap): marks are lifted
``LIFT_M`` along the region normal and sit up to ~6 mm off the porcelain where
the region OBJ does — acceptable for a static visual, stated here.
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np
import yaml

from .soil import (RAISED, build_fields, load_soil_classes, place_from_plan,
                   regions_in_state)
from .waste import asset, load_waste_classes

LIFT_M = 0.0025
DRAW_THRESHOLD = 0.03
PORCELAIN = np.array([0.93, 0.93, 0.92])
#: colour bands per overlay: a gradient needs more than one MTL entry
COLOUR_BANDS = 6


# --------------------------------------------------------------------------- transforms
def _axis_angle(axis, angle_deg: float) -> np.ndarray:
    a = np.asarray(axis, dtype=float); a /= max(np.linalg.norm(a), 1e-12)
    t = math.radians(float(angle_deg))
    k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(t) * k + (1 - math.cos(t)) * (k @ k)


def part_transforms(scene_yaml: str | Path, state: str) -> dict:
    """{component: (R, t)} part-local -> fixture BODY frame, for `state`.

    `scene.yaml` gives each hinged part a joint origin, an axis and its open /
    closed angles; the part mesh is hinge-relative (fixture_rig, fixture_physics
    and the Gazebo SDF all agree on this).
    """
    with open(scene_yaml) as handle:
        scene = (yaml.safe_load(handle) or {}).get("fixture") or {}
    out = {"body": (np.eye(3), np.zeros(3))}
    raised = RAISED.get(state, ())
    for part in ("lid", "seat"):
        joint = (scene.get(part) or {}).get("joint") or {}
        origin = np.asarray(joint.get("origin") or [0.0, 0.0, 0.0], dtype=float)
        angles = joint.get("angles_deg") or {}
        angle = float(angles.get("open", -110.0)) if part in raised else float(angles.get("closed", 0.0))
        rotation = _axis_angle(joint.get("axis") or [0.0, 1.0, 0.0], angle)
        out[part] = (rotation, origin)
    return out


# --------------------------------------------------------------------------- OBJ
def _colour_bands(colours: np.ndarray, bands: int = COLOUR_BANDS) -> tuple[np.ndarray, np.ndarray]:
    """Quantise per-face colours by luminance into `bands` materials. -> (palette, index per face)."""
    lum = colours @ np.array([0.299, 0.587, 0.114])
    if len(colours) <= bands or lum.max() - lum.min() < 1e-6:
        return np.array([colours.mean(axis=0)]), np.zeros(len(colours), dtype=int)
    edges = np.linspace(lum.min(), lum.max(), bands + 1)
    index = np.clip(np.searchsorted(edges, lum, side="right") - 1, 0, bands - 1)
    palette = np.array([colours[index == b].mean(axis=0) if (index == b).any() else colours.mean(axis=0)
                        for b in range(bands)])
    return palette, index


def write_obj(path: str | Path, vertices: np.ndarray, faces: np.ndarray, face_colours: np.ndarray,
              name: str) -> Path:
    """OBJ + MTL with one material per colour band. Returns the OBJ path."""
    path = Path(path)
    palette, index = _colour_bands(np.asarray(face_colours, dtype=float).reshape(-1, 3))
    mtl = path.with_suffix(".mtl")
    with mtl.open("w") as handle:
        for b, colour in enumerate(palette):
            handle.write(f"newmtl {name}_{b}\nKd {colour[0]:.4f} {colour[1]:.4f} {colour[2]:.4f}\n"
                         f"Ka 0 0 0\nKs 0.05 0.05 0.05\nNs 20\nd 1\n\n")
    with path.open("w") as handle:
        handle.write(f"mtllib {mtl.name}\no {name}\n")
        for v in np.asarray(vertices, dtype=float):
            handle.write(f"v {v[0]:.6f} {v[1]:.6f} {v[2]:.6f}\n")
        faces = np.asarray(faces, dtype=int)
        for b in range(len(palette)):
            rows = np.flatnonzero(index == b)
            if not len(rows):
                continue
            handle.write(f"usemtl {name}_{b}\n")
            for f in faces[rows]:
                handle.write(f"f {f[0] + 1} {f[1] + 1} {f[2] + 1}\n")
    return path


def _sdf_model(name: str, visuals: list[tuple[str, Path]]) -> str:
    parts = [f'<?xml version="1.0"?>\n<sdf version="1.8">\n  <model name="{name}">\n'
             f'    <static>true</static>\n    <link name="link">\n']
    for visual_name, mesh in visuals:
        parts.append(
            f'      <visual name="{visual_name}">\n        <cast_shadows>false</cast_shadows>\n'
            f'        <geometry><mesh><uri>file://{mesh.resolve()}</uri></mesh></geometry>\n'
            f'      </visual>\n')
    parts.append('    </link>\n  </model>\n</sdf>\n')
    return "".join(parts)


# --------------------------------------------------------------------------- the export
def soil_colour(cls, amount: np.ndarray) -> np.ndarray:
    """Same mapping as the Isaac rig: porcelain -> thin colour -> dark colour."""
    amount = np.clip(np.asarray(amount, dtype=float), 0.0, 1.0)
    thin = np.asarray(cls.color, dtype=float)
    dark = np.asarray(cls.appearance.color_dark or cls.color, dtype=float)
    t1 = np.clip(amount / 0.35, 0.0, 1.0)[:, None]
    t2 = np.clip((amount - 0.35) / 0.65, 0.0, 1.0)[:, None]
    colour = PORCELAIN[None, :] * (1.0 - t1) + thin[None, :] * t1
    return np.clip(colour * (1.0 - t2) + dark[None, :] * t2, 0.0, 1.0)


def export_gazebo(models_dir: str | Path, scene_yaml: str | Path, plan, out_dir: str | Path,
                  state: str = "closed", seed: int | None = None, cell_m: float = 0.005) -> dict:
    """Write the soil and floor-waste models for Gazebo. -> paths and the truth.

    `plan` is an `SdgPlan` (its `soil:` / `waste:` blocks and `seed`). The
    placement draws from the same RNG sequence the Isaac rig uses for the same
    seed, so the two simulators soil the fixture alike.
    """
    out_dir = Path(out_dir)
    (out_dir / "soil" / "meshes").mkdir(parents=True, exist_ok=True)
    (out_dir / "floor_waste" / "meshes").mkdir(parents=True, exist_ok=True)
    classes = load_soil_classes()
    fields, region_states = build_fields(models_dir, classes, cell_m=cell_m)
    transforms = part_transforms(scene_yaml, state)
    seed = int(plan.seed if seed is None else seed)
    rng = np.random.default_rng(seed)

    # gravity in each part's frame: the fixture stands upright, so body -Z is down
    down = {c: r.T @ np.array([0.0, 0.0, -1.0]) for c, (r, _) in transforms.items() if c in fields}

    soil_block = plan.soil or {}
    blob_range = soil_block.get("blobs", [0, 0])
    n_blobs = int(rng.integers(int(blob_range[0]), int(blob_range[1]) + 1)) if soil_block else 0
    blobs = place_from_plan(fields, region_states, state, rng, n_blobs, soil_block.get("regions"),
                            soil_block.get("class_weights"), soil_block.get("region_weights"),
                            soil_block.get("exclude_regions"), down_local=down,
                            class_region_weights=soil_block.get("class_region_weights")) if n_blobs else []

    visuals = []
    for component, field in fields.items():
        rotation, translation = transforms.get(component, (np.eye(3), np.zeros(3)))
        soiled = field.soiled_cells(DRAW_THRESHOLD)
        for name in sorted(set(field.cls[soiled]) - {""}) if len(soiled) else []:
            cells = soiled[field.cls[soiled] == name]
            tri = field.vertices[field.faces[cells]] + field.normals[cells][:, None, :] * LIFT_M
            body_pts = (tri.reshape(-1, 3) @ rotation.T) + translation
            faces = np.arange(len(body_pts)).reshape(-1, 3)
            colours = soil_colour(classes[name], field.amount[cells])
            mesh = write_obj(out_dir / "soil" / "meshes" / f"{component}_{name}.obj",
                             body_pts, faces, colours, f"{component}_{name}")
            visuals.append((f"{component}_{name}", mesh))

    # waste: on the fixture (body frame) or on the floor (base_link frame)
    waste_block = plan.waste or {}
    waste_range = waste_block.get("items", [0, 0])
    n_waste = int(rng.integers(int(waste_range[0]), int(waste_range[1]) + 1)) if waste_block else 0
    floor_xy = waste_block.get("floor_xy", [[0.3, 1.1], [-0.4, 0.4]])
    on_fixture_fraction = float(waste_block.get("on_fixture_fraction", 0.4))
    waste_classes = load_waste_classes()
    names = sorted(waste_classes)
    floor_visuals, waste_truth = [], []
    for index in range(n_waste):
        cls = waste_classes[str(rng.choice(names))]
        size_m = float(rng.uniform(*cls.size_mm)) / 1000.0
        vertices, faces, colours = asset(cls, rng, size_m)
        yaw = float(rng.uniform(0.0, 2.0 * math.pi))
        spin = np.array([[math.cos(yaw), -math.sin(yaw), 0.0], [math.sin(yaw), math.cos(yaw), 0.0],
                         [0.0, 0.0, 1.0]])
        vertices = vertices @ spin.T
        on_fixture = cls.allowed_on("fixture") and (not cls.allowed_on("floor")
                                                    or rng.random() < on_fixture_fraction)
        placed = None
        if on_fixture:
            spot = _resting_spot(fields, region_states, state, transforms, rng)
            if spot is not None:
                centre, normal, region = spot
                pts = vertices @ _align_z_to(normal).T + centre + normal * 0.0005
                mesh = write_obj(out_dir / "soil" / "meshes" / f"waste_{index:02d}_{cls.name}.obj",
                                 pts, faces, colours, f"waste_{index:02d}_{cls.name}")
                visuals.append((f"waste_{index:02d}_{cls.name}", mesh))
                placed = ("fixture:" + region, pts.mean(axis=0))
        if placed is None:
            x = float(rng.uniform(*floor_xy[0])); y = float(rng.uniform(*floor_xy[1]))
            pts = vertices + np.array([x, y, 0.0005])
            mesh = write_obj(out_dir / "floor_waste" / "meshes" / f"waste_{index:02d}_{cls.name}.obj",
                             pts, faces, colours, f"waste_{index:02d}_{cls.name}")
            floor_visuals.append((f"waste_{index:02d}_{cls.name}", mesh))
            placed = ("floor", pts.mean(axis=0))
        waste_truth.append({"label": cls.name, "action": cls.action, "hazard": cls.hazard,
                            "surface": placed[0], "size_mm": round(size_m * 1000.0, 1),
                            "position": [round(float(v), 4) for v in placed[1]],
                            "frame": "fixture_body" if placed[0] != "floor" else "base_link"})

    soil_sdf = out_dir / "soil" / "model.sdf"
    soil_sdf.write_text(_sdf_model("alix_soil", visuals))
    floor_sdf = out_dir / "floor_waste" / "model.sdf"
    floor_sdf.write_text(_sdf_model("alix_floor_waste", floor_visuals))

    truth = {
        "plan": plan.name, "seed": seed, "state": state, "frame": "fixture_body",
        "blobs": [{"label": b.soil_class, "kind": classes[b.soil_class].kind, "pattern": b.pattern,
                   "region": b.region, "component": b.component, "size_mm": round(b.size_mm, 1),
                   "mass_initial": round(b.mass_initial, 8),
                   "position_body": [round(float(v), 5) for v in
                                     (transforms[b.component][0] @ b.center_local + transforms[b.component][1])]}
                  for b in blobs],
        # every region that received soil, a drip's tail included — keyed by
        # name, `(unmapped)` for cells of OBJ groups no region claims
        "regions": {(r or "(unmapped)"): {"mass_initial": round(sum(
            (f.mass_initial(r) if r else float(f.placed_mass[f.cell_region == ""].sum()))
            for f in fields.values()), 8)}
                    for r in sorted({r for f in fields.values() for r in f.regions_with_soil()})},
        "waste": waste_truth,
    }
    (out_dir / "soil_truth.json").write_text(json.dumps(truth, indent=1))
    return {"soil_sdf": soil_sdf, "floor_sdf": floor_sdf, "truth": out_dir / "soil_truth.json",
            "blobs": len(blobs), "waste": len(waste_truth), "visuals": len(visuals),
            "floor_visuals": len(floor_visuals)}


def _resting_spot(fields, region_states, state, transforms, rng):
    """An upward-facing cell of a visible region, in the BODY frame. -> (centre, normal, region)."""
    choices = []
    for component, field in fields.items():
        rotation, _ = transforms.get(component, (np.eye(3), np.zeros(3)))
        up = (field.normals @ rotation.T)[:, 2]
        for region in regions_in_state(field, region_states, state):
            cells = field.cells_in_region(region)
            cells = cells[up[cells] > 0.6]
            if len(cells):
                choices.append((component, region, cells, float(field.areas[cells].sum())))
    if not choices:
        return None
    areas = np.array([c[3] for c in choices]); areas /= areas.sum()
    component, region, cells, _ = choices[int(rng.choice(len(choices), p=areas))]
    field = fields[component]
    cell = int(rng.choice(cells, p=field.areas[cells] / field.areas[cells].sum()))
    rotation, translation = transforms.get(component, (np.eye(3), np.zeros(3)))
    normal = rotation @ field.normals[cell]
    return rotation @ field.centers[cell] + translation, normal / max(np.linalg.norm(normal), 1e-12), region


def _align_z_to(normal: np.ndarray) -> np.ndarray:
    z = np.array([0.0, 0.0, 1.0])
    n = np.asarray(normal, dtype=float); n /= max(np.linalg.norm(n), 1e-12)
    v = np.cross(z, n); s = np.linalg.norm(v); c = float(z @ n)
    if s < 1e-9:
        return np.eye(3) if c > 0 else np.diag([1.0, -1.0, -1.0])
    k = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + k + k @ k * ((1 - c) / (s * s))
