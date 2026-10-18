"""Soil as a FIELD on the fixture surface, with a state that wiping changes.

Newton, 2026-10-05, on what the synthetic stains have to be:

    "we have to be able to model such stains which are realistic such that on
    scrubbing or wiping, based on water content, they can be removed or spread,
    realistically how they'd behave"

A decal cannot do that. A decal is a picture pasted on; it has no quantity, no
moisture, and no response to a pad passing over it. Yesterday's design chose
decals and they were the wrong primitive for the stated objective.

THE MODEL
---------
Soil lives on the triangles of the fixture's REGION meshes
(``<component>_regions.obj``), subdivided until no edge exceeds ``cell_m``.
Every cell carries:

    amount    s in [0, 1]   how much soil is there
    moisture  m in [0, 1]   how wet it is (fresh = wet, dried-on = dry)
    class     c             what it is: a row of `soil_classes.yaml`

and a WIPE is a pad of known footprint (``glove_frames.yaml``: 35 x 80 mm palm,
20 x 80 mm finger pads) moved along a stroke with a wetness and a pressure. For
every cell under the pad:

    lift      = s * k(c, pad_wet, m) * pressure      what the pad takes off
    s        -= lift
    pad_load += lift * area                           the pad is now dirtier
    m        <- max(m, pad_wet)                       a wet pad wets the surface

and a fraction ``spread(c)`` of what the pad carries is REDEPOSITED on the
cells at the pad's trailing edge — which is what smearing is. Dry pad on a wet
organic stain: high lift, high spread, the mark moves rather than disappears.
Wet pad on the same stain: high lift, low spread, it comes off. Any pad on
limescale: near-zero lift, because limescale needs acid, not water — and a
model that let a damp cloth remove it would teach the robot to keep wiping.

Every parameter is DATA (`config/soil_classes.yaml`) and every one is a stated
assumption, not a measurement: there is no measurement of real limescale or
faecal transfer coefficients anywhere in this repository. The structure is what
is defensible — mass is conserved between surface, pad and removed; wet removes
more than dry for water-soluble soil; nothing water-based moves limescale — and
the numbers are the first thing a real wipe test should replace.

WHAT THIS BUYS, beyond the rendering
------------------------------------
* **Ground truth that is a quantity.** The field IS the label: a mask, a region
  (the cell knows which group it came from), a 3D centroid in fixture-local
  coordinates, and an amount. No second render pass, no decal/opacity trick.
* **Localisation can be scored.** The 3D lift in `geometry.py` has never been
  checked against a known position; the field gives one.
* **The cleaning stack can be closed on.** Replay a generated cleaning config's
  strokes over the field and read what is left. ``remaining_fraction`` per region
  is the number `pass / target-clean / re-clean` should actually be decided on,
  instead of a detection count.

SHAPE IS IN THE FIELD (2026-10-05, second pass)
-----------------------------------------------
Newton on the first dataset: the marks "do not look like real stains". A disc
with a quadratic falloff does not. So a deposit now has a `pattern`
(`config/soil_classes.yaml`, `appearance:`): a splatter with satellite spots, a
drip with a tail running the way gravity runs on that surface, a ring that is
heaviest at its rim (a dried droplet, a limescale line), a speckle (mould), a
film (soap scum), each with an irregular outline and per-cell grain. The shape
lives in the per-cell AMOUNT, so the mask, the dynamics and the picture are the
same object — a smear of a splatter is a smear of exactly those cells.

The second primitive, `wipe_pad`, is the live one: the pad at a POSE (centre,
press axis, long axis) with a motion since the last step, which is what the
Isaac loop has every frame from the glove-frame prims. `wipe` (a straight
stroke) stays for the replay tool and the tests; both share the same lift and
smear.

Pure numpy. No Isaac, no ROS, no trimesh: the OBJ reader below handles the one
dialect these files use, and the subdivision is midpoint splitting, so the
whole model is unit-testable on any machine.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import yaml

SOIL_CLASSES_FILE = Path(__file__).resolve().parent.parent / "config" / "soil_classes.yaml"


class SoilError(ValueError):
    """A soil model could not be built or driven as asked."""


# --------------------------------------------------------------------------- classes
PATTERNS = ("blob", "splatter", "drip", "ring", "speckle", "film")


@dataclass(frozen=True)
class Appearance:
    """How a class of soil is SHAPED when deposited, and how it should be shown.

    The shape parameters act on the field (the amount per cell). The two
    roughness values are rendering hints the Isaac rig reads: a fresh wet mark
    is glossy, a dried one is matte. Every value is a visual judgement stated
    in `soil_classes.yaml`, not a measurement.
    """

    pattern: str = "blob"
    #: outline radius modulation, as a fraction of the radius (0 = a circle)
    edge_noise: float = 0.25
    #: [min, max] count of small satellite spots around a `splatter`
    satellites: tuple = (0, 0)
    #: [min, max] tail length along the surface's own 'down', mm, for a `drip`
    drip_mm: tuple = (0.0, 0.0)
    #: `ring`: the deposit band as a fraction of the radius, heaviest at the rim
    ring_width: float = 0.3
    #: `speckle`: fraction of the cells inside the outline that carry soil
    speckle_density: float = 0.35
    #: per-cell multiplicative noise on the amount (texture)
    grain: float = 0.2
    #: amount at the centre of a fresh deposit
    core_amount: float = 1.0
    #: colour at full amount; the class `color` is the thin edge. None = `color`
    color_dark: tuple | None = None
    roughness_wet: float = 0.08
    roughness_dry: float = 0.55


def _appearance(row: dict, name: str) -> Appearance:
    pattern = str(row.get("pattern", "blob"))
    if pattern not in PATTERNS:
        raise SoilError(f"soil class '{name}': appearance.pattern '{pattern}' is not one of "
                        f"{', '.join(PATTERNS)}")
    pair = lambda key, default: tuple(float(v) for v in row.get(key, default))  # noqa: E731
    dark = row.get("color_dark")
    return Appearance(
        pattern=pattern,
        edge_noise=float(row.get("edge_noise", 0.25)),
        satellites=tuple(int(v) for v in row.get("satellites", (0, 0))),
        drip_mm=pair("drip_mm", (0.0, 0.0)),
        ring_width=float(row.get("ring_width", 0.3)),
        speckle_density=float(row.get("speckle_density", 0.35)),
        grain=float(row.get("grain", 0.2)),
        core_amount=float(row.get("core_amount", 1.0)),
        color_dark=tuple(float(v) for v in dark) if dark else None,
        roughness_wet=float(row.get("roughness_wet", 0.08)),
        roughness_dry=float(row.get("roughness_dry", 0.55)),
    )


@dataclass(frozen=True)
class SoilClass:
    """One kind of soil, as `soil_classes.yaml` declares it."""

    name: str
    #: fraction lifted per pass by a DRY pad, on a WET deposit
    lift_dry: float
    #: fraction lifted per pass by a WET pad
    lift_wet: float
    #: how much a dried-on deposit resists: lift is multiplied by
    #: ``1 - dried_resistance * (1 - moisture)``
    dried_resistance: float
    #: fraction of the pad's load re-deposited at its trailing edge per pass.
    #: Smearing. High for greasy/organic soil under a dry pad, ~0 for soil a wet
    #: pad dissolves and carries away.
    spread_dry: float
    spread_wet: float
    #: rendering, so a frame shows the class as itself
    color: tuple
    #: `types.KIND_*` this maps to when a detector reports it: `stain` for the
    #: wipeable, `permanent` for what a wipe cannot shift
    kind: str = "stain"
    #: starting moisture when freshly placed
    initial_moisture: float = 0.6
    #: typical blob size on the surface, millimetres, for the placer
    size_mm: tuple = (15.0, 45.0)
    description: str = ""
    #: shape when deposited, and rendering hints
    appearance: Appearance = Appearance()
    #: a body-fluid spill with its own procedure (blood): the robot still
    #: cleans it, the flag selects the handling
    hazard: bool = False
    #: `wipe` or `absorb_disinfect` (the NHS two-step: soak up the bulk, then
    #: chlorine, then detergent — the wipe model is the last step only)
    procedure: str = "wipe"

    def roughness(self, moisture: float) -> float:
        """Rendering hint: glossy when wet, matte when dried on."""
        m = max(0.0, min(1.0, float(moisture)))
        return self.appearance.roughness_dry + (self.appearance.roughness_wet
                                                - self.appearance.roughness_dry) * m

    def lift_fraction(self, pad_wet: float, moisture: float) -> float:
        """Fraction of the cell's soil one pass removes."""
        base = self.lift_dry + (self.lift_wet - self.lift_dry) * float(pad_wet)
        resist = 1.0 - self.dried_resistance * (1.0 - float(moisture))
        return max(0.0, min(1.0, base * max(0.0, resist)))

    def spread_fraction(self, pad_wet: float) -> float:
        return self.spread_dry + (self.spread_wet - self.spread_dry) * float(pad_wet)


def load_soil_classes(path: str | Path | None = None) -> dict[str, SoilClass]:
    path = Path(path) if path else SOIL_CLASSES_FILE
    if not path.is_file():
        raise SoilError(f"no soil class table at {path}")
    with path.open() as handle:
        doc = yaml.safe_load(handle) or {}
    out: dict[str, SoilClass] = {}
    for name, row in (doc.get("classes") or {}).items():
        try:
            out[name] = SoilClass(
                name=name,
                lift_dry=float(row["lift_dry"]), lift_wet=float(row["lift_wet"]),
                dried_resistance=float(row.get("dried_resistance", 0.0)),
                spread_dry=float(row.get("spread_dry", 0.0)),
                spread_wet=float(row.get("spread_wet", 0.0)),
                color=tuple(float(v) for v in row.get("color", (0.4, 0.3, 0.2))),
                kind=str(row.get("kind", "stain")),
                initial_moisture=float(row.get("initial_moisture", 0.6)),
                size_mm=tuple(float(v) for v in row.get("size_mm", (15.0, 45.0))),
                description=str(row.get("description", "")),
                appearance=_appearance(row.get("appearance") or {}, name),
                hazard=bool(row.get("hazard", False)),
                procedure=str(row.get("procedure", "wipe")),
            )
        except KeyError as exc:
            raise SoilError(f"{path}: class '{name}' is missing {exc}") from exc
    if not out:
        raise SoilError(f"{path}: declares no classes")
    return out


# --------------------------------------------------------------------------- mesh
def read_obj_groups(path: str | Path) -> dict[str, tuple[np.ndarray, np.ndarray]]:
    """`<component>_regions.obj` -> {group name: (vertices (N,3), faces (M,3))}.

    The one dialect these files use: `v` lines, `o`/`g` group headers, `f`
    lines with `v`, `v/vt`, `v//vn` or `v/vt/vn` tokens, triangles and quads
    (quads are split). Vertex indices are GLOBAL in OBJ, so each group's faces
    are re-indexed onto the vertices it actually uses.
    """
    vertices: list[list[float]] = []
    groups: dict[str, list[list[int]]] = {}
    current = "_default"
    with open(path, "r", encoding="utf-8") as handle:
        for line in handle:
            if line.startswith("v "):
                vertices.append([float(v) for v in line.split()[1:4]])
            elif line.startswith(("o ", "g ")):
                current = line.split(maxsplit=1)[1].strip() if len(line.split()) > 1 else "_default"
                groups.setdefault(current, [])
            elif line.startswith("f "):
                idx = [int(tok.split("/")[0]) - 1 for tok in line.split()[1:]]
                faces = groups.setdefault(current, [])
                for k in range(1, len(idx) - 1):           # fan-triangulate
                    faces.append([idx[0], idx[k], idx[k + 1]])

    all_vertices = np.asarray(vertices, dtype=float)
    out = {}
    for name, faces in groups.items():
        if not faces:
            continue
        face_array = np.asarray(faces, dtype=int)
        used, inverse = np.unique(face_array.ravel(), return_inverse=True)
        out[name] = (all_vertices[used], inverse.reshape(face_array.shape))
    return out


def load_region_tables(models_dir: str | Path) -> tuple[dict, dict, dict]:
    """-> (groups_by_component, region_of_group, region_states) for a fixture's models/.

    `regions.yaml`'s `groups:` is the thing every first draft got wrong: a
    region is a declared UNION of OBJ groups, four of the 21 have groups named
    differently from themselves, and a group may belong to another component
    (`lid_side` spans lid + seat). Read from `models/corrected/` when present,
    which is the oracle copy (Newton, 2026-09-22), never the originals.

    `region_of_group` is keyed by `(component, group)`; `region_states` maps a
    region to `{"lid": open|closed|None, "seat": ...}`.
    """
    models_dir = Path(models_dir)
    corrected = models_dir / "corrected"
    base = corrected if corrected.is_dir() else models_dir
    regions_file = base / "regions.yaml" if (base / "regions.yaml").is_file() else models_dir / "regions.yaml"
    with open(regions_file) as handle:
        doc = yaml.safe_load(handle) or {}
    regions = doc.get("regions", doc)

    region_of_group: dict[tuple[str, str], str] = {}
    region_states: dict[str, dict] = {}
    for key, entry in regions.items():
        if not isinstance(entry, dict):
            continue
        region_states[key] = {"lid": entry.get("lid_state"), "seat": entry.get("seat_state")}
        if entry.get("generate", True) is False:
            # `deep` = bowl_inside_ring + bowl_inside_back, both regions in
            # their own right. A cell can carry one region label, so unions
            # that merely cover other regions do not claim cells.
            continue
        component = str(entry.get("component") or "body")
        for group in entry.get("groups") or [key]:
            if isinstance(group, dict):
                region_of_group[(str(group.get("component") or component),
                                 str(group["name"]))] = key
            else:
                region_of_group[(component, str(group))] = key

    groups_by_component = {}
    for component in ("body", "lid", "seat"):
        path = base / f"{component}_regions.obj"
        if not path.is_file():
            path = models_dir / f"{component}_regions.obj"
        if path.is_file():
            groups_by_component[component] = read_obj_groups(path)
    return groups_by_component, region_of_group, region_states


def build_fields(models_dir: str | Path, classes: dict | None = None, cell_m: float = 0.005
                 ) -> tuple[dict, dict]:
    """-> ({component: SoilField}, region_states) for a fixture's models/ directory."""
    classes = classes or load_soil_classes()
    groups_by_component, region_of_group, region_states = load_region_tables(models_dir)
    fields = {}
    for component, groups in groups_by_component.items():
        mapping = {g: r for (c, g), r in region_of_group.items() if c == component}
        fields[component] = SoilField(component, groups, mapping, classes, cell_m=cell_m)
    if not fields:
        raise SoilError(f"no region meshes under {models_dir}")
    return fields, region_states


#: Which parts are raised in each fixture state; the taxonomy `regions.yaml`
#: and `fixture_rig.STATES` share.
RAISED = {"closed": (), "lid_open": ("lid",), "lid_seat_open": ("lid", "seat")}


def regions_in_state(field, region_states: dict, state: str) -> list[str]:
    """Regions on `field` that exist in `state`.

    A lid-underside region is not a place for soil while the lid is down over
    it — not because it could not be dirty, but because nothing could see or
    wipe it, and a label nobody can act on is noise in the data. A region whose
    `lid_state` / `seat_state` the state does not satisfy is skipped; one with
    no constraint is always in.
    """
    lid_open = "lid" in RAISED.get(state, ())
    seat_open = "seat" in RAISED.get(state, ())
    out = []
    for region in sorted(set(r for r in field.cell_region if r)):
        need = region_states.get(region, {})
        lid_ok = need.get("lid") is None or (need["lid"] == "open") == lid_open
        seat_ok = need.get("seat") is None or (need["seat"] == "open") == seat_open
        if lid_ok and seat_ok:
            out.append(region)
    return out


def place_from_plan(fields: dict, region_states: dict, state: str, rng: np.random.Generator,
                    n: int, allowed_regions: dict | None = None, class_weights: dict | None = None,
                    region_weights: dict | None = None, exclude_regions: dict | None = None,
                    down_local: dict | None = None, class_region_weights: dict | None = None) -> list:
    """Deposit `n` blobs across `fields` the way an SDG plan's `soil:` block asks. -> blobs.

    Regions are drawn by AREA times `region_weights[region]` (default 1), over
    the regions that exist in `state`. Without the weights the pedestal's
    `side` and `front` — large, vertical, never in view from the inspection
    poses — take most of the soil (39 of 47 blobs occluded in the first run).
    A state missing from `allowed_regions` is UNRESTRICTED; `exclude_regions`
    exists for what `regions.yaml` cannot express (`toplid` has no state
    constraint but faces away when the lid is raised). `down_local` gives each
    component's gravity direction in its own frame, for the drip patterns.

    `class_region_weights` ({class: {region: weight}}) multiplies the region
    weight per class, so the placement can follow the evidence (2026-10-05
    research pass): urine lands on the rim, the seat underside and the lid
    underside (splash studies), faecal soil on the bowl sides and under the
    rim, mould in the hinge, scale at the waterline. A class WITH a table is
    confined to the regions it lists — a region it does not name gets the
    table's `_default` weight, 0 unless given — so limescale cannot land on a
    plastic seat because the seat was simply not mentioned (MEASURED: it did,
    on the first run). The class is drawn first, then its region; a class
    with no available region in this state is re-drawn.
    """
    classes = next(iter(fields.values())).classes
    names = sorted(classes)
    weights = np.array([float((class_weights or {}).get(c, 1.0)) for c in names])
    weights /= weights.sum()
    allowed = (allowed_regions or {}).get(state)
    excluded = set((exclude_regions or {}).get(state, []))
    choices = []
    for component, field in fields.items():
        for region in regions_in_state(field, region_states, state):
            if allowed is not None and region not in allowed:
                continue
            if region in excluded:
                continue
            area = float(field.areas[field.cells_in_region(region)].sum())
            area *= float((region_weights or {}).get(region, 1.0))
            if area > 0.0:
                choices.append((component, region, area))
    if not choices:
        return []
    placed = []
    for _ in range(int(n)):
        for _attempt in range(10):
            soil_class = str(rng.choice(names, p=weights))
            per_class = (class_region_weights or {}).get(soil_class)
            if per_class:
                fallback = float(per_class.get("_default", 0.0))
                areas = np.array([c[2] * float(per_class.get(c[1], fallback)) for c in choices])
            else:
                areas = np.array([c[2] for c in choices])
            if areas.sum() > 0.0:
                break
        else:
            continue                      # nothing drawable in this state
        areas /= areas.sum()
        component, region, _ = choices[int(rng.choice(len(choices), p=areas))]
        down = (down_local or {}).get(component, (0.0, 0.0, -1.0))
        placed.append(fields[component].place(soil_class, region, rng, down_local=down))
    return placed


def subdivide(vertices: np.ndarray, faces: np.ndarray, max_edge_m: float
              ) -> tuple[np.ndarray, np.ndarray]:
    """Midpoint-split every face whose longest edge exceeds `max_edge_m`.

    Repeated until no edge does. Each split is 1 -> 4, so the count grows
    quickly; a 5 mm cell on a 40k-triangle body is a few hundred thousand cells,
    which numpy handles and a renderer does not notice because only SOILED cells
    are ever emitted.
    """
    vertices = np.asarray(vertices, dtype=float)
    faces = np.asarray(faces, dtype=int)
    for _ in range(12):                                    # hard stop; 4^12 is absurd
        edges = vertices[faces[:, [1, 2, 0]]] - vertices[faces]
        longest = np.linalg.norm(edges, axis=2).max(axis=1)
        split = longest > max_edge_m
        if not split.any():
            break
        keep = faces[~split]
        big = faces[split]
        # midpoints, shared between neighbouring faces by (lo, hi) vertex key
        key_to_index: dict[tuple[int, int], int] = {}
        new_vertices = [vertices]
        next_index = len(vertices)

        def midpoint(a: int, b: int) -> int:
            nonlocal next_index
            key = (a, b) if a < b else (b, a)
            found = key_to_index.get(key)
            if found is not None:
                return found
            new_vertices.append(((vertices[a] + vertices[b]) * 0.5)[None, :])
            key_to_index[key] = next_index
            next_index += 1
            return next_index - 1

        quads = []
        for a, b, c in big:
            ab, bc, ca = midpoint(a, b), midpoint(b, c), midpoint(c, a)
            quads.extend([[a, ab, ca], [ab, b, bc], [ca, bc, c], [ab, bc, ca]])
        vertices = np.vstack(new_vertices)
        faces = np.vstack([keep, np.asarray(quads, dtype=int)]) if len(keep) else np.asarray(quads, dtype=int)
    return vertices, faces


# --------------------------------------------------------------------------- shapes
def _outline_modulation(rng: np.random.Generator, edge_noise: float, theta: np.ndarray) -> np.ndarray:
    """Radius modulation r(theta)/R - 1: a few low harmonics, bounded by `edge_noise`."""
    if edge_noise <= 0.0:
        return np.zeros_like(theta)
    harmonics = np.arange(2, 7)
    weights = rng.uniform(-1.0, 1.0, len(harmonics))
    phases = rng.uniform(0.0, 2.0 * math.pi, len(harmonics))
    total = sum(w * np.sin(k * theta + phi) for k, w, phi in zip(harmonics, weights, phases))
    return float(edge_noise) * total / max(float(np.abs(weights).sum()), 1e-9)


def _profile(rng: np.random.Generator, app: Appearance, a: np.ndarray, b: np.ndarray,
             r: np.ndarray, outline: np.ndarray, radius: float, drip_m: float) -> np.ndarray:
    """Amount profile in [0, 1] over the candidate cells, by pattern."""
    ratio = r / np.maximum(outline, 1e-9)
    soft = np.clip(1.0 - ratio ** 2, 0.0, 1.0)

    if app.pattern == "film":
        return np.clip(1.0 - ratio ** 6, 0.0, 1.0) * 0.6 * (1.0 + 0.3 * rng.random(len(r)))

    if app.pattern == "ring":
        band = outline * app.ring_width
        rim = np.exp(-((r - (outline - band / 2.0)) / np.maximum(band / 2.0, 1e-9)) ** 2)
        inner = soft * 0.15
        return np.where(ratio <= 1.05, np.maximum(rim, inner), 0.0)

    if app.pattern == "speckle":
        inside = ratio <= 1.0
        on = rng.random(len(r)) < app.speckle_density
        return np.where(inside & on, rng.uniform(0.4, 1.0, len(r)), 0.0)

    profile = soft.copy()
    if app.pattern == "splatter":
        lo, hi = app.satellites
        for _ in range(int(rng.integers(int(lo), int(hi) + 1))):
            angle = rng.uniform(0.0, 2.0 * math.pi)
            dist = rng.uniform(0.6, 1.8) * radius
            size = rng.uniform(0.12, 0.4) * radius
            d = np.hypot(a - dist * math.cos(angle), b - dist * math.sin(angle))
            profile = np.maximum(profile, np.clip(1.0 - (d / size) ** 2, 0.0, 1.0) * 0.9)
    if app.pattern == "drip" and drip_m > 0.0:
        # a tail along +u (down), narrowing, ending in a droplet
        width = 0.3 * radius * (1.0 - 0.5 * np.clip(a / drip_m, 0.0, 1.0))
        tail = (a >= 0.0) & (a <= drip_m) & (np.abs(b) <= width)
        tail_profile = (0.85 - 0.5 * np.clip(a / max(drip_m, 1e-9), 0.0, 1.0)) \
            * np.clip(1.0 - (b / np.maximum(width, 1e-9)) ** 2, 0.0, 1.0)
        profile = np.where(tail, np.maximum(profile, tail_profile), profile)
        d = np.hypot(a - drip_m, b)
        profile = np.maximum(profile, np.clip(1.0 - (d / (0.3 * radius)) ** 2, 0.0, 1.0) * 0.9)
    return profile


# --------------------------------------------------------------------------- field
@dataclass
class Blob:
    """One placed deposit — the unit the manifest records."""

    index: int
    soil_class: str
    region: str
    group: str
    component: str
    center_local: np.ndarray            # (3,), fixture/part-local
    normal_local: np.ndarray
    size_mm: float
    cells: np.ndarray                   # cell indices this blob touched
    #: the appearance pattern it was deposited with (`Appearance.pattern`)
    pattern: str = "blob"
    #: MASS (amount x area) this deposit ADDED to the surface. Not a sum of
    #: per-cell amounts: cells differ in area, and a blob laid over another only
    #: adds what the `maximum` left room for. Measured before/after, so the
    #: conservation check and `remaining_fraction` have an exact baseline.
    mass_initial: float = 0.0


@dataclass
class WipeResult:
    lifted: float               # soil mass taken off the surface this stroke
    redeposited: float          # mass smeared back down at the trailing edge
    cells_touched: int
    pad_load_after: float


class SoilField:
    """Soil state on one component's region meshes."""

    def __init__(self, component: str, groups: dict[str, tuple[np.ndarray, np.ndarray]],
                 region_of_group: dict[str, str], classes: dict[str, SoilClass],
                 cell_m: float = 0.005):
        self.component = component
        self.classes = classes
        self.cell_m = float(cell_m)

        verts, faces, cell_group = [], [], []
        offset = 0
        for group, (v, f) in groups.items():
            v2, f2 = subdivide(v, f, self.cell_m)
            verts.append(v2)
            faces.append(f2 + offset)
            cell_group.extend([group] * len(f2))
            offset += len(v2)
        if not faces:
            raise SoilError(f"{component}: no region groups to carry soil")

        self.vertices = np.vstack(verts)
        self.faces = np.vstack(faces)
        self.cell_group = np.asarray(cell_group)
        self.cell_region = np.asarray([region_of_group.get(g, "") for g in self.cell_group])

        tri = self.vertices[self.faces]
        self.centers = tri.mean(axis=1)
        cross = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
        norms = np.linalg.norm(cross, axis=1)
        self.areas = 0.5 * norms
        self.normals = cross / np.maximum(norms, 1e-12)[:, None]

        n = len(self.faces)
        self.amount = np.zeros(n)
        self.moisture = np.zeros(n)
        self.cls = np.full(n, "", dtype=object)
        #: MASS deposited on each cell at placement (amount x area, summed over
        #: every blob that touched it). The per-REGION baseline: a drip placed
        #: on `bowl_inside_back` runs its tail into the neighbouring region, and
        #: MEASURED (live run, 2026-10-05) a per-blob baseline then read 86 %
        #: remaining on the nominal region before any pad had touched anything.
        self.placed_mass = np.zeros(n)
        self.blobs: list[Blob] = []
        #: soil the pad is carrying, by class — mass that has left the surface
        #: and may come back as a smear
        self.pad_load: dict[str, float] = {}
        #: soil gone for good (left on the pad at the end, or rinsed)
        self.removed: dict[str, float] = {}

    # -- placement ---------------------------------------------------
    def cells_in_region(self, region: str) -> np.ndarray:
        return np.flatnonzero(self.cell_region == region)

    def place(self, soil_class: str, region: str, rng: np.random.Generator,
              size_mm: float | None = None, amount: float = 1.0,
              center_cell: int | None = None, down_local=(0.0, 0.0, -1.0)) -> Blob:
        """Deposit `soil_class` on a random cell of `region`, shaped by its pattern.

        The deposit is every cell within the pattern's reach of the centre whose
        normal faces the same way (dot > 0), so a mark cannot bleed through a
        1 mm bowl wall onto the far side — the failure the design pass measured
        on decals. Within that set the AMOUNT is the pattern (see `Appearance`):
        an irregular outline, satellites, a drip tail along `down_local` (the
        stage's gravity expressed in this part's frame, so a run on a raised
        lid runs the way a real one would), a ring, speckle or film, with
        per-cell grain. Everything about the shape is in the field.
        """
        cls = self.classes.get(soil_class)
        if cls is None:
            raise SoilError(f"unknown soil class '{soil_class}'; known: "
                            f"{', '.join(sorted(self.classes))}")
        candidates = self.cells_in_region(region)
        if not len(candidates):
            raise SoilError(f"{self.component}: region '{region}' has no cells")
        app = cls.appearance

        size = float(size_mm if size_mm is not None else rng.uniform(*cls.size_mm))
        radius = size / 2000.0
        centre_index = int(center_cell if center_cell is not None
                           else rng.choice(candidates, p=self.areas[candidates] / self.areas[candidates].sum()))
        centre = self.centers[centre_index]
        normal = self.normals[centre_index]

        # tangent basis: u runs the way gravity runs ON this surface, v across
        down = np.asarray(down_local, dtype=float)
        u = down - (down @ normal) * normal
        if np.linalg.norm(u) < 1e-3:                          # a horizontal surface: no 'down'
            u = np.cross(normal, [1.0, 0.0, 0.0])
            if np.linalg.norm(u) < 1e-3:
                u = np.cross(normal, [0.0, 1.0, 0.0])
        u /= np.linalg.norm(u)
        v = np.cross(normal, u)

        drip = (rng.uniform(*app.drip_mm) / 1000.0) if app.pattern == "drip" else 0.0
        reach = radius * (1.0 + app.edge_noise) * (2.3 if app.pattern == "splatter" else 1.15) + drip + 0.4 * radius
        near = np.flatnonzero(np.linalg.norm(self.centers - centre, axis=1) <= reach)
        same_side = near[(self.normals[near] @ normal) > 0.0]
        if not len(same_side):
            same_side = np.array([centre_index])

        rel = self.centers[same_side] - centre
        a, b = rel @ u, rel @ v
        r = np.hypot(a, b)
        theta = np.arctan2(b, a)
        outline = radius * (1.0 + _outline_modulation(rng, app.edge_noise, theta))

        profile = _profile(rng, app, a, b, r, outline, radius, drip)
        if app.grain > 0.0:
            profile = profile * np.clip(1.0 + app.grain * rng.normal(0.0, 1.0, len(profile)), 0.3, 1.7)
        deposit = np.clip(profile * app.core_amount * float(amount), 0.0, 1.0)
        keep = deposit > 1e-3
        cells, deposit = same_side[keep], deposit[keep]
        if not len(cells):
            cells, deposit = np.array([centre_index]), np.array([app.core_amount * float(amount)])

        # a class already there is not overwritten — a second deposit of a
        # different class on the same cell keeps the stronger one
        old_amount = self.amount[cells].copy()
        self.amount[cells] = np.maximum(self.amount[cells], deposit)
        added_per_cell = (self.amount[cells] - old_amount) * self.areas[cells]
        self.placed_mass[cells] += added_per_cell
        added = float(added_per_cell.sum())
        self.moisture[cells] = np.maximum(self.moisture[cells], cls.initial_moisture)
        self.cls[cells] = np.where(self.cls[cells] == "", soil_class, self.cls[cells])

        blob = Blob(index=len(self.blobs), soil_class=soil_class, region=region,
                    group=str(self.cell_group[centre_index]), component=self.component,
                    center_local=centre.copy(), normal_local=normal.copy(),
                    size_mm=size, cells=cells, pattern=app.pattern, mass_initial=added)
        self.blobs.append(blob)
        return blob

    # -- dynamics ----------------------------------------------------
    def dry(self, dt_s: float, rate_per_s: float = 1.0 / 600.0) -> None:
        """Moisture decays. Default: e-fold in ten minutes — an assumption."""
        self.moisture *= math.exp(-rate_per_s * float(dt_s))

    def wipe(self, start_local: np.ndarray, end_local: np.ndarray,
             pad_width_m: float, pad_length_m: float, pad_wet: float,
             pressure: float = 1.0, contact_m: float = 0.004) -> WipeResult:
        """One straight stroke of the pad from `start` to `end` (part-local).

        The pad's LONG edge lies across the stroke (`PAD_LONG_AXIS_INDEX`: the
        long edge is tool Y and the stroke travels along tool X), so the swath is
        `pad_length_m` wide and the pad's `pad_width_m` is its extent along the
        stroke. A cell is under the pad when it lies within the swept rectangle
        and within `contact_m` of the stroke plane — the glove thickness is the
        contact band (`glove_frames.yaml: thickness_m 0.02` for the whole
        glove; 4 mm here is the band over which the pad face actually presses).
        """
        start = np.asarray(start_local, dtype=float)
        end = np.asarray(end_local, dtype=float)
        direction = end - start
        length = float(np.linalg.norm(direction))
        if length < 1e-9:
            return WipeResult(0.0, 0.0, 0, sum(self.pad_load.values()))
        direction /= length

        # the stroke plane: the pad face at the centre cell's normal, averaged
        # over what it passes
        rel = self.centers - start
        along = rel @ direction
        in_run = (along >= -pad_width_m / 2.0) & (along <= length + pad_width_m / 2.0)
        if not in_run.any():
            return WipeResult(0.0, 0.0, 0, sum(self.pad_load.values()))
        # local surface normal of the cells in the run defines "across"
        mean_normal = self.normals[in_run].mean(axis=0)
        mean_normal /= max(np.linalg.norm(mean_normal), 1e-9)
        across = np.cross(mean_normal, direction)
        across /= max(np.linalg.norm(across), 1e-9)
        lateral = np.abs(rel @ across)
        height = np.abs(rel @ mean_normal)
        under = in_run & (lateral <= pad_length_m / 2.0) & (height <= contact_m)
        under_cells = np.flatnonzero(under)
        touched = under_cells[self.amount[under_cells] > 1e-6]

        lifted_total = self._lift(touched, pad_wet, pressure)
        # a wet pad wets everything it touches, soiled or not
        if pad_wet > 0.0:
            self.moisture[under_cells] = np.maximum(self.moisture[under_cells], float(pad_wet))

        # SMEAR: redeposit a fraction of the pad's load at the trailing edge
        trailing = under & (along >= length - pad_width_m) & (along <= length + pad_width_m / 2.0)
        redeposited = self._smear(np.flatnonzero(trailing), pad_wet)
        return WipeResult(lifted_total, redeposited, int(len(touched)),
                          sum(self.pad_load.values()))

    def wipe_pad(self, center_local, press_local, long_local, half_width_m: float,
                 half_length_m: float, motion_local=None, pad_wet: float = 0.0,
                 pressure: float = 1.0, contact_m: float = 0.010,
                 candidates: np.ndarray | None = None, facing_cos: float = 0.3) -> WipeResult:
        """The pad at a POSE, pressed once, having moved `motion_local` since last time.

        This is the live primitive: the Isaac loop reads each glove frame's
        transform every physics step and calls this with where the pad is now
        and how far it moved. `center_local` is the contact face centre,
        `press_local` the axis the pad presses along (into the surface),
        `long_local` the pad's long edge (tool Y). A cell is under the pad when
        it lies inside the W x L rectangle, within `contact_m` of the face plane
        along the press axis, and FACES the pad (its normal against the press
        axis by at least `facing_cos`) — a pad hovering over a thin bowl wall
        must not wipe the far side.

        `candidates` restricts the search (the rig passes a KD-tree ball around
        the pad; 490k body cells x 8 pads x 60 Hz is not a brute-force job).
        The trailing edge is the rear band of the rectangle relative to the
        motion; a stationary pad lifts and smears nothing.
        """
        center = np.asarray(center_local, dtype=float)
        press = np.asarray(press_local, dtype=float)
        press /= max(np.linalg.norm(press), 1e-12)
        long_axis = np.asarray(long_local, dtype=float)
        long_axis = long_axis - (long_axis @ press) * press
        long_axis /= max(np.linalg.norm(long_axis), 1e-12)
        short_axis = np.cross(press, long_axis)

        idx = np.arange(len(self.faces)) if candidates is None else np.asarray(candidates, dtype=int)
        if not len(idx):
            return WipeResult(0.0, 0.0, 0, sum(self.pad_load.values()))
        rel = self.centers[idx] - center
        under = ((np.abs(rel @ press) <= contact_m)
                 & (np.abs(rel @ long_axis) <= half_length_m)
                 & (np.abs(rel @ short_axis) <= half_width_m)
                 & ((self.normals[idx] @ press) <= -facing_cos))
        cells = idx[under]
        if not len(cells):
            return WipeResult(0.0, 0.0, 0, sum(self.pad_load.values()))

        touched = cells[self.amount[cells] > 1e-6]
        motion = None if motion_local is None else np.asarray(motion_local, dtype=float)
        travel = 0.0 if motion is None else float(np.linalg.norm(motion))
        m = motion / travel if travel > 1e-6 else short_axis
        extent = abs(m @ long_axis) * half_length_m + abs(m @ short_axis) * half_width_m
        # EVERY COEFFICIENT IS PER PASS — one pad extent of travel over the cell.
        # A 60 Hz loop stamps a cell ~40 times while the pad crosses it, so a
        # stamp lifts the fraction that makes the product over one pass come to
        # exactly the per-pass figure: 1 - (1 - k) ** (travel / extent). MEASURED
        # before this (probe_live_wipe.py): applying k per stamp took 28% off
        # LIMESCALE (k = 0.02) in one sweep. A pad that does not move lifts
        # nothing: scrubbing is motion, and a resting pad is a resting pad.
        scale = min(1.0, travel / max(2.0 * extent, 1e-9))
        lifted_total = self._lift(touched, pad_wet, pressure, exponent=scale) if scale > 0.0 else 0.0
        if pad_wet > 0.0:
            self.moisture[cells] = np.maximum(self.moisture[cells], float(pad_wet))

        redeposited = 0.0
        if travel > 1e-6:
            along = rel[under] @ m
            band = max(2.0 * self.cell_m, 0.25 * extent)
            redeposited = self._smear(cells[along <= -(extent - band)], pad_wet, scale)
        return WipeResult(lifted_total, redeposited, int(len(touched)),
                          sum(self.pad_load.values()))

    def _lift(self, touched: np.ndarray, pad_wet: float, pressure: float,
              exponent: float = 1.0) -> float:
        """Take soil off `touched` cells onto the pad. -> mass lifted.

        `exponent` is the fraction of a pass this contact amounts to: the lift
        applied is ``1 - (1 - k) ** exponent`` so that a pass made of many
        small stamps lifts exactly what one pass does.
        """
        lifted_total = 0.0
        if not len(touched):
            return 0.0
        for name in np.unique(self.cls[touched]):
            if name == "":
                continue
            cls = self.classes[name]
            cells = touched[self.cls[touched] == name]
            base = cls.lift_dry + (cls.lift_wet - cls.lift_dry) * float(pad_wet)
            resist = 1.0 - cls.dried_resistance * (1.0 - self.moisture[cells])
            fraction = np.clip(base * np.clip(resist, 0.0, None), 0.0, 1.0) * float(pressure)
            fraction = np.clip(fraction, 0.0, 1.0)
            if exponent != 1.0:
                fraction = 1.0 - (1.0 - fraction) ** float(exponent)
            lift = self.amount[cells] * fraction
            self.amount[cells] -= lift
            mass = float((lift * self.areas[cells]).sum())
            self.pad_load[name] = self.pad_load.get(name, 0.0) + mass
            lifted_total += mass
        return lifted_total

    def _smear(self, trail_cells: np.ndarray, pad_wet: float, scale: float = 1.0) -> float:
        """Redeposit `scale` x the class's spread fraction of the pad's load on
        `trail_cells`. -> mass."""
        redeposited = 0.0
        if not len(trail_cells):
            return 0.0
        for name, load in list(self.pad_load.items()):
            cls = self.classes[name]
            mass = load * cls.spread_fraction(pad_wet) * float(scale)
            if mass <= 0.0:
                continue
            per_area = mass / max(self.areas[trail_cells].sum(), 1e-12)
            add = np.minimum(per_area, 1.0 - self.amount[trail_cells])
            added = float((add * self.areas[trail_cells]).sum())
            self.amount[trail_cells] += add
            self.cls[trail_cells] = np.where(self.cls[trail_cells] == "", name, self.cls[trail_cells])
            self.pad_load[name] = load - added
            redeposited += added
        return redeposited

    def rinse_pad(self) -> float:
        """Everything on the pad is gone from the problem. -> mass removed."""
        total = 0.0
        for name, load in self.pad_load.items():
            self.removed[name] = self.removed.get(name, 0.0) + load
            total += load
        self.pad_load = {}
        return total

    # -- scoring -----------------------------------------------------
    def mass(self, region: str | None = None) -> float:
        cells = self.cells_in_region(region) if region else slice(None)
        return float((self.amount[cells] * self.areas[cells]).sum())

    def mass_initial(self, region: str | None = None) -> float:
        """Mass deposited ON this region's cells at placement — not the mass of
        the blobs nominally placed there. The two differ whenever a mark runs
        over a region border, which a drip does by design."""
        cells = self.cells_in_region(region) if region else slice(None)
        return float(self.placed_mass[cells].sum())

    def remaining_fraction(self, region: str | None = None) -> float:
        """What fraction of the soil placed on the region is still on it.

        THE NUMBER a pass / re-clean decision should be made on. A detection
        count says how many marks a model found; this says how much dirt there
        is, by mass, against what there was. 0.0 is clean; 1.0 is untouched; a
        value ABOVE 1.0 means wiping made it worse, which a dry pad on a wet
        organic stain genuinely does. A region nothing was placed on but that
        now carries soil (a smear from next door) has no baseline: `inf`, so a
        caller cannot mistake it for clean.
        """
        initial = self.mass_initial(region)
        now = self.mass(region)
        if initial <= 0.0:
            return 0.0 if now <= 1e-15 else math.inf
        return now / initial

    def regions_with_soil(self) -> list[str]:
        """Every region that was soiled at placement or carries soil now,
        including the empty name for cells of OBJ groups no region claims."""
        carrying = (self.placed_mass > 0.0) | (self.amount * self.areas > 1e-15)
        return sorted(set(self.cell_region[carrying]))

    def soiled_cells(self, threshold: float = 0.02) -> np.ndarray:
        return np.flatnonzero(self.amount > threshold)

    def blob_centroid_now(self, blob: Blob) -> np.ndarray | None:
        """Where the blob's mass is NOW — moves when it is smeared."""
        cells = blob.cells
        weights = self.amount[cells] * self.areas[cells]
        if weights.sum() <= 0.0:
            return None
        return (self.centers[cells] * weights[:, None]).sum(axis=0) / weights.sum()

    def conservation_error(self) -> float:
        """|placed - (surface + pad + removed)|, in mass units. Should be ~0.

        Not quite zero by construction: a cell saturates at amount 1.0, so a
        smear onto an already-full cell is clipped and the excess stays on the
        pad. That is handled above; this exists so a test can assert it.
        """
        placed = self.mass_initial()
        held = self.mass() + sum(self.pad_load.values()) + sum(self.removed.values())
        return abs(placed - held)
