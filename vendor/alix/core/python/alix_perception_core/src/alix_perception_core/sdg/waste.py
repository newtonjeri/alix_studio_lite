"""Waste: objects in the washroom, and what the robot is told to do about them.

Newton, 2026-10-05: *"to be able to detect and command an action of detected
and identified waste on the floor or on the fixture being cleaned, e.g. tissue,
and other waste categories that are commonly found in the toilets or
washrooms."*

A stain is soil ON a surface and the response is to wipe it. Waste is a THING on
a surface and the response is to pick it up. That is why this is a separate
taxonomy from `soil_classes.yaml`, with a separate action vocabulary, rather
than a seventh kind of stain.

Every shipped class is the robot's job. The first draft of the table sent
sanitary waste and sharps to a person; Newton, 2026-10-05: *"Sanitary waste and
sharps should also be handled by the robot."* So the action is `clear` for all
ten, and `hazard: true` is a HANDLING flag that travels with the detection
(`ROI3D.hazard`) so the orchestrator can route such an item to its hazard
handling. It does not change the action. `escalate` stays in the vocabulary
for a label the table does not know — "I do not know what this is" still reads
as "a person looks" — and for a deployment that adds a class it wants a person
for.

The table is `config/waste_classes.yaml`; this module reads it and answers
three questions for the analysis stage:

    action_for("tissue")          -> "clear"
    action_for("sharps")          -> "clear"      is_hazard("sharps") -> True
    action_for("odd_thing")       -> "escalate"   (unknown label)

and builds the procedural assets for the placer (`asset()`): a crumpled tissue,
a folded towel, a crinkled wrapper, a cigarette end, a hair clump, a sanitary
pad, a needle, a glove, a mask, a bottle — each a mesh with per-face colours and
a material hint, so a frame shows ten different kinds of thing rather than ten
white lumps.

The crumple is here rather than in the Isaac scripts because it is pure numpy
and the unit tests should be able to assert its size and its watertightness
without a GPU. PhysX cloth was checked and is NOT in this build
(`PhysxParticleClothAPI`, `PhysxDeformableSurfaceAPI` absent from
`omni.usd.schema.physx`), and a settled-cloth probe cost 104 s a sheet; a
procedural crumple costs 5 ms and looks like what a crumpled tissue looks like
from a wrist camera, which is a white irregular lump.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml

WASTE_CLASSES_FILE = Path(__file__).resolve().parent.parent / "config" / "waste_classes.yaml"

#: Per-item actions the analysis stage emits for waste, written into
#: `AnalyzeScene.Response.roi_recommendations` beside the stain decisions.
ACTION_CLEAR = "clear"
ACTION_ESCALATE = "escalate"
WASTE_ACTIONS = (ACTION_CLEAR, ACTION_ESCALATE)


class WasteError(ValueError):
    """The waste table is missing or malformed."""


@dataclass(frozen=True)
class WasteClass:
    name: str
    action: str
    hazard: bool
    surfaces: str              # floor | fixture | both
    size_mm: tuple
    color: tuple
    description: str = ""
    #: colours the generator draws from instead of `color`, if any
    palette: tuple = ()
    #: rendering hints: roughness, metallic, clearcoat (UsdPreviewSurface terms)
    material: dict = None

    def allowed_on(self, surface: str) -> bool:
        return self.surfaces == "both" or self.surfaces == surface

    def pick_color(self, rng: np.random.Generator) -> np.ndarray:
        if self.palette:
            return np.asarray(self.palette[int(rng.integers(len(self.palette)))], dtype=float)
        return np.asarray(self.color, dtype=float)


def load_waste_classes(path: str | Path | None = None) -> dict[str, WasteClass]:
    path = Path(path) if path else WASTE_CLASSES_FILE
    if not path.is_file():
        raise WasteError(f"no waste class table at {path}")
    with path.open() as handle:
        doc = yaml.safe_load(handle) or {}
    out: dict[str, WasteClass] = {}
    for name, row in (doc.get("classes") or {}).items():
        action = str(row.get("action", "")).strip()
        if action not in WASTE_ACTIONS:
            raise WasteError(
                f"{path}: waste class '{name}' declares action '{action}'; "
                f"expected one of {', '.join(WASTE_ACTIONS)}")
        # `hazard` is independent of `action` (Newton, 2026-10-05: hazards are
        # the robot's job too). The loader used to refuse a hazard that was not
        # escalated; it now refuses only an action outside the vocabulary.
        hazard = bool(row.get("hazard", False))
        material = dict(row.get("material") or {})
        material = {"roughness": float(material.get("roughness", 0.7)),
                    "metallic": float(material.get("metallic", 0.0)),
                    "clearcoat": float(material.get("clearcoat", 0.0))}
        out[name] = WasteClass(
            name=name, action=action, hazard=hazard,
            surfaces=str(row.get("surfaces", "both")),
            size_mm=tuple(float(v) for v in row.get("size_mm", (40.0, 140.0))),
            color=tuple(float(v) for v in row.get("color", (0.9, 0.9, 0.9))),
            description=str(row.get("description", "")),
            palette=tuple(tuple(float(v) for v in c) for c in (row.get("palette") or [])),
            material=material,
        )
    if not out:
        raise WasteError(f"{path}: declares no classes")
    return out


_CLASSES: dict[str, WasteClass] | None = None


def classes() -> dict[str, WasteClass]:
    global _CLASSES
    if _CLASSES is None:
        _CLASSES = load_waste_classes()
    return _CLASSES


def action_for(label: str, default: str = ACTION_ESCALATE) -> str:
    """The action for a detected waste label.

    Unknown labels ESCALATE. A detector that reports a waste class the table
    does not know has found something nobody decided a policy for, and the
    conservative reading of "I do not know what this is" is "a person looks".
    """
    found = classes().get(label)
    return found.action if found is not None else default


def is_hazard(label: str) -> bool:
    """Does this label need the hazard handling? Unknown labels do."""
    found = classes().get(label)
    return bool(found.hazard) if found is not None else True


# --------------------------------------------------------------------------- assets
def crumple(rng: np.random.Generator, size_m: float = 0.09, grid: int = 21,
            height_ratio: float = 0.9, folds: int = 3
            ) -> tuple[np.ndarray, np.ndarray]:
    """A crumpled sheet as a height field on a disc. -> (vertices (N,3), faces (M,3)).

    A `grid` x `grid` lattice over a square of side `size_m`, lifted by a sum
    of a few radial folds and a low-octave noise, then clipped to a disc so the
    silhouette is round-ish rather than square. Z is up and the lowest point is
    at z = 0, so the asset sits on whatever it is placed on. The result is a
    single open sheet, watertight nowhere, which is what a crumpled tissue is.
    """
    n = int(grid)
    half = float(size_m) / 2.0
    xs = np.linspace(-half, half, n)
    gx, gy = np.meshgrid(xs, xs, indexing="xy")
    r = np.hypot(gx, gy) / max(half, 1e-9)

    z = np.zeros_like(gx)
    for _ in range(int(folds)):
        angle = rng.uniform(0.0, 2.0 * math.pi)
        freq = rng.uniform(1.5, 4.0)
        phase = rng.uniform(0.0, 2.0 * math.pi)
        z += np.abs(np.sin(freq * (gx * math.cos(angle) + gy * math.sin(angle)) / half * math.pi + phase))
    # low-frequency lumps so it is not a pure ridge pattern
    for octave in (1.0, 2.0, 4.0):
        px, py = rng.uniform(0.0, 2.0 * math.pi, 2)
        z += (0.6 / octave) * (np.sin(octave * gx / half * math.pi + px)
                               * np.cos(octave * gy / half * math.pi + py))
    z -= z.min()
    z /= max(z.max(), 1e-9)
    z *= height_ratio * half
    # drop towards the rim so the sheet lies down at its edge
    z *= np.clip(1.15 - r, 0.0, 1.0)

    vertices = np.stack([gx.ravel(), gy.ravel(), z.ravel()], axis=1)
    faces = []
    for j in range(n - 1):
        for i in range(n - 1):
            a = j * n + i
            b, c, d = a + 1, a + n, a + n + 1
            if r[j, i] <= 1.05 and r[j + 1, i + 1] <= 1.05:
                faces.append([a, b, d])
                faces.append([a, d, c])
    return vertices, np.asarray(faces, dtype=int)


# --------------------------------------------------------------------------- assets, by class
#: What `asset()` returns: vertices (N,3) with min z = 0, triangle faces (M,3),
#: and one linear-RGB colour per FACE (M,3), so a cigarette end can have a tan
#: filter and a white body on one mesh.
Asset = tuple[np.ndarray, np.ndarray, np.ndarray]


def asset(cls: WasteClass, rng: np.random.Generator, size_m: float | None = None) -> Asset:
    """A procedural mesh for one item of `cls`, sized from its table row.

    Ten generators, one per shipped class, each a few dozen lines of numpy: a
    tissue is a crumpled sheet, a towel a folded one, a wrapper a crinkled
    foil, a cigarette end a bent two-tone cylinder, hair a few thin strands, a
    pad a rounded slab, a sharp a needle or a blade, a glove a deflated hand, a
    mask a pleated rectangle with loops, a bottle a cylinder with a neck and a
    cap. None of them is a scan of the real thing; they are the SHAPES, with
    the right proportions, colours and (through `material`) surface finish, so
    a frame shows ten different kinds of object rather than ten white lumps —
    which is what the first dataset showed and what Newton rejected.
    """
    size = float(size_m if size_m is not None else rng.uniform(*cls.size_mm) / 1000.0)
    maker = _MAKERS.get(cls.name, _tissue)
    vertices, faces, colours = maker(cls, rng, size)
    vertices = np.asarray(vertices, dtype=float)
    vertices[:, 2] -= vertices[:, 2].min()                 # rests on whatever it is placed on
    return vertices, np.asarray(faces, dtype=int), np.asarray(colours, dtype=float)


def _vary(colour, rng, n, amount=0.06):
    base = np.asarray(colour, dtype=float)[None, :]
    return np.clip(base * (1.0 + amount * rng.normal(0.0, 1.0, (n, 1))), 0.0, 1.0)


def _tissue(cls, rng, size):
    v, f = crumple(rng, size_m=size, grid=23, height_ratio=rng.uniform(0.5, 0.9),
                   folds=int(rng.integers(2, 5)))
    c = _vary(cls.pick_color(rng), rng, len(f), 0.04)
    return v, f, c


def _sheet(size_x, size_y, n_x, n_y):
    xs = np.linspace(-size_x / 2.0, size_x / 2.0, n_x)
    ys = np.linspace(-size_y / 2.0, size_y / 2.0, n_y)
    gx, gy = np.meshgrid(xs, ys, indexing="xy")
    faces = []
    for j in range(n_y - 1):
        for i in range(n_x - 1):
            a = j * n_x + i
            faces.append([a, a + 1, a + n_x + 1]); faces.append([a, a + n_x + 1, a + n_x])
    return gx, gy, np.asarray(faces, dtype=int)


def _paper_towel(cls, rng, size):
    """A hand towel folded once, lying with a soft crease and a lifted corner."""
    gx, gy, f = _sheet(size, size * 0.55, 19, 11)
    z = 0.012 * np.abs(np.sin(gx / size * math.pi * 1.0 + rng.uniform(0, 1)))      # one fold
    z += 0.004 * np.sin(gy / size * 9.0 + rng.uniform(0, 6)) * np.cos(gx / size * 7.0)
    z += 0.02 * np.clip((gx / size + gy / size * 0.3) - 0.25, 0.0, 1.0)             # a lifted corner
    v = np.stack([gx.ravel(), gy.ravel(), z.ravel()], axis=1)
    c = _vary(cls.pick_color(rng), rng, len(f), 0.05)
    return v, f, c


def _wrapper(cls, rng, size):
    """Crinkled foil: high-frequency creases, low height, saturated colour."""
    gx, gy, f = _sheet(size, size * rng.uniform(0.6, 1.0), 21, 17)
    z = np.zeros_like(gx)
    for _ in range(6):
        ang = rng.uniform(0, 2 * math.pi); freq = rng.uniform(6.0, 14.0); ph = rng.uniform(0, 6)
        z += np.abs(np.sin(freq * (gx * math.cos(ang) + gy * math.sin(ang)) / size + ph))
    z *= 0.004
    z += 0.01 * (1.0 - np.clip(np.hypot(gx, gy) / (size * 0.6), 0, 1)) * rng.uniform(0.5, 1.5)
    v = np.stack([gx.ravel(), gy.ravel(), z.ravel()], axis=1)
    base = cls.pick_color(rng)
    c = _vary(base, rng, len(f), 0.12)
    return v, f, c


def _cylinder(radius, length, n=10, segments=1, axis_profile=None):
    """A closed cylinder along +x from 0 to `length`. -> (v, f, ring index per face)."""
    rings = []
    v = []
    for s in range(segments + 1):
        t = s / segments
        x = t * length
        r = radius if axis_profile is None else axis_profile(t)
        y_off = 0.0
        rings.append(len(v))
        for k in range(n):
            ang = 2 * math.pi * k / n
            v.append([x, r * math.cos(ang) + y_off, r * math.sin(ang) + r])
    f, which = [], []
    for s in range(segments):
        a0, b0 = rings[s], rings[s + 1]
        for k in range(n):
            k1 = (k + 1) % n
            f.append([a0 + k, b0 + k, b0 + k1]); f.append([a0 + k, b0 + k1, a0 + k1])
            which += [s, s]
    # end caps
    cap0 = len(v); v.append([0.0, 0.0, radius]); cap1 = len(v); v.append([length, 0.0, radius])
    for k in range(n):
        k1 = (k + 1) % n
        f.append([cap0, rings[0] + k1, rings[0] + k]); which.append(0)
        f.append([cap1, rings[-1] + k, rings[-1] + k1]); which.append(segments - 1)
    return np.asarray(v, float), np.asarray(f, int), np.asarray(which, int)


def _cigarette_end(cls, rng, size):
    """A butt: tan filter for the first third, white paper, a dark ash tip; bent."""
    radius = 0.0040
    v, f, which = _cylinder(radius, size, n=12, segments=6)
    t = v[:, 0] / size
    v[:, 1] += 0.004 * np.sin(t * math.pi) * rng.uniform(-1, 1)           # a slight bend
    seg_t = (which + 0.5) / 6.0
    c = np.where(seg_t[:, None] < 0.34, _vary(cls.pick_color(rng), rng, len(f), 0.05),
                 _vary((0.92, 0.90, 0.86), rng, len(f), 0.04))
    c[seg_t > 0.9] = np.array([0.25, 0.22, 0.20]) * (1 + 0.1 * rng.normal(0, 1, (int((seg_t > 0.9).sum()), 1)))
    return v, f, np.clip(c, 0, 1)


def _hair(cls, rng, size):
    """A clump: a few thin strands along smooth random curves."""
    vs, fs, cs = [], [], []
    colour = cls.pick_color(rng)
    strands = int(rng.integers(3, 7))
    for _ in range(strands):
        n = 24
        t = np.linspace(0.0, 1.0, n)
        ax, ay = rng.uniform(0.3, 1.0, 2) * size
        px = ax * t + 0.15 * size * np.sin(rng.uniform(2, 5) * t * math.pi + rng.uniform(0, 6))
        py = ay * (t - 0.5) * rng.uniform(-1, 1) + 0.15 * size * np.cos(rng.uniform(2, 5) * t * math.pi + rng.uniform(0, 6))
        pz = 0.0015 + 0.004 * np.abs(np.sin(rng.uniform(1, 3) * t * math.pi + rng.uniform(0, 6)))
        path = np.stack([px, py, pz], axis=1)
        r = 0.0007
        base = sum(len(x) for x in vs)
        ring = []
        for i in range(n):
            d = path[min(i + 1, n - 1)] - path[max(i - 1, 0)]
            d /= max(np.linalg.norm(d), 1e-9)
            side = np.cross(d, [0, 0, 1.0]); side /= max(np.linalg.norm(side), 1e-9)
            up = np.cross(side, d)
            for k in range(4):
                ang = 2 * math.pi * k / 4
                ring.append(path[i] + r * (math.cos(ang) * side + math.sin(ang) * up))
        ring = np.asarray(ring)
        f = []
        for i in range(n - 1):
            for k in range(4):
                k1 = (k + 1) % 4
                a, b = base + i * 4 + k, base + (i + 1) * 4 + k
                f.append([a, b, base + (i + 1) * 4 + k1]); f.append([a, base + (i + 1) * 4 + k1, base + i * 4 + k1])
        vs.append(ring); fs.append(np.asarray(f)); cs.append(_vary(colour, rng, len(f), 0.1))
    return np.vstack(vs), np.vstack(fs), np.vstack(cs)


def _rounded_slab(size_x, size_y, height, n=16, power=2.5):
    """A superellipse slab: a sanitary pad, a soap bar. Top domed, edges rounded."""
    xs = np.linspace(-1, 1, n); ys = np.linspace(-1, 1, n)
    gx, gy = np.meshgrid(xs, ys, indexing="xy")
    inside = (np.abs(gx) ** power + np.abs(gy) ** power) <= 1.0
    dome = np.clip(1.0 - (np.abs(gx) ** power + np.abs(gy) ** power), 0, 1) ** 0.5
    top = np.stack([gx.ravel() * size_x / 2, gy.ravel() * size_y / 2, (0.3 + 0.7 * dome.ravel()) * height], axis=1)
    bottom = top.copy(); bottom[:, 2] = 0.0
    v = np.vstack([top, bottom]); off = len(top)
    f = []
    ins = inside.ravel()
    for j in range(n - 1):
        for i in range(n - 1):
            a = j * n + i; b, c, d = a + 1, a + n, a + n + 1
            if ins[a] and ins[d]:
                f.append([a, b, d]); f.append([a, d, c])
                f.append([off + a, off + d, off + b]); f.append([off + a, off + c, off + d])
            # side walls where inside meets outside
            for p_, q_ in ((a, b), (a, c)):
                if ins[p_] != ins[q_]:
                    f.append([p_, q_, off + q_]); f.append([p_, off + q_, off + p_])
    return v, np.asarray(f, int)


def _sanitary_waste(cls, rng, size):
    v, f = _rounded_slab(size, size * rng.uniform(0.35, 0.5), rng.uniform(0.008, 0.016), n=18)
    c = _vary(cls.pick_color(rng), rng, len(f), 0.04)
    # a few soiled patches on the top faces
    top = v[f].mean(axis=1)[:, 2] > 0.002
    patch = rng.random(len(f)) < 0.15
    c[top & patch] = _vary((0.55, 0.22, 0.18), rng, int((top & patch).sum()), 0.1)
    return v, f, c


def _sharps(cls, rng, size):
    """A needle with a hub, or a razor blade. Both shiny; neither a bare-pad job."""
    if rng.random() < 0.6:
        length = size
        v1, f1, _ = _cylinder(0.0006, length * 0.65, n=8, segments=2)                 # the cannula
        v2, f2, _ = _cylinder(0.0045, length * 0.35, n=10, segments=2)                 # the hub
        v2[:, 0] += length * 0.65
        v2[:, 2] -= 0.0045 - 0.0006
        v = np.vstack([v1, v2]); f = np.vstack([f1, f2 + len(v1)])
        c = np.vstack([_vary((0.75, 0.76, 0.78), rng, len(f1), 0.03),
                       _vary((0.55, 0.75, 0.85), rng, len(f2), 0.05)])
        return v, f, c
    gx, gy, f = _sheet(size * 0.45, size * 0.22, 7, 4)
    v = np.stack([gx.ravel(), gy.ravel(), np.full(gx.size, 0.0004)], axis=1)
    c = _vary((0.72, 0.73, 0.75), rng, len(f), 0.03)
    return v, f, c


def _glove(cls, rng, size):
    """A discarded glove: a flattened palm with five limp finger tubes."""
    palm_len = size * 0.45
    v, f = _rounded_slab(palm_len, size * 0.32, 0.009, n=14, power=3.0)
    vs, fs = [v], [f]
    colour = cls.pick_color(rng)
    cs = [_vary(colour, rng, len(f), 0.04)]
    for k in range(5):
        y = (k - 2) * size * 0.07
        length = size * (0.3 if k not in (0, 4) else 0.22)
        vk, fk, _ = _cylinder(0.0075, length, n=8, segments=4,
                              axis_profile=lambda t: 0.0075 * (1 - 0.3 * t))
        vk[:, 0] += palm_len * 0.45; vk[:, 1] += y
        vk[:, 2] *= 0.6                                                        # deflated
        vk[:, 1] += np.sin(vk[:, 0] / size * 3) * size * 0.03 * (k - 2)       # splayed
        base = sum(len(x) for x in vs)
        vs.append(vk); fs.append(fk + base); cs.append(_vary(colour, rng, len(fk), 0.04))
    return np.vstack(vs), np.vstack(fs), np.vstack(cs)


def _face_mask(cls, rng, size):
    """A surgical mask: pleated rectangle, slightly bowed, two ear loops."""
    gx, gy, f = _sheet(size, size * 0.55, 25, 13)
    pleats = 0.004 * np.abs(np.sin(gy / (size * 0.55) * math.pi * 3.0))
    bow = 0.015 * (1.0 - (gx / (size / 2)) ** 2)
    z = pleats + bow * rng.uniform(0.3, 1.0)
    v = np.stack([gx.ravel(), gy.ravel(), z.ravel()], axis=1)
    colour = cls.pick_color(rng)
    c = _vary(colour, rng, len(f), 0.03)
    vs, fs, cs = [v], [f], [c]
    for side in (-1, 1):
        n = 12
        t = np.linspace(0, math.pi, n)
        loop = np.stack([side * (size / 2 + 0.02 * np.sin(t)), (size * 0.55 / 2) * np.cos(t),
                         0.002 + 0.0 * t], axis=1)
        base = sum(len(x) for x in vs)
        ring, fl = [], []
        for i in range(n):
            d = loop[min(i + 1, n - 1)] - loop[max(i - 1, 0)]; d /= max(np.linalg.norm(d), 1e-9)
            sd = np.cross(d, [0, 0, 1.0]); sd /= max(np.linalg.norm(sd), 1e-9); up = np.cross(sd, d)
            for k in range(4):
                ang = 2 * math.pi * k / 4
                ring.append(loop[i] + 0.0012 * (math.cos(ang) * sd + math.sin(ang) * up))
        for i in range(n - 1):
            for k in range(4):
                k1 = (k + 1) % 4
                a, b = base + i * 4 + k, base + (i + 1) * 4 + k
                fl.append([a, b, base + (i + 1) * 4 + k1]); fl.append([a, base + (i + 1) * 4 + k1, base + i * 4 + k1])
        vs.append(np.asarray(ring)); fs.append(np.asarray(fl)); cs.append(_vary((0.92, 0.92, 0.92), rng, len(fl), 0.02))
    return np.vstack(vs), np.vstack(fs), np.vstack(cs)


def _bottle(cls, rng, size):
    """A drinks bottle on its side: body, shoulder, neck, cap."""
    radius = size * 0.115
    body_len = size * 0.72
    v1, f1, _ = _cylinder(radius, body_len, n=16, segments=3)
    v2, f2, _ = _cylinder(radius, size * 0.12, n=16, segments=3,
                          axis_profile=lambda t: radius * (1 - 0.6 * t))
    v2[:, 0] += body_len; v2[:, 2] += 0.0
    v3, f3, _ = _cylinder(radius * 0.45, size * 0.16, n=12, segments=1)
    v3[:, 0] += body_len + size * 0.12; v3[:, 2] += radius - radius * 0.45
    colour = cls.pick_color(rng)
    v = np.vstack([v1, v2, v3]); f = np.vstack([f1, f2 + len(v1), f3 + len(v1) + len(v2)])
    c = np.vstack([_vary(colour, rng, len(f1), 0.03), _vary(colour, rng, len(f2), 0.03),
                   _vary((0.2, 0.3, 0.65), rng, len(f3), 0.03)])
    # a label band
    band = (v[f].mean(axis=1)[:, 0] > body_len * 0.3) & (v[f].mean(axis=1)[:, 0] < body_len * 0.6)
    c[band] = _vary((0.95, 0.95, 0.92), rng, int(band.sum()), 0.02)
    return v, f, c


_MAKERS = {
    "tissue": _tissue, "paper_towel": _paper_towel, "wrapper": _wrapper,
    "cigarette_end": _cigarette_end, "hair": _hair, "sanitary_waste": _sanitary_waste,
    "sharps": _sharps, "glove": _glove, "face_mask": _face_mask, "bottle": _bottle,
}
