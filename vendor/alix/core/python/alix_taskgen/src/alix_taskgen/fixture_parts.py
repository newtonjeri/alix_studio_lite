"""Fixture part registry — the single source of truth for a fixture's parts.

Every layer that used to hard-code the commode triad (``body``/``lid``/``seat``)
reads part names, articulation joints, glove-frame contact suffixes and
state-implication rules from this registry instead, so any fixture (urinal,
washtub, …) with any part set works without code changes.

Schemas understood (in precedence order):

1. Declarative ``fixture.parts`` (new, preferred for new fixtures)::

       fixture:
         parts:
           basin:
             mesh: "package://.../basin.stl"
             primary: true              # default component for regions
           cover:
             mesh: "package://.../cover.stl"
             parent: basin              # joint origin is in this part's frame
             glove_frame_suffix: "_2"   # -> *_glove_frame_2 (Z contact)
             joint:
               origin: [x, y, z]
               origin_rpy_deg: [r, p, y]
               axis: [0, 1, 0]
               angles_deg: {closed: 0.0, open: -95.0}
             implies:
               open: {other_part: open}   # opening this forces other_part open

2. Legacy component blocks (``fixture.body`` / ``fixture.lid`` / … each with a
   ``mesh`` key) — the schema the C++ ArticulatedSceneBuilder reads. Any block
   with a ``mesh`` is a part; a block with a ``joint`` is articulated. When
   both ``lid`` and ``seat`` exist the commode reality rule (an open seat rests
   against an open lid) is synthesized as ``seat.implies.open = {lid: open}``.

3. Legacy mesh map (``fixture.meshes.<name>``) — static single/multi-mesh
   fixtures (urinal_a, commode_a).

The registry is deliberately read-only over scene.yaml: existing files keep
working untouched, and the C++ scene builder keeps reading the legacy blocks.
"""

import functools
from dataclasses import dataclass, field
from pathlib import Path

import yaml


# Keys under ``fixture:`` that are metadata, never part blocks.
_RESERVED_FIXTURE_KEYS = {
    "display_name", "type", "articulated", "pose", "scale", "meshes", "parts",
}

# Articulation states every revolute/prismatic joint implicitly supports.
DEFAULT_CLOSED_STATE = "closed"

# Legacy glove-frame rule: articulated parts (lid/seat) are cleaned with the
# Z-contact glove face (*_glove_frame_2); static parts with the X-contact
# face (*_glove_frame_1). New-schema parts may override per part.
_LEGACY_ARTICULATED_GLOVE_SUFFIX = "_2"


@dataclass(frozen=True)
class FixturePart:
    name: str
    mesh_uri: str | None = None
    static: bool = True
    primary: bool = False
    parent: str | None = None          # parent part name; None = fixture root
    glove_frame_suffix: str = ""       # "" -> *_glove_frame_1, "_2" -> *_glove_frame_2
    joint: dict | None = None          # {origin, origin_rpy_deg, axis, angles_deg{state: deg}}
    implies: dict = field(default_factory=dict)   # {state: {other_part: state}}
    scale: tuple = (1.0, 1.0, 1.0)

    @property
    def states(self):
        """Articulation state names this part supports (closed always valid)."""
        if self.static:
            return ("static",)
        angles = (self.joint or {}).get("angles_deg", {}) or {}
        names = list(angles)
        if DEFAULT_CLOSED_STATE not in names:
            names.insert(0, DEFAULT_CLOSED_STATE)
        return tuple(names)

    def joint_angle_deg(self, state):
        """Angle for an articulation state. ``closed``/``static`` default to 0.0
        when the joint declares no explicit angle for them."""
        if self.static or state in (None, "static"):
            return 0.0
        angles = (self.joint or {}).get("angles_deg", {}) or {}
        if state in angles:
            return float(angles[state])
        if state == DEFAULT_CLOSED_STATE:
            return 0.0
        valid = ", ".join(sorted(angles)) or DEFAULT_CLOSED_STATE
        raise ValueError(
            f"Unknown articulation state '{state}' for part '{self.name}'. Valid states: {valid}")


@dataclass(frozen=True)
class PartRegistry:
    parts: tuple = ()
    fixture_type: str | None = None
    source: str = "empty"     # parts | legacy_components | legacy_meshes | empty

    def __bool__(self):
        return bool(self.parts)

    @property
    def names(self):
        return tuple(p.name for p in self.parts)

    def part(self, name):
        for p in self.parts:
            if p.name == name:
                return p
        return None

    @property
    def primary_part(self):
        for p in self.parts:
            if p.primary:
                return p
        return self.parts[0] if self.parts else None

    @property
    def static_parts(self):
        return tuple(p for p in self.parts if p.static)

    @property
    def is_combined(self):
        """True for a fixture shipped as WHOLE-FIXTURE meshes, one per state.

        Two shapes exist, and they are not interchangeable:

        * **articulated** (commode_b/c) — separate `body` / `lid` / `seat`
          meshes plus joints; a region names the component it lives on and each
          component is posed by its own joint angle.
        * **combined** (commode_a) — the parts are fused, so the fixture ships
          one pre-baked mesh per articulation STATE (`closed`, `lid_open`,
          `lid_seat_open`). There is no `lid` to pose; there is a whole commode
          whose lid happens to be open.

        The regions CSV is per FAMILY, so it names `lid`/`seat`/`body` for every
        commode including the combined one. Asking a combined fixture for a
        `lid` part is therefore normal, and must resolve to the state mesh
        rather than raise.

        A single-mesh fixture (`urinal_a`, one part named `default`) is NOT
        combined — it has no states to choose between.
        """
        return bool(self.parts) and not self.articulated_parts and len(self.parts) > 1

    def state_part(self, state_name):
        """The whole-fixture mesh for a state name, or None."""
        return self.part(state_name) if state_name else None

    @property
    def articulated_parts(self):
        return tuple(p for p in self.parts if not p.static)

    @property
    def articulated_names(self):
        return tuple(p.name for p in self.articulated_parts)

    @property
    def has_articulation(self):
        return bool(self.articulated_parts)

    def default_states(self):
        """``{part: 'closed'}`` for every articulated part."""
        return {p.name: DEFAULT_CLOSED_STATE for p in self.articulated_parts}

    def apply_implies(self, states):
        """Return *states* with every part's ``implies`` rules applied.

        Iterates to a fixed point (bounded by part count) so chained rules
        settle; e.g. the commode rule ``seat: open  =>  lid: open``.
        """
        states = dict(states)
        for _ in range(max(len(self.parts), 1)):
            changed = False
            for p in self.articulated_parts:
                implied = (p.implies or {}).get(states.get(p.name)) or {}
                for other, other_state in implied.items():
                    if states.get(other) != other_state:
                        states[other] = other_state
                        changed = True
            if not changed:
                break
        return states


def load_part_registry(scene_path):
    """Part registry for a fixture scene.yaml (cached on path + mtime)."""
    scene_path = Path(scene_path)
    try:
        mtime = scene_path.stat().st_mtime_ns
    except OSError:
        return PartRegistry()
    return _load_part_registry_cached(str(scene_path), mtime)


#: Which openable parts each commode classification HAS (Newton, 2026-08-18
#: — a new axis, orthogonal to `articulated`). Consumers: the reality-logic
#: validator (a region requiring a seat state is impossible on has_lid_only),
#: mechanism-task invocation (which lid/seat open/close tasks exist for the
#: fixture), and the future perception package's inspection stages. Every
#: commode in the current fleet (a-c) is has_lid_and_seat.
COMMODE_CLASSIFICATION_PARTS = {
    "has_lid_only": frozenset({"lid"}),
    "has_seat_only": frozenset({"seat"}),
    "has_lid_and_seat": frozenset({"lid", "seat"}),
}


def commode_classification(scene_path):
    """The fixture's declared ``commode_classification``, or None.

    Read raw from scene.yaml (not through the registry cache — the field is
    fixture metadata, not part geometry). Unknown values raise: a typo here
    would silently disable the classification checks it exists to feed.
    """
    try:
        with open(Path(scene_path), "r", encoding="utf-8") as stream:
            scene = yaml.safe_load(stream) or {}
    except (OSError, yaml.YAMLError):
        return None
    value = ((scene.get("fixture") or {}).get("commode_classification"))
    if value is None:
        return None
    value = str(value).strip().lower()
    if value not in COMMODE_CLASSIFICATION_PARTS:
        valid = ", ".join(sorted(COMMODE_CLASSIFICATION_PARTS))
        raise ValueError(
            f"{scene_path}: commode_classification '{value}' is not one of: "
            f"{valid}")
    return value


def registry_from_fixture_dict(fixture):
    """Build a registry from a parsed ``fixture:`` mapping (schema detection)."""
    fixture = fixture or {}
    if isinstance(fixture.get("parts"), dict) and fixture["parts"]:
        return _from_parts_schema(fixture)
    components = {
        name: block for name, block in fixture.items()
        if name not in _RESERVED_FIXTURE_KEYS and isinstance(block, dict)
        and ("mesh" in block or "joint" in block)
    }
    if components:
        return _from_legacy_components(fixture, components)
    meshes = fixture.get("meshes") or {}
    if isinstance(meshes, dict) and meshes:
        return _from_legacy_meshes(fixture, meshes)
    return PartRegistry(fixture_type=fixture.get("type"))


# ---------------------------------------------------------------------------


@functools.lru_cache(maxsize=32)
def _load_part_registry_cached(scene_path_str, _mtime_ns):
    try:
        with open(scene_path_str, "r", encoding="utf-8") as stream:
            scene = yaml.safe_load(stream) or {}
    except (OSError, yaml.YAMLError):
        return PartRegistry()
    return registry_from_fixture_dict(scene.get("fixture", {}) or {})


def _norm_scale(value):
    if value is None:
        return (1.0, 1.0, 1.0)
    if isinstance(value, (int, float)):
        return (float(value),) * 3
    return tuple(float(v) for v in value)


def _norm_joint(raw):
    """Normalize a joint block; accepts legacy ``origin``/``angles_deg`` keys."""
    if not isinstance(raw, dict) or not raw:
        return None
    angles = raw.get("angles_deg") or raw.get("states") or {}
    return {
        "type": str(raw.get("type", "revolute")),
        "origin": [float(v) for v in (raw.get("origin") or [0.0, 0.0, 0.0])],
        "origin_rpy_deg": [float(v) for v in (raw.get("origin_rpy_deg") or [0.0, 0.0, 0.0])],
        "axis": [float(v) for v in (raw.get("axis") or [0.0, 1.0, 0.0])],
        "angles_deg": {str(k): float(v) for k, v in angles.items()},
    }


def _norm_implies(raw):
    if not isinstance(raw, dict):
        return {}
    return {str(state): {str(p): str(s) for p, s in (targets or {}).items()}
            for state, targets in raw.items()}


def _pick_primary(entries, explicit=None):
    """Primary-part name: explicit flag > 'body' > 'default' > first static > first."""
    if explicit:
        return explicit
    names = [name for name, _ in entries]
    for candidate in ("body", "default"):
        if candidate in names:
            return candidate
    for name, static in entries:
        if static:
            return name
    return names[0] if names else None


def _from_parts_schema(fixture):
    raw_parts = fixture["parts"]
    explicit_primary = next((n for n, b in raw_parts.items() if (b or {}).get("primary")), None)
    primary = _pick_primary(
        [(n, "joint" not in (b or {})) for n, b in raw_parts.items()], explicit_primary)
    parts = []
    for name, block in raw_parts.items():
        block = block or {}
        joint = _norm_joint(block.get("joint"))
        static = bool(block.get("static", joint is None))
        suffix = block.get("glove_frame_suffix")
        if suffix is None:
            suffix = "" if static else _LEGACY_ARTICULATED_GLOVE_SUFFIX
        parts.append(FixturePart(
            name=str(name),
            mesh_uri=block.get("mesh"),
            static=static,
            primary=(name == primary),
            parent=block.get("parent"),
            glove_frame_suffix=str(suffix),
            joint=None if static else joint,
            implies=_norm_implies(block.get("implies")),
            scale=_norm_scale(block.get("scale", fixture.get("scale"))),
        ))
    return PartRegistry(parts=tuple(parts), fixture_type=fixture.get("type"), source="parts")


def _from_legacy_components(fixture, components):
    primary = _pick_primary([(n, "joint" not in b) for n, b in components.items()])
    names = set(components)
    parts = []
    for name, block in components.items():
        joint = _norm_joint(block.get("joint"))
        static = joint is None
        # Commode reality rule, now data on the registry instead of code in
        # three separate modules: the seat swings up against the lid, so an
        # open seat is only physically possible with the lid open.
        implies = {}
        if name == "seat" and "lid" in names and not static:
            implies = {"open": {"lid": "open"}}
        parts.append(FixturePart(
            name=str(name),
            mesh_uri=block.get("mesh"),
            static=static,
            primary=(name == primary),
            parent=None if name == primary else primary,
            glove_frame_suffix="" if static else _LEGACY_ARTICULATED_GLOVE_SUFFIX,
            joint=joint,
            implies=implies,
            scale=_norm_scale(block.get("scale", fixture.get("scale"))),
        ))
    return PartRegistry(parts=tuple(parts), fixture_type=fixture.get("type"),
                        source="legacy_components")


def _from_legacy_meshes(fixture, meshes):
    primary = _pick_primary([(n, True) for n in meshes])
    parts = tuple(
        FixturePart(
            name=str(name),
            mesh_uri=uri,
            static=True,
            primary=(name == primary),
            scale=_norm_scale(fixture.get("scale")),
        )
        for name, uri in meshes.items()
    )
    return PartRegistry(parts=parts, fixture_type=fixture.get("type"), source="legacy_meshes")
