"""Route a 3D detection to a fixture region by the fixture's own region meshes.

Until 2026-10-06 a detection's region came from `region_bands.yaml`: bands on the
status model's fixture box, authored against butterfli's camera pose, which
assume the fixture is seen face-on with its back at the top of the box. From the
wrist camera at alix's inspection poses neither holds — `inspection_pose_4`
looks straight down, `inspection_pose_1` rolls the image -87 degrees, and the
real frames of 2026-03 have the cistern on the right — so the bands cannot be
right at every pose. The meshes can.

    locator = RegionLocator.for_fixture("commode_c")
    region, distance_m = locator.locate(point_in_body_frame, "closed")

The point is a lifted detection centre (`geometry.roi_from_detection`, in the
output frame) carried into the fixture BODY frame by the fixture pose the node
supplies. The region meshes are the `*_regions.obj` groups that `regions.yaml`
declares — the SDG soil field's own tables (`sdg.soil.load_region_tables`) —
with the hinged parts placed by `sdg.export.part_transforms` for the state, and
only the regions that exist in that state compete (`sdg.soil.regions_in_state`):
the lid's underside cannot claim a point while the lid is down over it.

What this needs that the bands did not: the fixture pose in the output frame. In
the scene it is `scene.yaml`'s `fixture.pose` (the body frame in base_link with
the UGV docked at the default standoff); `load_fixture_pose` reads it and the
node composes it with TF. A point farther than `max_distance_m` from every
visible region cell routes NOWHERE — unrouted, counted — never to the nearest
region regardless: that is the cloth beside the fixture, or a pose that is wrong,
and either way the arm must not be sent.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import yaml

from . import paths
from .sdg.export import part_transforms
from .sdg.soil import build_fields, regions_in_state

#: Cell size the region meshes are subdivided to for routing. The soil field
#: uses 5 mm; routing to the nearest cell does not need that, and 1 cm is a
#: quarter of the memory and the query cost.
DEFAULT_CELL_M = 0.01

#: Farther than this from every visible region cell = unrouted. A lifted
#: detection carries the depth error (a few mm measured, the full fallback
#: error otherwise) and the fixture pose error; 3 cm covers the measured case
#: and refuses the fallback-and-wrong one.
DEFAULT_MAX_DISTANCE_M = 0.03


class RegionError(RuntimeError):
    """The fixture's region meshes or scene could not be loaded."""


def rpy_deg_to_matrix(rpy_deg) -> np.ndarray:
    """Roll, pitch, yaw in degrees (x, then y, then z, extrinsic) -> 3x3."""
    r, p, y = (math.radians(float(v)) for v in rpy_deg)
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


#: The venue every fixture scene is composed with unless told otherwise — the
#: same default as `fixture_pose.DEFAULT_VENUE` in the Isaac scripts.
DEFAULT_VENUE = "washroom_800mm_wide_scene.yaml"


def default_venue_yaml() -> Path | None:
    root = paths.task_config_root()
    if root is None:
        return None
    candidate = Path(root) / "scene" / DEFAULT_VENUE
    return candidate if candidate.is_file() else None


def load_venue_rotation(venue_yaml: str | Path) -> np.ndarray:
    """3x3 rotation of the venue (`washroom_scene.washroom_structure.pose.orientation_rpy_deg`)."""
    with open(venue_yaml) as handle:
        loaded = yaml.safe_load(handle) or {}
    structure = (loaded.get("washroom_scene") or {}).get("washroom_structure") or {}
    rpy = (structure.get("pose") or {}).get("orientation_rpy_deg") or [0.0, 0.0, 0.0]
    return rpy_deg_to_matrix(rpy)


def load_fixture_pose(scene_yaml: str | Path, standoff_m: float | None = None,
                      venue_yaml: str | Path | None = "") -> np.ndarray:
    """4x4 pose of the fixture BODY frame in `scene.yaml`'s `relative_to` frame (base_link).

    THE VENUE SUPPLIES ITS YAW TO THE FIXTURE (`fixture_pose.compose`, the
    scene builder, the Isaac rig all do this): the washroom's
    `orientation_rpy_deg` is [0, 0, 180], the scene's own rotation is zero, so
    the body frame's +x points AT the robot and the lid top — (0.306, 0, 0.028)
    in the body frame — sits at x ≈ 0.70 in base_link, not 1.31. MEASURED on
    the first real capture (2026-10-07): the lid plane from the depth and the
    recorded TF is at (0.697, 0.007, 0.462); without the yaw the projected box
    missed the fixture entirely, and against the renderer's own boxes the
    projection error went from a median 588 px to 25 px with it.

    `venue_yaml`: "" (default) = the default washroom venue beside the task
    configs; None = no venue (the scene's own rotation only); a path = that
    venue. `fixture.pose.position` is written for the UGV at
    `default_ugv_standoff_m`; a docking at another standoff moves the fixture
    along +X by the difference. None keeps the scene's own position.
    """
    with open(scene_yaml) as handle:
        fixture = (yaml.safe_load(handle) or {}).get("fixture") or {}
    pose = fixture.get("pose") or {}
    position = np.asarray(pose.get("position") or [0.0, 0.0, 0.0], dtype=float)
    if standoff_m is not None:
        default = float(pose.get("default_ugv_standoff_m", standoff_m))
        position = position + np.array([float(standoff_m) - default, 0.0, 0.0])
    rotation = rpy_deg_to_matrix(pose.get("orientation_rpy_deg") or [0.0, 0.0, 0.0])
    if venue_yaml == "":
        venue_yaml = default_venue_yaml()
        if venue_yaml is None:
            raise RegionError(f"no default venue ({DEFAULT_VENUE}) beside the task configs; "
                              f"pass venue_yaml=None to compose without one")
    if venue_yaml is not None:
        rotation = load_venue_rotation(venue_yaml) @ rotation
    out = np.eye(4)
    out[:3, :3] = rotation
    out[:3, 3] = position
    return out


def default_calibration_file(model: str = "commode_c") -> Path | None:
    """`config/fixtures/<model>_calibration.yaml` beside this module, if it exists."""
    path = Path(__file__).resolve().parent / "config" / "fixtures" / f"{model}_calibration.yaml"
    return path if path.is_file() else None


class RegionLocator:
    """Nearest visible region cell to a point in the fixture body frame."""

    def __init__(self, models_dir: str | Path, scene_yaml: str | Path,
                 cell_m: float = DEFAULT_CELL_M, max_distance_m: float = DEFAULT_MAX_DISTANCE_M,
                 calibration: "str | Path | dict | None" = None):
        self.models_dir = Path(models_dir)
        self.scene_yaml = Path(scene_yaml)
        self.max_distance_m = float(max_distance_m)
        try:
            self.fields, self.region_states = build_fields(self.models_dir, cell_m=cell_m)
        except Exception as exc:                       # noqa: BLE001 — one error type for the caller
            raise RegionError(f"region meshes under {self.models_dir}: {exc}") from exc
        if not self.scene_yaml.is_file():
            raise RegionError(f"no scene.yaml at {self.scene_yaml}")
        # The REAL fixture against the model, MEASURED (tools/perception/calibrate_fixture.py):
        # {component: {region: metres}} along the part's local +z, its top normal. The real
        # commode_c has a board lid 15-20 mm above the modelled lid and a seat ring and rim
        # about half a ring thickness off (2026-10-07/08); from above that offset is the whole
        # signal between lid_open and lid_seat_open, so the cells carry it.
        self.calibration: dict = self._load_calibration(calibration)
        if self.calibration:
            for component, regions in self.calibration.items():
                field = self.fields.get(component)
                if field is None:
                    continue
                shift = np.zeros(len(field.centers))
                for region, metres in regions.items():
                    shift[field.cell_region == region] = float(metres)
                field.centers = field.centers + np.stack([np.zeros_like(shift), np.zeros_like(shift), shift], axis=1)
        self._cells: dict[str, tuple[np.ndarray, np.ndarray]] = {}
        self._components: dict[str, np.ndarray] = {}
        self._trees: dict = {}
        self._distinguishing: dict = {}
        self._surface: dict[str, np.ndarray] = {}

    @classmethod
    def for_fixture(cls, model: str = "commode_c", fixture_type: str = "commode", **kw) -> "RegionLocator":
        """The fixture's locator. Its MEASURED calibration (`config/fixtures/<model>_calibration.yaml`
        in this package) is applied when the file exists; `calibration=None` is the raw model."""
        base = paths.fixture_model_dir(fixture_type, model)
        if base is None:
            raise RegionError(f"no fixture directory for {fixture_type}/{model}")
        base = Path(base)
        models = base / "models" if (base / "models").is_dir() else base
        scene = base / "scene.yaml" if (base / "scene.yaml").is_file() else base.parent / "scene.yaml"
        if "calibration" not in kw:
            kw["calibration"] = default_calibration_file(model)
        return cls(models, scene, **kw)

    @staticmethod
    def _load_calibration(source) -> dict:
        if not source:
            return {}
        if isinstance(source, dict):
            doc = source
        else:
            path = Path(source)
            if not path.is_file():
                return {}
            with open(path) as handle:
                doc = yaml.safe_load(handle) or {}
        offsets = doc.get("offsets_m", doc) or {}
        return {str(c): {str(r): float(v) for r, v in (regions or {}).items()} for c, regions in offsets.items()}

    @property
    def regions(self) -> list[str]:
        return sorted({str(r) for f in self.fields.values() for r in set(f.cell_region) if r})

    def cells(self, state: str) -> tuple[np.ndarray, np.ndarray]:
        """-> (cell centres in the BODY frame (N, 3), region per cell (N,)) for `state`."""
        if state not in self._cells:
            transforms = part_transforms(self.scene_yaml, state)
            points, regions, components = [], [], []
            for component, field in self.fields.items():
                visible = set(regions_in_state(field, self.region_states, state))
                mask = np.isin(field.cell_region, list(visible)) & (field.cell_region != "")
                if not mask.any():
                    continue
                rotation, translation = transforms.get(component, (np.eye(3), np.zeros(3)))
                points.append(field.centers[mask] @ rotation.T + translation)
                regions.append(field.cell_region[mask])
                components.append(np.full(int(mask.sum()), component, dtype=object))
            if points:
                self._cells[state] = (np.vstack(points), np.concatenate(regions))
                self._components[state] = np.concatenate(components)
            else:
                self._cells[state] = (np.zeros((0, 3)), np.zeros((0,), dtype=object))
                self._components[state] = np.zeros((0,), dtype=object)
        return self._cells[state]

    def surface(self, state: str) -> np.ndarray:
        """EVERY cell of every part, placed for `state` — no region filter. The
        routing's `cells()` drops regions nobody can clean in a state (the bowl
        under a closed seat); a depth hypothesis needs the surfaces that are
        physically there, and lets occlusion sort out what the camera sees."""
        return self.surface_parts(state)[0]

    def surface_parts(self, state: str) -> tuple[np.ndarray, np.ndarray]:
        """(every cell placed for `state`, a flag per cell: on a RAISED part).

        A raised lid or seat stands at whatever angle the real one has, never
        at the nominal one, so a depth pixel on it cannot tell states apart."""
        from .sdg.soil import RAISED
        if state not in self._surface:
            transforms = part_transforms(self.scene_yaml, state)
            raised = set(RAISED.get(state, ()))
            pts, flags = [], []
            for component, field in self.fields.items():
                rotation, translation = transforms.get(component, (np.eye(3), np.zeros(3)))
                pts.append(field.centers @ rotation.T + translation)
                flags.append(np.full(len(field.centers), component in raised, dtype=bool))
            self._surface[state] = (np.vstack(pts), np.concatenate(flags)) if pts else (np.zeros((0, 3)), np.zeros(0, bool))
        return self._surface[state]

    def static_cells(self, state: str) -> np.ndarray:
        """The cells of `state` on parts that do NOT move with the hinge angle:
        the body and the parts lying closed. A raised lid or seat is at whatever
        angle the real one happens to be, never at the nominal one, so its
        surfaces cannot be trusted to tell states apart."""
        from .sdg.soil import RAISED
        points, _ = self.cells(state)
        components = self._components[state]
        raised = set(RAISED.get(state, ()))
        keep = np.array([c not in raised for c in components], dtype=bool)
        return points[keep]

    def distinguishing(self, a: str, b: str, apart_m: float = 0.008) -> tuple[np.ndarray, np.ndarray]:
        """(surfaces of `a` that `b` lacks, surfaces of `b` that `a` lacks), static
        parts only — the seat ring lying on the bowl against the bare rim it
        covers. Cached; this is geometry, the same for every frame."""
        key = (a, b)
        if key not in self._distinguishing:
            a_only = self.static_cells(a)
            a_only = a_only[self.nearest(a_only, b) > apart_m]
            b_only = self.static_cells(b)
            b_only = b_only[self.nearest(b_only, a) > apart_m]
            self._distinguishing[key] = (a_only, b_only)
            self._distinguishing[(b, a)] = (b_only, a_only)
        return self._distinguishing[key]

    def nearest(self, points_body: np.ndarray, state: str) -> np.ndarray:
        """Distance from each point (body frame, (N, 3)) to the nearest visible
        cell of `state`. A KD-tree when scipy is present (cached per state),
        else brute force against a 20k subsample of the cells."""
        cells, _ = self.cells(state)
        if len(cells) == 0 or len(points_body) == 0:
            return np.full(len(points_body), math.inf)
        try:
            from scipy.spatial import cKDTree
            tree = self._trees.get(state)
            if tree is None:
                tree = self._trees[state] = cKDTree(cells)
            return tree.query(np.asarray(points_body, dtype=float), workers=-1)[0]
        except ImportError:
            pass
        if len(cells) > 20000:
            cells = cells[np.random.default_rng(0).choice(len(cells), 20000, replace=False)]
        out = np.empty(len(points_body))
        for i in range(0, len(points_body), 1000):
            p = np.asarray(points_body[i:i + 1000], dtype=float)
            out[i:i + 1000] = np.sqrt(((p[:, None, :] - cells[None, :, :]) ** 2).sum(axis=2).min(axis=1))
        return out

    def locate(self, point_body: np.ndarray, state: str) -> tuple[str, float]:
        """-> (region, distance to its nearest cell) — ("", distance) beyond `max_distance_m`."""
        points, regions = self.cells(state)
        if len(points) == 0:
            return "", math.inf
        p = np.asarray(point_body, dtype=float)[:3]
        d2 = ((points - p) ** 2).sum(axis=1)
        i = int(np.argmin(d2))
        distance = float(math.sqrt(d2[i]))
        if distance > self.max_distance_m:
            return "", distance
        return str(regions[i]), distance


def nearest_to(points: np.ndarray, cells: np.ndarray) -> np.ndarray:
    """Distance from each point to the nearest of `cells` (KD-tree if scipy, else brute force)."""
    points = np.asarray(points, dtype=float)
    if len(cells) == 0 or len(points) == 0:
        return np.full(len(points), math.inf)
    try:
        from scipy.spatial import cKDTree
        return cKDTree(cells).query(points, workers=-1)[0]
    except ImportError:
        pass
    if len(cells) > 20000:
        cells = cells[np.random.default_rng(0).choice(len(cells), 20000, replace=False)]
    out = np.empty(len(points))
    for i in range(0, len(points), 1000):
        p = points[i:i + 1000]
        out[i:i + 1000] = np.sqrt(((p[:, None, :] - cells[None, :, :]) ** 2).sum(axis=2).min(axis=1))
    return out


# --------------------------------------------------------------------------- state from depth
STATES = ("closed", "lid_open", "lid_seat_open")

#: The second-best state must be at least this much farther from the points
#: than the best for the depth to DECIDE. The lid top, the seat ring and the
#: bowl are 20-100 mm apart and the D435 is a few mm at this range; MEASURED
#: on the first real capture (2026-10-07, 56 frames) the margins were 16-35 mm.
MIN_MARGIN_MM = 8.0
#: Fewer points than this on the fixture = no decision (the fixture is not in
#: view, or the depth is empty).
MIN_POINTS = 500
#: A depth point farther than this from EVERY state's surfaces is not the
#: fixture (the floor, the hand, the cloth) and does not vote.
NEAR_ANY_M = 0.06
#: A depth point votes between two states only when it lies within this of a
#: surface that one state has and the other lacks (the common surfaces are silent).
VOTE_NEAR_M = 0.02
#: Fewer voting points than this between two states = the surface that tells
#: them apart is not in view: undecided.
MIN_DISCRIMINATING = 60
#: The winner's share of the votes between two states — with MIN_MARGIN_MM of
#: separation at this share, or on its own from CLEAR_VOTE_SHARE up.
MIN_VOTE_SHARE = 0.7
CLEAR_VOTE_SHARE = 0.85


@dataclass
class DepthState:
    """What the depth says the fixture state is, and how surely."""
    state: str                                  # "" = undecided
    margin_mm: float
    residual_mm: dict = field(default_factory=dict)
    points: int = 0
    confident: bool = False
    reason: str = ""
    offset_m: float = 0.0                       # a decided state's registration: how much higher the real surfaces sat than its render (body +z)

    @property
    def confidence(self) -> float:
        """0.5 at the minimum margin, 1.0 from 20 mm up: a number the lift
        check's threshold (0.5) can read like the classifier's."""
        if not self.confident:
            return 0.0
        return min(1.0, 0.5 + 0.5 * max(0.0, self.margin_mm - MIN_MARGIN_MM) / (20.0 - MIN_MARGIN_MM))


_EMPTY = 1.0e6     # "no surface" in a depth render; finite, so OpenCV's filters keep it


def render_hypothesis(points_body: np.ndarray, body_from_cam: np.ndarray, fx: float, fy: float,
                      cx: float, cy: float, width: int, height: int, stride: int,
                      raised: np.ndarray | None = None):
    """The depth image a camera would see of `points_body` (one hypothesis's
    surfaces), at 1/`stride` resolution: the NEAREST cell per pixel, so a
    surface behind another is hidden exactly as it is for the real camera.
    Holes between 1 cm cells are closed with a 5x5 minimum filter. 0 = nothing.
    With `raised` (a flag per cell) also -> a mask of the pixels whose nearest
    cell is on a raised part."""
    import cv2
    cam_from_body = np.linalg.inv(body_from_cam)
    cam = points_body @ cam_from_body[:3, :3].T + cam_from_body[:3, 3]
    front = cam[:, 2] > 0.05
    cam = cam[front]
    flags = raised[front] if raised is not None else None
    u = ((fx * cam[:, 0] / cam[:, 2] + cx) / stride).astype(int)
    v = ((fy * cam[:, 1] / cam[:, 2] + cy) / stride).astype(int)
    w, h = width // stride, height // stride
    ok = (u >= 0) & (u < w) & (v >= 0) & (v < h)
    u, v, z = u[ok], v[ok], cam[ok, 2].astype(np.float32)
    img = np.full((h, w), _EMPTY, np.float32)
    order = np.argsort(-z)                       # far first, near overwrites
    img[v[order], u[order]] = z[order]
    img = cv2.erode(img, np.ones((5, 5), np.uint8))   # minimum filter: fills the gaps between cells
    img[img >= _EMPTY * 0.5] = 0.0
    if flags is None:
        return img
    rz = np.full((h, w), _EMPTY, np.float32)                   # the raised cells' own depth image
    fr = flags[ok]
    rz[v[order][fr[order]], u[order][fr[order]]] = z[order][fr[order]]
    rz = cv2.erode(rz, np.ones((5, 5), np.uint8))
    raised_mask = (img > 0) & (rz < _EMPTY * 0.5) & (rz <= img + 0.002)   # the nearest surface here is raised
    return img, raised_mask


def state_from_depth(depth_m: np.ndarray, fx: float, fy: float, cx: float, cy: float,
                     camera_to_target: np.ndarray, fixture_pose: np.ndarray, locator,
                     states=STATES, stride: int = 6, explained_m: float = 0.03,
                     registration_m: float = 0.0) -> DepthState:
    """The fixture state from geometry: which state's rendered depth does the measured depth match?

    Newton, 2026-10-07: the lift check decides from depth and is consistent, so
    why not the state. This is that, by RENDER AND COMPARE: every hypothesis's
    surfaces (`locator.surface(state)`, the parts placed for the state) are
    projected into the camera as a depth image, which hides what the camera
    could not see exactly as the real camera's does; the measured depth is
    compared with each render per pixel. Only the pixels where two renders
    DIFFER by more than MIN_MARGIN_MM vote — the seat ring lying on the bowl
    against the bare rim, the closed lid against the ring under it — and only
    where the measurement is explained by one of the two (within
    `explained_m`: a hand or a cloth in front of the fixture explains neither
    and does not vote). The winner needs MIN_VOTE_SHARE of at least
    MIN_DISCRIMINATING voters against EVERY other state; the margin is the
    smallest pairwise median separation. No colour, no training, no dots, no
    ink; it needs the fixture pose right to a few centimetres and says
    (undecided) when the surfaces that tell states apart are not in view.

    `registration_m` raises every hypothesis by that much along the body's +z
    before rendering: the robot's placement puts the whole real fixture a
    centimetre or so above or below the nominal pose, differently each session
    (MEASURED 2026-10-08: +10 mm in one session, -4 in the one before), and
    that is close to the 14 mm between the lid top and the seat ring. A decided
    state reports its own `offset_m` — how much higher the real surfaces sat
    than the winning render — so a caller can keep it as the session's
    registration for the frames that follow at the same standoff.
    """
    depth = np.asarray(depth_m, dtype=np.float32)
    if depth.ndim != 2:
        return DepthState("", 0.0, reason="depth is not a 2-D image")
    height, width = depth.shape
    measured = depth[0:height:stride, 0:width:stride]
    measured = np.where(np.isfinite(measured) & (measured > 0), measured, 0.0)
    if not (measured > 0).any():
        return DepthState("", 0.0, reason="no valid depth")
    body_from_cam = np.linalg.inv(np.asarray(fixture_pose, dtype=float)) @ np.asarray(camera_to_target, dtype=float)
    lift = np.array([0.0, 0.0, float(registration_m)])
    # the view's tilt from straight down: a surface raised by dz along body z is
    # nearer by dz * cos(tilt) along the optical axis (exact top-down)
    optical_axis_body = body_from_cam[:3, :3] @ np.array([0.0, 0.0, 1.0])
    cos_tilt = float(abs(optical_axis_body[2]) / max(np.linalg.norm(optical_axis_body), 1e-9))
    renders, raised = {}, {}
    for state in states:
        pts, flags = locator.surface_parts(state) if hasattr(locator, "surface_parts") else (locator.surface(state), None)
        if registration_m:
            pts = np.asarray(pts, dtype=float) + lift
        r = render_hypothesis(pts, body_from_cam, fx, fy, cx, cy, width, height, stride, raised=flags)
        r, rm = r if isinstance(r, tuple) else (r, np.zeros_like(r, dtype=bool))
        h = min(r.shape[0], measured.shape[0]); w = min(r.shape[1], measured.shape[1])
        renders[state], raised[state] = r[:h, :w], rm[:h, :w]
    measured = measured[:renders[states[0]].shape[0], :renders[states[0]].shape[1]]
    valid = {s: (measured > 0) & (renders[s] > 0) for s in states}
    on_fixture = int(sum(int(v.sum()) for v in valid.values()) / max(len(states), 1))
    if on_fixture < MIN_POINTS // 4:              # pixels at 1/stride resolution
        return DepthState("", 0.0, points=on_fixture, reason=f"only {on_fixture} depth pixels on the fixture")
    residual = {s: round(float(np.median(np.abs(measured[valid[s]] - renders[s][valid[s]]))) * 1000.0, 1)
                if valid[s].any() else math.inf for s in states}

    def pairwise(a: str, b: str):
        both = valid[a] & valid[b] & ~raised[a] & ~raised[b]      # a raised part's angle is unknown: it does not vote
        differ = both & (np.abs(renders[a] - renders[b]) > MIN_MARGIN_MM / 1000.0)
        # NO per-frame correction. TRIED (2026-10-07): a depth offset, then a
        # plane, estimated where the two hypotheses agree (the bowl, the
        # pedestal) — it made every render decide (238/240) and broke the real
        # frames (closed 46 -> 26 decided): the real fixture differs from the
        # model UNEVENLY (a flat board lid 15-20 mm high, a bowl 13 mm deep, a
        # tilt), so an offset from the pedestal says nothing about the lid. The
        # raw vote is 98-99 % on the real closed frames; the real open bowl
        # stays at 57 % — undecided, and the colour model decides it — until
        # the model's rim and ring heights are measured on the real fixture.
        err_a = np.abs(measured - renders[a]); err_b = np.abs(measured - renders[b])
        voters = differ & (np.minimum(err_a, err_b) < explained_m)
        n = int(voters.sum())
        if n < MIN_DISCRIMINATING:
            return None, n, 0.0
        share = float((err_a[voters] < err_b[voters]).mean())
        # how much better the winner explains the voters, in mm (signed for a)
        margin = float(np.median(err_b[voters] - err_a[voters])) * 1000.0
        return share, n, margin

    plain = sorted(residual.items(), key=lambda kv: kv[1])
    reason = ""
    for a, _ in plain:
        margins = []
        verdict = ""
        for b in states:
            if b == a:
                continue
            share, n, margin = pairwise(a, b)
            if share is None:
                verdict = f"{a} vs {b}: only {n} distinguishing pixels in view"
                break
            # an overwhelming vote decides on its own; a clear one needs the
            # separation too (CLEAR_VOTE_SHARE: 8 of 8 closed renders left
            # undecided at 85-97 % of thousands of votes were decided right by it)
            if share < MIN_VOTE_SHARE or (share < CLEAR_VOTE_SHARE and margin < MIN_MARGIN_MM):
                verdict = f"{a} vs {b}: {share:.0%} of {n} votes, {margin:.1f} mm"
                break
            margins.append(max(margin, 0.0))
        if not verdict:
            # the winner's own registration: how much higher the real surfaces sat
            # than its render, over the pixels it explains on parts that do not move
            own = valid[a] & ~raised[a] & (np.abs(measured - renders[a]) < explained_m)
            offset = (float(-np.median(measured[own] - renders[a][own])) * cos_tilt + float(registration_m)
                      if own.sum() >= 50 else float(registration_m))
            return DepthState(a, round(min(margins), 1), residual, on_fixture, True, "", round(offset, 4))
        reason = reason or verdict
    return DepthState("", 0.0, residual, on_fixture, False, reason or "no state wins its pairwise votes")
