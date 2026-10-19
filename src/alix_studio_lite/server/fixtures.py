"""The fixture pool: commodes A, B and C, urinal A and wash basin A (decision D29).

Each fixture is described by ALIX's own ``scene.yaml`` (vendored): its pose in the robot's base
frame, its meshes, and either an articulation (lid and seat on hinges, commodes B and C) or one
mesh per lid/seat state (commode A). Commode C is the default.

Region catalogues:

- commode C: the signed-off deck catalogue (``assets/regions/commode_c_catalogue.json``);
- the others: a provisional starter catalogue built from ALIX's ``models/regions.yaml``, one
  unit per region and arm (``arm_coverage`` L, R or RL_paired), coded L-R01, R-R01, ... until a
  regions deck exists for them.
"""
from __future__ import annotations

import json
import math
from functools import lru_cache
from pathlib import Path

import yaml

from .. import vendor_paths

CONFIG = vendor_paths.TASKS_PKG / "config"
FIXTURES = {                       # id -> folder under alix_tasks/config
    "commode_a": "commode_cleaning/commode_a",
    "commode_b": "commode_cleaning/commode_b",
    "commode_c": "commode_cleaning/commode_c",
    "urinal_a": "urinal_cleaning/urinal_a",
    "washtub_a": "washtub_cleaning/washtub_a",
}
DEFAULT = "commode_c"
TYPE_TEXT = {"commode": "Commode", "urinal": "Urinal", "washtub": "Wash basin"}
DECK_CATALOGUES = {"commode_c": vendor_paths.ASSETS / "regions/commode_c_catalogue.json"}
PHASE_OF_COMPONENT = {"closed": "lid_closed", "lid_open": "lid_open", "lid_seat_open": "seat_open"}
NAME_FIX = {"basin_siide": "basin_side"}       # a typo in ALIX's washtub regions.yaml
STATE_TEXT = {"lid_closed": "Lid closed", "lid_open": "Lid open", "seat_open": "Seat open", None: "Any"}


def _folder(fid: str) -> Path:
    if fid not in FIXTURES:
        raise KeyError(f"unknown fixture {fid!r}")
    return CONFIG / FIXTURES[fid]


def _mesh_url(fid: str, uri: str) -> str:
    return f"/mesh/fixture/{fid}/{Path(uri).name}"


def mesh_path(fid: str, name: str) -> Path | None:
    """The vendored STL behind ``/mesh/fixture/<fid>/<name>`` (None if not part of the fixture)."""
    p = _folder(fid) / "models" / Path(name).name
    return p if p.suffix.lower() == ".stl" and p.is_file() else None


PHASE_STATE = {"lid_closed": "closed", "lid_open": "lid_open", "seat_open": "lid_seat_open"}


def washroom_rotation(cubicle_mm: int):
    """The venue's orientation (yaw 180 for both cubicles). ALIX composes it into every fixture part's
    orientation (``scene_pose._fixture_root_transform``); without it a fixture faces the wrong way."""
    from scipy.spatial.transform import Rotation

    venue = yaml.safe_load(vendor_paths.cubicle_yaml(cubicle_mm).read_text(encoding="utf-8")) or {}
    rpy = (venue.get("washroom_scene", {}).get("washroom_structure", {}).get("pose", {})
           .get("orientation_rpy_deg"))
    return Rotation.from_euler("xyz", rpy, degrees=True) if rpy is not None else None


@lru_cache(maxsize=None)
def wall_clamp(fid: str, back_mm: float | None, cubicle_mm: int = 1400) -> dict | None:
    """How far the lid and the seat open before touching the wall behind the commode (the owner, 19 Oct: the lid
    angle must obey the wall, "open only up to the point where it just touches the wall", at a 0 mm gap and at a
    larger one): ``{back_mm, parts: {lid | seat: {alix_deg, used_deg, touches_wall}}}``, or None without a gap (no
    room measured) or for a fixture with no hinged parts. The wall's face stands ``back_mm`` behind the commode's
    back (``footprint``); a part's angle is the largest, from closed toward ALIX's open angle, at which no point of
    its mesh is past that face, found to 0.01 degrees. The seat rests on the lid, so it never opens past it."""
    import numpy as np
    from alix_taskgen.fixture_parts import load_part_registry
    from alix_taskgen.scene_pose import resolve_component_pose, resolve_mesh_path, resolve_mesh_scale

    if back_mm is None:
        return None
    scene = _folder(fid) / "scene.yaml"
    registry = load_part_registry(scene)
    if not registry or registry.is_combined:
        return None
    rot = washroom_rotation(cubicle_mm)
    wall_x = footprint(fid)["back_x"] + float(back_mm) / 1000.0
    parts, lid_used = {}, None
    for name in ("lid", "seat"):
        part = registry.part(name)
        if part is None or part.static or not part.joint:
            continue
        nominal = float(part.joint_angle_deg("open"))
        path = resolve_mesh_path(scene, name, vendor_paths.ALIX)
        mesh = mesh_path(fid, Path(str(path)).name) if path else None
        if abs(nominal) < 1e-9 or mesh is None:
            continue
        pts = _stl_points(mesh) * np.asarray(resolve_mesh_scale(scene, name), dtype=float)

        def farthest(angle, pts=pts, name=name):
            """The part's farthest point toward the wall (m, base frame) at this hinge angle."""
            T = resolve_component_pose(scene, name, joint_angle_deg=angle, washroom_rotation=rot)
            return float((pts @ T.rotation.as_matrix().T + np.asarray(T.translation, dtype=float))[:, 0].max())

        used = _angle_before(farthest, nominal, wall_x)
        if abs(used) < abs(nominal) - 1e-9:
            used = math.copysign(math.floor(abs(used) * 100.0) / 100.0, nominal)     # to 0.01 deg, towards closed
        if name == "seat" and lid_used is not None and abs(used) > abs(lid_used):
            used = lid_used
        if name == "lid":
            lid_used = used
        parts[name] = dict(alix_deg=round(nominal, 2), used_deg=used, touches_wall=abs(used) < abs(nominal) - 1e-6)
    return dict(back_mm=round(float(back_mm), 1), parts=parts) if parts else None


def _angle_before(farthest, nominal: float, wall_x: float) -> float:
    """The largest angle from 0 toward ``nominal`` (its sign) at which ``farthest(angle)`` is not past ``wall_x``:
    1 degree steps from closed, then bisection to under 0.01 degrees."""
    tol = 1e-6
    if farthest(nominal) <= wall_x + tol:
        return nominal
    sign, span = (1.0 if nominal > 0 else -1.0), abs(nominal)
    ok, bad, a = 0.0, span, 1.0
    while a < span:
        if farthest(sign * a) > wall_x + tol:
            bad = a
            break
        ok, a = a, a + 1.0
    for _ in range(12):
        mid = (ok + bad) / 2.0
        if farthest(sign * mid) > wall_x + tol:
            bad = mid
        else:
            ok = mid
    return sign * ok


@lru_cache(maxsize=None)
def spec(fid: str, cubicle_mm: int = 1400, back_mm: float | None = None) -> dict:
    """What the 3D view needs: every part's mesh and its 4x4 pose in the robot base frame for each
    lid/seat state, computed with ALIX's own ``scene_pose.resolve_component_pose`` (row-major). ``back_mm``: the
    gap from the commode's back to the wall behind (placement); the lid and the seat then open no further than
    that wall lets them (``wall_clamp``, said in ``lid_seat``)."""
    import numpy as np
    from alix_taskgen.fixture_parts import load_part_registry
    from alix_taskgen.scene_pose import resolve_component_pose, resolve_mesh_path, resolve_mesh_scale

    scene = _folder(fid) / "scene.yaml"
    fx = yaml.safe_load(scene.read_text(encoding="utf-8"))["fixture"]
    rot = washroom_rotation(cubicle_mm)
    registry = load_part_registry(scene)
    states = {"closed": {"lid": "closed", "seat": "closed"}, "lid_open": {"lid": "open", "seat": "closed"},
              "lid_seat_open": {"lid": "open", "seat": "open"}}
    clamp = wall_clamp(fid, back_mm, cubicle_mm) if back_mm is not None else None
    used = {p: c["used_deg"] for p, c in ((clamp or {}).get("parts") or {}).items() if c["touches_wall"]}

    def matrix(part, state):
        st = states[state].get(part)
        T = resolve_component_pose(scene, part, articulation_state=st, joint_angle_deg=used.get(part) if st == "open" else None,
                                   washroom_rotation=rot)
        M = np.eye(4)
        M[:3, :3] = T.rotation.as_matrix()
        M[:3, 3] = np.asarray(T.translation, dtype=float)
        return [round(float(v), 7) for v in M.reshape(-1)]

    def url(part, state=None):
        path = resolve_mesh_path(scene, part, vendor_paths.ALIX, fixture_state=state)
        return _mesh_url(fid, str(path)) if path else None

    parts = []
    if registry and registry.is_combined:
        # one pre-baked mesh per state (commode A), or a single mesh (urinal, basin), at the root
        state_meshes = {}
        for phase, state in PHASE_STATE.items():
            part = registry.state_part(state)
            if part is not None and part.mesh_uri:
                state_meshes[phase] = _mesh_url(fid, part.mesh_uri)
        primary = registry.primary_part
        default = _mesh_url(fid, primary.mesh_uri) if primary is not None and primary.mesh_uri else None
        parts.append(dict(name="body", scale=list(resolve_mesh_scale(scene, "body")), state_meshes=state_meshes,
                          mesh=default, matrices={ph: matrix("body", st) for ph, st in PHASE_STATE.items()}))
    else:
        for name in (registry.names if registry else ["body"]):
            parts.append(dict(name=name, scale=list(resolve_mesh_scale(scene, name)), mesh=url(name),
                              matrices={ph: matrix(name, st) for ph, st in PHASE_STATE.items()}))
    return dict(id=fid, name=fx.get("display_name", fid), type=fx.get("type", "fixture"),
                articulated=bool(fx.get("articulated")), cubicle_mm=cubicle_mm,
                washroom_rpy_deg=None if rot is None else [round(float(v), 4) for v in rot.as_euler("xyz", degrees=True)],
                parts=parts, back_mm=back_mm, lid_seat=clamp)


def list_fixtures() -> list[dict]:
    out = []
    for fid in FIXTURES:
        fx = yaml.safe_load((_folder(fid) / "scene.yaml").read_text(encoding="utf-8"))["fixture"]
        s = dict(name=fx.get("display_name", fid), type=fx.get("type", "fixture"))
        out.append(dict(id=fid, name=s["name"], type=s["type"], type_text=TYPE_TEXT.get(s["type"], s["type"]),
                        has_deck=fid in DECK_CATALOGUES, default=fid == DEFAULT))
    return out


def catalogue(fid: str) -> dict:
    """The fixture's region catalogue: the deck's, or a provisional one from regions.yaml."""
    if fid in DECK_CATALOGUES:
        return json.loads(DECK_CATALOGUES[fid].read_text(encoding="utf-8"))
    regions = (yaml.safe_load((_folder(fid) / "models/regions.yaml").read_text(encoding="utf-8")) or {}).get("regions") or {}
    units, n = [], {"left": 0, "right": 0}
    for step, (rid, r) in enumerate(regions.items(), 1):
        cov = str(r.get("arm_coverage", "RL_paired")).upper()
        arms = [a for a, flag in (("left", "L" in cov), ("right", "R" in cov)) if flag]
        phase = PHASE_OF_COMPONENT.get(r.get("component"))
        for arm in arms:
            n[arm] += 1
            code = f"{'L' if arm == 'left' else 'R'}-R{n[arm]:02d}"
            units.append(dict(
                key=code, code=code, part=None, name=NAME_FIX.get(rid, rid).replace("_", " ").upper(), arm=arm, step=step,
                deck_task=None, deck_slide=None, type="Cleaning", kind="cleaning",
                toilet_state=STATE_TEXT[phase], phase=phase, first_release=True, pole_mm=None,
                hand_preset="GRIP", hand_counts=[0, 0, 0, 0, 150, 1000], target_s=None, legacy_v6="none",
                note=f"provisional: from ALIX's regions.yaml ({rid}); no regions deck yet", picture=None,
                slide_picture=None, provisional=True))
    phases = [dict(id=p, label=STATE_TEXT[p], question={"lid_closed": "Is the lid closed?",
               "lid_open": "Is the lid UP and the seat DOWN?", "seat_open": "Are the lid and the seat both UP?"}[p])
              for p in ("lid_closed", "lid_open", "seat_open")]
    return dict(fixture=fid, deck=None, source=f"vendored {FIXTURES[fid]}/models/regions.yaml (provisional)",
                phases=phases, hand_presets={"GRIP": [0, 0, 0, 0, 150, 1000], "ALL OPEN": [1000] * 6}, units=units)


def _stl_points(path: Path):
    """An STL's vertices (binary or ASCII), N x 3, in the mesh's own units."""
    import struct

    import numpy as np

    data = path.read_bytes()
    if data[:5] == b"solid" and b"facet" in data[:512]:
        return np.array([[float(v) for v in ln.split()[1:4]] for ln in data.decode(errors="ignore").splitlines()
                         if ln.strip().startswith("vertex")])
    n = struct.unpack("<I", data[80:84])[0]
    rows = np.frombuffer(data[84:84 + n * 50], dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]))
    return rows["v"].reshape(-1, 3).astype(float)


@lru_cache(maxsize=None)
def footprint(fid: str) -> dict:
    """The fixture's extent in the robot base frame at ALIX's calibrated pose (lid and seat closed),
    from its own meshes as ALIX places them: ``front_x`` its front tip (nearest the robot), ``back_x``,
    ``left_y`` / ``right_y`` its edges on the robot's left (+Y) and right (-Y), in metres; and
    ``standoff_m``, the UGV standoff that pose was calibrated at (``scene_pose.default_ugv_standoff``)."""
    import numpy as np
    from alix_taskgen.scene_pose import default_ugv_standoff

    pts = []
    for part in spec(fid)["parts"]:
        url = (part.get("state_meshes") or {}).get("lid_closed") or part.get("mesh")
        path = mesh_path(fid, url.rsplit("/", 1)[-1]) if url else None
        if path is None:
            continue
        p = _stl_points(path) * np.asarray(part.get("scale") or [1, 1, 1], dtype=float)
        M = np.asarray(part["matrices"]["lid_closed"], dtype=float).reshape(4, 4)
        pts.append(p @ M[:3, :3].T + M[:3, 3])
    if not pts:
        raise ValueError(f"no meshes for {fid}")
    p = np.vstack(pts)
    lo, hi = p.min(0), p.max(0)
    return dict(front_x=float(lo[0]), back_x=float(hi[0]), right_y=float(lo[1]), left_y=float(hi[1]),
                standoff_m=float(default_ugv_standoff(_folder(fid) / "scene.yaml")))
