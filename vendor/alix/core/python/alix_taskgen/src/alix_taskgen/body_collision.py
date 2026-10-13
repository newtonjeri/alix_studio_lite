"""Full-body collision world for the reach gate — G33 item 3, driven by G35.

The live stack's last remaining failure class is the arm's BODY in the
environment at the SOLVED configuration: MTC's ComputeIK discards every
sampled branch as colliding and the task dies at its first arm move with
"IK failed for pose target" — with FCL naming pairs the offline gate could
not see (`L_Link5 <-> commode_body`, `IRH_right_middle_pad <-> bin_link`,
`R_ConnectorLink <-> lid_opening_cover1`, `R_Link4 <-> washroom_structure`).

This module gives the SDK oracle the same eyes. One
:class:`BodyCollisionWorld` holds the WHOLE runtime world, offline:

* the full-robot URDF (`alix_workspace/urdf/alix.urdf`) — both arms, hands,
  poles, UGV, and the BIN with its opening covers, which are ROBOT LINKS in
  the live scene (the FCL log types them 'Robot link'), so one ACM-filtered
  robot-internal check covers arm-vs-bin, arm-vs-partner-arm and arm-vs-UGV
  with no scene assembly at all;
* the washroom walls, posed per the venue scene yaml the C++ builder
  actually loads (`washroom_800mm_wide_scene.yaml` — G23);
* the commode fixture meshes at the config's own `commode_fixture_type`
  articulation, posed by the same `scene_pose` math the generator authored
  the points with.

The `pose_filter` produced by :meth:`make_pose_filter` maps one active-arm
joint solution (degrees, the Algo convention) to the named collision pairs
the RUNTIME would report for it — empty means the runtime's ComputeIK would
accept the branch. `tools/verify_task_motion.verify` calls it per candidate
branch (rejection = branch search over the remaining seeds) and per 10 mm
segment sample (parity with computeCartesianPath's per-sample validity
check).

Allowance parity (G12/G35): the task's `allow_collision` stage is in force
from before the approach to after the path, so the filter allows exactly
what the stage allows — the ACTIVE HAND's links against the
`contact_surface` surfaces (plus any explicit extra `collisions` pairs) —
and nothing else. Arm structural links against ANY surface, and anything
against the bin, walls, UGV or partner arm, stay checked: those are the
pairs the runtime kills tasks with, and Newton's ruling keeps the bin
checked as-is.
"""

from __future__ import annotations

import math
import threading
from functools import lru_cache
from pathlib import Path

import numpy as np

#: Fixtures whose offline world has been validated against the LIVE stack
#: (G20): archive controls at 100%/0 pairs AND offline pair names matching
#: the live FCL log. The gate consults the world only for these — an
#: unvalidated world must not gate (the urinal_a authoring control read
#: 92.6% on R_Link5<->washroom_structure pairs no live run has ever
#: confirmed). To extend: run that fixture's control live, compare pairs,
#: add it here with the evidence in GENERATION_FINDINGS.
WORLD_VALIDATED_FIXTURES = frozenset({"commode_c"})

#: The glove pad's compression travel (CLEANING_MOTION_SPEC §4). A declared
#: per-config `collisions:` pair (Newton's escape hatch, G40) is legitimate
#: only while its measured penetration stays within this — brushing the pad
#: can absorb. Deeper contact on an ACCEPTED branch means the declaration is
#: masking a real authoring defect, and the gate fails the config for it.
PAD_COMPLIANCE_MM = 20.0

#: SRDF group state of the NON-task arm's hand while the task arm works, when no
#: schedule-derived partner is given. Newton, 2026-09-17: the idle arm is at rest
#: with its hand "fully grasped" — `grasp` (alix.srdf, inspire_hand_left/right:
#: fingers 1.3 rad, thumb 0.0698 / 0.454).
IDLE_PARTNER_HAND_STATE = "grasp"
#: THE IDLE PARTNER HAND IS AN ENVELOPE, not its meshes (Newton, 2026-09-17): "a
#: union of cuboids fitting the hands at full open and grasped, a little bit bigger
#: than the mesh bounding box would be the best and cheapest collision checking
#: solution, bigger by almost .5cm in all dimensions" — "at the end of the day you
#: do not want to move too close to the fingers, or the hand of the other arm".
#: One box per state, each the bounding box of the WHOLE hand (palm and every
#: finger link) in that state, taken in the hand's own base frame so it rides the
#: partner arm; the union is both boxes at once, so the hand may be in either.
IDLE_PARTNER_HAND_STATES = ("grasp", "release")
#: How much bigger than the fitted bounding box each envelope is, per DIMENSION
#: (length, width and height each grow by this — half of it on every face).
IDLE_HAND_ENVELOPE_GROWTH_M = 0.005
#: Collision-object name stem. It starts with the hand's own link prefix (ILH_/IRH_)
#: so the pair filter treats a contact as cross-arm — never excusable by a config
#: declaration — and the text before '#' is what a contact reports.
IDLE_HAND_ENVELOPE_NAME = "{prefix}_hand_envelope"
#: Where an envelope not in force for this filter is parked (a schedule named the
#: partner's hand state, so only that state's box applies): far outside any world.
_PARKED = np.eye(4)
_PARKED[:3, 3] = (1.0e4, 1.0e4, 1.0e4)

#: contact_surface -> the fixture collision-object names the C++ allowance
#: expands to (task_base's allow/forbid stage; log-verified pair names).
CONTACT_SURFACES = {
    "all": ("commode_body", "commode_seat", "commode_lid"),
    "body": ("commode_body",),
    "seat": ("commode_seat",),
    "lid": ("commode_lid",),
    "seat_and_lid": ("commode_seat", "commode_lid"),
}

#: commode_fixture_type -> per-part articulation states for scene_pose.
FIXTURE_TYPE_STATES = {
    "closed": {"lid": "closed", "seat": "closed"},
    "lid_open": {"lid": "open", "seat": "closed"},
    "lid_seat_open": {"lid": "open", "seat": "open"},
}

_ARM_CHAIN = ("Link1", "Link2", "Link3", "Link4", "Link5", "Link6", "Link7",
              "ConnectorLink", "CameraHolderLink")


def _repo_root() -> Path:
    here = Path(__file__).resolve()
    for parent in here.parents:
        if (parent / "ARCHITECTURE.md").is_file() and (parent / "core").is_dir():
            return parent
    raise RuntimeError("repo root not found")


def _T(transform) -> np.ndarray:
    """alix_taskgen Transform -> 4x4."""
    M = np.eye(4)
    M[:3, :3] = transform.rotation.as_matrix()
    M[:3, 3] = np.asarray(transform.translation, dtype=float)
    return M


class BodyCollisionWorld:
    """The runtime's collision world, rebuilt offline. Build once per process
    (mesh BVHs are the expensive part); query per joint solution."""

    #: Venue scenes live together; a short name is enough to pick one.
    VENUE_DIR = ("ros", "orchestration", "alix_tasks", "config", "scene")
    DEFAULT_VENUE = "washroom_800mm_wide_scene.yaml"

    def __init__(self, repo_root: Path | None = None,
                 scene_yaml: str | Path | None = None):
        self.repo = Path(repo_root) if repo_root else _repo_root()
        #: The VENUE, as an argument. It used to be a literal buried in
        #: `_ensure`, which meant the one thing most likely to change about a
        #: deployment — how wide the cubicle is — could only be changed by
        #: editing this file. The 800 mm room stays the default because it is
        #: the standardised operating space; 1000 mm, 1400 mm and the Amsterdam
        #: scene sit beside it and are now reachable by name.
        self.scene_yaml = self._resolve_venue(scene_yaml)
        self._built = False
        # One world per process (`shared_world` is lru_cache'd) and ONE
        # trimesh CollisionManager pair per side inside it, so every closure
        # `make_pose_filter` ever hands out mutates the SAME FCL objects.
        # Studio runs generation in-process on a ThreadingHTTPServer, so two
        # overlapping requests put two threads into `set_transform` /
        # `in_collision_other` on the same manager and python-fcl walks a
        # broadphase tree another thread is rewriting: reproduced twice as a
        # hard SIGSEGV (exit 139, 2026-08-20) with faulthandler naming
        # trimesh/collision.py:423 in_collision_other <- _pairs <- filter_fn.
        # A control run with four threads on their OWN manager pairs and
        # 12,000 iterations survived clean — the SHARING is the defect, not
        # python-fcl — so the fix is to serialize access to the managers.
        # RE-ENTRANT because the lock nests: `make_pose_filter` takes it to
        # pose the static manager and to build the fixture manager, and the
        # baseline probe `_pairs(None)` takes it again from inside the same
        # call; a plain Lock self-deadlocks there.
        # SCOPE, explicitly: this lock protects the MANAGERS (and the caches
        # that build them) only. The module-global `frame_bands._WORLD_FILTER`
        # is a separate race on a separate object, closed by the server's
        # generation-wide lock; this lock is not a substitute for it, and the
        # generation-wide lock is what also keeps one request's static-manager
        # pose from being overwritten by another request between the posing
        # here and the queries in `_pairs`.
        self._lock = threading.RLock()
        #: side -> {hand state: envelope box record} (`_managers`)
        self._envelopes = {}

    # ------------------------------------------------------------------ build
    def _resolve_venue(self, spec) -> Path:
        """A venue path, a bare filename, or a short name like "1000"/"800mm"."""
        base = self.repo.joinpath(*self.VENUE_DIR)
        if spec is None:
            return base / self.DEFAULT_VENUE
        p = Path(str(spec)).expanduser()
        if p.is_absolute() and p.is_file():
            return p
        cand = base / p.name
        if cand.is_file():
            return cand
        key = str(spec).strip().lower().replace("mm", "").replace("_", "")
        if key.isdigit():
            named = base / f"washroom_{int(key)}mm_wide_scene.yaml"
            if named.is_file():
                return named
        named = base / f"washroom_scene_{key}.yaml"
        if named.is_file():
            return named
        raise FileNotFoundError(
            f"venue {spec!r} not found under {base} — have "
            f"{sorted(x.name for x in base.glob('*.yaml'))}")

    @classmethod
    def venues(cls, repo_root: Path | None = None) -> list:
        """Every venue scene on disk, for a caller that wants to offer a choice."""
        base = (Path(repo_root) if repo_root else _repo_root()).joinpath(*cls.VENUE_DIR)
        return sorted(x.name for x in base.glob("washroom*.yaml"))

    def _ensure(self):
        if self._built:
            return
        # Check-then-set under the lock: `self._built = True` is set
        # LAST, so two concurrent first-callers otherwise both rebuild
        # and replace the manager dicts mid-flight under each other.
        with self._lock:
            if self._built:
                return
            import sys
            ws_src = self.repo / "core" / "python" / "alix_workspace" / "src"
            if str(ws_src) not in sys.path:
                sys.path.insert(0, str(ws_src))
            from alix_workspace.collision import RobotCollisionModel
            from alix_workspace.urdf_kinematics import UrdfModel
            from alix_workspace.srdf_states import (SrdfStates, find_srdf,
                                                    load_initial_positions)
            from alix_workspace.dual_arm_plan_collision import load_disabled_pairs
            from alix_workspace.plan_scene_validation import load_wall_manager

            urdf = ws_src / "alix_workspace" / "urdf" / "alix.urdf"
            self.model = UrdfModel.from_file(str(urdf))
            self.rcm = RobotCollisionModel.from_file(str(urdf), tag="collision")
            srdf_path = find_srdf(str(urdf.parent))
            self.acm = load_disabled_pairs(srdf_path) if srdf_path else set()
            self.srdf = SrdfStates.from_file(srdf_path) if srdf_path else None
            try:
                self.initial = load_initial_positions(self.rcm.index)
            except Exception:  # noqa: BLE001
                self.initial = {}
            self.walls, _ = load_wall_manager(self.scene_yaml)
            self._movers = {}        # side -> (CollisionManager, entries)
            self._statics = {}       # side -> (CollisionManager, entries)
            self._selfs = {}         # side -> (lo, hi, distal) managers + entries
            self._fixtures = {}      # (fixture, fixture_type) -> CollisionManager
            # Which filter last posed each side's SHARED static manager. Every
            # filter re-poses it from its own `home` before querying when the
            # token differs (see make_pose_filter) — without this, creating a
            # second filter on the same world silently re-posed the partner
            # arm for every filter already alive (measured 2026-08-27: a
            # filter built with the partner at rest reported 3 partner pairs,
            # then [] once another filter had moved the partner away).
            self._static_owner = {}
            self._built = True

    def _managers(self, side: str):
        """(moving mgr+entries, static mgr+entries) for *side*. Moving = the
        active arm chain + its hand; static = everything else (partner arm,
        UGV, bin + covers, poles, glove unit). Splitting keeps static-static
        overlap out of every query and roughly halves the pair count."""
        # The built managers are process-wide shared FCL state; the
        # check-then-build below is the same first-caller race as
        # `_ensure`, and every later query mutates what it returns.
        with self._lock:
            import trimesh
            if side in self._movers:
                return self._movers[side], self._statics[side]
            p = "L" if side == "left" else "R"
            # FK-derived split: a link is MOVING iff its world transform depends
            # on the active arm's joints. A name-prefix list misses riders — the
            # camera_link rides the right arm and, frozen at its rest pose,
            # produced a phantom R_Link3<->camera_link contact on the G20
            # control (toplid_right) until this was measured instead of assumed.
            at_zero = self.model.link_world_transforms({})
            bent = self.model.link_world_transforms(
                {f"{p}_joint{i}": 0.5 for i in range(1, 8)})
            moving_links = {link for link, T in at_zero.items()
                            if not np.allclose(T, bent.get(link, T), atol=1e-9)}

            def is_moving(link):
                return link in moving_links

            def build(pred):
                mgr = trimesh.collision.CollisionManager()
                entries = []
                for link, geoms in self.rcm.links.items():
                    if not pred(link):
                        continue
                    for gi, geom in enumerate(geoms):
                        mesh = self.rcm._load_mesh(geom)
                        if mesh is None or not len(mesh.faces):
                            continue
                        name = f"{link}#{gi}"
                        mgr.add_object(name, mesh, np.eye(4))
                        entries.append((name, link, np.asarray(geom.origin, float)))
                return mgr, entries

            partner = "left" if side == "right" else "right"
            partner_prefix = ("ILH" if partner == "left" else "IRH") + "_"
            self._movers[side] = build(is_moving)
            # The partner hand is NOT in the static set as meshes: it is its
            # envelopes (`IDLE_PARTNER_HAND_STATES`, Newton 2026-09-17).
            self._statics[side] = build(lambda l: not is_moving(l) and not l.startswith(partner_prefix))
            sta_mgr, _sta_entries = self._statics[side]
            envelopes = {}
            states = (self.srdf.states_in_group(f"inspire_hand_{partner}") if self.srdf is not None else {})
            for state in sorted(states):
                fitted = self._hand_envelope(partner, state)
                if fitted is None:
                    continue
                root, T_box, extents = fitted
                name = f"{IDLE_HAND_ENVELOPE_NAME.format(prefix=partner_prefix[:-1])}#{state}"
                sta_mgr.add_object(name, trimesh.creation.box(extents=extents), _PARKED)
                envelopes[state] = {"name": name, "root": root, "T_box": T_box,
                                    "extentsM": [round(float(v), 4) for v in extents]}
            self._envelopes[side] = envelopes
            return self._movers[side], self._statics[side]

    def _hand_envelope(self, hand_side: str, state: str):
        """``(root_link, T_root_box, extents)``: the whole hand's bounding box at *state*
        in its own base frame, grown by `IDLE_HAND_ENVELOPE_GROWTH_M` per dimension."""
        prefix = ("ILH" if hand_side == "left" else "IRH") + "_"
        try:
            joints = self.srdf.states_in_group(f"inspire_hand_{hand_side}").get(state) or {}
        except Exception:  # noqa: BLE001
            return None
        tf = self.model.link_world_transforms(dict(joints))
        by_child = getattr(self.model, "joints_by_child", {}) or {}
        hand = [l for l in self.rcm.links if l.startswith(prefix) and l in tf]
        roots = [l for l in hand if not str(getattr(by_child.get(l), "parent", "")).startswith(prefix)]
        if not hand or len(roots) != 1:
            return None
        root = roots[0]
        to_root = np.linalg.inv(tf[root])
        pts = []
        for link in hand:
            for geom in self.rcm.links[link]:
                mesh = self.rcm._load_mesh(geom)
                if mesh is None or not len(mesh.vertices):
                    continue
                M = to_root @ tf[link] @ np.asarray(geom.origin, float)
                V = np.asarray(mesh.vertices, dtype=float)
                pts.append(V @ M[:3, :3].T + M[:3, 3])
        if not pts:
            return None
        P = np.vstack(pts)
        half = IDLE_HAND_ENVELOPE_GROWTH_M / 2.0
        lo, hi = P.min(axis=0) - half, P.max(axis=0) + half
        T_box = np.eye(4)
        T_box[:3, 3] = (lo + hi) / 2.0
        return root, T_box, hi - lo

    #: Active-arm links whose pose is FIXED by the tool pose: the flange and
    #: everything hanging off it. With the tool frame held, no swivel or wrist
    #: branch moves them, so their contact with the world is a property of the
    #: POSE — one query per pose, not one per cell.
    _DISTAL_TOKENS = ("Link7", "ConnectorLink", "CameraHolderLink", "camera_link")

    def _self_managers(self, side: str):
        """Three sub-managers of the mover set, for self-collision and the
        distal-only query: lo = Link1..3, hi = Link4..6, distal = the flange,
        the camera holder/camera and the whole hand.

        Why sub-managers rather than ``in_collision_internal`` on the mover
        set: that call costs 95 ms (measured) because it pairs all 37 objects,
        27 of which are hand links whose 351 mutual pairs are ALL ACM-disabled.
        The SRDF leaves exactly 66 intra-arm pairs enabled (right; 62 left):
        Link1/2 vs Link6/7/ConnectorLink/CameraHolderLink, Link3 vs
        CameraHolderLink, and the arm chain vs its own hand. lo x hi, lo x
        distal and hi x distal cover every one of them, and the adjacent pairs
        they also produce are dropped by the same ACM filter as everything
        else.
        """
        with self._lock:
            if side in self._selfs:
                return self._selfs[side]
            import trimesh
            (mov_mgr, mov_entries), _ = self._managers(side)
            p = "L" if side == "left" else "R"
            hand = "ILH" if side == "left" else "IRH"

            def which(link):
                if link.startswith(hand) or any(t in link for t in self._DISTAL_TOKENS):
                    return "distal"
                if link in (f"{p}_Link1", f"{p}_Link2", f"{p}_Link3"):
                    return "lo"
                if link in (f"{p}_Link4", f"{p}_Link5", f"{p}_Link6"):
                    return "hi"
                return "distal"        # any other rider: treat as tool-fixed

            groups = {"lo": ([], trimesh.collision.CollisionManager()),
                      "hi": ([], trimesh.collision.CollisionManager()),
                      "distal": ([], trimesh.collision.CollisionManager())}
            for name, link, origin in mov_entries:
                geoms = self.rcm.links[link]
                gi = int(name.split("#")[1])
                mesh = self.rcm._load_mesh(geoms[gi])
                if mesh is None or not len(mesh.faces):
                    continue
                entries, mgr = groups[which(link)]
                mgr.add_object(name, mesh, np.eye(4))
                entries.append((name, link, origin))
            self._selfs[side] = {k: (mgr, entries) for k, (entries, mgr) in groups.items()}
            return self._selfs[side]

    def fixture_manager(self, fixture: str, fixture_type: str,
                        ugv_standoff_m: float | None = None):
        """CollisionManager with `commode_body`/`commode_seat`/`commode_lid`
        posed at *fixture_type*'s articulation — the same scene_pose math that
        authored the cleaning points, so gate and generator share one world.

        *ugv_standoff_m* is the config's DECLARED standoff; the fixture
        shifts by its delta from the scene default exactly as the C++ places
        it (G38 — before this the world posed every fixture at the default,
        so a 0.38-standoff config was gated against a fixture 130 mm closer
        than the one the runtime checks)."""
        # Same shared-cache race as `_managers`: two threads missing on
        # the same key both build a manager and the second replaces the
        # first while a query may already hold it.
        with self._lock:
            import trimesh
            from alix_taskgen.scene_pose import (default_ugv_standoff,
                                                 load_component_mesh,
                                                 resolve_component_pose,
                                                 standoff_delta)
            key = (fixture, fixture_type,
                   round(float(ugv_standoff_m), 4) if ugv_standoff_m else None)
            if key in self._fixtures:
                return self._fixtures[key]
            states = FIXTURE_TYPE_STATES.get(fixture_type,
                                             FIXTURE_TYPE_STATES["closed"])
            config_root = (self.repo / "ros" / "orchestration" / "alix_tasks"
                           / "config")
            candidates = [d / fixture / "scene.yaml"
                          for d in sorted(config_root.iterdir()) if d.is_dir()]
            scene_path = next((c for c in candidates if c.is_file()), None)
            mgr = trimesh.collision.CollisionManager()
            if scene_path is None:
                self._fixtures[key] = mgr      # no scene -> no fixture objects,
                return mgr                     # robot + walls still checked
            from alix_taskgen.fixture_parts import load_part_registry
            from alix_taskgen.scene_pose import load_washroom_rotation
            registry = load_part_registry(scene_path)
            parts = list(registry.names) if registry else ["body"]
            # The venue's washroom rotation (yaw 180) rotates the FIXTURE too —
            # the C++ builder composes it into every part pose, and the authoring
            # pipeline passes it at every resolve_component_pose call site. This
            # was the one call site without it (G37): the offline commode faced
            # the wrong way, its open lid/seat leaned AWAY where the live ones
            # lean TOWARD the robot (live seat hinge measured at x=0.890,
            # rpy(180, -70, 0) vs this world's x=1.12 leaning back), and every
            # open-state fixture verdict was fiction.
            washroom_rotation = load_washroom_rotation(scene_path)
            delta = standoff_delta(scene_path, ugv_standoff_m)
            for part in dict.fromkeys([*parts, "body"]):
                try:
                    mesh = load_component_mesh(scene_path, part, self.repo)
                except Exception:  # noqa: BLE001
                    mesh = None
                if mesh is None:
                    continue
                T = _T(resolve_component_pose(
                    scene_path, part, articulation_state=states.get(part),
                    washroom_rotation=washroom_rotation,
                    standoff_delta_m=delta))
                mgr.add_object(f"commode_{part}" if part in ("body", "seat", "lid")
                               else f"{fixture}_{part}", mesh, transform=T)
            self._fixtures[key] = mgr
            return mgr

    def _contact_links_for(self, frames):
        """Union of contact_links.yaml's per-frame link sets for *frames*,
        or None when the file is unreadable (caller falls back to the old
        hand-prefix rule — over-allowing, never stricter than the runtime)."""
        if not hasattr(self, "_contact_groups"):
            import yaml
            path = (self.repo / "ros" / "orchestration" / "alix_tasks"
                    / "config" / "contact_links.yaml")
            try:
                raw = yaml.safe_load(path.read_text()) or {}
                self._contact_groups = {
                    str(k): frozenset(str(l) for l in v or [])
                    for k, v in (raw.get("contact_link_groups") or {}).items()}
                # The C++ allowance's OBJECT side, from the same file: the
                # contact_surface -> collision-object map. Covers the
                # non-commode surfaces (bin_link, bin_lid_covers, ...) the
                # builtin CONTACT_SURFACES table never knew (mechanism/bin/
                # glove gate extension, contract C11, 2026-08-18).
                self._surface_objects = {
                    str(k): tuple(str(o) for o in v or [])
                    for k, v in (raw.get("contact_surface_objects")
                                 or {}).items()}
            except Exception:  # noqa: BLE001
                self._contact_groups = None
                self._surface_objects = None
        if not self._contact_groups:
            return None
        out = set()
        for frame in frames:
            out |= self._contact_groups.get(str(frame), frozenset())
        return out if out else None

    # ------------------------------------------------------------------ query
    def make_pose_filter(self, config: dict, fixture: str,
                         partner_state: dict | None = None,
                         pole_override_m: float | None = None):
        """(filter_fn, info) for one task config.

        ``filter_fn(joints_deg7_or_None)`` FKs the whole robot with the active
        arm at the given Algo solution (None = both arms at rest) and returns
        the sorted list of blocking pair strings ``"A<->B"`` — ACM-filtered,
        allowance-filtered, rest-baseline-subtracted. Empty list = the runtime
        would accept this branch. ``info`` records the world knobs and any
        baseline artifacts for the caller to report once.

        *partner_state* is the schedule-derived state of the NON-task arm
        (``schedule_chain.partner_state_for``): ``{"arm_state": srdf name,
        "pole_m": float|None, "hand_state": srdf name|None}``. The runtime
        pins the whole robot to the chained ``start_joint_state`` seed, so
        the partner is a FixedState obstacle at wherever the SCHEDULE left
        it — ``ready`` + pole full_length for row 0's opening jobs, not the
        blanket rest_pose + half_length this world froze before. ``None``
        keeps the legacy default (both arms rest, partner pole half_length).

        *pole_override_m* pins the ACTIVE arm's sliding plate to a metre value
        instead of the config's own ``move_pole`` state. It exists for the
        redundancy atlas, which treats pole height as a coordinate to sweep: an
        atlas evaluated at one pole while this world checks collision at another
        would report the arm clear of a wall it is actually inside. Everything
        else leaves it None and gets the config's state, unchanged.
        """
        self._ensure()
        params = config.get("task_parameters") or {}
        ik_frame = str(params.get("ik_frame") or "")
        side = "left" if ik_frame.startswith("L_") else "right"
        p = "L" if side == "left" else "R"
        hand = "ILH" if side == "left" else "IRH"
        fixture_type = str(params.get("commode_fixture_type") or "closed")

        surfaces, extra_pairs = set(), set()
        pole_state, hand_state = "quarter_length", None
        frames_used = {ik_frame} if ik_frame else set()
        seen_execute = False
        for st in config.get("stage_sequence") or []:
            stage = str(st.get("stage") or "")
            group = str(st.get("planning_group") or "")
            # A path stage ends the pre-path phase whatever its NAME —
            # bin/glove path stages (carry_to_*, slide into glove) carry a
            # `path:` key, not the execute_path name (C11 gate extension).
            if stage == "execute_path" or st.get("path"):
                seen_execute = True
            elif stage == "allow_collision":
                # UNION over every allow window: bin/glove bracket several
                # allow/forbid windows (bin_link at the grasp, covers at the
                # stow); last-wins modelled only cleaning's single bracket.
                surfaces.add(str(st.get("contact_surface") or "all"))
                for pair in st.get("collisions") or []:
                    if len(pair) == 2:
                        extra_pairs.add(frozenset((str(pair[0]), str(pair[1]))))
                frames_used |= {str(f) for f in
                                st.get("additional_ik_frames") or []}
                # Stage-level contact_links_groups override (bin: only the
                # finger chains touch bin_link) — the keys live in the same
                # contact_link_groups table the frames resolve from.
                frames_used |= {str(g) for g in
                                st.get("contact_links_groups") or []}
            elif stage.startswith("move_pole"):
                # "move_pole_to_<x>_height" is the C++-canonical spelling; a
                # bare "move_pole" appears in the generator's synth config.
                # The strict prefix silently dropped the synth's pole and the
                # authoring filter judged EVERY pose at quarter_length — the
                # arm base 75-215 mm below where the config actually runs
                # (found via seat_ring_bottom_left: phantom pad-vs-lid
                # contact at poses the gate cleared, G42).
                # Side + phase aware: partner-group stages and post-path
                # stows must not set the cleaning-phase state — front_*
                # commands the PARTNER pole (stow) and re-stows its own pole
                # after the path; either one taken blindly judges the whole
                # walk at the wrong arm-base height. The rule is EXCLUSIVE
                # (skip only stages explicitly addressed to the partner):
                # the generator's synth stage carries a placeholder group
                # ("pole_x"), and an inclusive allow-list would silently
                # drop it — the G42 bug all over again.
                _partner = "left" if side == "right" else "right"
                if not seen_execute and group != f"pole_{_partner}":
                    pole_state = str(st.get("pose_name") or pole_state)
            elif stage.startswith("hand_"):
                # ANY hand stage sets the pre-path hand pose, not just
                # hand_grasp: glove entry runs hand_release -> hand_thumb_grasp
                # before its path, and judging the wear path with fully
                # grasped fingers put finger links where the task never puts
                # them (C11 gate extension, 2026-08-18). The C++ uses the
                # stage name as the pose when pose_name is absent.
                _partner = "left" if side == "right" else "right"
                if not seen_execute and group != f"inspire_hand_{_partner}":
                    hand_state = (st.get("pose_name")
                                  or stage[len("hand_"):] or None)
            if st.get("ik_frame"):
                frames_used.add(str(st["ik_frame"]))   # per-stage band frames
        if not surfaces:
            surfaces = {"all"}
        contact_surface = "+".join(sorted(surfaces))
        # Object side of the allowance: contact_links.yaml's own
        # contact_surface_objects map (bin_link, bin_lid_covers, ... —
        # exactly what the C++ resolves), builtin commode table as fallback.
        self._contact_links_for(frozenset())        # ensure the cache exists
        allowed = set()
        for s in surfaces:
            table = getattr(self, "_surface_objects", None) or {}
            allowed |= set(table.get(s)
                           or CONTACT_SURFACES.get(s, CONTACT_SURFACES["all"]))
            # GRANULARITY GAP, deliberate: the C++ scene splits equipment into
            # finer objects (surface key `bin_link` maps to bin_handle_left/
            # right) that this URDF world does not have — its bin is ONE
            # `bin_link` body. Allowing the KEY itself lets the declared grasp
            # window excuse finger<->bin contact at the handle; the cost is
            # that finger-vs-bin-BODY contact inside the window is excused
            # too, which the C++ would block. Recorded, not hidden — live
            # parity is the check (2026-08-18 gate extension).
            allowed.add(s)
        # The allowed LINK side comes from contact_links.yaml — the same
        # per-frame surgical sets the C++ allowance resolves (G39: measured
        # from the hand URDF at each frame's own hand state; the palm and
        # base are in NO frame's set). Union over every frame the config
        # presses with. Fallback to the hand prefix only when the file is
        # unreadable — over-allowing is the old behavior, never worse.
        allowed_links = self._contact_links_for(frames_used)

        home = dict(self.initial)
        idle_partner_hand: dict = {}
        partner_of_task = "left" if side == "right" else "right"
        if self.srdf is not None:
            try:
                home.update(self.srdf.group_state("bin_rail", "BIN_LOWERED"))
            except Exception:  # noqa: BLE001
                pass
            for arm_side in ("left", "right"):
                try:
                    home.update(self.srdf.group_state(
                        f"rm75_arm_{arm_side}", "rest_pose"))
                except Exception:  # noqa: BLE001
                    pass
            # THE IDLE PARTNER'S HAND. Newton, 2026-09-17, generating the
            # right-arm bowl_inside_front: "the left arm is at rest pose and the
            # left hand fully grasped position". The arm half of that was
            # already the default above; the hand was never posed, so the
            # partner's fingers sat wherever initial_positions left them. A
            # schedule-derived `partner_state` below still overrides both.
            try:
                idle_partner_hand = dict(self.srdf.hand_state(partner_of_task,
                                                              IDLE_PARTNER_HAND_STATE))
                home.update(idle_partner_hand)
            except Exception:  # noqa: BLE001
                idle_partner_hand = {}
            try:
                # PARTNER pole: the live harness seeds BOTH sliding plates at
                # 0.15 (tools/stacks/run_cleaning_tasks.sh START string, = SRDF
                # half_length) — NOT initial_positions.yaml's 0.3, which is
                # 150 mm away from the world the runtime actually checks.
                home.update(self.srdf.group_state("dual_poles", "half_length"))
                # ACTIVE pole: the task's own move_pole stage runs before the
                # approach, so approach + path plan at the config's state.
                pole = self.srdf.group_state("dual_poles", pole_state)
                home.update({k: v for k, v in pole.items()
                             if k.startswith(f"{p}_")})
                if pole_override_m is not None:
                    home[f"{p}_sliding_plate_joint"] = float(pole_override_m)
            except Exception:  # noqa: BLE001
                pass
            if hand_state:
                try:
                    home.update(self.srdf.hand_state(side, str(hand_state)))
                except Exception:  # noqa: BLE001
                    pass
            if partner_state:
                # Schedule-derived partner (G50): overrides the rest_pose /
                # half_length defaults above. Applied last so a task whose
                # own stages command the partner (front_*) wins via the spec
                # the walker already folded those stages into.
                partner_side = "left" if side == "right" else "right"
                q = "L" if partner_side == "left" else "R"
                name = partner_state.get("arm_state")
                if name:
                    try:
                        home.update(self.srdf.group_state(
                            f"rm75_arm_{partner_side}", str(name)))
                    except Exception:  # noqa: BLE001
                        pass
                if partner_state.get("pole_m") is not None:
                    home[f"{q}_sliding_plate_joint"] = float(
                        partner_state["pole_m"])
                if partner_state.get("hand_state"):
                    try:
                        home.update(self.srdf.hand_state(
                            partner_side, str(partner_state["hand_state"])))
                    except Exception:  # noqa: BLE001
                        pass
                # Explicit partner JOINTS, applied last so they win over the
                # named state. A paired task needs the partner pinned where its
                # own authored trajectory puts it at this moment, which is not
                # any SRDF group state — `{"joints": [7 values in RADIANS]}` or
                # a `{joint_name: value}` map. Anything else is unchanged.
                joints = partner_state.get("joints")
                if joints is not None:
                    if isinstance(joints, dict):
                        home.update({str(k): float(v) for k, v in joints.items()})
                    else:
                        vals = [float(v) for v in joints]
                        for i, v in enumerate(vals[:7], start=1):
                            home[f"{q}_joint{i}"] = v

        (mov_mgr, mov_entries), (sta_mgr, sta_entries) = self._managers(side)
        selfs = self._self_managers(side)
        joint_names = [f"{p}_joint{i}" for i in range(1, 8)]

        transforms = self.model.link_world_transforms(home)
        # THE ENVELOPES IN FORCE: both idle states while the partner's hand state is
        # UNKNOWN (the union), or only the state a schedule names. Every other
        # envelope is parked, so no box one filter needs can linger in another's.
        named_hand = str(partner_state.get("hand_state")) if (partner_state and partner_state.get("hand_state")) else None
        envelope_states = (named_hand,) if named_hand else IDLE_PARTNER_HAND_STATES
        union = named_hand is None
        # The static manager is SHARED, and posing it is exactly the mutation
        # `_pairs` races with — the crash is not confined to `_pairs`, so the
        # posing loop and the fixture-manager build hold the lock too.
        #
        # The pose is also REMEMBERED per filter: `static_pose` is re-applied
        # by `_pairs` whenever another filter has posed the manager since
        # (`self._static_owner[side]`), so two filters with different partner
        # states on one world each see their own partner, whatever order they
        # are called in. ~66 set_transform calls, microseconds each.
        token = object()
        static_pose = []
        with self._lock:
            for name, link, origin in sta_entries:
                T = transforms.get(link)
                if T is not None:
                    static_pose.append((name, T @ origin))
                    sta_mgr.set_transform(name, T @ origin)
            envelopes_posed = {}
            for state, env in self._envelopes.get(side, {}).items():
                T_root = transforms.get(env["root"])
                T = (T_root @ env["T_box"]) if (state in envelope_states and T_root is not None) else _PARKED
                static_pose.append((env["name"], T))
                sta_mgr.set_transform(env["name"], T)
                if T is not _PARKED:
                    envelopes_posed[state] = env["extentsM"]
            self._static_owner[side] = token

            fixture_mgr = self.fixture_manager(
                fixture, fixture_type,
                ugv_standoff_m=params.get("ugv_standoff_m"))

        def _ensure_static_pose():
            """Under the lock: re-pose the shared static manager if a later
            filter moved it."""
            if self._static_owner.get(side) is not token:
                for name, T in static_pose:
                    sta_mgr.set_transform(name, T)
                self._static_owner[side] = token

        def _pose_movers(joints_deg7):
            """Under the lock: FK the whole robot and pose every mover manager."""
            joints = dict(home)
            if joints_deg7 is not None:
                joints.update({n: math.radians(v)
                               for n, v in zip(joint_names, joints_deg7)})
            tf = self.model.link_world_transforms(joints)
            for name, link, origin in mov_entries:
                T = tf.get(link)
                if T is not None:
                    mov_mgr.set_transform(name, T @ origin)
            for mgr, entries in selfs.values():
                for name, link, origin in entries:
                    T = tf.get(link)
                    if T is not None:
                        mgr.set_transform(name, T @ origin)

        def _collect(found, mgr_a, mgr_b):
            hit, names, data = mgr_a.in_collision_other(
                mgr_b, return_names=True, return_data=True)
            if not hit:
                return
            for a, b in names:
                key = tuple(sorted((a.split("#")[0], b.split("#")[0])))
                found.setdefault(key, 0.0)
            for contact in data:
                key = tuple(sorted(n.split("#")[0] for n in contact.names))
                found[key] = max(found.get(key, 0.0), float(contact.depth))

        def _pairs(joints_deg7, scope="all"):
            """(blocking {pair: depth_m}, declared {pair: depth_m}) at one
            branch. Depth is the deepest FCL contact of the pair — the
            physical evidence behind both the escape-hatch certification and
            the residual-pair audit (G40).

            *scope*: ``all`` (the full check), ``distal`` (only the tool-fixed
            links against the world — a property of the POSE), ``proximal``
            (the arm chain against the world plus self), ``self`` (self only).
            """
            # Whole body under the world lock: the shared mover manager
            # is posed and then queried, and a second thread stepping in
            # between the two segfaults FCL (see __init__). Held across
            # the ~3 ms query, which is the cost of sharing one world.
            with self._lock:
                _ensure_static_pose()
                _pose_movers(joints_deg7)
                found = {}
                if scope in ("all", "proximal"):
                    # The environment class: partner arm/bin/covers/UGV/poles
                    # (static robot links), walls, and the fixture at articulation.
                    for other in (sta_mgr, self.walls, fixture_mgr):
                        _collect(found, mov_mgr if scope == "all" else selfs["lo"][0], other)
                        if scope == "proximal":
                            _collect(found, selfs["hi"][0], other)
                if scope == "distal":
                    for other in (sta_mgr, self.walls, fixture_mgr):
                        _collect(found, selfs["distal"][0], other)
                if scope in ("all", "proximal", "self"):
                    # SELF-collision, curated (see _self_managers): the 66
                    # ACM-enabled intra-arm pairs and nothing else. It used to be
                    # skipped outright on cost grounds (in_collision_internal:
                    # 95 ms); as three small cross-manager queries it is ~1 ms.
                    _collect(found, selfs["lo"][0], selfs["hi"][0])
                    _collect(found, selfs["lo"][0], selfs["distal"][0])
                    _collect(found, selfs["hi"][0], selfs["distal"][0])
                out, declared = {}, {}
                for (a, b), depth in found.items():
                    if a == b or frozenset((a, b)) in self.acm:
                        continue
                    if frozenset((a, b)) in extra_pairs:
                        # NEVER excusable, whatever a config declares: contact
                        # between the ACTIVE arm and the PARTNER arm's chain or
                        # hand (standing G50 policy — cross-arm safety is the
                        # partner-stow prelude's job, not a declaration's).
                        _q = "R" if side == "left" else "L"
                        _ph = "IRH" if side == "left" else "ILH"
                        cross_arm = any(x.startswith((f"{_q}_Link", _ph))
                                        for x in (a, b))
                        # Fixture-surface pairs keep the G40 escape hatch: excused
                        # ONLY within the pad compliance — the declaration
                        # licenses BRUSHING, and a deeper branch must keep
                        # blocking so the search finds a shallower one. Pairs
                        # against EQUIPMENT (bin_link, glove unit, covers, own
                        # base links) are grasps/insertions the runtime's binary
                        # ACM allows outright — the gate mirrors that (runtime
                        # parity, C11 gate extension 2026-08-18) and records the
                        # depth for the audit instead of failing it.
                        involves_fixture = any(
                            x.startswith(("commode_", f"{fixture}_"))
                            for x in (a, b))
                        if not cross_arm and (
                                not involves_fixture
                                or depth * 1000.0 <= PAD_COMPLIANCE_MM):
                            pair = f"{a}<->{b}"
                            declared[pair] = max(declared.get(pair, 0.0), depth)
                            continue
                    surface = a if a in allowed else (b if b in allowed else None)
                    if surface is None:
                        surface = a if a.startswith(("commode_", f"{fixture}_")) else (
                            b if b.startswith(("commode_", f"{fixture}_")) else None)
                    if surface is not None and (
                            surface in allowed
                            or surface.startswith(f"{fixture}_")):
                        # commode surfaces follow the task's contact_surface
                        # exactly; other fixtures' surfaces are what the task is
                        # pressing (their allow stage grants the hand wholesale)
                        partner = b if surface == a else a
                        if (partner in allowed_links
                                if allowed_links is not None
                                else partner.startswith(hand)):
                            continue        # the allowance the task grants
                    out[f"{a}<->{b}"] = depth
                return out, declared

        baseline = set(_pairs(None)[0])

        # Filled IN PLACE as filter_fn runs — read them after the walk.
        #   declaredDepthsMm: max penetration of each declared pair over the
        #     ACCEPTED branches only. A branch rejected for other contact is
        #     one the walk never uses; folding its (possibly deep) declared
        #     contact in would poison the certification.
        #   blockingDepthsMm: (min, max, hits) per BLOCKING pair over every
        #     attempted branch. The min is the audit signal: if even the
        #     shallowest branch penetrates beyond the pad compliance, no
        #     declaration can legitimately rescue the pose.
        declared_depths, blocking_depths = {}, {}

        def filter_fn(joints_deg7):
            # `_pairs` locks for itself, but the accumulator dicts above are
            # filled IN PLACE and `info` hands the very same objects to the
            # caller, so a read-modify-write of `blocking_depths[pair]` split
            # across two threads would lose a min/max/count. The lock is
            # re-entrant and the FCL query dominates the cost, so take it once
            # for the whole update instead of racing the dicts.
            with self._lock:
                res, declared = _pairs(joints_deg7)
                out = [p for p in sorted(res) if p not in baseline]
                for pair in out:
                    mm = res[pair] * 1000.0
                    lo, hi, n = blocking_depths.get(pair, (mm, mm, 0))
                    blocking_depths[pair] = (min(lo, mm), max(hi, mm), n + 1)
                if not out:
                    for pair, depth in declared.items():
                        mm = depth * 1000.0
                        declared_depths[pair] = max(
                            declared_depths.get(pair, 0.0), mm)
                return out

        def _scoped(scope):
            def fn(joints_deg7):
                with self._lock:
                    res, _declared = _pairs(joints_deg7, scope=scope)
                    return [p for p in sorted(res) if p not in baseline]
            return fn

        def depths(joints_deg7, scope="all"):
            """``{pair: depth_mm}`` for the blocking pairs at one branch — the
            per-query number `filter_fn` drops (it returns names only)."""
            with self._lock:
                res, _declared = _pairs(joints_deg7, scope=scope)
                return {p: res[p] * 1000.0 for p in sorted(res) if p not in baseline}

        # Scoped queries, as attributes so every caller that already holds a
        # filter gets them without a second make_pose_filter (which would
        # re-pose the shared static manager — see `_ensure_static_pose`).
        filter_fn.distal = _scoped("distal")
        filter_fn.proximal = _scoped("proximal")
        filter_fn.self_only = _scoped("self")
        filter_fn.depths = depths

        info = {"side": side, "fixture_type": fixture_type,
                # The venue this world was actually built against. Reporting the
                # REQUEST instead would let a typo'd or unresolved venue name
                # read back as if it had been honoured.
                "venue": self.scene_yaml.name, "venue_path": str(self.scene_yaml),
                "contact_surface": contact_surface, "pole_state": pole_state,
                "pole_override_m": pole_override_m,
                "hand_state": hand_state, "partner_state": partner_state,
                # What the idle partner hand was ACTUALLY posed at — the joints,
                # not the name asked for: an SRDF miss here is swallowed above,
                # and an empty map is how a reader tells it happened.
                "idlePartnerHand": {"state": IDLE_PARTNER_HAND_STATE,
                                    "joints": dict(idle_partner_hand),
                                    "applied": bool(idle_partner_hand),
                                    "model": "envelope",
                                    "union": union,
                                    "envelopeStates": list(envelope_states),
                                    "envelopesPosed": envelopes_posed,
                                    "growthPerDimensionM": IDLE_HAND_ENVELOPE_GROWTH_M,
                                    "scheduleOverride": bool(partner_state and (
                                        partner_state.get("hand_state")
                                        or partner_state.get("joints") is not None))},
                "baseline": sorted(baseline),
                "declaredDepthsMm": declared_depths,
                "blockingDepthsMm": blocking_depths}
        return filter_fn, info


def shared_world(scene_yaml: str | None = None) -> BodyCollisionWorld:
    """Process-wide world per venue (mesh BVHs load once; ~seconds).

    Keyed by the venue so that comparing a task across cubicle widths does not
    silently reuse the walls of whichever venue was asked for first — which is
    what a `maxsize=1` cache with no key did. Three is deliberate: enough to
    hold the 800 / 1000 / 1400 mm rooms at once, few enough that the meshes do
    not accumulate. `shared_world()` with no argument is unchanged and still
    returns the 800 mm world every existing caller expects.

    The key is the RESOLVED path, not the spec, which is why this wrapper
    exists. `()`, `(None,)` and `("washroom_800mm_wide_scene.yaml",)` are three
    spellings of the same venue and all three occur in this tree — the first
    from `generate_cleaning_config` and `sdk_reach`, the second from the atlas
    endpoint when the venue picker sits on its blank default, the third when the
    operator picks the "800mm (default)" entry the picker itself appends. Keying
    on the spec would fill all three slots with identical 800 mm worlds, each a
    full URDF re-parse and a complete rebuild of every mesh BVH, and would then
    evict the generation world the moment anyone compared against 1400 mm.

    Resolving costs a bare `BodyCollisionWorld.__init__`, which builds nothing:
    `_ensure` is where the seconds go, and it runs on first use of the cached
    instance.
    """
    try:
        key = str(BodyCollisionWorld(scene_yaml=scene_yaml).scene_yaml)
    except Exception:  # noqa: BLE001 — an unresolvable venue must raise on USE
        key = None
    return _shared_world_resolved(key)


@lru_cache(maxsize=3)
def _shared_world_resolved(scene_path: str | None = None) -> BodyCollisionWorld:
    """One world per resolved venue path. Do not call directly."""
    return BodyCollisionWorld(scene_yaml=scene_path)
