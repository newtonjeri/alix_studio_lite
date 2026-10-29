"""Contact forces of the lid and seat closings, predicted before the robot runs them (the owner, 29 Oct: "a contact that
makes sense should be around very few newtons ... the simulation side can tell us the forces ... before deploying to the
real hardware"; "we will work on these tasks only": Close the lid and Close the seat).

Why: the lab's Close the lid of 29 Oct drew 50 to 100 N (the wrist sensors; 15 to 25 A on J3, J4 and J6) where the
hold, the lid resting on the back of the right hand, took 6 to 8 N. The left glove pressed on the lid's edge at its start
and through its pull, and the right hand, still under the lid after its release, lifted it on its way up to the
inspection pose. ALIX's planner lets the hand touch the lid at any depth (allow_collision), and S2's collision check
leaves ALIX's plans out, so nothing said so before the robot moved.

The model (quasi-static, both arms on the run's clock):
- the part the task moves (TASK_PART) is a rigid body on its hinge, posed by ALIX's own scene_pose at any angle, with its
  weight (PART_KG), between its open stop (ALIX's open angle, or the wall behind where the room is measured:
  fixtures.wall_clamp) and closed;
- each hand wears a glove of GLOVE_MM over its model links: a spring of GLOVE_N_PER_MM, so the force on a part is
  GLOVE_N_PER_MM times how far the glove is pressed in (the model's hand-to-part distance short of GLOVE_MM, by FCL). An
  arm link past the hand has no glove: it presses from 0 mm;
- at each moment the moving part takes the angle of least energy (its weight's and the gloves') reachable from where it
  was: a hand pushing it along its swing moves it for a few newtons, one pushing it where it cannot go presses;
- the parts that do not move (the seat under the lid, the lid open behind the seat, the bowl) press back the same way;
- where the plan's config says where the part is (update_fixture_scene, after a carry): a part still on the open side of
  its balance angle takes that angle (its tip-over goes faster than a quasi-static model can follow); one past its
  balance falls toward it and stops where a hand holds it, and that is said (29 Oct: the right's release did not let the
  lid down).

Calibrated on both closings the lab ran on 29 Oct (CALIBRATION, LAB_SHIFT_MM): with the commode where the arms met it
that day, 20 mm further, 5 mm to the left and 10 mm lower than ALIX's place, and the glove at 3.5 N/mm, the model reads
the five contacts measured within about a quarter. The force moves about GLOVE_N_PER_MM for every millimetre the
commode stands off, so ``simulate`` takes a placement shift and a path is made to hold within about 10 mm of it. The model
does not follow a finger hooking the seat's opening: the seat's tip-over is the plan's config's word for it.

FCL is not safe across threads: like collision_check, this runs in the worker process (workers.pool()).
"""
from __future__ import annotations

import json
import time
from functools import lru_cache
from pathlib import Path

import numpy as np

GLOVE_MM = 20.0                 #: the glove over the model's hand links (collision_check: a real contact comes 20 mm early)
GLOVE_N_PER_MM = 3.5            #: its stiffness (calibrated: CALIBRATION)
LAB_SHIFT_MM = (20.0, 5.0, -10.0)   #: the commode as the arms met it at the lab on 29 Oct, off ALIX's nominal place (x away
                                    #: from the robot, y to the left, z up; the room's own offset added): CALIBRATION
#: The 29 Oct runs of both closings (the plans the lab ran, the wrist sensors' change) against the model with the commode
#: shifted by 0 to 20 mm in x, 0 to 5 in y, 0 to -10 in z: at 20, 5, -10 and 3.5 N/mm it reads the lid's start 50 N (66
#: measured), its pull 103 (90), the right's way up 96 (100), the seat's start 75 (66), its carry 92 (81); at the nominal
#: place it read 42 / 133 / 145 / 151 / 152 at 5 N/mm. The placement moves with the UGV's: a path is made to hold within
#: about 10 mm of it (tools/contact_check.py --band 10).
CALIBRATION = "29 Oct 2026, Close the lid 15:17:57 and Close the seat 15:15:49 / 15:17:09"
PART_KG = {"lid": 1.2, "seat": 1.0}     #: assumed, not weighed yet
G = 9.81
WARN_N, STOP_N = 20.0, 40.0     #: the owner, 29 Oct: a sensible contact is a few newtons (the hold: 6 to 8 N)
SUSTAIN_S = 0.2                 #: refused when over STOP_N this long (the lab's three of 29 Oct: 0.3 s and more); a brush
                                #: shorter is a warning (29 Oct: the left's way to rest past the lid, 0.1 s in the model,
                                #: 6 to 9 N measured)
SAMPLE_S = 0.04                 #: the run's moments this far apart (the 3D's own step is 20 ms)
NEAR_M = 0.30                   #: an arm whose links' origins are all farther than this from a part's box is not asked of FCL
STEP_DEG, FINE_DEG = 1.0, 0.125
TASK_PART = {"lid_close": "lid", "seat_close": "seat"}
OTHER = {"lid": "seat", "seat": "lid"}
HELD_DEG = 2.0                  #: a part this far short of closed where the plan's config has it closed is said to be held


def task_part(task: str) -> str | None:
    """The part ``task`` moves (Close the lid: the lid; Close the seat: the seat), or None when it is not checked."""
    return TASK_PART.get(task)


@lru_cache(maxsize=None)
def default_home() -> dict:
    """ALIX Studio's standing pose (robot_model._home) without building the 3D's meshes."""
    from . import collision_check, robot_model

    return robot_model._home(robot_model.fk_model(), collision_check._rcm().index)


class Fixture:
    """The commode for one task: the part it moves on its hinge, the others where the task leaves them, in the robot's base
    frame as Lite's 3D puts them (the room's offset), moved by ``shift_mm`` (a placement off its place)."""

    def __init__(self, task: str, fid: str = "commode_c", back_mm: float | None = None, offset_m=(0.0, 0.0),
                 cubicle_mm: int = 1400, shift_mm=(0.0, 0.0, 0.0)):
        import trimesh
        from alix_taskgen.fixture_parts import load_part_registry
        from alix_taskgen.scene_pose import resolve_component_pose, resolve_mesh_path, resolve_mesh_scale

        from .. import vendor_paths
        from . import fixtures

        self.part = TASK_PART[task]
        self._scene = fixtures._folder(fid) / "scene.yaml"
        self._rot = fixtures.washroom_rotation(cubicle_mm)
        self._pose = resolve_component_pose
        clamp = fixtures.wall_clamp(fid, None if back_mm is None else round(float(back_mm), 1), cubicle_mm) \
            if back_mm is not None else None
        registry = load_part_registry(self._scene)
        stops = {}
        for name in ("lid", "seat"):
            p = registry.part(name)
            used = ((clamp or {}).get("parts") or {}).get(name) or {}
            stops[name] = float(used.get("used_deg", p.joint_angle_deg("open")))
        self.open = stops[self.part]
        self.wall = bool((((clamp or {}).get("parts") or {}).get(self.part) or {}).get("touches_wall"))
        self.shift = np.eye(4)
        self.shift[:3, 3] = [float((offset_m or [0, 0])[0]), float((offset_m or [0, 0])[1]), 0.0]
        self.shift[:3, 3] += np.asarray(shift_mm, dtype=float) / 1000.0

        def mesh(name):
            path = resolve_mesh_path(self._scene, name, vendor_paths.ALIX)
            m = trimesh.load(str(fixtures.mesh_path(fid, Path(str(path)).name)), force="mesh")
            scale = np.asarray(resolve_mesh_scale(self._scene, name), dtype=float)
            if not np.allclose(scale, 1.0):
                m.apply_scale(scale)
            return m

        self._T: dict = {}
        self.mesh = mesh(self.part)
        self.com = np.asarray(self.mesh.centroid, dtype=float)
        self.kg = PART_KG[self.part]
        self.moving = _Obj(self.part, self.mesh, self.T(0.0))
        # the others: the seat closed under the lid (Close the lid), the lid open at its stop behind the seat (Close the
        # seat: the wall behind it, the seat in front), the bowl
        still = {OTHER[self.part]: 0.0 if self.part == "lid" else stops["lid"], "body": None}
        self.still = [_Obj(name, mesh(name), self.shift @ self._matrix(name, angle)) for name, angle in still.items()]
        self._z: dict = {}
        grid = np.arange(self.open, 0.0 + 1e-9, 0.5)
        self.balance = float(grid[int(np.argmax([self.com_z(a) for a in grid]))])
        corners = []                                  # the moving part's box over its whole swing, and the others'
        for a in np.linspace(self.open, 0.0, 9):
            corners.append(trimesh.transform_points(self.mesh.bounds, self.T(a)))
        for name, angle in still.items():
            corners.append(trimesh.transform_points(mesh(name).bounds, self.shift @ self._matrix(name, angle)))
        pts = np.vstack(corners)
        self.box = (pts.min(0) - NEAR_M, pts.max(0) + NEAR_M)

    def _matrix(self, name: str, angle: float | None) -> np.ndarray:
        T = self._pose(self._scene, name, joint_angle_deg=angle, washroom_rotation=self._rot)
        M = np.eye(4)
        M[:3, :3] = T.rotation.as_matrix()
        M[:3, 3] = np.asarray(T.translation, dtype=float)
        return M

    def T(self, angle: float) -> np.ndarray:
        a = round(float(angle), 3)
        if a not in self._T:
            self._T[a] = self.shift @ self._matrix(self.part, a)
        return self._T[a]

    def at(self, angle: float) -> "_Obj":
        self.moving.place(self.T(angle))
        return self.moving

    def com_z(self, angle: float) -> float:
        a = round(float(angle), 3)
        if a not in self._z:
            T = self.T(a)
            self._z[a] = float((T[:3, :3] @ self.com + T[:3, 3])[2])
        return self._z[a]

    def near(self, origins: np.ndarray) -> bool:
        lo, hi = self.box
        return bool(np.any(np.all((origins >= lo) & (origins <= hi), axis=1)))


class _Obj:
    """A part of the commode: its collision manager, its box in its own frame and where it is."""

    def __init__(self, name: str, mesh, T: np.ndarray):
        import trimesh

        self.name, self.lo, self.hi = name, np.asarray(mesh.bounds[0], dtype=float), np.asarray(mesh.bounds[1], dtype=float)
        self.mgr = trimesh.collision.CollisionManager()
        self.mgr.add_object(name, mesh, transform=T)
        self.T = np.asarray(T, dtype=float)

    def place(self, T: np.ndarray) -> None:
        if not np.array_equal(T, self.T):
            self.mgr.set_transform(self.name, T)
            self.T = T

    def gap(self, centers: np.ndarray, radii: np.ndarray) -> float:
        """The least gap (mm) from spheres to the part's box: a bound below its true distance, so FCL is asked only when
        it is small."""
        local = (centers - self.T[:3, 3]) @ self.T[:3, :3]
        out = np.maximum(np.maximum(self.lo - local, 0.0), local - self.hi)
        return float(np.min(np.linalg.norm(out, axis=1) - radii)) * 1000.0


@lru_cache(maxsize=None)
def _spheres(side: str, part: str) -> tuple:
    """Each collision mesh of the arm's ``part`` (hand, structure) as a sphere about its link: (links, origins, centres,
    radii)."""
    from . import collision_check

    rcm, arm = collision_check._rcm(), collision_check._arm(side, part)
    links, origins, centres, radii = [], [], [], []
    for name, link, origin in arm.entries:
        mesh = rcm._load_mesh(rcm.links[link][int(name.split("#", 1)[1])])
        v = np.asarray(mesh.vertices, dtype=float)
        c = (v.min(0) + v.max(0)) / 2.0
        links.append(link)
        origins.append(np.asarray(origin, dtype=float))
        centres.append(c)
        radii.append(float(np.max(np.linalg.norm(v - c, axis=1))))
    return links, np.asarray(origins), np.asarray(centres), np.asarray(radii)


def _signed(arm, mgr) -> tuple[float, str]:
    """The arm's distance to the object (mm; into it negative) and its link nearest (or deepest)."""
    hit, data = arm.mgr.in_collision_other(mgr, return_data=True)
    mine = {name for name, _, _ in arm.entries}
    if hit:
        best = max(data, key=lambda c: float(getattr(c, "depth", 0.0) or 0.0))
        link = next((n for n in best.names if n in mine), "")
        return -1000.0 * float(getattr(best, "depth", 0.0) or 0.0), link.split("#", 1)[0]
    d, names = arm.mgr.min_distance_other(mgr, return_names=True)
    link = next((n for n in names if n in mine), "")
    return 1000.0 * float(d), link.split("#", 1)[0]


class _Arms:
    """Both arms posed at one moment: each one's hand (gloved) and the rest of its links (bare), near the commode or not."""

    def __init__(self, fx: Fixture):
        from . import collision_check, robot_model

        self.fx = fx
        self.model = robot_model.fk_model()
        self.parts = {s: (collision_check._arm(s, "hand"), collision_check._arm(s, "structure")) for s in ("left", "right")}
        self.near, self.balls = {}, {}

    def pose(self, values: dict):
        tf = self.model.link_world_transforms(values)
        for s, (hand, rest) in self.parts.items():
            self.balls[s] = {}
            for part in ("hand", "structure"):
                links, origins, centres, radii = _spheres(s, part)
                T = np.stack([tf[ln] for ln in links]) @ origins
                self.balls[s][part] = (np.einsum("nij,nj->ni", T[:, :3, :3], centres) + T[:, :3, 3], radii)
            pts = np.vstack([c for c, _ in self.balls[s].values()])
            self.near[s] = self.fx.near(pts)
            if self.near[s]:
                hand.pose(tf)
                rest.pose(tf)

    def pressed(self, obj: _Obj) -> list:
        """[(side, mm pressed in, link)] of each arm near the commode against ``obj``: its glove, or a bare link."""
        out = []
        for s, (hand, rest) in self.parts.items():
            if not self.near.get(s):
                continue
            ph = pr = 0.0
            lh = lr = ""
            if obj.gap(*self.balls[s]["hand"]) < GLOVE_MM:
                dh, lh = _signed(hand, obj.mgr)
                ph = max(0.0, GLOVE_MM - dh)
            if obj.gap(*self.balls[s]["structure"]) < 0.0 and rest.mgr.in_collision_other(obj.mgr):
                dr, lr = _signed(rest, obj.mgr)
                pr = max(0.0, -dr)
            if ph > 0 or pr > 0:
                out.append((s, ph + pr, lh if ph >= pr else lr))
        return out


def _energy(fx: Fixture, arms: _Arms, angle: float, memo: dict | None = None) -> tuple[float, list]:
    a = round(float(angle), 3)
    if memo is not None and a in memo:
        return memo[a]
    pressed = arms.pressed(fx.at(a))
    out = fx.kg * G * fx.com_z(a) * 1000.0 + sum(0.5 * GLOVE_N_PER_MM * p * p for _, p, _ in pressed), pressed   # N mm
    if memo is not None:
        memo[a] = out
    return out


def _settle(fx: Fixture, arms: _Arms, a0: float) -> tuple[float, list]:
    """The least energy angle reached from ``a0`` (steps of STEP_DEG halved to FINE_DEG), between the stops."""
    memo: dict = {}
    a, (e, pressed) = a0, _energy(fx, arms, a0, memo)
    step = STEP_DEG
    while step >= FINE_DEG:
        moved = False
        for s in (-step, step):
            b = min(0.0, max(fx.open, a + s))
            if b == a:
                continue
            eb, pb = _energy(fx, arms, b, memo)
            if eb < e - 1e-6:
                a, e, pressed, moved = b, eb, pb, True
                break
        if not moved:
            step /= 2.0
    return a, pressed


def _fall(fx: Fixture, arms: _Arms, a0: float, target: float) -> float:
    """From ``a0`` toward ``target`` a step at a time while the energy does not rise (a hand holds it there)."""
    a, (e, _) = a0, _energy(fx, arms, a0)
    sign = 1.0 if target > a0 else -1.0
    while abs(target - a) > 1e-6:
        b = a + sign * min(STEP_DEG, abs(target - a))
        eb, _ = _energy(fx, arms, b)
        if eb > e + 1e-6:
            break
        a, e = b, eb
    return a


def simulate(task: str, times, arms: dict, home: dict | None = None, fid: str = "commode_c", back_mm: float | None = None,
             offset_m=(0.0, 0.0), cubicle_mm: int = 1400, updates=(), shift_mm=None, start_deg: float | None = None,
             progress=None) -> dict:
    """The contact forces of ``task`` over ``times`` (s, ascending; thinned to SAMPLE_S): ``arms`` {side: {pole: [n] mm,
    hand: [n][6] counts, q: [n][7] deg}} on those times, ``home`` the robot's other joints, ``updates`` [(t, deg)] where
    the plan's config puts the moving part (update_fixture_scene, at the end of the move before it), ``start_deg`` the
    part's angle at the first moment (its open stop by default). -> {trace, peaks, over, held, level, ...} (contact_view)."""
    from .. import vendor_paths

    vendor_paths.activate()
    from . import staging

    t_start = time.monotonic()
    shift_mm = LAB_SHIFT_MM if shift_mm is None else tuple(float(v) for v in shift_mm)
    fx = Fixture(task, fid, back_mm, offset_m, cubicle_mm, shift_mm)
    poser = _Arms(fx)
    home = dict(home if home is not None else default_home())
    joints = {s: [f"{staging.SIDE_PREFIX[s]}_joint{i}" for i in range(1, 8)] for s in ("left", "right")}
    times = [float(t) for t in times]
    keep, last = [], None
    for i, t in enumerate(times):                    # thinned to SAMPLE_S, the last moment kept
        if last is None or t - last >= SAMPLE_S - 1e-9 or i == len(times) - 1:
            keep.append(i)
            last = t
    pending = sorted((float(t), float(d)) for t, d in updates)
    angle = fx.open if start_deg is None else float(start_deg)
    reached = angle                                  # the most closed it has been since the last update
    trace, held, placed = [], [], []
    for n, i in enumerate(keep):
        t = times[i]
        values = dict(home)
        for s, d in arms.items():
            values.update(staging.joint_values(s, d["pole"][i], [int(round(c)) for c in d["hand"][i]]))
            values.update(zip(joints[s], np.radians(np.asarray(d["q"][i], dtype=float)).tolist()))
        poser.pose(values)
        while pending and pending[0][0] <= t + 1e-9:
            _, target = pending.pop(0)
            target = min(0.0, max(fx.open, target))
            if angle < fx.balance:                   # not tipped yet here: the plan's word for where it lands, when the carry
                placed.append(dict(t=round(t, 3), deg=round(target, 1), was=round(angle, 1),        # took it past its
                                   tipped=bool(reached >= fx.balance or target < fx.balance)))   # balance
                angle = target
            else:
                got = _fall(fx, poser, angle, target)
                if abs(target) < 1e-6 and abs(got - target) > HELD_DEG:     # the plan has it closed, a hand keeps it up
                    _, pressed = _energy(fx, poser, got)
                    by = max(pressed, key=lambda p: p[1]) if pressed else ("", 0.0, "")
                    held.append(dict(t=round(t, 3), want_deg=round(target, 1), at_deg=round(got, 1), side=by[0], link=by[2]))
                angle = got
            reached = angle
        angle, pressed = _settle(fx, poser, angle)
        reached = max(reached, angle)
        row = dict(t=round(t, 3), deg=round(angle, 2))
        for s, p, link in pressed:
            row[s] = dict(part=fx.part, n=round(GLOVE_N_PER_MM * p, 1), link=link)
        for obj in fx.still:
            for s, p, link in poser.pressed(obj):
                f = GLOVE_N_PER_MM * p
                if f > (row.get(s) or {}).get("n", 0.0):
                    row[s] = dict(part=obj.name, n=round(f, 1), link=link)
        trace.append(row)
        if progress is not None and n % 25 == 0:
            progress(n, len(keep))
    return contact_view(task, trace, held, placed, fx, shift_mm, time.monotonic() - t_start)


def contact_view(task, trace, held, placed, fx, shift_mm, seconds) -> dict:
    """The result: each arm's peak (N, when, on what, with which link), the stretches over WARN_N, the held parts, the
    level (stop at STOP_N, warn at WARN_N), the trace for a chart."""
    peaks, over = {}, []
    for s in ("left", "right"):
        rows = [r for r in trace if s in r]
        if rows:
            top = max(rows, key=lambda r: r[s]["n"])
            peaks[s] = dict(n=top[s]["n"], t=top["t"], part=top[s]["part"], link=top[s]["link"])
        cur = None
        for r in trace:
            f = (r.get(s) or {}).get("n", 0.0)
            if f >= WARN_N:
                if cur is None:
                    cur = dict(side=s, t0=r["t"], t1=r["t"], n=f, part=r[s]["part"], link=r[s]["link"])
                cur["t1"] = r["t"]
                if f > cur["n"]:
                    cur.update(n=f, part=r[s]["part"], link=r[s]["link"])
            elif cur is not None:
                over.append(cur)
                cur = None
        if cur is not None:
            over.append(cur)
    top = max([p["n"] for p in peaks.values()] or [0.0])
    untipped = [p for p in placed if not p["tipped"]]
    sustained = 0.0                                  # the longest stretch over STOP_N, either arm
    for s in ("left", "right"):
        run = 0.0
        for a, b in zip(trace, trace[1:] + [None]):
            if (a.get(s) or {}).get("n", 0.0) >= STOP_N:
                run += (b["t"] - a["t"]) if b is not None else SAMPLE_S
                sustained = max(sustained, run)
            else:
                run = 0.0
    level = "stop" if sustained >= SUSTAIN_S - 1e-9 else "warn" if top >= WARN_N or held or untipped else "note"
    return dict(task=task, part=fx.part, level=level, peak_n=round(top, 1), over_stop_s=round(sustained, 2), peaks=peaks, over=sorted(over, key=lambda o: o["t0"]),
                held=held, placed=placed, trace=trace, open_deg=round(fx.open, 2), wall=fx.wall, balance_deg=round(fx.balance, 1),
                shift_mm=[float(v) for v in shift_mm], seconds=round(seconds, 2),
                model=dict(glove_mm=GLOVE_MM, n_per_mm=GLOVE_N_PER_MM, kg=fx.kg, warn_n=WARN_N, stop_n=STOP_N))


# ── the plan's own clock (a task on its own, and the preview of a config) ────────────────────────────────────────────

def fixture_updates(task: str, plan=None, cfg_path=None) -> list:
    """[(plan entry index, deg)]: where the task's config puts the moving part after each of its moves
    (update_fixture_scene), by the move it follows (the plan's entries are the config's moves in order: alix_sync)."""
    import yaml

    from .. import vendor_paths
    from . import alix_sync, mechanism

    plan = plan or mechanism.cached(task)
    part = TASK_PART[task]
    raw = json.loads(plan.path.read_text(encoding="utf-8"))
    root = vendor_paths.ALIX
    cfg = yaml.safe_load(Path(cfg_path or root / alix_sync.CONFIGS_REL / f"{mechanism.TASKS[task].get('variant_of', task)}.yaml").read_text())
    flags = {k: v for k, v in (raw.get("task_flags") or {}).items() if isinstance(v, bool)}
    tp, defaults = cfg.get("task_parameters") or {}, alix_sync._defaults(root)
    out, moves = [], 0
    for st in cfg.get("stage_sequence") or []:
        if st.get("conditional"):
            k = st.get("condition_param")
            if not flags.get(k, tp.get(k, defaults.get(k, alix_sync.PLANNER_DEFAULTS.get(k)))):
                continue
        name = str(st.get("stage"))
        if name.startswith("update_fixture"):
            deg = st.get(f"{part}_angle_deg")
            if deg is not None and moves:
                out.append((moves - 1, float(deg)))
        elif "check" not in st and "hold_time" not in st and not name.startswith(alix_sync.NON_MOTION):
            moves += 1
    return out


def plan_samples(task: str, plan=None, dt: float = SAMPLE_S) -> tuple:
    """The plan's entries one after the other (as ALIX made it): (times, arms, entry end times). Each arm stands at its
    first move's first line until then (the MoveJ to the plan's start), its pole and hand where the plan's entries set them."""
    from . import mechanism, staging

    plan = plan or mechanism.cached(task)
    entries = list(plan.entries)
    first = {}
    for e in entries:
        if e.kind == "arm" and e.side not in first:
            first[e.side] = [v / 1000.0 for v in e.rows[0]]
    state = {s: dict(q=first.get(s, [0.0] * 7), pole=None, hand=[1000] * 6) for s in ("left", "right")}
    for e in entries:                                # the pole each arm starts at: its first pole entry
        if e.kind == "pole" and state[e.side]["pole"] is None:
            state[e.side]["pole"] = round(e.last()[0] * 1000.0, 1)
    for s in state:
        state[s]["pole"] = state[s]["pole"] if state[s]["pole"] is not None else 150.0
    segs, clock, ends = [], 0.0, []
    for e in entries:
        st = state[e.side]
        if e.kind == "arm":
            rows = np.asarray(e.rows, dtype=float) / 1000.0
            segs.append((e.side, clock, clock + e.seconds, rows, st["pole"], list(st["hand"])))
            st["q"] = rows[-1].tolist()
            dur = e.seconds
        elif e.kind == "pole":
            mm = round(e.last()[0] * 1000.0, 1)
            dur = staging.pole_travel_s(st["pole"], mm)
            segs.append((e.side, clock, clock + dur, None, mm, list(st["hand"])))
            st["pole"] = mm
        else:
            vals = [int(c) if c is not None else int(h) for c, h in zip(e.hand_counts(), st["hand"])]
            dur = staging.hand_move_s(st["hand"], vals)
            segs.append((e.side, clock, clock + dur, None, st["pole"], vals))
            st["hand"] = vals
        clock += dur
        ends.append(clock)
    times = np.arange(0.0, clock + dt, dt)
    arms = {}
    for s in ("left", "right"):
        q = np.tile(np.asarray(first.get(s, [0.0] * 7), dtype=float), (len(times), 1))
        pole = np.full(len(times), 150.0)
        hand = np.tile(np.array([1000] * 6, dtype=float), (len(times), 1))
        init_pole = next((p for side, _, _, rows, p, _ in segs if side == s), 150.0)
        pole[:] = init_pole
        for side, t0, t1, rows, p, h in segs:
            if side != s:
                continue
            after = times >= t0 - 1e-9
            pole[after] = p
            hand[after] = h
            if rows is not None:
                idx = np.clip(np.round((times - t0) / mechanism.CLOCK_S).astype(int), 0, len(rows) - 1)
                q[after] = rows[idx[after]]
        arms[s] = dict(pole=pole.tolist(), hand=hand.tolist(), q=q.tolist())
    return times.tolist(), arms, ends


def check_plan(task: str, back_mm: float | None = None, offset_m=(0.0, 0.0), shift_mm=None, home=None,
               fid: str = "commode_c", cubicle_mm: int = 1400, progress=None, plan_path=None, cfg_path=None) -> dict:
    """``task``'s plan on its own clock through ``simulate`` (a task run on its own, and the plans' preview); ``plan_path``
    and ``cfg_path`` another plan of it and its config (a kept one: tests/data/contact)."""
    from . import mechanism

    plan = mechanism.Plan(task, plan_path) if plan_path is not None else mechanism.cached(task)
    times, arms, ends = plan_samples(task, plan)
    updates = [(ends[i], deg) for i, deg in fixture_updates(task, plan, cfg_path) if i < len(ends)]
    out = simulate(task, times, arms, home, fid, back_mm, offset_m, cubicle_mm, updates, shift_mm, progress=progress)
    out["stages"] = [dict(i=e.i, side=e.side, stage=e.stage, t1=round(t, 3)) for e, t in zip(plan.entries, ends)]
    return out


def stage_at(stages: list, t: float, side: str | None = None) -> str:
    """The plan entry running at ``t`` (its side's when given): 'LEFT execute_path'."""
    t0 = 0.0
    for st in stages:
        if t0 - 1e-9 <= t <= st["t1"] + 1e-9 and (side is None or st["side"] == side):
            return f"{st['side'].upper()} {st['stage']}"
        t0 = st["t1"]
    return ""


def words(out: dict, title: str, at=None) -> str:
    """One finding's text: the peak, where, the level's rule; ``at(t, side)`` names the stage then."""
    if not out["peaks"]:
        return f"{title}: the hands do not touch the commode in the model (the glove {GLOVE_MM:g} mm)."
    parts = []
    for s, p in sorted(out["peaks"].items(), key=lambda kv: -kv[1]["n"]):
        where = at(p["t"], s) if at else ""
        parts.append(f"{s.upper()} {p['n']:.0f} N on the {p['part']} ({p['link']}" + (f", {where}" if where else "") + f", {p['t']:.1f} s)")
    text = f"{title}: the hands press up to " + "; ".join(parts) + "."
    if out["over"]:
        text += " Over " + f"{WARN_N:g} N: " + "; ".join(
            f"{o['side'].upper()} {o['t0']:.1f}-{o['t1']:.1f} s up to {o['n']:.0f} N" + (f" ({at(o['t0'], o['side'])})" if at else "")
            for o in out["over"][:6]) + "."
    for p in out["placed"]:
        if not p["tipped"]:
            text += (f" The carry does not take the {out['part']} past its balance ({out['balance_deg']:g} deg) in the model: it "
                     f"stays at {p['was']:g} deg, where the plan's config has it at {p['deg']:g} deg at {p['t']:.1f} s.")
    for h in out["held"]:
        text += (f" The {out['part']} does not come down to {h['want_deg']:g} deg where the plan's config puts it: held at "
                 f"{h['at_deg']:g} deg by the {h['side'].upper()} hand ({h['link']}) at {h['t']:.1f} s.")
    text += (f" A sensible contact takes a few newtons (the 29 Oct hold: 6 to 8 N); Lite refuses Real over {STOP_N:g} N for "
             f"{SUSTAIN_S:g} s or more (here {out['over_stop_s']:g} s; model: glove {GLOVE_MM:g} mm at {GLOVE_N_PER_MM:g} N/mm, the commode "
             f"{out['shift_mm']} mm off ALIX's place, the {out['part']} {PART_KG[out['part']]:g} kg, "
             f"open to {out['open_deg']:g} deg" + (" at the wall" if out["wall"] else "") + ").")
    return text


_GATE: dict = {}


def gate(task: str, job=None) -> dict | None:
    """A lid or seat closing run on its own in Real (mechanism.run): its plan on its own clock, ALIX's room (the wall
    not known here; S2 uses the project's), checked in the worker process once per plan file. Raises UserError
    (code ``contact``) from STOP_N; the result otherwise (None for a task not checked, or with LITE_CONTACT_CHECK=0)."""
    import hashlib
    import os

    from . import mechanism, workers
    from .jobs import UserError

    if task_part(task) is None or os.environ.get("LITE_CONTACT_CHECK") == "0":
        return None
    plan = mechanism.cached(task)
    key = (task, hashlib.sha256(plan.path.read_bytes()).hexdigest())
    if key not in _GATE:
        if job is not None:
            job.progress(f"{plan.title}: checking the hands' contact forces (the plan's first run here)")
        fut = workers.pool().submit(check_plan, task)
        while not fut.done():
            if job is not None:
                job.sleep(0.2)
            else:
                time.sleep(0.2)
        _GATE[key] = fut.result()
    out = _GATE[key]
    if out["level"] == "stop":
        at = lambda t, s, st=out["stages"]: stage_at(st, t, s)          # noqa: E731
        raise UserError("Not run: " + words(out, plan.title, at), "Edit the path in ALIX so the hands press less (tools/"
                        "contact_check.py says by how much before re-planning), re-plan it, then run it again.", code="contact")
    return out
