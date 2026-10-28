"""Lite's lid and seat tasks and plans kept the same as ALIX's (the user, 28 Oct: "make sure the lid and seat tasks and
plans are synchronised with the alix_ws, any change in either should be reflected in the other").

What is compared: every file Lite vendors for the lid and the seat (the plans with their 5 ms tracks, the re-lift rungs,
the task configs, the check policy, the gap check's parameters: vendor/MANIFEST.json's entries under those paths), plus
any plan or rung ALIX holds for one of Lite's tasks that Lite has not vendored. Each file three ways: ALIX's copy, Lite's
copy and the manifest's sha256 (the file as it was when last copied, the common ancestor), so each difference says which
side changed: ALIX (pull it into Lite), Lite (push it into ALIX), both (a conflict, a person decides).

What is checked besides (on either tree, ALIX's or Lite's vendored copy):
- each plan is its config's: the config's stages in order (its conditions evaluated with the plan's own task flags, then
  the config's task parameters, then ALIX's defaults) are the plan's entries, checks and holds in order; each move to a
  pose of the arm's block ends there (forward kinematics of the block's ik_frame, ALIX's URDF), each move to a named
  state ends at the SRDF's values, each stroke stays on its sequence's polyline (from the start pose, as the planner
  prepends that leg; its corners rounded by a few mm) and ends at its last point. A config edited and its re-planning failed
  (the planner then leaves the old plan in place: 28 Oct, twice) shows here;
- a sequence plan (``<task>_seq``) is its task's plan less the checks its flag drops, the same moves;
- the pre-planned re-lifts chain from where the lift ends (mechanism.Plan.rungs_note);
- Lite's registry: every task has its plan (and its config), every catalogue row names a task, the commode's states are
  known before and after each, the vendor tool lists every file (else a full re-vendor drops it).

Nothing here moves a robot or plans anything: re-planning stays the owner's (ALIX's bringup and planner).
"""
from __future__ import annotations

import hashlib
import importlib.util
import json
import math
import os
import shutil
import subprocess
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path

from .. import vendor_paths

LITE = vendor_paths.LITE_ROOT
VENDOR = "vendor/alix"
DEFAULT_ALIX = Path.home() / "alix_ws" / "src" / "alix"
PLANS_REL = "Resource/plans/commode_c/hardware"
CONFIGS_REL = "ros/orchestration/alix_tasks/config/mechanism_tasks/commode_c"
#: the lid and seat files, by their path in ALIX (the vendor tool's section for them)
PREFIXES = (f"{PLANS_REL}/", "Resource/plans/README.md", f"{CONFIGS_REL}/",
            "ros/orchestration/alix_tasks/config/mechanism_tasks/task_parameters_defaults.yaml",
            "ros/orchestration/alix_tasks/scripts/lift_check.py", "ros/perception/alix_perception/config/perception.yaml")
#: ALIX's planner defaults for the condition flags it knows by name (config_loader.hpp: use_prestart_stage true)
PLANNER_DEFAULTS = {"use_prestart_stage": True}
POSE_MM, POSE_DEG, POINT_MM, STATE_DEG, POLE_MM = 3.0, 1.5, 5.0, 0.1, 1.0
STROKE_DEV_MM = 6.0       #: a stroke's samples off its config's polyline (the planner's own "deviation from polyline": <= 3 mm)


def _to_segment(X, a, b):
    """Each row of X (n x 3)'s distance to the segment a-b (metres)."""
    import numpy as np

    ab = b - a
    L2 = float(ab @ ab)
    t = np.zeros(len(X)) if L2 < 1e-12 else np.clip((X - a) @ ab / L2, 0.0, 1.0)
    return np.linalg.norm(X - (a + t[:, None] * ab), axis=1)
SEQ_DEG = 0.5            #: a sequence plan's moves end within this of its task's (the plannings' IK spread: 0.06-0.23 deg)


def alix_root() -> Path | None:
    """ALIX's checkout on this computer (LITE_ALIX_ROOT, else ~/alix_ws/src/alix), or None (the lab: Lite runs alone)."""
    p = Path(os.environ.get("LITE_ALIX_ROOT") or DEFAULT_ALIX)
    return p if (p / PLANS_REL).is_dir() else None


def _sha(p: Path) -> str | None:
    return hashlib.sha256(p.read_bytes()).hexdigest() if p.is_file() else None


def _tasks():
    from . import mechanism

    return mechanism.TASKS


def _ours(rel: str) -> bool:
    return rel.startswith(PREFIXES) and not rel.endswith((".pyc",))


def _task_files(root: Path, names) -> set:
    """The plan files (and rungs) of ``names`` under ``root`` (an ALIX tree or Lite's vendor/alix), as ALIX paths."""
    out = set()
    hw = root / PLANS_REL
    for n in names:
        for p in list(hw.glob(f"{n}_toppra.*")) + list((hw / "rungs").glob(f"{n}_toppra.rung*")):
            out.add(p.relative_to(root).as_posix())
    return out


@dataclass
class FileState:
    path: str              # its path in ALIX
    state: str             # same | alix | lite | conflict | new_alix | new_lite | gone_alix | gone_lite | manifest
    alix: str | None = None
    lite: str | None = None
    base: str | None = None

    def words(self) -> str:
        return {"same": "the same", "alix": "changed in ALIX: pull", "lite": "changed in Lite: push",
                "conflict": "changed in both: a person decides (pull --take alix, or push --take lite)",
                "new_alix": "new in ALIX for one of Lite's tasks: pull", "new_lite": "in Lite only: push",
                "gone_alix": "removed in ALIX: pull removes Lite's copy", "gone_lite": "removed from Lite: pull restores it",
                "manifest": "the same on both sides, the manifest's record behind: pull records it"}[self.state]


@dataclass
class Report:
    alix: Path | None
    files: list = field(default_factory=list)       # FileState, the ones that differ
    checks: list = field(default_factory=list)      # (level, where, text): level error | warn | note
    same: int = 0

    @property
    def in_sync(self) -> bool:
        return not self.files and not any(lv == "error" for lv, _, _ in self.checks)

    def lines(self) -> list:
        out = []
        if self.alix is None:
            out.append("ALIX's checkout is not on this computer (LITE_ALIX_ROOT): only Lite's own copies are checked.")
        else:
            out.append(f"ALIX: {self.alix}; {self.same} lid and seat file(s) the same on both sides.")
        for f in self.files:
            out.append(f"  {f.state.upper():9s} {f.path}: {f.words()}")
        for lv, where, text in self.checks:
            out.append(f"  {lv.upper():9s} {where}: {text}")
        out.append("IN SYNC" if self.in_sync else "NOT IN SYNC")
        return out


def _manifest(lite: Path) -> dict:
    return json.loads((lite / "vendor" / "MANIFEST.json").read_text(encoding="utf-8"))


def file_states(alix: Path, lite: Path = LITE) -> tuple[list, int]:
    """Every lid and seat file three ways (FileState for those that differ, how many are the same)."""
    man = _manifest(lite)
    base = {}
    for f in man["files"]:
        d = f["dest"]
        if d.startswith(VENDOR + "/") and _ours(d[len(VENDOR) + 1:]):
            base[d[len(VENDOR) + 1:]] = f.get("sha256_after_patch") or f["sha256"]
    names = list(_tasks())
    paths = set(base) | _task_files(alix, names) | _task_files(lite / VENDOR, names)
    out, same = [], 0
    for rel in sorted(paths):
        a, li, b = _sha(alix / rel), _sha(lite / VENDOR / rel), base.get(rel)
        if a is not None and a == li:
            if a == b:
                same += 1
            else:
                out.append(FileState(rel, "manifest", a, li, b))
            continue
        if b is None:
            state = "new_alix" if li is None else "new_lite" if a is None else "conflict"
        elif a is None and li is None:
            continue                                           # gone from both: the manifest's line is dropped by pull
        elif a is None:
            state = "gone_alix" if li == b else "conflict"
        elif li is None:
            state = "gone_lite" if a == b else "conflict"
        elif li == b:
            state = "alix"
        elif a == b:
            state = "lite"
        else:
            state = "conflict"
        out.append(FileState(rel, state, a, li, b))
    return out, same


# ── each plan against its config ──────────────────────────────────────────────────────────────────────────────────

NON_MOTION = ("setup_", "current_state", "allow_", "forbid_", "update_fixture")


def _defaults(root: Path) -> dict:
    import yaml

    p = root / "ros/orchestration/alix_tasks/config/mechanism_tasks/task_parameters_defaults.yaml"
    try:
        tp = (yaml.safe_load(p.read_text()) or {}).get("task_parameters") or {}
    except OSError:
        tp = {}
    return {k: v for k, v in tp.items() if isinstance(v, bool)}


def config_stages(cfg: dict, flags: dict, defaults: dict) -> list:
    """The config's stages the plan is made of, in order: ("move", stage dict) for each stage that plans a move,
    ("check", name) for a check, ("hold", name) for a dwell; its conditions as the planner evaluates them."""
    tp = cfg.get("task_parameters") or {}
    out = []
    for st in cfg.get("stage_sequence") or []:
        if st.get("conditional"):
            k = st.get("condition_param")
            v = flags.get(k, tp.get(k, defaults.get(k, PLANNER_DEFAULTS.get(k))))
            if not v:
                continue
        name = str(st.get("stage"))
        if "check" in st:
            out.append(("check", name))
        elif "hold_time" in st:
            out.append(("hold", name))
        elif not name.startswith(NON_MOTION):
            out.append(("move", st))
    return out


def plan_stages(plan: dict) -> list:
    """The plan's entries, checks and holds in order: ("move", sub_trajectory), ("check", name), ("hold", name)."""
    out = []
    for t in plan.get("sub_trajectories") or []:
        for pa in t.get("pre_actions") or []:
            out.append(("check" if pa.get("type") == "check_fixture_state" else "hold", pa.get("stage")))
        out.append(("move", t))
    return out


def _side(joint: str) -> str:
    return "left" if joint.startswith(("L_", "ILH_")) else "right"


_GROUP_STATES: dict = {}


def _group_states(root: Path) -> dict:
    """{(group, state): {joint: value}} from the SRDF of ``root``."""
    key = str(root)
    if key not in _GROUP_STATES:
        out = {}
        srdf = root / "ros/manipulation/alix_moveit_config/config/alix.srdf"
        if srdf.is_file():
            for gs in ET.parse(srdf).getroot().iter("group_state"):
                out[(gs.get("group"), gs.get("name"))] = {j.get("name"): float(j.get("value")) for j in gs.iter("joint")}
        _GROUP_STATES[key] = out
    return _GROUP_STATES[key]


def _fk():
    import numpy as np

    from . import robot_fk

    m = robot_fk.model()
    T_ref_inv = np.linalg.inv(m.fk(m.chain(m.root_link(), "alix_ref_frame"), []))

    def fk(side: str, frame: str, q7, pole_m: float):
        ch = m.chain(m.root_link(), frame)
        pfx = "L" if side == "left" else "R"
        cols = []
        for j in m.movable_joints(ch):
            if j.name == f"{pfx}_sliding_plate_joint":
                cols.append(np.full(len(q7), pole_m))
            elif j.name.startswith(f"{pfx}_joint"):
                cols.append(np.asarray(q7)[:, int(j.name[len(f"{pfx}_joint"):]) - 1])
            else:
                raise ValueError(f"the chain to {frame} moves {j.name}")
        return T_ref_inv @ m.fk_batch(ch, np.stack(cols, axis=1))

    return fk


def _pose_T(p7):
    import numpy as np
    from scipy.spatial.transform import Rotation as Rot

    T = np.eye(4)
    T[:3, 3] = p7[:3]
    T[:3, :3] = Rot.from_quat(p7[3:]).as_matrix()
    return T


def _rel_T(start7, rel: dict):
    import numpy as np
    from scipy.spatial.transform import Rotation as Rot

    T = np.eye(4)
    T[:3, 3] = np.asarray(start7[:3]) + np.asarray(rel.get("translation") or [0, 0, 0])
    T[:3, :3] = Rot.from_euler("XYZ", rel.get("rotation") or [0, 0, 0], degrees=True).as_matrix() @ Rot.from_quat(start7[3:]).as_matrix()
    return T


def _err(Ta, Tb) -> tuple:
    import numpy as np
    from scipy.spatial.transform import Rotation as Rot

    return (float(np.linalg.norm(Ta[:3, 3] - Tb[:3, 3]) * 1000.0),
            float(np.degrees(Rot.from_matrix(Ta[:3, :3] @ Tb[:3, :3].T).magnitude())))


def plan_against_config(plan: dict, cfg: dict, root: Path, what: str) -> list:
    """Why ``plan`` is not ``cfg``'s, as (level, where, text) rows; [] when it is."""
    import numpy as np

    rows = []
    flags = {k: v for k, v in (plan.get("task_flags") or {}).items() if isinstance(v, bool)}
    want, have = config_stages(cfg, flags, _defaults(root)), plan_stages(plan)
    names = lambda seq: [(k, x if isinstance(x, str) else str(x.get("stage") or x.get("stage_name"))) for k, x in seq]   # noqa: E731
    if names(want) != names(have):
        k = next((i for i, (a, b) in enumerate(zip(names(want), names(have))) if a != b), min(len(want), len(have)))
        a = names(want)[k] if k < len(want) else ("nothing", "")
        b = names(have)[k] if k < len(have) else ("nothing", "")
        rows.append(("error", what, f"the plan is not its config's: at step {k + 1} the config has {a[0]} {a[1]}, the plan "
                                     f"{b[0]} {b[1]} (the config edited after the plan, or its re-planning failed and left "
                                     "the old plan): re-plan it in ALIX"))
        return rows
    fk = _fk()
    groups = _group_states(root)
    pole = {"left": None, "right": None}
    for g in plan.get("track_groups") or []:
        pm = (g.get("pre_stroke") or {}).get("pole_m")
        if pm is not None:
            pole = {s: pole[s] if pole[s] is not None else float(pm) for s in pole}
    worst = {}
    for (kind, st), (_, t) in zip(want, have):
        if kind != "move":
            continue
        joints, wps = t.get("joint_names") or [], t.get("waypoints") or []
        if not joints or not wps:
            continue
        side = _side(joints[0])
        last = wps[-1]["positions"]
        pose_name, group = st.get("pose_name"), st.get("planning_group")
        stage = str(st.get("stage"))
        if "sliding_plate" in joints[0]:
            pole[side] = float(last[0])
            target = groups.get((group, pose_name), {})
            if joints[0] in target and abs(target[joints[0]] - last[0]) * 1000 > POLE_MM:
                rows.append(("error", what, f"{stage} ({side}) ends {last[0] * 1000:.1f} mm, its config's {pose_name} is "
                                            f"{target[joints[0]] * 1000:.1f} mm: re-plan it in ALIX"))
            continue
        if joints[0].startswith(("ILH_", "IRH_")):
            continue
        block = cfg.get(side) or {}
        poses = block.get("cartesian_poses") or {}
        frame = (block.get("task_parameters") or {}).get("ik_frame")
        q = np.asarray([w["positions"] for w in wps], dtype=float)
        pm = pole[side] if pole[side] is not None else 0.15
        if stage == "execute_path":
            seq = block.get("cleaning_sequence") or []
            pts = block.get("cleaning_points") or {}
            order = [p for pair in seq for p in pair]
            if not frame or "start_pose" not in poses or not order:
                continue
            if any(p not in pts for p in order):
                continue
            X = fk(side, frame, q, pm)[:, :3, 3]
            start = _pose_T(poses["start_pose"])[:3, 3]
            V = [start] + [_rel_T(poses["start_pose"], pts[p])[:3, 3] for p in order[:1]]    # the planner's leg from the start
            segs = [(V[0], V[1])] + [(_rel_T(poses["start_pose"], pts[a])[:3, 3], _rel_T(poses["start_pose"], pts[b])[:3, 3])
                                     for a, b in seq]
            dev = np.min(np.stack([_to_segment(X, a, b) for a, b in segs], axis=1), axis=1) * 1000.0
            k = int(np.argmax(dev))
            end = float(np.linalg.norm(X[-1] - _rel_T(poses["start_pose"], pts[order[-1]])[:3, 3])) * 1000.0
            worst[(side, stage)] = max(float(dev[k]), end)
            if dev[k] > STROKE_DEV_MM or end > POINT_MM:
                rows.append(("error", what, f"the {side} stroke leaves its config's path by {dev[k]:.1f} mm (the planner rounds its "
                                            f"corners by a few mm) and ends {end:.1f} mm from its last point {order[-1]}: the "
                                            "config's points changed after the plan: re-plan it in ALIX"))
            continue
        if pose_name in poses and frame:
            p = poses[pose_name]
            Tt = _pose_T(p) if isinstance(p, list) else _rel_T(poses["start_pose"], p)
            mm, deg = _err(fk(side, frame, q[-1:], pm)[0], Tt)
            if mm > POSE_MM or deg > POSE_DEG:
                rows.append(("error", what, f"{stage} ({side}) ends {mm:.1f} mm, {deg:.2f} deg from its config's {pose_name}: "
                                            "the config's pose changed after the plan: re-plan it in ALIX"))
            continue
        target = groups.get((group or f"rm75_arm_{side}", pose_name)) if pose_name else None
        if target:
            off = max((abs(target[j] - v) for j, v in zip(joints, last) if j in target), default=0.0)
            if math.degrees(off) > STATE_DEG:
                rows.append(("error", what, f"{stage} ({side}) ends {math.degrees(off):.2f} deg from the SRDF's {pose_name}: "
                                            "re-plan it in ALIX (or the SRDF changed: tools/sync_srdf_group_states.py)"))
    return rows


def seq_against_task(seq: dict, task: dict, cfg: dict, what: str) -> list:
    """Why a sequence plan is not its task's plan less the dropped checks, as rows; [] when it is. An arm move ending more
    than SEQ_DEG away in its joints but at the same hand pose (the arm's other configuration about its hand: the planner's
    IK on another bringup) is a warning; a hand pose apart is an error."""
    import numpy as np

    a = [(k, x if isinstance(x, str) else x.get("stage_name")) for k, x in plan_stages(seq)]
    b = [(k, x if isinstance(x, str) else x.get("stage_name")) for k, x in plan_stages(task)]
    dropped = [x for x in b if x not in a and x[0] == "check"]
    if [x for x in b if x not in dropped] != a:
        return [("error", what, f"its moves are not its task's plan's ({task.get('timestamp_human')}): re-plan the "
                                "sequence plan in ALIX with the task's config (its flag false)")]
    rows, fk = [], None
    for x, y in zip(seq["sub_trajectories"], task["sub_trajectories"]):
        joints = x.get("joint_names") or []
        if not joints or not joints[0][:2] in ("L_", "R_") or "sliding_plate" in joints[0]:
            continue
        qa, qb = x["waypoints"][-1]["positions"], y["waypoints"][-1]["positions"]
        d = max(math.degrees(abs(u - v)) for u, v in zip(qa, qb))
        if d <= SEQ_DEG:
            continue
        side = _side(joints[0])
        frame = ((cfg.get(side) or {}).get("task_parameters") or {}).get("ik_frame")
        mm = deg = None
        if frame:
            fk = fk or _fk()
            mm, deg = _err(fk(side, frame, np.asarray([qa]), 0.15)[0], fk(side, frame, np.asarray([qb]), 0.15)[0])
        j = max(range(len(qa)), key=lambda k: abs(qa[k] - qb[k]))
        stamp = f"(the sequence plan {seq.get('timestamp_human')}, its task's {task.get('timestamp_human')})"
        if mm is not None and mm <= POSE_MM and deg <= POSE_DEG:
            rows.append(("warn", what, f"the {side} {x['stage_name']} ends at the same hand pose ({mm:.1f} mm, {deg:.2f} deg) on "
                                       f"another arm configuration: joint {j + 1} {d:.1f} deg apart {stamp}; the plannings' IK "
                                       "took another branch: re-plan one from the other's start state if they should be one"))
        else:
            rows.append(("error", what, f"the {side} {x['stage_name']} ends {d:.1f} deg (joint {j + 1}) from its task's plan's"
                                        + (f", the hand {mm:.1f} mm, {deg:.2f} deg apart" if mm is not None else "")
                                        + f" {stamp}: one was re-planned alone: re-plan the other in ALIX"))
    return rows


def plan_checks(root: Path, where: str) -> list:
    """Each of Lite's plans under ``root`` (ALIX's tree or Lite's vendor/alix) against its config and its task's plan."""
    import yaml

    from . import mechanism

    rows = []
    for name, info in mechanism.TASKS.items():
        task = info.get("variant_of", name)
        p = root / PLANS_REL / f"{name}_{mechanism.MODE}.json"
        c = root / CONFIGS_REL / f"{task}.yaml"
        if not p.is_file():
            rows.append(("error", f"{where} {name}", f"Lite offers {info['title']} and {where} has no plan {p.name}"))
            continue
        if not c.is_file():
            rows.append(("error", f"{where} {name}", f"{where} has no config {c.name} for it"))
            continue
        plan = json.loads(p.read_text(encoding="utf-8"))
        try:
            rows += plan_against_config(plan, yaml.safe_load(c.read_text()), root, f"{where} {p.name}")
        except Exception as exc:                                                    # noqa: BLE001 - said, not hidden
            rows.append(("error", f"{where} {p.name}", f"could not be checked against {c.name} ({type(exc).__name__}: {exc})"))
        if task != name:
            tp = root / PLANS_REL / f"{task}_{mechanism.MODE}.json"
            if tp.is_file():
                rows += seq_against_task(plan, json.loads(tp.read_text(encoding="utf-8")), yaml.safe_load(c.read_text()),
                                         f"{where} {p.name}")
        try:
            note = mechanism.Plan(name, p).rungs_note()
        except Exception as exc:                                                    # noqa: BLE001
            note = f"its plan does not load ({type(exc).__name__}: {exc})"
        if note:
            rows.append(("error", f"{where} {p.name}", note))
    return rows


def registry_checks(lite: Path = LITE, alix: Path | None = None) -> list:
    """Lite's own lists against each other and against ALIX's configs."""
    import yaml

    from . import mechanism, workflow

    rows = []
    tasks = mechanism.TASKS
    for code in mechanism.ROWS:
        for t in mechanism.row_tasks(code):
            if t not in tasks:
                rows.append(("error", "mechanism.ROWS", f"{code} names {t}, which is not one of Lite's tasks"))
    for name, info in tasks.items():
        if info.get("variant_of") and info["variant_of"] not in tasks:
            rows.append(("error", "mechanism.TASKS", f"{name} is a variant of {info['variant_of']}, not one of Lite's tasks"))
        for table, words in ((mechanism.PHASE_BEFORE, "PHASE_BEFORE"), (mechanism.PHASE_AFTER, "PHASE_AFTER")):
            if name not in table:
                rows.append(("error", f"mechanism.{words}", f"{name} has no commode state"))
        if "variant_of" not in info and name not in workflow.HOLDS_AFTER:
            rows.append(("error", "workflow.HOLDS_AFTER", f"{name} has no states it leaves"))
    state = {"closed": "lid_closed", "lid_open": "lid_open", "lid_seat_open": "seat_open"}
    for name, info in tasks.items():
        c = lite / VENDOR / CONFIGS_REL / f"{info.get('variant_of', name)}.yaml"
        if c.is_file():
            tp = (yaml.safe_load(c.read_text()) or {}).get("task_parameters") or {}
            want = state.get(str(tp.get("commode_fixture_type")))
            if want and mechanism.PHASE_BEFORE.get(name) != want:
                rows.append(("error", "mechanism.PHASE_BEFORE", f"{name} starts in {mechanism.PHASE_BEFORE.get(name)}, its config "
                                                                f"says {tp.get('commode_fixture_type')}"))
    listed = set(_vendor_listed())
    for rel in sorted(_task_files(lite / VENDOR, list(tasks)) | {f"{CONFIGS_REL}/{n}.yaml" for n, i in tasks.items() if "variant_of" not in i}):
        if rel not in listed:
            rows.append(("warn", "tools/vendor_from_alix.py", f"{rel} is not in its list: a full re-vendor would drop it"))
    if alix is not None:
        for c in sorted((alix / CONFIGS_REL).glob("*.yaml")):
            try:
                tp = (yaml.safe_load(c.read_text()) or {}).get("task_parameters") or {}
            except Exception:                                                       # noqa: BLE001
                continue
            if tp.get("compound") and tp.get("mechanism_type") in ("lid", "seat") and c.stem not in tasks:
                rows.append(("warn", f"ALIX {c.name}", "a compound lid or seat task Lite does not offer"))
    return rows


def _vendor_listed() -> list:
    spec = importlib.util.spec_from_file_location("lite_vendor_tool", LITE / "tools" / "vendor_from_alix.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return list(mod.ALIX_FILES)


def _uncommitted(alix: Path, paths) -> list:
    try:
        out = subprocess.run(["git", "-C", str(alix), "status", "--porcelain", "--", *paths], capture_output=True, text=True,
                             timeout=30).stdout
    except (OSError, subprocess.SubprocessError):
        return []
    return [line[3:] for line in out.splitlines() if line.strip()]


def status(alix: Path | None = None, lite: Path = LITE, deep: bool = True) -> Report:
    """The lid and seat files and plans of both sides (``alix`` None: ALIX's checkout when on this computer)."""
    alix = alix if alix is not None else alix_root()
    rep = Report(alix)
    if alix is not None:
        rep.files, rep.same = file_states(alix, lite)
    rep.checks += registry_checks(lite, alix)
    if deep:
        rep.checks += plan_checks(lite / VENDOR, "Lite")
        if alix is not None and any(f.state not in ("same", "manifest") for f in rep.files):
            rep.checks += [r for r in plan_checks(alix, "ALIX") if r not in rep.checks]   # what a pull would bring
    if alix is not None:
        mine = sorted(_task_files(alix, list(_tasks())))
        loose = _uncommitted(alix, mine)
        if loose:
            rep.checks.append(("note", "ALIX git", f"{len(loose)} of the plan files Lite runs are not committed in ALIX"))
    return rep


# ── pull and push ──────────────────────────────────────────────────────────────────────────────────────────────────

def _record(lite: Path, alix: Path, rel: str, gone: bool = False) -> None:
    path = lite / "vendor" / "MANIFEST.json"
    man = json.loads(path.read_text(encoding="utf-8"))
    dest = f"{VENDOR}/{rel}"
    files = [f for f in man["files"] if f["dest"] != dest]
    if not gone:
        src = alix / rel
        e = dict(dest=dest, source=str(src), origin="alix", sha256=_sha(lite / dest), bytes=(lite / dest).stat().st_size)
        at = next((i for i, f in enumerate(files) if f["dest"] > dest), len(files))
        files.insert(at, e)
    man["files"] = files
    path.write_text(json.dumps(man, indent=2), encoding="utf-8")


def pull(alix: Path, lite: Path = LITE, take_alix: bool = False, dry_run: bool = False) -> list:
    """ALIX -> Lite: every file changed or new in ALIX (with ``take_alix`` the conflicts too), the ones removed in ALIX
    removed from Lite, the ones removed from Lite restored, the manifest's record brought up. [(path, what was done)]."""
    states, _ = file_states(alix, lite)
    done = []
    for f in states:
        if f.state == "conflict" and not take_alix:
            continue
        if f.state in ("lite", "new_lite"):
            continue
        dst = lite / VENDOR / f.path
        if f.state == "gone_alix" or (f.state == "conflict" and f.alix is None):
            if not dry_run:
                dst.unlink(missing_ok=True)
                _record(lite, alix, f.path, gone=True)
            done.append((f.path, "removed from Lite (ALIX removed it)"))
            continue
        if not dry_run:
            dst.parent.mkdir(parents=True, exist_ok=True)
            if f.state != "manifest":
                shutil.copy2(alix / f.path, dst)
            _record(lite, alix, f.path)
        done.append((f.path, {"manifest": "recorded", "gone_lite": "restored from ALIX"}.get(f.state, "copied from ALIX")))
    if done and not dry_run:
        from . import mechanism

        mechanism.cached.cache_clear()
    return done


def push(alix: Path, lite: Path = LITE, take_lite: bool = False, dry_run: bool = False) -> list:
    """Lite -> ALIX: every file changed in Lite or in Lite only (with ``take_lite`` the conflicts too), the manifest's
    record brought up. Nothing is removed from ALIX (a file removed from Lite is said; pull restores it). [(path, what)]."""
    states, _ = file_states(alix, lite)
    done = []
    for f in states:
        if f.state not in ("lite", "new_lite") and not (take_lite and f.state == "conflict" and f.lite is not None):
            continue
        if not dry_run:
            dst = alix / f.path
            dst.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(lite / VENDOR / f.path, dst)
            _record(lite, alix, f.path)
        done.append((f.path, "copied into ALIX"))
    return done
