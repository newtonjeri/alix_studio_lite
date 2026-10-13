"""Task-config loading for mode 1 — ROS-free.

Loads a task YAML plus its category defaults and merges them the way the task
tree itself defines: task values override category defaults, and the arm side
comes from the planning group or the task name.

Deliberately alix-side and standalone rather than importing RM_API2's
`task_config`: that module resolves geometry against the butterfli URDF and
lives in a tree sessions do not edit (contract C16.1).
"""

from __future__ import annotations

import pathlib
import re
from typing import Dict, Optional

import yaml

# Category layouts in the alix task tree (contract C11-4b).
_LAYOUTS = (
    "{cat}/{fixture}/{task}_cleaning_points.yaml",   # cleaning
    "{cat}/{fixture}/{task}.yaml",                   # mechanism
    "{cat}/{task}.yaml",                             # bin / glove (flat)
)


def _deep_merge(base: dict, over: dict) -> dict:
    out = dict(base or {})
    for k, v in (over or {}).items():
        out[k] = _deep_merge(out[k], v) if isinstance(v, dict) and isinstance(out.get(k), dict) else v
    return out


#: A SECTION config's stem (plan/P17 L5): ``<region>_s<k>of<N>[x<tag>]_<arm>``,
#: sealed or not -- the same net as the executive's `section_plans.SECTION_LIKE`.
_SECTION_STEM = re.compile(r"_s\d+of\d+(?:x[0-9A-Za-z]*)?_(?:left|right)(?:_|$)")


def section_refusal(task: str, doc: Optional[dict]) -> Optional[str]:
    """Why *task* must not be dispatched, when it is ONE SECTION of a sectioned
    region (plan/P17 L5; review S2, 2026-09-29), else None.

    A section is 1 of N configs that clean the region only TOGETHER, in order,
    each a stop, a retreat and a re-seed; a sealed set runs through
    `arm_task_executive <region>_<arm>` only.  Mode 1 dispatches ONE config by
    task name, so a section here would clean part of the region and report the
    task done.  Recognised by its block (``provenance.stroke.section``, written
    on every section config) or by its name (a hand-copied section)."""
    stroke = (((doc or {}).get("provenance") or {}).get("stroke") or {}) if isinstance(doc, dict) else {}
    if isinstance(stroke, dict) and stroke.get("section") is not None:
        sec = stroke["section"] if isinstance(stroke["section"], dict) else {}
        k = sec.get("index")                       # 0-based in the block (path_emission)
        return (f"task {task!r} is section {k + 1 if isinstance(k, int) else '?'} of {sec.get('of', '?')} of a SECTIONED "
                "region (provenance.stroke.section): alix_dispatch has no section-set route (plan/P17 L5) "
                "-- run the whole sealed set with arm_task_executive <region>_<arm>")
    if _SECTION_STEM.search(str(task).split("[")[0]):
        return (f"task {task!r} names ONE section of a SECTIONED region: alix_dispatch has no section-set "
                "route (plan/P17 L5) -- run the whole sealed set with arm_task_executive <region>_<arm>")
    return None


class TaskConfig:
    """A merged task config, with the accessors the dispatcher needs."""

    def __init__(self, task: str, doc: dict, defaults: dict, source: str):
        # plan/P17 L5: a section is never dispatched as a task (`section_refusal`)
        why = section_refusal(task, doc)
        if why is not None:
            raise ValueError(why)
        self.task = task
        self.doc = doc or {}
        self.source = source
        self.params = _deep_merge((defaults or {}).get("task_parameters", {}),
                                  self.doc.get("task_parameters", {}))
        self._defaults = defaults or {}
        group = self.params.get("primary_planning_group", "")
        if group.endswith("_left") or task.endswith("_left"):
            self.side = "left"
        elif group.endswith("_right") or task.endswith("_right"):
            self.side = "right"
        elif "_dual" in task or self.params.get("compound"):
            self.side = "dual"
        else:
            raise ValueError(f"cannot determine the arm for task {task!r}")
        arm_def = (self._defaults.get("arm_defaults", {})
                   .get(self.side, {}).get("task_parameters", {}))
        self.params = _deep_merge(arm_def, self.params)


    # ── compound (dual-arm) tasks ──
    #
    # A compound config is ONE task that drives both arms: shared
    # `task_parameters` with `compound: true` and an `arms` list, a `right:`
    # and `left:` block each carrying its own task_parameters, cartesian_poses,
    # cleaning_points and cleaning_sequence, and ONE `stage_sequence` whose
    # motion stages each name their `arm`.
    #
    # The sequence is therefore ALREADY SERIALISED — right does its part, then
    # left, then right again. There is no concurrency to invent and no
    # rendezvous to get wrong; the config states the order.

    @property
    def is_compound(self) -> bool:
        return bool(self.params.get("compound")) or self.side == "dual"

    @property
    def arms(self) -> list:
        declared = self.params.get("arms")
        if declared:
            return list(declared)
        return [s for s in ("right", "left") if s in self.doc]

    def arm_view(self, side: str) -> "TaskConfig":
        """The single-arm config this compound task carries for `side`.

        Returned as a TaskConfig so everything downstream — pose lookup, speed
        resolution, the stroke builder — works unchanged on one arm of a
        compound task.
        """
        block = self.doc.get(side)
        if block is None:
            raise KeyError(f"{self.task!r} has no {side!r} block (arms: {self.arms})")
        doc = {
            "task_parameters": _deep_merge(self.doc.get("task_parameters", {}),
                                           block.get("task_parameters", {})),
            "cartesian_poses": block.get("cartesian_poses", {}) or {},
            "cleaning_points": block.get("cleaning_points", {}) or {},
            "cleaning_sequence": block.get("cleaning_sequence", []) or [],
            # The stage sequence stays with the parent: a per-arm view must not
            # look like a standalone task that could be run on its own.
            "stage_sequence": [],
        }
        doc["task_parameters"]["primary_planning_group"] = f"rm75_arm_{side}"
        doc["task_parameters"].pop("compound", None)
        view = TaskConfig(f"{self.task}[{side}]", doc, self._defaults, self.source)
        return view

    # ── identity ──
    @property
    def prefix(self) -> str:
        return "L" if self.side == "left" else "R"

    @property
    def reference_frame(self) -> str:
        return self.params.get("reference_frame", "alix_ref_frame")

    @property
    def ik_frame(self) -> Optional[str]:
        return self.params.get("ik_frame")

    # ── geometry ──
    @property
    def cartesian_poses(self) -> dict:
        return self.doc.get("cartesian_poses", {}) or {}

    @property
    def cleaning_points(self) -> dict:
        return self.doc.get("cleaning_points") or self.doc.get("path_waypoints") or {}

    @property
    def cleaning_sequence(self) -> list:
        return self.doc.get("cleaning_sequence", []) or []

    @property
    def stage_sequence(self) -> list:
        return self.doc.get("stage_sequence", []) or []

    # ── speeds (contract C3; percentages are SDK v-percent semantics) ──
    def speed_pct(self, kind: str, stage: Optional[dict] = None) -> int:
        """Speed for a stage, as an SDK percentage.

        `speed_class` on the stage wins when present (mechanism/bin/glove use
        it); otherwise the per-device scaling from the merged parameters.
        """
        classes = self.params.get("speed_classes", {}) or {}
        if stage and stage.get("speed_class") in classes:
            return int(round(100 * float(classes[stage["speed_class"]])))
        key = {
            "arm": "max_velocity_scaling",
            "pole": "pole_max_velocity_scaling",
            "hand": "hand_max_velocity_scaling",
        }.get(kind, "max_velocity_scaling")
        return int(round(100 * float(self.params.get(key, 1.0))))

    @property
    def cleaning_line_speed_mps(self) -> Optional[float]:
        v = self.params.get("cleaning_line_speed_mps")
        return float(v) if v is not None else None

    @property
    def cleaning_line_acc_mps2(self):
        a = self.params.get("cleaning_line_acc_mps2")
        return float(a) if a is not None else None

    def turn_policy(self) -> Optional[Dict]:
        """The three-field turn policy, or None when the config carries none.

        `cleaning_turn_radius_mm` is the only required field, and it is a
        RADIUS, not a speed: A.2.1 turns a radius into a speed and A.2 turns it
        into a blend `r`. A config that names a turn speed instead would be
        naming an output.

        `cleaning_turn_overtravel_mm` is the permission Newton gave on
        2026-08-21 — with group frames the arm reaches past the lid edge, so a
        row turnaround may leave the workpiece rather than cut its corner off.
        It defaults to 0, which keeps every turn on the surface.
        """
        radius_mm = self.params.get("cleaning_turn_radius_mm")
        if radius_mm is None:
            return None
        cruise = self.params.get("cleaning_cruise_speed_mps")
        speed_cap = self.params.get("cleaning_turn_speed_cap_mps")
        over_mm = self.params.get("cleaning_turn_overtravel_mm") or 0.0
        tol_mm = self.params.get("cleaning_turn_surface_tol_mm")
        return {
            "radius_m": float(radius_mm) / 1000.0,
            "cruise_mps": float(cruise) if cruise is not None
                          else self.cleaning_line_speed_mps,
            "speed_cap_mps": float(speed_cap) if speed_cap is not None else None,
            "overtravel_m": float(over_mm) / 1000.0,
            "a_lat": self.cleaning_line_acc_mps2,
            "surface_tol_m": (None if tol_mm is None else float(tol_mm) / 1000.0),
            "strict_step": bool(self.params.get("cleaning_turn_strict_step", False)),
        }

    @property
    def arc_moves(self) -> list:
        """`arc_moves` spans, or [] — the turnarounds `rm_movec` may trace.

        A span names two points that are ALREADY in `cleaning_sequence` plus a
        `via_translation` that is NOT a waypoint. So the executable path and
        the point count are unchanged (C5), MoveIt still plans the polyline
        (C17), and only the controller collapses the span into one arc.
        """
        return list(self.doc.get("arc_moves") or [])


def load(task: str, config_root: str, fixture: str = "commode_c",
         category: str = "commode_cleaning") -> TaskConfig:
    """Load `task` from `config_root`, trying each known layout in order."""
    root = pathlib.Path(config_root)
    cat_dir = root / category
    defaults_path = cat_dir / "task_parameters_defaults.yaml"
    defaults: Dict = {}
    if defaults_path.is_file():
        defaults = yaml.safe_load(defaults_path.read_text()) or {}
    tried = []
    for pattern in _LAYOUTS:
        p = root / pattern.format(cat=category, fixture=fixture, task=task)
        tried.append(str(p))
        if p.is_file():
            return TaskConfig(task, yaml.safe_load(p.read_text()) or {}, defaults, str(p))
    raise FileNotFoundError("no task config; tried:\n  " + "\n  ".join(tried))


def load_file(path: str, task: Optional[str] = None,
              defaults_path: Optional[str] = None) -> TaskConfig:
    """Load one task file directly (the frozen test fixtures use this)."""
    p = pathlib.Path(path)
    defaults: Dict = {}
    dp = pathlib.Path(defaults_path) if defaults_path else p.parent / "task_parameters_defaults.yaml"
    if dp.is_file():
        defaults = yaml.safe_load(dp.read_text()) or {}
    name = task or p.name.replace("_cleaning_points.yaml", "").replace(".yaml", "")
    return TaskConfig(name, yaml.safe_load(p.read_text()) or {}, defaults, str(p))
