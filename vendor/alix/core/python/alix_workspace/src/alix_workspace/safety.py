"""Human-intrusion safety zone evaluation (ROS-free).

Pure-Python zone logic: turns a set of detected obstacle points (in the robot
base frame) plus a hygiene-lock ledger into a structured safety verdict.

Zone tiers (configurable radii):
* ``stall_entry_radius_m``  -> base halt + arms retract,
* ``human_retract_radius_m`` -> arms retract,
* ``caution_radius_m``       -> human detected, no mitigation,
* a latched e-stop overrides everything.

A hygiene lock prevents re-entry to a just-cleaned target for
``hygiene_hold_time_s``.

This module has **no ROS dependency**; consumers (a web route, a CLI, or an
external ROS bridge) feed it points and read the :class:`SafetyVerdict`.
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass, field
from typing import Dict, List, Sequence

import numpy as np

logger = logging.getLogger("alix_workspace.safety")

# Severity levels.
CLEAR = 0
CAUTION = 1
INTRUSION = 2
ESTOP = 3

_LEVEL_NAME = {CLEAR: "CLEAR", CAUTION: "CAUTION", INTRUSION: "INTRUSION", ESTOP: "ESTOP"}


@dataclass
class SafetyVerdict:
    status_level: int = CLEAR
    human_detected: bool = False
    nearest_human_distance_m: float = float("inf")
    arms_retract_required: bool = False
    base_halt_required: bool = False
    estop_asserted: bool = False
    active_zones: List[str] = field(default_factory=list)
    hygiene_locked_targets: List[str] = field(default_factory=list)
    message: str = "clear"

    def as_dict(self) -> dict:
        d = self.nearest_human_distance_m
        return {
            "status_level": self.status_level,
            "status_name": _LEVEL_NAME.get(self.status_level, "?"),
            "human_detected": self.human_detected,
            "nearest_human_distance_m": None if math.isinf(d) else round(d, 3),
            "arms_retract_required": self.arms_retract_required,
            "base_halt_required": self.base_halt_required,
            "estop_asserted": self.estop_asserted,
            "active_zones": self.active_zones,
            "hygiene_locked_targets": self.hygiene_locked_targets,
            "message": self.message,
        }


class SafetyZoneEvaluator:
    """Evaluate detected points against the configured safety zones."""

    def __init__(self, config: dict):
        cfg = config.get("safety", {})
        self.base_exclusion = float(cfg.get("base_exclusion_radius_m", 1.5))
        self.human_retract = float(cfg.get("human_retract_radius_m", 1.5))
        self.stall_entry = float(cfg.get("stall_entry_radius_m", 0.8))
        self.caution_radius = float(cfg.get("caution_radius_m", 2.0))
        self.arm_swept_buffer = float(cfg.get("arm_swept_buffer_m", 0.20))
        self.hygiene_hold_s = float(cfg.get("hygiene_hold_time_s", 30.0))
        self._hygiene_locks: Dict[str, float] = {}
        self._estop_latched = False

    # ------------------------------------------------------------------
    # Hygiene lock ledger
    # ------------------------------------------------------------------
    def register_cleaned_target(self, target: str, now_s: float) -> None:
        self._hygiene_locks[target] = now_s + self.hygiene_hold_s
        logger.info("Hygiene lock on '%s' for %.0fs", target, self.hygiene_hold_s)

    def hygiene_locked(self, now_s: float) -> List[str]:
        active = [t for t, exp in self._hygiene_locks.items() if exp > now_s]
        for t in list(self._hygiene_locks):
            if self._hygiene_locks[t] <= now_s:
                del self._hygiene_locks[t]
        return active

    def is_target_locked(self, target: str, now_s: float) -> bool:
        return target in self.hygiene_locked(now_s)

    # ------------------------------------------------------------------
    def set_estop(self, asserted: bool) -> None:
        self._estop_latched = bool(asserted)

    # ------------------------------------------------------------------
    def evaluate(self, points_xy: Sequence[Sequence[float]], now_s: float = 0.0) -> SafetyVerdict:
        """Evaluate obstacle points (Nx2, base frame, metres) at time ``now_s``."""
        verdict = SafetyVerdict()
        verdict.hygiene_locked_targets = self.hygiene_locked(now_s)

        pts = np.asarray(points_xy, dtype=float).reshape(-1, 2) if len(points_xy) else np.empty((0, 2))
        dists = np.linalg.norm(pts, axis=1) if pts.shape[0] else np.empty((0,))
        dists = dists[np.isfinite(dists)]
        nearest = float(np.min(dists)) if dists.size else math.inf
        if dists.size:
            verdict.nearest_human_distance_m = nearest
            verdict.human_detected = nearest <= self.caution_radius

        if self._estop_latched:
            verdict.status_level = ESTOP
            verdict.estop_asserted = True
            verdict.arms_retract_required = True
            verdict.base_halt_required = True
            verdict.active_zones.append("estop")
            verdict.message = "E-STOP asserted"
            return verdict

        if nearest <= self.stall_entry:
            verdict.status_level = INTRUSION
            verdict.arms_retract_required = True
            verdict.base_halt_required = True
            verdict.active_zones.extend(["stall_entry", "arm_retract"])
            verdict.message = f"Human in stall ({nearest:.2f} m): halt base, retract arms"
        elif nearest <= self.human_retract:
            verdict.status_level = INTRUSION
            verdict.arms_retract_required = True
            verdict.active_zones.append("arm_retract")
            verdict.message = f"Human within {self.human_retract:.1f} m ({nearest:.2f} m): retract arms"
        elif nearest <= self.caution_radius:
            verdict.status_level = CAUTION
            verdict.active_zones.append("caution")
            verdict.message = f"Human in caution zone ({nearest:.2f} m)"
        else:
            verdict.status_level = CLEAR
            verdict.message = "clear"
        return verdict

    # ------------------------------------------------------------------
    def zone_radii(self) -> dict:
        """Radii for visualization (concentric circles around the base)."""
        return {
            "stall_entry": self.stall_entry,
            "human_retract": self.human_retract,
            "caution": self.caution_radius,
            "base_exclusion": self.base_exclusion,
        }
