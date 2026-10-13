"""RealSense D435i wet-environment degradation model (Deliverable C).

Estimates depth quality and a per-fixture *perception confidence* score from
three observable environmental factors:

* steam / fog density       (0.0 clear .. 1.0 dense fog),
* reflective surface present (bool, e.g. mirrors, polished chrome),
* ambient light level        (lux; the D435i IR projector degrades both in the
  dark and in very bright, washed-out conditions).

When the confidence for a fixture falls below
``sensor_model.min_confidence_for_autonomy`` the fixture is flagged as requiring
a fallback strategy (force-torque guided cleaning or a pre-programmed
trajectory) instead of live depth servoing.

This is a pure-Python module (numpy only) so it is fully unit-testable offline.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import Dict, Optional

import numpy as np

logger = logging.getLogger("alix_workspace.sensor_model")


@dataclass
class PerceptionAssessment:
    """Per-fixture perception quality assessment."""

    fixture: str
    depth_noise_m: float
    valid_depth_fraction: float
    confidence: float
    autonomy_ok: bool
    recommended_fallback: Optional[str]

    def as_dict(self) -> dict:
        return {
            "fixture": self.fixture,
            "depth_noise_m": round(self.depth_noise_m, 4),
            "valid_depth_fraction": round(self.valid_depth_fraction, 3),
            "confidence": round(self.confidence, 3),
            "autonomy_ok": self.autonomy_ok,
            "recommended_fallback": self.recommended_fallback,
        }


class SensorDegradationModel:
    """RealSense D435i degradation / perception-confidence estimator."""

    # D435i ideal operating illuminance band (lux). Outside this band the
    # confidence is linearly attenuated toward the configured floor.
    _LUX_LOW = 100.0
    _LUX_HIGH = 5000.0

    def __init__(self, config: dict):
        cfg = config.get("sensor_model", {})
        self._base_noise = float(cfg.get("base_depth_noise_m", 0.002))
        # noise increase per *full unit* of fog density (config historically
        # documented "per 0.1 unit"; we normalise to per-unit here).
        self._fog_noise = float(cfg.get("fog_noise_per_unit_m", 0.05))
        self._reflective_dropout = float(cfg.get("reflective_depth_dropout_prob", 0.35))
        self._min_conf = float(cfg.get("min_confidence_for_autonomy", 0.45))
        # noise (m) at which confidence from noise alone reaches ~0.
        self._noise_zero_conf_m = float(cfg.get("noise_zero_confidence_m", 0.06))
        self._fixtures: Dict[str, dict] = config.get("fixtures", {})

    # ------------------------------------------------------------------
    # Core model
    # ------------------------------------------------------------------

    def depth_noise(self, fog_density: float, reflective: bool) -> float:
        """Expected depth std-dev (m) at the working distance."""
        fog_density = float(np.clip(fog_density, 0.0, 1.0))
        noise = self._base_noise + self._fog_noise * fog_density
        if reflective:
            # Reflective surfaces add IR speckle scatter on top of dropout.
            noise *= 1.5
        return noise

    def valid_depth_fraction(self, fog_density: float, reflective: bool) -> float:
        """Fraction of pixels expected to return a valid depth reading."""
        fog_density = float(np.clip(fog_density, 0.0, 1.0))
        # Fog scatters the IR pattern -> exponential loss of valid returns.
        frac = float(np.exp(-2.5 * fog_density))
        if reflective:
            frac *= 1.0 - self._reflective_dropout
        return float(np.clip(frac, 0.0, 1.0))

    def _lux_factor(self, lux: float) -> float:
        """Multiplicative confidence factor (0..1) for ambient illuminance."""
        if lux < self._LUX_LOW:
            return float(np.clip(lux / self._LUX_LOW, 0.1, 1.0))
        if lux > self._LUX_HIGH:
            # Bright daylight washes out the IR pattern.
            over = (lux - self._LUX_HIGH) / self._LUX_HIGH
            return float(np.clip(1.0 - 0.5 * over, 0.2, 1.0))
        return 1.0

    def confidence(
        self, fog_density: float, reflective: bool, ambient_lux: float = 500.0
    ) -> float:
        """Combined perception confidence in [0, 1]."""
        noise = self.depth_noise(fog_density, reflective)
        noise_conf = float(np.clip(1.0 - noise / self._noise_zero_conf_m, 0.0, 1.0))
        valid_conf = self.valid_depth_fraction(fog_density, reflective)
        lux_conf = self._lux_factor(ambient_lux)
        # Geometric mean keeps any single bad factor from being masked.
        conf = (noise_conf * valid_conf * lux_conf) ** (1.0 / 3.0)
        return float(np.clip(conf, 0.0, 1.0))

    # ------------------------------------------------------------------
    # Per-fixture assessment
    # ------------------------------------------------------------------

    def assess_fixture(
        self,
        fixture: str,
        fog_density: float,
        ambient_lux: float = 500.0,
        reflective: Optional[bool] = None,
    ) -> PerceptionAssessment:
        """Assess one fixture; ``reflective`` defaults to the fixture config."""
        spec = self._fixtures.get(fixture, {})
        if reflective is None:
            reflective = bool(spec.get("reflective", False))

        noise = self.depth_noise(fog_density, reflective)
        valid = self.valid_depth_fraction(fog_density, reflective)
        conf = self.confidence(fog_density, reflective, ambient_lux)
        ok = conf >= self._min_conf

        fallback = None
        if not ok:
            fallback = (
                "force_torque_guided"
                if spec.get("primitive") != "plane"
                else "preprogrammed_trajectory"
            )
            logger.warning(
                "Fixture '%s': perception confidence %.2f < %.2f (fog=%.2f, "
                "reflective=%s, lux=%.0f) -> fallback '%s'",
                fixture, conf, self._min_conf, fog_density, reflective,
                ambient_lux, fallback,
            )
        else:
            logger.info(
                "Fixture '%s': perception confidence %.2f OK (fog=%.2f, "
                "reflective=%s)", fixture, conf, fog_density, reflective,
            )

        return PerceptionAssessment(
            fixture=fixture,
            depth_noise_m=noise,
            valid_depth_fraction=valid,
            confidence=conf,
            autonomy_ok=ok,
            recommended_fallback=fallback,
        )

    def assess_all(
        self, fog_density: float, ambient_lux: float = 500.0
    ) -> Dict[str, PerceptionAssessment]:
        """Assess every configured fixture under one environmental condition."""
        return {
            name: self.assess_fixture(name, fog_density, ambient_lux)
            for name in self._fixtures
        }
