"""What goes into an optimisation and what comes out."""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from ..motion.checks import CheckReport

OPTIMISERS = ("manual", "toppra")       # manual: optimise.manual (the HTML's algorithm); toppra: engine


@dataclass(frozen=True)
class ToppraOptions:
    """ALIX's hardware-proven taught-path chain: 5 Hz zero-phase smoothing, pause removal,
    30 mm arc-length smoothing, then TOPP-RA within 0.95 x the ratings and 600 deg/s^2."""
    smooth_hz: float = 5.0                # 0 = off
    strip_pauses: bool = True
    smooth_mm: float = 30.0               # 0 = off
    pin_ends: bool = True                 # put the first and last pose back on the recording's


@dataclass
class OptRequest:
    md_raw: np.ndarray                    # (N, 7) int millidegrees, the recording
    side: str                             # "left" | "right"
    optimiser: str = "toppra"             # the time-optimal engine's request (manual tuning: optimise.manual)
    target_s: float | None = None         # seconds, (lines - 1) x 5 ms; None = as fast as the limits allow
    max_dev_mm: float | None = None       # the region's MAX DEV (tightens L5)
    toppra: ToppraOptions = field(default_factory=ToppraOptions)
    origin: str | None = None             # recording path, for the record
    origin_sha256: str | None = None
    natural: bool = False                 # no target: the plan's own start (TOPP-RA: its fastest)


@dataclass
class Binding:
    """One limit that sets the shortest time."""
    kind: str                             # "speed" | "acceleration" | "joint_limit"
    joint: int | None
    share: float                          # fraction of the file at >= 98 % of the limit
    line0: int                            # longest stretch, delivered lines (0-based)
    line1: int
    t0_s: float                           # the same, in seconds (5 ms per line)
    t1_s: float
    peak: float                           # highest utilisation (1.0 = at the limit)
    text: str


@dataclass
class OptResult:
    md: np.ndarray                        # (N, 7) int64 millidegrees, the delivered file
    optimiser: str
    side: str
    target_s: float | None
    achieved_s: float                     # (lines - 1) x 5 ms
    lines: int
    t_min_s: float                        # the fastest feasible
    lines_min: int
    recorded_s: float                     # the recording
    met: bool                             # target reached (True when no target was given)
    uploadable: bool                      # every hard limit passes
    binding: list = field(default_factory=list)
    checks: CheckReport = field(default_factory=CheckReport)
    hints: list = field(default_factory=list)
    message: str = ""
    recipe: dict = field(default_factory=dict)

    def summary(self) -> dict:
        from ..motion.checks import failures_text

        return {
            "optimiser": self.optimiser, "side": self.side,
            "target_s": self.target_s, "achieved_s": round(self.achieved_s, 3), "lines": self.lines,
            "t_min_s": round(self.t_min_s, 3), "lines_min": self.lines_min,
            "recorded_s": round(self.recorded_s, 3), "met": self.met, "uploadable": self.uploadable,
            "limits_ok": self.uploadable, "enforced": True, "limits_text": failures_text(self.checks.to_dict()),
            "message": self.message, "hints": list(self.hints),
            "binding": [b.__dict__ for b in self.binding],
        }
