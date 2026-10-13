"""What goes into an optimisation and what comes out."""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from ..motion.checks import CheckReport

OPTIMISERS = ("default", "toppra")


@dataclass(frozen=True)
class Segment:
    """A stretch of the recording, in its own 1-based line numbers (inclusive), as the
    prototype and the editor show them.

    ``pct`` means:

    - Default optimiser: time as a percentage of the recording's (75 = faster, 115 = slower),
      relative to the rest of the file. The region's target time then scales everything.
    - Time-optimal: a speed cap, as a percentage of the joint ratings, inside the stretch.
    """
    start: int
    end: int
    pct: float
    label: str = ""

    def __post_init__(self):
        if int(self.start) < 1 or int(self.end) < int(self.start):
            raise ValueError(f"segment lines must satisfy 1 <= start <= end, got {self.start}-{self.end}")
        if not 0 < float(self.pct) <= 1000:
            raise ValueError(f"segment percentage must be in (0, 1000], got {self.pct}")


@dataclass(frozen=True)
class DefaultOptions:
    """The trajectory prototype's algorithm (trim idle, moving-mean smoothing, per-segment
    time) with Lite's three fixes: mirrored edges (F-a), blended segment boundaries (F-b) and
    a 41-line window checked against the taught path (F-c)."""
    trim: bool = True
    idle_window: int = 11
    idle_threshold_md: float = 8.0        # millidegrees per line, every joint
    idle_min_run: int = 20
    smooth_window: int = 41               # lines; 0 or 1 = no smoothing; made odd
    blend_s: float = 0.25                 # boundary blend (F-b); 0 = hard steps (prototype)


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
    optimiser: str = "default"            # "default" | "toppra"
    target_s: float | None = None         # seconds, (lines - 1) x 5 ms; None = as fast as the limits allow
    segments: tuple = ()
    max_dev_mm: float | None = None       # the region's MAX DEV (tightens L5)
    default: DefaultOptions = field(default_factory=DefaultOptions)
    toppra: ToppraOptions = field(default_factory=ToppraOptions)
    origin: str | None = None             # recording path, for the record
    origin_sha256: str | None = None
    cuts: tuple = ()                      # (start, end) 1-based recording lines to remove first
    natural: bool = False                 # no target: keep the recorded pace (with the segment %)
                                          # instead of the fastest the limits allow


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
        return {
            "optimiser": self.optimiser, "side": self.side,
            "target_s": self.target_s, "achieved_s": round(self.achieved_s, 3), "lines": self.lines,
            "t_min_s": round(self.t_min_s, 3), "lines_min": self.lines_min,
            "recorded_s": round(self.recorded_s, 3), "met": self.met, "uploadable": self.uploadable,
            "message": self.message, "hints": list(self.hints),
            "binding": [b.__dict__ for b in self.binding],
        }
