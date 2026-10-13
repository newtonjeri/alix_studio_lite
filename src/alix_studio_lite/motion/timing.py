"""Time in Lite: the controller cycle is 5 ms, and it is the only time base (decision D26).

One line of a track is one 5 ms cycle. A file of N lines lasts (N - 1) x 5 ms: the arm is at
the first line at 0 s and reaches the last line N - 1 cycles later. Operator target times use
the same rule (a target of T seconds is T / 5 ms + 1 lines), and every limit is judged at 5 ms.
"""
from __future__ import annotations

import math

CYCLE_S = 0.005             # the controller cycle: one line per cycle
DT_GATE_S = CYCLE_S         # limits are judged at the cycle
DT_RECORD_S = CYCLE_S       # the pendant records one line per cycle


def duration_for(lines: int) -> float:
    """Seconds from the first line of a file to its last."""
    return CYCLE_S * max(int(lines) - 1, 0)


def lines_for(seconds: float) -> int:
    """The line count whose duration is closest to ``seconds`` (at least 2 lines)."""
    s = float(seconds)
    if not math.isfinite(s) or s <= 0:
        raise ValueError(f"a target time must be a positive number of seconds, got {seconds!r}")
    return max(int(round(s / CYCLE_S)) + 1, 2)
