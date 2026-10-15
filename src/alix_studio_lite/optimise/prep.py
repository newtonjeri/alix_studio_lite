"""A preparation step of ALIX's taught-path chain, for the time-optimal tuning (``toppra_opt``).

The trajectory HTML's own steps (trim, mean smoothing, region scaling) are manual tuning's, in
``optimise.manual``; the time-optimal tuning never uses them.
"""
from __future__ import annotations

import numpy as np


def pause_mask(q_deg: np.ndarray, tol_deg: float = 0.05, min_lines: int = 3) -> np.ndarray:
    """The rows ``taught_tracks.strip_pauses`` keeps (it returns only the kept rows; Lite also
    needs which ones). A run of at least ``min_lines`` lines all within ``tol_deg`` of the run's
    first line collapses to that first line; the last line is kept. Parity with ALIX is tested."""
    q = np.asarray(q_deg, dtype=float)
    n = q.shape[0]
    keep = np.ones(n, dtype=bool)
    if n < 3:
        return keep
    k = 0
    while k < n - 1:
        anchor = q[k]
        j = k + 1
        while j < n and np.abs(q[j] - anchor).max() <= tol_deg:
            j += 1
        if j - k >= int(min_lines):
            keep[k + 1:j] = False
            k = j
        else:
            k += 1
    keep[-1] = True
    return keep
