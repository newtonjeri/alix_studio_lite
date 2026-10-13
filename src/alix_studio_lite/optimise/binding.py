"""Which limit sets the shortest time, in plain language.

Input is a per-line utilisation series per limit (1.0 = at the limit). A limit binds where its
utilisation is at least 98 % of the limit. Limits are ranked by the share of the file they
bind on, and each is given with its longest binding stretch:

    "joint 6 acceleration (38 % of the time; longest 3.3-3.5 s)"

Times are seconds into the delivered file at 5 ms per line.
"""
from __future__ import annotations

import numpy as np

from ..motion.timing import CYCLE_S
from .types import Binding

KIND_TEXT = {"speed": "speed", "acceleration": "acceleration", "joint_limit": "approach to its end stop"}


def _longest_run(mask: np.ndarray) -> tuple[int, int]:
    best, best_len, start = (0, 0), 0, None
    for i, m in enumerate(np.append(mask, False)):
        if m and start is None:
            start = i
        elif not m and start is not None:
            if i - start > best_len:
                best, best_len = (start, i - 1), i - start
            start = None
    return best


def binding_report(util: dict, threshold: float = 0.98, top: int = 3) -> list[Binding]:
    found = []
    for key, u in util.items():
        u = np.asarray(u, dtype=float)
        if not u.size:
            continue
        mask = u >= threshold
        if not mask.any():
            continue
        kind, _, j = key.rpartition("_J")
        l0, l1 = _longest_run(mask)
        found.append(Binding(kind=kind, joint=int(j), share=float(mask.mean()), line0=int(l0), line1=int(l1),
                             t0_s=round(l0 * CYCLE_S, 3), t1_s=round(l1 * CYCLE_S, 3),
                             peak=float(u.max()), text=""))
    found.sort(key=lambda b: (-b.share, -b.peak))
    found = found[:top]
    for b in found:
        what = f"joint {b.joint} {KIND_TEXT.get(b.kind, b.kind)}"
        if b.share < 0.01:                      # a single peak, not a stretch
            b.text = f"{what} around {0.5 * (b.t0_s + b.t1_s):.1f} s"
        else:
            b.text = f"{what} ({100 * b.share:.0f} % of the time; longest {b.t0_s:.1f}-{b.t1_s:.1f} s)"
    return found


def describe(bindings: list[Binding]) -> str:
    if not bindings:
        return ""
    if len(bindings) == 1:
        return f"limited by {bindings[0].text}"
    rest = ", then ".join(b.text for b in bindings[1:])
    return f"limited mostly by {bindings[0].text}, then {rest}"
