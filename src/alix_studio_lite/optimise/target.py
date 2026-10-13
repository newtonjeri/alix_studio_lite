"""The target-time rule, the same for both optimisers.

A plan turns a line count into bytes (``plan.make(n)``, n = lines - 1). More lines play the
same path more slowly, so the speed-dependent limits (speed, acceleration, step, the
speed part of the joint-limit margin) pass for every n from some n_min upwards:

- ``find_min_lines`` finds n_min by an exponential search from ``plan.n_start``, then
  bisection, testing the integer bytes each time;
- if the target's line count is at least n_min, the target is delivered exactly; otherwise
  n_min is (the closest feasible time) and the binding limit is named.

Rounding to integer millidegrees can make feasibility very slightly non-monotone near n_min,
so the chosen n is re-tested and, if needed, raised in 0.5 % steps (``settle``).
"""
from __future__ import annotations

import math
from typing import Callable

import numpy as np


class Search:
    """Memoised feasibility of ``make(n)`` under ``ok(bytes)``."""

    def __init__(self, make: Callable[[int], np.ndarray], ok: Callable[[np.ndarray], bool]):
        self.make = make
        self.ok = ok
        self.cache: dict[int, bool] = {}

    def __call__(self, n: int) -> bool:
        n = int(n)
        if n not in self.cache:
            self.cache[n] = bool(self.ok(self.make(n)))
        return self.cache[n]

    @property
    def evaluations(self) -> int:
        return len(self.cache)


def find_min_lines(f: Search, n_start: int, n_floor: int = 1, n_cap: int | None = None) -> int | None:
    """Smallest n in [n_floor, n_cap] with f(n) True, assuming monotone feasibility.
    None if even ``n_cap`` fails."""
    n_floor = max(1, int(n_floor))
    n = max(n_floor, int(n_start))
    n_cap = max(n, int(n_cap if n_cap is not None else 20 * n))
    if f(n):
        hi = n
        lo = None
        while lo is None:
            if hi == n_floor:
                return n_floor
            m = max(n_floor, hi // 2)
            if f(m):
                hi = m
            else:
                lo = m
    else:
        lo = n
        hi = None
        while hi is None:
            if lo >= n_cap:
                return None
            m = min(n_cap, lo * 2)
            if f(m):
                hi = m
            else:
                lo = m
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if f(mid):
            hi = mid
        else:
            lo = mid
    return hi


def settle(f: Search, n: int, max_steps: int = 40) -> int | None:
    """The first n' >= n (in 0.5 % steps) whose bytes pass."""
    n = int(n)
    for _ in range(max_steps + 1):
        if f(n):
            return n
        n += max(1, math.ceil(0.005 * n))
    return None
