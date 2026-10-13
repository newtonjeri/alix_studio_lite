"""Where the shoulder would have to be — the inverse of the reach question.

The atlas can prove a waypoint is beyond the arm, and it can name the fix class
("standoff, pole height, or the other arm"). It could not, until this, say
**how much**. That left the most actionable failure with the least actionable
report: an operator told "12 to 21 mm beyond reach" still has to guess whether
25 mm of standoff fixes it or 250 mm does.

The inverse is exact and cheap. The wrist centre of a waypoint is fixed by the
pose alone, and the arm can hold it if and only if the shoulder is within
``a + b`` of it. So the shoulder positions from which the WHOLE stroke is
reachable are the intersection of one ball per waypoint — and the shoulder is
not free: it moves only along the pole (0 to 0.315 m of travel) and with the UGV's
standoff. Two knobs, so the answer is a 2-D map that can be read directly.

**Sign convention, stated once because it is easy to get backwards.**
``ugv_standoff_m`` places the FIXTURE: a larger standoff moves it further from
the robot, +x in ``base_link`` (`server._standoff_dx_for_request` is
``standoff - default``). This module sweeps the SHOULDER instead, because that
is what the reach test is about. The two are opposite:

    shoulder +dx  ==  fixture -dx  ==  standoff REDUCED by dx  ==  move closer

Every result therefore carries ``standoff_delta_m = -dx``, so a caller never has
to re-derive it.

Measured on ``toplid_right``, whose 16 far samples sit up to 20.8 mm beyond the
arm: reducing the standoff by 25 mm makes the whole stroke reachable at the pole
height it already uses, and 30 mm clears the 20-degree straight-elbow gate too.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

import numpy as np

logger = logging.getLogger("alix_workspace.placement")

#: The pole's physical travel. Sweeping outside it produces answers that are
#: geometrically true and mechanically unavailable, so the map marks them.
#: 0.315 is the full stroke (pole.xacro upper="0.315"); it was 0.3 before the
#: 2026-08-30 unit harmonization, which under-reported the top 15 mm.
POLE_TRAVEL_M = (0.0, 0.315)


@dataclass
class PlacementWindow:
    """Which (standoff, pole) placements put the whole stroke within reach."""

    dx_grid: np.ndarray                #: (NX,) shoulder shift along +x, metres
    pole_grid: np.ndarray              #: (NP,) absolute pole height, metres
    worst_wrist_mm: np.ndarray         #: (NX, NP) the binding wrist distance
    reach_ok: np.ndarray               #: (NX, NP) bool
    gate_ok: np.ndarray                #: (NX, NP) bool
    pole_reachable: np.ndarray         #: (NP,) is that pole height mechanically available
    max_reach_mm: float
    gate_reach_mm: float
    gate_deg: float
    #: (NX,) is that standoff shift within the fleet bounds. Empty = unconstrained
    #: (no standoff was declared), NOT "all available" — the distinction matters,
    #: because an unconstrained answer must not be reported as a permitted one.
    dx_available: np.ndarray = field(default_factory=lambda: np.zeros(0, dtype=bool))
    standoff_m: Optional[float] = None  #: the task's own standoff, if it declared one
    standoff_range_m: Optional[Tuple[float, float]] = None
    current_dx: float = 0.0
    current_pole_m: float = 0.0
    binding_waypoint: int = -1         #: whose wrist centre is furthest, as authored
    binding_wrist: np.ndarray = field(default_factory=lambda: np.zeros(3))
    names: List[str] = field(default_factory=list)

    # ------------------------------------------------------------------
    @property
    def feasible_now(self) -> bool:
        i = int(np.argmin(np.abs(self.dx_grid - self.current_dx)))
        j = int(np.argmin(np.abs(self.pole_grid - self.current_pole_m)))
        return bool(self.reach_ok[i, j])

    def minimal_change(self, gate: bool = False, pole_only: bool = False
                       ) -> Optional[dict]:
        """The smallest move from where the robot is now that fixes the stroke.

        Cost is millimetres of standoff plus millimetres of pole, weighted
        equally on purpose: both are cheap to change and the point is to show
        the operator the nearest working placement, not to rank the two.
        """
        ok = (self.gate_ok if gate else self.reach_ok).copy()
        ok &= self.pole_reachable[None, :]
        # And the standoff has a floor. This is not a detail: the fleet minimum
        # is 0.25 m (UGV front to fixture front tip) and it is ALSO the default,
        # so most tasks already sit exactly on it — and moving CLOSER is the only
        # direction that ever fixes a reach failure. Without this mask the module
        # confidently recommends a move that `validate_ugv_standoff` silently
        # clamps back, leaving the operator with a warning in a server log and a
        # stroke that still does not reach.
        if self.dx_available.size:
            ok &= self.dx_available[:, None]
        if pole_only:
            keep = np.zeros_like(ok)
            keep[int(np.argmin(np.abs(self.dx_grid - self.current_dx)))] = True
            ok &= keep
        if not ok.any():
            return None
        cost = (np.abs(self.dx_grid - self.current_dx)[:, None] * 1000.0
                + np.abs(self.pole_grid - self.current_pole_m)[None, :] * 1000.0)
        c = np.where(ok, cost, np.inf)
        i, j = np.unravel_index(int(np.argmin(c)), c.shape)
        dx = float(self.dx_grid[i])
        return {
            "shoulderDxMm": round(dx * 1000.0, 1),
            "standoffDeltaMm": round(-dx * 1000.0, 1),   # closer is negative
            "poleM": round(float(self.pole_grid[j]), 4),
            "poleDeltaMm": round((float(self.pole_grid[j]) - self.current_pole_m) * 1000.0, 1),
            "worstWristMm": round(float(self.worst_wrist_mm[i, j]), 2),
            "costMm": round(float(cost[i, j]), 1),
        }

    @property
    def standoff_at_floor(self) -> bool:
        return bool(self.standoff_range_m and self.standoff_m is not None
                    and self.standoff_m <= self.standoff_range_m[0] + 1e-9)

    def pole_line(self, gate: bool = False) -> List[dict]:
        """Feasibility along the pole at the CURRENT standoff, for the viewport.

        This is the one shell an operator can act on without moving the robot:
        the shoulder's achievable positions are a 315 mm vertical segment, and
        colouring it by whether the whole stroke is reachable from each height
        turns "the pole might help" into a line you can look at.
        """
        i = int(np.argmin(np.abs(self.dx_grid - self.current_dx)))
        ok = self.gate_ok if gate else self.reach_ok
        return [{"poleM": round(float(h), 4),
                 "available": bool(self.pole_reachable[k]),
                 "ok": bool(ok[i, k]),
                 "worstWristMm": round(float(self.worst_wrist_mm[i, k]), 2)}
                for k, h in enumerate(self.pole_grid)]

    def to_dict(self) -> dict:
        return {
            "feasibleNow": self.feasible_now,
            "maxReachMm": round(self.max_reach_mm, 2),
            "gateReachMm": round(self.gate_reach_mm, 2),
            "gateDeg": self.gate_deg,
            "currentPoleM": round(self.current_pole_m, 4),
            "bindingWaypoint": self.binding_waypoint,
            "bindingWaypointName": (self.names[self.binding_waypoint]
                                    if 0 <= self.binding_waypoint < len(self.names) else None),
            "bindingWrist": [round(float(v), 5) for v in self.binding_wrist],
            "minimalForReach": self.minimal_change(),
            "minimalForGate": self.minimal_change(gate=True),
            "minimalPoleOnlyForReach": self.minimal_change(pole_only=True),
            "standoffM": self.standoff_m,
            "standoffRangeM": (list(self.standoff_range_m)
                               if self.standoff_range_m else None),
            "standoffAtFloor": bool(
                self.standoff_range_m and self.standoff_m is not None
                and self.standoff_m <= self.standoff_range_m[0] + 1e-9),
            # The unconstrained answer, kept beside the constrained one so a
            # reader can see WHY there is no placement fix: "25 mm closer would
            # do it, and 25 mm closer is not available" is a different sentence
            # from "nothing helps", and only one of them is true here.
            "minimalIgnoringStandoffLimits": self._unbounded_minimal(),
            "poleLine": self.pole_line(),
            "standoffWindowMm": self._window(),
            "note": ("shoulder +dx == standoff REDUCED by dx (move closer); "
                     "pole travel is 0..315 mm"),
        }

    def _unbounded_minimal(self) -> Optional[dict]:
        """What would fix it if the standoff floor did not exist."""
        saved = self.dx_available
        object.__setattr__(self, "dx_available", np.zeros(0, dtype=bool))
        try:
            return self.minimal_change(gate=True) or self.minimal_change()
        finally:
            object.__setattr__(self, "dx_available", saved)

    def _window(self) -> Optional[List[float]]:
        ok = self.reach_ok & self.pole_reachable[None, :]
        if self.dx_available.size:
            ok &= self.dx_available[:, None]
        if not ok.any():
            return None
        ii = np.where(ok.any(axis=1))[0]
        return [round(-float(self.dx_grid[ii.max()]) * 1000.0, 1),
                round(-float(self.dx_grid[ii.min()]) * 1000.0, 1)]


def placement_window(arm, poses: Sequence[np.ndarray], shoulder: np.ndarray,
                     pole_m: float, *,
                     dx_range_m: float = 0.20, dx_step_m: float = 0.005,
                     pole_step_m: float = 0.005, gate_deg: float = 20.0,
                     standoff_m: Optional[float] = None,
                     standoff_range_m: Optional[Tuple[float, float]] = None,
                     names: Optional[Sequence[str]] = None) -> PlacementWindow:
    """Sweep standoff and pole; report where the whole stroke comes within reach.

    Pure geometry, like the tilt search: the binding quantity is one distance
    per waypoint, so the whole 2-D map costs a single vectorised pass. It is a
    NECESSARY condition — joint limits, singularity and collision are the
    atlas's job — so a placement this map admits still has to be rebuilt and
    judged, and one it rejects can be dismissed without building anything.
    """
    poses = [np.asarray(T, dtype=float) for T in poses]
    wc = np.array([arm.wrist_centre_of_pose(T) for T in poses])
    shoulder = np.asarray(shoulder, dtype=float)

    n = int(round(dx_range_m / dx_step_m))
    dxs = np.arange(-n, n + 1, dtype=float) * dx_step_m
    lo, hi = POLE_TRAVEL_M
    # Sweep a little past the travel so the map can SAY "the pole would reach it
    # but it cannot go that far", instead of silently ending at the stop.
    poles = np.arange(lo - 0.05, hi + 0.05 + 1e-9, pole_step_m)

    # shoulder(dx, pole) = shoulder_at(pole_m) + [dx, 0, pole - pole_m]
    shifts = np.zeros((len(dxs), len(poles), 3))
    shifts[..., 0] = dxs[:, None]
    shifts[..., 2] = (poles - pole_m)[None, :]
    cand = shoulder[None, None, :] + shifts                     # (NX, NP, 3)
    d = np.linalg.norm(wc[None, None, :, :] - cand[:, :, None, :], axis=-1)
    worst = d.max(axis=2)                                       # (NX, NP)

    max_reach = arm.max_reach_m
    gate_reach = float(arm.distance_from_elbow(np.radians(gate_deg)))
    here = np.linalg.norm(wc - shoulder, axis=1)
    binding = int(np.argmax(here))

    # shoulder +dx == standoff -dx, so dx is available only while the resulting
    # standoff stays inside the fleet bounds. Without a declared standoff there
    # is nothing to bound against and the mask stays empty (unconstrained).
    if standoff_m is not None and standoff_range_m is not None:
        s_lo, s_hi = standoff_range_m
        resulting = float(standoff_m) - dxs
        dx_ok = (resulting >= s_lo - 1e-9) & (resulting <= s_hi + 1e-9)
    else:
        dx_ok = np.zeros(0, dtype=bool)

    win = PlacementWindow(
        dx_grid=dxs, pole_grid=poles, worst_wrist_mm=worst * 1000.0,
        reach_ok=worst <= max_reach + 1e-12, gate_ok=worst <= gate_reach + 1e-12,
        pole_reachable=(poles >= lo - 1e-9) & (poles <= hi + 1e-9),
        dx_available=dx_ok, standoff_m=standoff_m,
        standoff_range_m=(tuple(standoff_range_m) if standoff_range_m else None),
        max_reach_mm=max_reach * 1000.0, gate_reach_mm=gate_reach * 1000.0,
        gate_deg=float(gate_deg), current_pole_m=float(pole_m),
        binding_waypoint=binding, binding_wrist=wc[binding],
        names=list(names or []))
    logger.info("placement: feasible now=%s; smallest fix %s",
                win.feasible_now, win.minimal_change())
    return win
