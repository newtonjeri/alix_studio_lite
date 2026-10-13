"""Optimise a drag-taught recording.

Two optimisers, chosen per region by the operator or engineer (decision D24):

- ``"default"``: the trajectory prototype's algorithm (trim idle, moving-mean smoothing,
  per-segment time %) with three fixes, then uniform timing to the target;
- ``"toppra"`` (Time-optimal): ALIX's hardware-proven TOPP-RA chain.

Both follow the same rule: the operator's time is the target; if the limits do not allow it,
the result is the closest feasible time and names the limit that binds. Both deliver integer
bytes that pass the same hard limits (``motion.checks``).

    from alix_studio_lite.optimise import optimise, request_from_file
    res = optimise(request_from_file("FILE_01.1.txt", "left", optimiser="toppra", target_s=25))
    print(res.message)
"""
from .engine import optimise, optimise_together, request_from_file
from .sidecar import write_result
from .toppra_opt import available as toppra_available
from .types import Binding, DefaultOptions, OptRequest, OptResult, Segment, ToppraOptions

__all__ = ["optimise", "optimise_together", "request_from_file", "write_result", "toppra_available",
           "Binding", "DefaultOptions", "OptRequest", "OptResult", "Segment", "ToppraOptions"]
