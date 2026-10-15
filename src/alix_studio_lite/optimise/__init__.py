"""Tune a drag-taught recording. Two separate tunings, chosen per region (the owner, 15 Oct):

- ``"manual"`` (the default, ``optimise.manual``): the trajectory HTML's algorithm as it is
  (trim idle, mean smoothing, region scaling), the operator's settings and scale regions. The
  limits are checked and reported, never enforced: the hardware decides;
- ``"toppra"`` (Time-optimal, ``optimise.engine``): ALIX's hardware-proven TOPP-RA chain on the
  recording as taught, the fastest time within the limits (or a longer target). It never goes
  through the manual tuning's steps, and its limits stay enforced.

    from alix_studio_lite.optimise import manual, optimise, request_from_file
    res = manual.optimise(md, "left", {"segments": [{"start": 1, "end": 2000, "pct": 75}]})
    res = optimise(request_from_file("FILE_01.1.txt", "left", target_s=25))      # TOPP-RA
"""
from . import manual
from .engine import optimise, optimise_together, request_from_file
from .sidecar import write_result
from .toppra_opt import available as toppra_available
from .types import Binding, OptRequest, OptResult, ToppraOptions

__all__ = ["manual", "optimise", "optimise_together", "request_from_file", "write_result", "toppra_available",
           "Binding", "OptRequest", "OptResult", "ToppraOptions"]
