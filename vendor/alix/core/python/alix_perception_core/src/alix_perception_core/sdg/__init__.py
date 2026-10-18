"""Synthetic data generation: the plan, and the record of what it produced.

ROS-free, Isaac-free, GPU-free. The renderer lives in
`ros/simulation/isaacsim/scripts/sdg_rig.py`; everything here is the contract it
writes against, so it can be unit-tested on any machine in milliseconds rather
than discovered to be wrong after a twenty-minute render.

    from alix_perception_core.sdg import load_plan, ManifestWriter
"""

from .manifest import (CORPUS_NAME, MANIFEST_NAME, FrameRecord, ManifestWriter,
                       SoilInstance, load_manifest, sha256_of)
from .plan import (DEFAULT_STATES, KNOWN_AOVS, Resolution, SdgPlan,
                   SdgPlanError, load_plan)

__all__ = [
    "SdgPlan", "SdgPlanError", "load_plan", "Resolution", "KNOWN_AOVS",
    "DEFAULT_STATES",
    "FrameRecord", "SoilInstance", "ManifestWriter", "load_manifest",
    "sha256_of", "MANIFEST_NAME", "CORPUS_NAME",
]
