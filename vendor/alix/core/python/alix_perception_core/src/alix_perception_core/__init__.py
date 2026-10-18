"""ROS-free perception core for alix's wrist-mounted RealSense D435.

Local inference is the default backend (Newton, 2026-09-08): the models are
developed in-house and a customer washroom has no network guarantee. Nothing here
imports ROS — the node in `ros/perception/alix_perception` is the only thing that
knows `rclpy` exists (LAYERS.md §1).

    from alix_perception_core import PerceptionCore, Intrinsics
    core = PerceptionCore()                       # local backend
    result = core.analyse(image, intr, cam_to_ref, depth_image=depth)

Background, and the measurements the defaults come from:
`Resource/reports/2026-09-08_perception_port/DESIGN.md`.
"""

from .analysis import PerceptionCore
from .camera import (DEFAULT_DEPTH_PROFILE, DEPTH_640x480, DEPTH_848x480,
                     DEPTH_1280x720, Intrinsics, image_roll_deg)
from .regions import RegionVocabulary, load_vocabulary
from .types import (Detection, FixtureStatus, ROI3D, SceneAssessment,
                    FIXTURE_STATES, FIXTURE_TYPES, RECOMMENDATIONS)

# Importing the package registers the backends, so `create()` works without the
# caller knowing which module each lives in. Registration is NOT selection:
# `DEFAULT_BACKEND` is "local" and `create()` raises on an unknown name, so the
# cloud backend being importable is not a route to reaching it by accident.
# Neither module imports its own runtime at import time — ultralytics and
# google-genai are both loaded inside the backend's __init__.
from .backends import local as _local  # noqa: F401
from .backends import cloud as _cloud  # noqa: F401

__all__ = [
    "PerceptionCore", "Intrinsics", "image_roll_deg",
    "DEFAULT_DEPTH_PROFILE", "DEPTH_640x480", "DEPTH_848x480", "DEPTH_1280x720",
    "RegionVocabulary", "load_vocabulary",
    "Detection", "FixtureStatus", "ROI3D", "SceneAssessment",
    "FIXTURE_TYPES", "FIXTURE_STATES", "RECOMMENDATIONS",
]
