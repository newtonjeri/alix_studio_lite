"""Shared configuration loading for the Butterfli workspace analysis package.

Centralises the (otherwise duplicated) logic of locating and parsing
``alix_workspace_config.yaml`` so every node/module resolves the same
file in the same order:

1. an explicit path argument,
2. the ``BUTTERFLI_WORKSPACE_CONFIG`` environment variable,
3. the installed package ``share`` directory,
4. the in-source ``config/`` directory (for ``colcon`` symlink-install / dev).
"""

from __future__ import annotations

import logging
import os
from pathlib import Path
from typing import Optional

import yaml

logger = logging.getLogger("alix_workspace.config_loader")

_CONFIG_BASENAME = "alix_workspace_config.yaml"
_ENV_VAR = "BUTTERFLI_WORKSPACE_CONFIG"


def default_config_path() -> str:
    """Return the best-guess path to the workspace config YAML.

    Raises
    ------
    FileNotFoundError
        If no candidate path exists.
    """
    candidates = []

    env_path = os.environ.get(_ENV_VAR)
    if env_path:
        candidates.append(Path(env_path))

    # In-source / installed package path:
    #   alix_workspace/alix_workspace/config_loader.py
    #   config/ is PACKAGE DATA and lives beside this module, so it resolves
    #   from __file__ directly. It used to sit at project level and be reached
    #   with parents[1]; that broke silently the moment the package moved under
    #   src/ (F30), pointing at src/config which does not exist.
    candidates.append(Path(__file__).resolve().parent / "config" / _CONFIG_BASENAME)

    for path in candidates:
        if path and path.is_file():
            return str(path)

    raise FileNotFoundError(
        f"Could not locate {_CONFIG_BASENAME}. Looked in: "
        + ", ".join(str(c) for c in candidates)
    )


def load_config(config_path: Optional[str] = None) -> dict:
    """Load and parse the workspace config YAML into a dict.

    Parameters
    ----------
    config_path : optional explicit path; if ``None`` :func:`default_config_path`
        is used.
    """
    path = config_path or default_config_path()
    logger.info("Loading workspace config from %s", path)
    with open(path, "r") as fh:
        data = yaml.safe_load(fh)
    if not isinstance(data, dict):
        raise ValueError(f"Config at {path} did not parse to a mapping")
    return data
