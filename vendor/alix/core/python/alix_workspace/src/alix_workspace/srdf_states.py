"""Read SRDF ``group_state`` presets (ROS/MoveIt-free).

Used to pose the inspire hands at a named grasp state (e.g. ``grasp``,
``open_tenth``, ``half_grasp``) when checking IK/collision, so the finger
geometry matches the real configuration for a given tool frame. The SRDF is the
single source of truth for the joint VALUES — edit the state there and it flows
through automatically.

    states = SrdfStates.from_file("alix.srdf")
    joints = states.group_state("inspire_hand_right", "grasp")   # {joint: value}
"""

from __future__ import annotations

import logging
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from typing import Dict, Optional

logger = logging.getLogger("alix_workspace.srdf_states")


@dataclass
class SrdfStates:
    # (group, state) -> {joint_name: value}
    states: Dict[str, Dict[str, float]] = field(default_factory=dict)

    @classmethod
    def from_file(cls, srdf_path: str) -> "SrdfStates":
        root = ET.parse(srdf_path).getroot()
        out: Dict[str, Dict[str, float]] = {}
        for gs in root.findall("group_state"):
            key = f"{gs.get('group','')}/{gs.get('name','')}"
            out[key] = {
                j.get("name"): float(j.get("value", 0.0))
                for j in gs.findall("joint")
                if j.get("name") is not None
            }
        logger.info("Parsed SRDF '%s': %d group states", srdf_path, len(out))
        return cls(states=out)

    def group_state(self, group: str, state: str) -> Dict[str, float]:
        return dict(self.states.get(f"{group}/{state}", {}))

    def states_in_group(self, group: str) -> Dict[str, Dict[str, float]]:
        """All states defined for a group: {state_name: {joint: value}}."""
        prefix = f"{group}/"
        return {k[len(prefix):]: dict(v) for k, v in self.states.items() if k.startswith(prefix)}

    def hand_state(self, arm: str, state: str) -> Dict[str, float]:
        """Finger joint values for an arm ('left'/'right') at a named state."""
        if not state:
            return {}
        group = "inspire_hand_left" if "left" in arm.lower() else "inspire_hand_right"
        vals = self.group_state(group, state)
        if not vals:
            logger.warning("SRDF has no state '%s' for group '%s'", state, group)
        return vals


def load_initial_positions(index) -> Dict[str, float]:
    """Merge ``config/initial_positions.yaml`` from the source ``*_description``
    packages (the home joint configuration used in simulation), resolved via a
    :class:`PackageIndex` so it works on any machine from the source tree.

    Returns ``{joint_name: value}`` for all numeric entries found."""
    import yaml
    out: Dict[str, float] = {}
    files = 0
    dirs = getattr(index, "name_to_dir", {}) or {}
    for pkg_dir in dict.fromkeys(dirs.values()):   # dedupe shared dirs
        p = pkg_dir / "config" / "initial_positions.yaml"
        if not p.is_file():
            continue
        try:
            data = yaml.safe_load(p.read_text()) or {}
        except Exception:
            continue
        files += 1
        for k, v in data.items():
            if isinstance(v, (int, float)):
                out[k] = float(v)
    logger.info("Loaded %d initial joint positions from %d file(s)", len(out), files)
    return out


#: Directory names never searched for an SRDF. A stale SRDF is worse than no
#: SRDF: it supplies joint limits and named states to collision and
#: reachability, so the wrong one produces confident, wrong answers.
_SRDF_SKIP_DIRS = frozenset({
    ".git", ".Trash", "Trash", "node_modules", "__pycache__",
    "build", "install", "log", "site-packages", ".cache", ".venv", "venv",
})


def _srdf_search_ceiling(start):
    """Stop the upward walk at a project boundary, never at ``/``.

    The original walked every parent to the filesystem root and ran a
    RECURSIVE ``**`` glob at each level. From a tree with no SRDF that meant
    globbing all of ``$HOME`` — and it duly returned one out of
    ``~/.local/share/Trash``. Searching past the project is not
    "best-effort"; it is how a deleted robot's joint limits end up in a
    reachability result.
    """
    from pathlib import Path
    p = Path(start).resolve()
    for parent in [p, *p.parents]:
        if (parent / "ARCHITECTURE.md").is_file() or (parent / ".git").exists():
            return parent
    # No marker: allow a few levels, but never the home directory or root.
    home = Path.home().resolve()
    for parent in [p, *p.parents][:5]:
        if parent in (home, Path(parent.anchor)):
            break
        ceiling = parent
    return ceiling


def find_srdf(start: str) -> Optional[str]:
    """Locate ``*.srdf`` in a sibling ``*_moveit_config`` package.

    ROS-free by design (no ament index). Bounded by ``_srdf_search_ceiling``
    and blind to ``_SRDF_SKIP_DIRS``; returns the first match in **sorted**
    order so the answer is deterministic rather than filesystem-dependent.

    ``ALIX_SRDF`` overrides entirely.
    """
    import os
    from pathlib import Path

    override = os.environ.get("ALIX_SRDF")
    if override:
        return override if Path(override).is_file() else None

    p = Path(start).resolve()
    ceiling = _srdf_search_ceiling(p)

    def _acceptable(hit: Path) -> bool:
        return not any(part in _SRDF_SKIP_DIRS for part in hit.parts)

    parents = [p, *p.parents]
    try:
        parents = parents[: parents.index(ceiling) + 1]
    except ValueError:
        pass

    for parent in parents:
        for pattern in ("*_moveit_config/config/*.srdf",
                        "**/*_moveit_config/config/*.srdf"):
            hits = sorted(h for h in parent.glob(pattern) if _acceptable(h))
            if hits:
                return str(hits[0])
    return None
