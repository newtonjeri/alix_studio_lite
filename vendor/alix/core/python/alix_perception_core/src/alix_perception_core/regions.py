"""The region vocabulary, read from the fixture's own `regions.yaml`.

butterfli kept a hand-written `VALID_REGIONS` tuple in `ai_prompts.py`. On alix
that would be a second copy of a list whose first copy is annotated "SINGLE
SOURCE OF TRUTH", and the two would drift the first time a region was added in
the Studio. So this module derives the vocabulary instead, and the answer to
"may perception say this word" is always the fixture model's own answer.

The derivation also settles three things that a hand-written list cannot.

**Not every region can be cleaned.** `generate: false` marks a region whose
coverage belongs to a covering task; and a region may be declared with no shipped
`*_cleaning_points.yaml` at all. Both are reportable observations and neither is
an actionable target. `Region.servable` separates them, so a caller sees the gap
rather than a silently dropped detection — the case that matters is
`bowl_inside_*`, butterfli's most-reported region in the field and one whose
configs are all currently retired.

**A region is only visible in some fixture states.** `lid_state` / `seat_state`
say which. A detector that reports `seat_ring_bottom` on a closed commode has
made an error that the geometry can catch for free.

**Side parity is not perception's business.** `arm_coverage` (R | L | RL_paired |
RL_compound) already decides which arm covers a region, so perception emits the
bare name and the consumer resolves the side. That is also how
`AnalyzeScene.srv` is shaped, and it is why nothing here appends `_left`/`_right`.
"""

from __future__ import annotations

import functools
import logging
from dataclasses import dataclass
from pathlib import Path

import yaml

from .paths import fixture_model_dir

logger = logging.getLogger("alix_perception_core.regions")

#: butterfli's class names for three regions alix names differently. Kept as a
#: translation at the BOUNDARY — a model trained with the old labels stays
#: usable without re-labelling its dataset, and nothing downstream sees the old
#: word. Not a general alias table: these three are renames of the same surface,
#: verified against regions.yaml, and new names belong in regions.yaml.
LEGACY_ALIASES = {
    "commode_front": "front",
    "commode_side": "side",
    "commode_top": "top",
}


@dataclass(frozen=True)
class Region:
    """One region as the fixture model declares it."""

    name: str
    region_id: int | None
    component: str
    arm_coverage: str
    description: str
    lid_state: str | None
    seat_state: str | None
    generates_task: bool
    has_config: bool

    @property
    def servable(self) -> bool:
        """Can a cleaning task actually be run against this region today?"""
        return self.generates_task and self.has_config

    @property
    def required_fixture_state(self) -> str | None:
        """The commode state this region is cleaned in, or None if unconstrained.

        Reality logic, stated in regions.yaml and relied on here: an open seat
        forces an open lid, because the seat swings up against it.
        """
        if self.seat_state == "open":
            return "lid_seat_open"
        if self.lid_state == "open":
            return "lid_open"
        if self.lid_state == "closed" or self.seat_state == "closed":
            return "closed"
        return None


class RegionVocabulary:
    """The regions one fixture model declares, and what may be said about them."""

    def __init__(self, regions: dict[str, Region], fixture_type: str, model: str):
        self._regions = regions
        self.fixture_type = fixture_type
        self.model = model

    # -- construction ------------------------------------------------
    @classmethod
    def load(cls, model: str = "commode_c",
             fixture_type: str = "commode") -> "RegionVocabulary":
        """Read `<model>/models/regions.yaml`. Empty vocabulary if absent.

        Empty rather than raising: a missing fixture model is a condition the
        service should report, not one that stops the node from starting.
        """
        base = fixture_model_dir(fixture_type, model)
        if base is None:
            return cls({}, fixture_type, model)

        manifest = base / "models" / "regions.yaml"
        if not manifest.is_file():
            return cls({}, fixture_type, model)

        with manifest.open() as handle:
            loaded = yaml.safe_load(handle) or {}
        raw = loaded.get("regions", loaded)

        regions: dict[str, Region] = {}
        for name, spec in raw.items():
            if not isinstance(spec, dict):
                continue
            regions[name] = Region(
                name=name,
                region_id=spec.get("region_id"),
                component=spec.get("component", "body"),
                arm_coverage=spec.get("arm_coverage", ""),
                description=spec.get("description", ""),
                lid_state=spec.get("lid_state"),
                seat_state=spec.get("seat_state"),
                # `generate: false` means "covered by another task", not "absent".
                generates_task=spec.get("generate", True) is not False,
                has_config=_has_cleaning_config(base, name),
            )
        return cls(regions, fixture_type, model)

    # -- queries -----------------------------------------------------
    def __contains__(self, name: str) -> bool:
        return self.canonical(name) in self._regions

    def __len__(self) -> int:
        return len(self._regions)

    @property
    def names(self) -> tuple[str, ...]:
        return tuple(sorted(self._regions))

    def get(self, name: str) -> Region | None:
        return self._regions.get(self.canonical(name))

    @staticmethod
    def canonical(name: str) -> str:
        """Fold a legacy butterfli region name onto its alix name."""
        return LEGACY_ALIASES.get(name, name)

    def servable(self) -> tuple[str, ...]:
        """Regions a cleaning task can be run against today."""
        return tuple(sorted(n for n, r in self._regions.items() if r.servable))

    def visible_in(self, fixture_state: str) -> tuple[str, ...]:
        """Regions that can legitimately be reported in this fixture state."""
        return tuple(sorted(
            n for n, r in self._regions.items()
            if r.required_fixture_state in (None, fixture_state)
        ))

    @staticmethod
    def normalise(name: str) -> str:
        """Fold case, hyphens and spaces onto the underscore spelling.

        butterfli's first two tiers (`_best_region_match`, ai_client.py:528-557).
        A model's class names and a human's prompt both spell regions loosely —
        `Seat-Ring`, `seat ring`, `SEAT_RING` — and none of those differences is
        information.
        """
        return name.strip().lower().replace("-", "_").replace(" ", "_")

    def best_match(self, text: str) -> str:
        """Resolve a loose name to a declared region, or "" if it cannot be.

        Three tiers, in order, and a REFUSAL rather than a pick when the answer is
        ambiguous:

        1. the normalised name IS a region (after the legacy alias fold);
        2. the name carries a region as a suffix or prefix around a label —
           `stain_seat_ring`, `seat_ring_stain` — which is how a retrained model
           would encode the region it saw;
        3. otherwise nothing.

        Tier 2 takes the LONGEST match, because alix's vocabulary contains names
        that are prefixes of other names (`seat_ring` and `seat_ring_sides`,
        `lid_seat_front` and `front`), and the shortest match would route a
        `seat_ring_sides` detection to `seat_ring` — a different surface, cleaned
        by a different task. If two different regions tie at the same length the
        answer is "", because the one thing worse than not knowing which surface
        a stain is on is being confidently wrong about it: the consequence is an
        arm wiping the wrong place and reporting success.

        butterfli's version had a fuzzy `difflib` tier below these. It is not
        carried: `get_close_matches` on a 21-name vocabulary full of shared
        stems maps `lid_side` onto `lidsides_back` at a ratio that looks
        convincing, and a wrong region here is not a cosmetic error.
        """
        normal = self.canonical(self.normalise(text))
        if normal in self._regions:
            return normal

        best: list[str] = []
        best_len = 0
        for region in self._regions:
            for alias in {region} | {k for k, v in LEGACY_ALIASES.items() if v == region}:
                if not (normal.endswith("_" + alias) or normal.startswith(alias + "_")):
                    continue
                if len(alias) > best_len:
                    best, best_len = [region], len(alias)
                elif len(alias) == best_len and region not in best:
                    best.append(region)
        if len(best) == 1:
            return best[0]
        if len(best) > 1:
            logger.warning("'%s' matches %s equally well; left unrouted rather "
                           "than guessed", text, ", ".join(sorted(best)))
        return ""

    def validate(self, name: str, fixture_state: str | None = None
                 ) -> tuple[bool, str]:
        """Is this a region this fixture has, in this state?

        Returns (ok, reason). `reason` is empty when ok, and is written to be
        readable in a service `message` field.
        """
        canonical = self.canonical(name)
        region = self._regions.get(canonical)
        if region is None:
            return False, (f"'{name}' is not a region of {self.model}; "
                           f"declared regions: {', '.join(self.names) or 'none'}")
        if fixture_state is not None:
            required = region.required_fixture_state
            if required is not None and required != fixture_state:
                return False, (f"'{canonical}' is cleaned with the fixture "
                               f"{required}, but the fixture is {fixture_state}")
        return True, ""


def _has_cleaning_config(base: Path, region: str) -> bool:
    """Is there a shipped `*_cleaning_points.yaml` for this region?

    Only the model directory itself counts. `retired_*/` and `hand_authored/`
    deliberately do not: a retired config is one the current gates reject, and
    treating it as available would promise a task that cannot run.

    Matched by prefix because configs carry the arm parity the region name does
    not (`front` -> `front_left_cleaning_points.yaml`).
    """
    try:
        return any(p.name.startswith(f"{region}_") and
                   p.name.endswith("_cleaning_points.yaml")
                   for p in base.iterdir() if p.is_file())
    except OSError:
        return False


@functools.lru_cache(maxsize=8)
def load_vocabulary(model: str = "commode_c",
                    fixture_type: str = "commode") -> RegionVocabulary:
    """Cached `RegionVocabulary.load` — the node asks per request."""
    return RegionVocabulary.load(model=model, fixture_type=fixture_type)
