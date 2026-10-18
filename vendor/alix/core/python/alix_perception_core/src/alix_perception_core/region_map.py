"""Which fixture region a detection sits in, from where it falls on the fixture.

Ported from `butterfli_perception/scripts/ai_client.py`
(`_COMMODE_REGION_RULES`, `_map_to_region`, `_get_fixture_bbox`). Without this a
stain detector produces positions nobody can act on: `RunCleaningTask` is keyed
by region, so an unrouted observation is real and useless.

The rules are **bands on the fixture's own bounding box**, not on the image. That
is the part worth keeping: normalising against the fixture makes the mapping
survive a change in camera distance and small viewpoint changes, which
normalising against the frame does not. If the status model found no fixture
box, there is no denominator and the mapping declines rather than guessing
against the frame.

Two changes from butterfli's version:

* **The table is data, not a class attribute.** It ships as `region_bands.yaml`
  and can be overridden per fixture model, because these numbers are a
  *geometric guess about one fixture seen from one pose* and they are the first
  thing that will need tuning when either changes. There is no second copy in
  code: if the file is unreadable the table is empty and every detection comes
  back unrouted, which is visible, rather than silently falling back to numbers
  nobody has looked at since they were written.
* **The output is validated.** butterfli's fallback returned `commode_side` for
  anything unmatched, which silently routes an unrecognised position to a real
  cleanable region. Here an unmatched position returns `""` — unrouted, counted,
  and visible — because inventing a region sends the arm somewhere on the
  strength of nothing.

WHAT THESE NUMBERS ARE, HONESTLY. They are hand-set bands, authored against
butterfli's camera pose, and they are not measured. They assume the image is
upright (the core de-rotates before inference, so that holds) and that the
fixture is seen roughly face-on. They are a reasonable starting point and they
are not a substitute for ray-casting against the labelled region meshes, which
needs a measured fixture pose — see
`Resource/reports/2026-09-08_perception_port` §6.1.
"""

from __future__ import annotations

import functools
import logging
from pathlib import Path

import yaml

logger = logging.getLogger("alix_perception_core.region_map")

#: Bands live in `config/region_bands.yaml`, not here. They were a class
#: attribute in butterfli (`_COMMODE_REGION_RULES`) and a module constant here
#: until 2026-10-03; both made a hand-tuned geometric guess about ONE fixture
#: seen from ONE pose into something you needed a code change to re-tune. They
#: are the first numbers that will move when either changes.
BANDS_FILE = Path(__file__).resolve().parent / "config" / "region_bands.yaml"


@functools.lru_cache(maxsize=1)
def packaged_bands() -> dict[str, dict[str, list]]:
    """The shipped band table. Empty — and loud — if the file is unreadable.

    Empty rather than a built-in copy: a silent fallback table would be a second
    source for numbers that exist to be tuned, and the two would drift the first
    time someone edited the file. An empty table routes nothing, which
    `analysis.py` already reports as `unrouted_no_region` — visible, countable
    and obviously wrong, which a stale duplicate is not.
    """
    try:
        with BANDS_FILE.open() as handle:
            loaded = yaml.safe_load(handle) or {}
    except OSError as exc:
        logger.error("region bands not readable at %s (%s); every detection "
                     "will come back unrouted", BANDS_FILE, exc)
        return {}
    return _parse(loaded.get("bands") or {})


def _parse(bands: dict) -> dict[str, dict[str, list]]:
    return {
        fixture: {state: [tuple(row) for row in rows]
                  for state, rows in (states or {}).items()}
        for fixture, states in bands.items()
    }


class RegionMapper:
    """Position -> region, for one band table."""

    def __init__(self, bands: dict | None = None):
        self.bands = bands if bands is not None else packaged_bands()

    @classmethod
    def load(cls, path: str | Path | None = None) -> "RegionMapper":
        """Load a band table, falling back to the packaged defaults."""
        if path is None:
            path = Path(__file__).resolve().parent / "config" / "region_bands.yaml"
        path = Path(path)
        if not path.is_file():
            return cls()
        with path.open() as handle:
            loaded = yaml.safe_load(handle) or {}
        bands = loaded.get("bands")
        if not bands:
            return cls()
        return cls(_parse(bands))

    def region_for(self, fixture_type: str, fixture_state: str,
                   detection_center: tuple[float, float],
                   fixture_bbox: tuple[float, float, float, float] | None) -> str:
        """Region name for a detection centre, or "" if it cannot be decided.

        Args:
            detection_center: (x, y) in image pixels.
            fixture_bbox: (x0, y0, x1, y1) of the fixture in the same image.
                None means the status model found no fixture box, and then there
                is no denominator — the answer is "" rather than a guess against
                the frame, which would move with the camera.
        """
        if fixture_bbox is None:
            return ""
        x0, y0, x1, y1 = fixture_bbox
        width, height = x1 - x0, y1 - y0
        if width <= 0 or height <= 0:
            return ""

        cx, cy = detection_center
        fx = (cx - x0) / width
        fy = (cy - y0) / height
        # Outside the fixture box entirely: not on the fixture, so not a region
        # of it. Clamping would put every background detection on an edge region.
        if not (0.0 <= fx <= 1.0 and 0.0 <= fy <= 1.0):
            return ""

        states = self.bands.get(fixture_type)
        if not states:
            return ""
        rows = states.get(fixture_state)
        if rows is None:
            # No band table for this state. butterfli fell back to the
            # lid_seat_open table here; that maps a closed lid onto bowl
            # interiors, so it is not carried over.
            return ""

        for y_min, y_max, x_min, x_max, region in rows:
            if y_min <= fy < y_max and x_min <= fx < x_max:
                return region
        return ""


@functools.lru_cache(maxsize=4)
def load_mapper(path: str | None = None) -> RegionMapper:
    return RegionMapper.load(path)
