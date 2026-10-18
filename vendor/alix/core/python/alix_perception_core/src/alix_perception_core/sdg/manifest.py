"""What a synthetic run produced, recorded so it can be scored and trusted.

Two outputs, and the split matters:

**`manifest.jsonl`** — one row per frame, carrying everything the renderer knew:
the seed, the camera's world pose, every randomised value it drew, each placed
soil instance with its REGION, and a sha256 of the image. This is the record, and
it is deliberately richer than any training format.

**`corpus.yaml`** — the same frames in the exact shape
`tools/perception/eval_models.py` already reads. This is the shortest route from
"we have frames" to "we have a number": the two shipped models can be scored on a
synthetic set with **zero code change** to the evaluator.

WHY THE REGION IS THE PART WORTH HAVING
----------------------------------------
No human annotator can label a photograph with *which fixture region* a stain is
on — not reliably, not at scale, and not in a way that agrees with
`regions.yaml`. A renderer can, because it placed the stain there. Today region
routing runs on `config/region_bands.yaml`, whose own header says it is *"a
hand-set geometric guess about ONE fixture seen from ONE pose, NOT measured"*.
A region-labelled dataset is the first thing that could score it.

`corpus.yaml` has no slot for a region, so it goes in the manifest and the
manifest is the file to keep.

WHAT THE MANIFEST PROMISES, AND WHAT IT DOES NOT
-------------------------------------------------
It does **not** promise that re-running the plan reproduces the images. That was
measured and is unavailable: identical seeds give identical scene state to six
decimal places but different pixels under RTX RealTime at any `rt_subframes`. So
the contract is **scene and label reproducibility with the images archived**, and
every row carries a hash so a later claim about a frame can be checked against
the frame.

Every row also carries `ideal_depth: true`. Isaac's depth has no MinZ blind zone,
no stereo dropout on textureless porcelain, no specular holes and never a zero
no-data pixel — `d435.yaml` says so outright with `simulates_min_z: false`. A
consumer that forgets this will conclude the depth path is more robust than it is.
"""

from __future__ import annotations

import hashlib
import json
from dataclasses import asdict, dataclass, field
from pathlib import Path

import yaml

MANIFEST_NAME = "manifest.jsonl"
CORPUS_NAME = "corpus.yaml"


@dataclass
class SoilInstance:
    """One placed piece of soil, as the renderer placed it."""

    #: The class name written into the label map, e.g. `stain` or `limescale`.
    label: str

    #: THE REGION IT WAS PLACED ON — the thing no photograph can carry. Comes
    #: from the named group in `<component>_regions.obj` that was sampled, mapped
    #: through `regions.yaml`'s `groups:` to the declared region name.
    region: str = ""

    #: Which of body/lid/seat it sits on. A region can span components
    #: (`lid_side` is lid + seat), and the two carry different transforms, so the
    #: component is not recoverable from the region name alone.
    component: str = ""

    #: Where, in the fixture's own frame — so a frame remains interpretable if
    #: the fixture is later re-posed.
    position_local: tuple = (0.0, 0.0, 0.0)
    normal_local: tuple = (0.0, 0.0, 1.0)

    #: Size on the surface, millimetres. Comparable with `min_stain_mm`, which is
    #: the gate this dataset finally makes measurable at a KNOWN range.
    size_mm: float = 0.0

    #: Filled from the label pass. 0 means the instance was placed and is not
    #: visible — occluded, off-frame, or facing away. A frame whose every
    #: instance has 0 visible pixels is labelled "stained" and shows nothing,
    #: which is a training error, so it is recorded rather than assumed.
    visible_px: int = 0

    #: (x0, y0, x1, y1) pixels, from `bounding_box_2d_tight` on the label pass.
    bbox_px: tuple | None = None

    #: Where the blob's centroid PROJECTS in the image, (u, v, depth_m), whether
    #: or not it is visible. The thing that makes a 0-visible blob diagnosable:
    #: off-frame, behind the lid, or a transform error are three different
    #: failures and this tells them apart.
    uv_px: tuple | None = None

    #: Where in the stage it was when photographed — the 3D truth a
    #: localisation estimate is scored against.
    position_stage: tuple = (0.0, 0.0, 0.0)

    #: The appearance pattern it was deposited with (`Appearance.pattern`:
    #: blob / splatter / drip / ring / speckle / film) and its mean moisture
    #: when photographed. A detector that finds every splatter and no ring is
    #: a finding only if the manifest says which was which.
    pattern: str = ""
    moisture: float = 0.0


@dataclass
class FrameRecord:
    """One rendered frame."""

    index: int
    image: str                        # relative to the dataset directory
    fixture_state: str = ""
    fixture_model: str = "commode_c"

    soil: list = field(default_factory=list)

    depth: str = ""                   # relative path, if depth was captured
    #: Isaac's depth is IDEAL. See the module docstring. Never drop this.
    ideal_depth: bool = True

    #: Camera pose in the frame positions are expressed against, so a consumer
    #: can re-derive geometry without re-running the sim.
    camera_position: tuple = (0.0, 0.0, 0.0)
    camera_quat_xyzw: tuple = (0.0, 0.0, 0.0, 1.0)
    intrinsics: dict = field(default_factory=dict)

    #: Everything the randomiser drew for this frame, by name. A plan records
    #: ranges; this records the values, which is what makes a surprising frame
    #: explicable afterwards.
    randomised: dict = field(default_factory=dict)

    seed: int = 0
    sha256: str = ""

    @property
    def stain_count(self) -> int:
        """Instances that are actually VISIBLE — what a detector could find.

        Not the placed count. A model scored against placed-but-occluded soil is
        being blamed for not seeing through porcelain.
        """
        return sum(1 for s in self.soil if s.visible_px > 0)


def sha256_of(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


class ManifestWriter:
    """Append frame records, then export a corpus the evaluator already reads."""

    def __init__(self, directory: str | Path, plan=None):
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=True)
        self.plan = plan
        self.records: list[FrameRecord] = []
        self._path = self.directory / MANIFEST_NAME
        # Truncated on open: a half-written manifest from a crashed run must not
        # be silently appended to, because the row count is what a later
        # integrity check compares against the file count.
        self._path.write_text("")

    def add(self, record: FrameRecord, hash_image: bool = True) -> FrameRecord:
        if hash_image and not record.sha256:
            image_path = self.directory / record.image
            if image_path.is_file():
                record.sha256 = sha256_of(image_path)
        self.records.append(record)
        with self._path.open("a") as handle:
            handle.write(json.dumps(_as_jsonable(record), sort_keys=True) + "\n")
        return record

    # ------------------------------------------------------------------
    def export_corpus(self, path: str | Path | None = None,
                      relative_to: Path | None = None) -> Path:
        """Write `corpus.yaml` in the exact shape `eval_models.load_corpus` reads.

        That schema is `{images: [{path, fixture_state, stains}]}`, with a
        relative `path` resolved against the corpus file's own directory
        (`eval_models.py:73-75`). Writing it means a synthetic set is scored by
        the SAME tool, with the SAME thresholds, as the real photographs — which
        is the only way the two numbers can be put side by side.

        `stains` is the VISIBLE count, not the placed count. See
        `FrameRecord.stain_count`.
        """
        path = Path(path) if path else (self.directory / CORPUS_NAME)
        base = relative_to or path.parent

        images = []
        for record in self.records:
            image_path = self.directory / record.image
            try:
                written = image_path.relative_to(base)
            except ValueError:
                written = image_path.resolve()
            row = {"path": str(written), "stains": record.stain_count}
            if record.fixture_state:
                row["fixture_state"] = record.fixture_state
            images.append(row)

        header = _corpus_header(self.plan, len(images), self.directory)
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("w") as handle:
            handle.write(header)
            yaml.safe_dump({"images": images}, handle, sort_keys=False,
                           default_flow_style=False)
        return path

    def summary(self) -> dict:
        placed = sum(len(r.soil) for r in self.records)
        visible = sum(r.stain_count for r in self.records)
        regions: dict[str, int] = {}
        states: dict[str, int] = {}
        for record in self.records:
            states[record.fixture_state] = states.get(record.fixture_state, 0) + 1
            for instance in record.soil:
                if instance.visible_px > 0:
                    regions[instance.region] = regions.get(instance.region, 0) + 1
        return {
            "frames": len(self.records),
            "soil_placed": placed,
            "soil_visible": visible,
            # Placed-but-invisible is the quiet failure: a frame labelled
            # "stained" showing nothing teaches the detector that porcelain is a
            # stain. Surfaced here rather than left to be noticed.
            "soil_occluded": placed - visible,
            "clean_frames": sum(1 for r in self.records if r.stain_count == 0),
            "by_state": states,
            "by_region": dict(sorted(regions.items())),
        }


def load_manifest(directory: str | Path) -> list[FrameRecord]:
    """Read a manifest back. Rows with unknown keys are refused, not ignored.

    A manifest written by a newer version carrying a field this one drops would
    otherwise be read as if the field had never existed, and the dropped field is
    exactly the sort of thing (`ideal_depth`) a consumer must not miss.
    """
    path = Path(directory)
    if path.is_dir():
        path = path / MANIFEST_NAME
    known = set(FrameRecord.__dataclass_fields__)
    soil_known = set(SoilInstance.__dataclass_fields__)

    records = []
    for number, line in enumerate(path.read_text().splitlines(), start=1):
        line = line.strip()
        if not line:
            continue
        row = json.loads(line)
        unknown = set(row) - known
        if unknown:
            raise ValueError(
                f"{path}:{number} carries unknown field(s) "
                f"{', '.join(sorted(unknown))} — written by a newer manifest "
                f"version. Silently dropping them would hide exactly the kind of "
                f"caveat they are added to carry.")
        soil = []
        for entry in row.pop("soil", []):
            extra = set(entry) - soil_known
            if extra:
                raise ValueError(
                    f"{path}:{number} soil entry carries unknown field(s) "
                    f"{', '.join(sorted(extra))}")
            soil.append(SoilInstance(**entry))
        records.append(FrameRecord(soil=soil, **row))
    return records


def _as_jsonable(record: FrameRecord) -> dict:
    row = asdict(record)
    row["soil"] = [asdict(s) if not isinstance(s, dict) else s for s in record.soil]
    return row


def _corpus_header(plan, count: int, directory: Path) -> str:
    name = getattr(plan, "name", "unnamed")
    seed = getattr(plan, "seed", "?")
    return (
        "# =============================================================\n"
        f"# SYNTHETIC corpus — generated, not photographed\n"
        "# =============================================================\n"
        f"# plan: {name}   seed: {seed}   frames: {count}\n"
        f"# dataset: {directory}\n"
        "#\n"
        "# Read by tools/perception/eval_models.py, the same tool and the same\n"
        "# thresholds that score the real photographs — which is the point: the\n"
        "# two numbers can be compared.\n"
        "#\n"
        "# THIS IS A DIAGNOSTIC, NOT A GATE. Resource/Data/corpus.yaml is the\n"
        "# gate: 16 real images, 13 of them clean. A score here never appears\n"
        "# without the real one beside it, because a model that learned this\n"
        "# renderer scores well here by construction.\n"
        "#\n"
        "# `stains` is the VISIBLE instance count, not the placed count: soil\n"
        "# that was placed and then occluded is not something a detector can be\n"
        "# blamed for missing. manifest.jsonl carries both, plus each instance's\n"
        "# REGION, which no photographic corpus can.\n"
        "# =============================================================\n\n")
