"""Local inference — the default and priority backend.

Models are developed in-house (Newton, 2026-09-08) and a customer washroom has no
network guarantee, so this path is the one that has to work. Two design choices
follow from "we will be developing ML models locally", and both are about making
the NEXT model cheap rather than making this one work:

**The class-to-meaning map lives in `models.yaml`, not in code.** butterfli
hard-coded a `_categorize_classes` routine that substring-matched class names
against five pattern lists and mapped them to states inline; a retrained model
with different labels needed a code change, and two of those lists shipped with
the comment *"Placeholder — update with actual class names from the target
model"*. Here a model declares what each of its classes MEANS, so a new model is
a manifest entry:

    models:
      status:
        task: status
        runtime: ultralytics
        weights: models/commode_status_v2.pt
        confidence: 0.25
        classes:
          lid_closed:    {fixture_type: commode, fixture_state: closed}
          lid_open:      {fixture_type: commode, fixture_state: lid_open}
          biohazard:     {kind: escalate}

**The runtime is per-model and imported lazily.** `ultralytics` for `.pt`,
`onnxruntime` for `.onnx` — declared by the model, not by this package, so the
Jetson can run ONNX while a workstation runs PyTorch weights against the same
manifest and the same code.

Both models ported from `butterfli_perception` ship in `models/` and are wired as
the packaged default, so `create("local")` works with no configuration. Their
measured quality is poor — 76 false positives over 13 clean images, fixture state
correct on 7 of 14 (`config/models.yaml` header, and
`Resource/reports/2026-09-08_perception_port` §2.1-§2.2) — and that is a model
problem to fix by retraining, not a reason for the pipeline to refuse to run.
What IS an error rather than a fallback is a manifest that cannot be loaded or
declares nothing: silence there would leave the robot with no perception and no
message saying so.

Against that failure specifically, `min_stain_mm` gates detections on PHYSICAL
size rather than pixels. A fiducial dot is about 8-10 mm; a stain worth sending
an arm to is bigger. A pixel threshold cannot express that — it changes meaning
with range — but a millimetre threshold survives a resolution change, a new
inspection pose and a retrained model.

WHAT THE CLASS KINDS BUY (ported from butterfli's buckets, ai_client.py:940-993).
Five meanings, and the behaviour each one earns:

    fixture     locates the fixture. Its box is the denominator every region
                band is measured against, so without one the regions are unknown
                rather than guessed.
    status      locates it AND names its state. Implies `fixture`.
    stain       cleanable. Counted, routed, and allowed to drive a re-clean.
    permanent   real, located, and not removable. Reported separately and NEVER
                allowed to drive a re-clean: a mark that survives a perfect clean
                would otherwise loop the robot forever.
    escalate    a person must look. Outranks every other recommendation.
    ignore      detected on purpose and meaningless (printed branding, fiducials).
"""

from __future__ import annotations

import logging
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import yaml

from ..paths import models_manifest, repo_root
from ..region_map import RegionMapper, load_mapper
from ..regions import RegionVocabulary, load_vocabulary
from ..types import (FIXTURE_TYPES, KIND_ESCALATE, KIND_FIXTURE, KIND_IGNORE,
                     KIND_PERMANENT, KIND_STAIN, KIND_STATUS, KINDS, PERMANENT,
                     Detection, FixtureStatus, severity_for)
from .base import Backend, BackendError, register

logger = logging.getLogger("alix_perception_core.local")

#: Smallest stain worth reporting, millimetres on the surface. Below this a
#: detection is a marker, a screw head, a water speck or a shadow. Overridable
#: per-deployment in models.yaml; the default is set by the fiducial dots on the
#: test rig, which are the known false positive.
DEFAULT_MIN_STAIN_MM = 15.0

#: The state assumed when the model has NO state-classification classes at all.
#: butterfli carried two of these tables — `_DEFAULT_STATUS_STATE` (commode ->
#: closed) and `_DEFAULT_STAIN_STATE` (commode -> lid_seat_open, ai_client.py:
#: 639-649) — so the same frame was called `closed` by check_status and
#: `lid_seat_open` by detect_stains, and the stains were mapped with bands for a
#: state the status pass had just denied. One table here, and the stain pass
#: takes the state from the status pass rather than re-deriving it.
#:
#: `closed` is the conservative choice: it maps nothing onto a bowl interior, so
#: a model that cannot read state produces no confident interior routing. The
#: real fix is a model with state classes, and `state_is_default` says loudly
#: which case you are in. Override per deployment with `default_fixture_state:`.
DEFAULT_FIXTURE_STATE = {
    "commode": "closed",
    "urinal": "default",
    "washtab": "default",
}

#: Confidence credited to a caller-supplied `fixture_hint` that the model does
#: not corroborate. butterfli's value (ai_client.py:1023). It is deliberately
#: below the 0.65 state gate in analysis.py: a hint says WHAT the fixture is, and
#: nothing at all about what state it is in.
HINT_CONFIDENCE = 0.7


@dataclass
class ClassMeaning:
    """What one model class means, as `models.yaml` declares it."""

    name: str
    kind: str
    fixture_type: str = ""
    fixture_state: str = ""
    region: str = ""
    severity: str = ""

    @classmethod
    def from_dict(cls, name: str, spec, task: str, model_key: str) -> "ClassMeaning":
        """Parse one `classes:` entry, including the pre-`kind:` spellings.

        The manifest format grew a `kind:` field; the two shipped models were
        written before it. Rather than rewrite data that is already correct, the
        older spellings are read as what they plainly mean:

            fixture_state present   -> status  (it names a state, so it reads one)
            fixture_type only       -> fixture (it finds a fixture, not a state)
            is_stain: false         -> ignore
            on a stain model        -> stain

        An explicit `kind:` always wins.
        """
        if spec is None:
            spec = {}
        if not isinstance(spec, dict):
            raise BackendError(
                f"model '{model_key}' class '{name}': expected a mapping, got "
                f"{type(spec).__name__}")

        kind = str(spec.get("kind") or "").strip()
        if not kind:
            if spec.get("fixture_state"):
                kind = KIND_STATUS
            elif spec.get("fixture_type"):
                kind = KIND_FIXTURE
            elif spec.get("is_stain") is False:
                kind = KIND_IGNORE
            elif task == "stain":
                kind = KIND_STAIN
            else:
                raise BackendError(
                    f"model '{model_key}' class '{name}' declares no kind and "
                    f"nothing that implies one. Add kind: one of "
                    f"{', '.join(KINDS)}.")
        if kind not in KINDS:
            raise BackendError(
                f"model '{model_key}' class '{name}' declares kind '{kind}'; "
                f"known kinds: {', '.join(KINDS)}")

        fixture_type = str(spec.get("fixture_type") or "")
        if fixture_type and fixture_type not in FIXTURE_TYPES:
            raise BackendError(
                f"model '{model_key}' class '{name}' declares fixture_type "
                f"'{fixture_type}'; known: {', '.join(FIXTURE_TYPES)}")
        if kind == KIND_STATUS and not spec.get("fixture_state"):
            raise BackendError(
                f"model '{model_key}' class '{name}' is kind 'status' but names "
                f"no fixture_state; a state you cannot name is not one to act on.")

        return cls(name=name, kind=kind, fixture_type=fixture_type,
                   fixture_state=str(spec.get("fixture_state") or ""),
                   region=str(spec.get("region") or ""),
                   severity=str(spec.get("severity") or ""))

    @property
    def locates_fixture(self) -> bool:
        """Does a detection of this class tell you where the fixture is?

        A status class does: it fired because it saw a commode with its lid down,
        so its box IS the commode's box. butterfli got this for free because its
        `_categorize_classes` filed a name like `toilet_lid_closed` into the
        status bucket AND the fixture bucket (the two loops are independent,
        ai_client.py:948-968) — which is the only reason `_get_fixture_bbox`
        returned anything for the shipped status model, whose five classes are
        all state classes.
        """
        return self.kind in (KIND_FIXTURE, KIND_STATUS)


@dataclass
class ModelSpec:
    """One model, as `models.yaml` declares it."""

    key: str
    task: str                      # "status" | "stain"
    runtime: str                   # "ultralytics" | "onnx"
    weights: Path
    confidence: float = 0.25
    classes: dict[str, ClassMeaning] = field(default_factory=dict)

    @classmethod
    def from_dict(cls, key: str, spec: dict, base_dir: Path) -> "ModelSpec":
        try:
            weights = Path(spec["weights"])
        except KeyError as exc:
            raise BackendError(f"model '{key}' has no 'weights'") from exc
        if not weights.is_absolute():
            root = repo_root()
            for candidate in (base_dir / weights, (root / weights) if root else None):
                if candidate is not None and candidate.is_file():
                    weights = candidate
                    break
        task = spec.get("task", "")
        if task not in ("status", "stain"):
            raise BackendError(
                f"model '{key}' has task '{task}'; expected 'status' or 'stain'")
        classes = {
            name: ClassMeaning.from_dict(name, meaning, task, key)
            for name, meaning in (spec.get("classes") or {}).items()
        }
        return cls(key=key, task=task,
                   runtime=spec.get("runtime", "ultralytics"),
                   weights=weights,
                   confidence=float(spec.get("confidence", 0.25)),
                   classes=classes)

    def of_kind(self, *kinds: str) -> tuple[str, ...]:
        return tuple(sorted(n for n, m in self.classes.items() if m.kind in kinds))


class _UltralyticsRunner:
    """YOLO `.pt` weights. Imported here so the dep is optional."""

    def __init__(self, spec: ModelSpec):
        try:
            from ultralytics import YOLO
        except ImportError as exc:
            raise BackendError(
                f"model '{spec.key}' declares runtime 'ultralytics', which is not "
                f"installed. pip install 'alix_perception_core[ultralytics]'") from exc
        if not spec.weights.is_file():
            raise BackendError(
                f"model '{spec.key}': weights not found at {spec.weights}")
        self._spec = spec
        self._model = YOLO(str(spec.weights))
        self.names = dict(self._model.names)

    def infer(self, image: np.ndarray) -> list[tuple[str, float, tuple]]:
        """-> [(class_name, confidence, (x0, y0, x1, y1)), ...] in image pixels."""
        result = self._model.predict(image, conf=self._spec.confidence,
                                     verbose=False)[0]
        out = []
        for box in result.boxes:
            name = self.names.get(int(box.cls), str(int(box.cls)))
            xyxy = tuple(float(v) for v in box.xyxy[0])
            out.append((name, float(box.conf), xyxy))
        return out


class _OnnxRunner:
    """ONNX weights — the Jetson deployment path.

    Deliberately unimplemented rather than half-implemented: the output tensor
    layout is a property of how the model was exported, and guessing it produces
    plausible boxes in the wrong places. Wire it against a real exported model,
    with `tools/perception/eval_models.py` as the gate.
    """

    def __init__(self, spec: ModelSpec):
        try:
            import onnxruntime  # noqa: F401
        except ImportError as exc:
            raise BackendError(
                f"model '{spec.key}' declares runtime 'onnx', which is not "
                f"installed. pip install 'alix_perception_core[onnx]'") from exc
        raise BackendError(
            f"model '{spec.key}': the onnx runtime is declared but its output "
            f"decoding is not implemented — the layout depends on how the model "
            f"was exported. Implement it against the real export and gate it with "
            f"tools/perception/eval_models.py.")


_RUNNERS = {"ultralytics": _UltralyticsRunner, "onnx": _OnnxRunner}


class LocalBackend(Backend):
    """Offline inference from locally-developed models."""

    name = "local"

    def __init__(self, manifest_path: str | Path | None = None,
                 min_stain_mm: float | None = None,
                 require_fixture_bbox: bool = True,
                 fixture_model: str = "commode_c"):
        path = Path(manifest_path) if manifest_path else models_manifest()
        if path is None or not Path(path).is_file():
            raise BackendError(
                "no local model manifest. Point ALIX_PERCEPTION_MODELS at a "
                "models.yaml, or pass models_manifest to the node. See "
                "core/python/alix_perception_core/src/alix_perception_core/config/models.yaml "
                "for the format.")
        path = Path(path)
        with path.open() as handle:
            loaded = yaml.safe_load(handle) or {}

        self._manifest_path = path
        self.min_stain_mm = float(
            min_stain_mm if min_stain_mm is not None
            else loaded.get("min_stain_mm", DEFAULT_MIN_STAIN_MM))

        # butterfli's "frame not aimed at the fixture" refusal (ai_client.py:
        # 1066-1071). A stain box is only meaningful as a fraction of the fixture
        # box; with no fixture found, every region would be a guess against the
        # frame, which moves with the camera. Returning nothing is the honest
        # answer. Switchable because a stain-only model has no fixture class at
        # all, and for that model the refusal would reject every frame.
        self.require_fixture_bbox = bool(require_fixture_bbox)

        self._default_state = dict(DEFAULT_FIXTURE_STATE)
        self._default_state.update(loaded.get("default_fixture_state") or {})

        specs = {
            key: ModelSpec.from_dict(key, spec, path.parent)
            for key, spec in (loaded.get("models") or {}).items()
        }
        if not specs:
            raise BackendError(
                f"{path} declares no models, so there is nothing to infer with. "
                f"The packaged manifest declares two; if you are pointing at your "
                f"own, add a 'status' and a 'stain' entry — see "
                f"core/python/alix_perception_core/src/alix_perception_core/"
                f"config/models.yaml for the format.")

        self._status = self._one(specs, "status")
        self._stain = self._one(specs, "stain")
        self._runners: dict[str, object] = {}
        self._mapper: RegionMapper = load_mapper()
        self._vocabulary: RegionVocabulary = load_vocabulary(model=fixture_model)

        logger.info("local backend: status=%s stain=%s (manifest %s)",
                    self._status.key if self._status else None,
                    self._stain.key if self._stain else None, path)
        logger.info("capabilities: state=%s escalation=%s permanent=%s",
                    self.supports_state_classification,
                    self.supports_escalation, self.supports_permanent_detection)

    @staticmethod
    def _one(specs: dict[str, ModelSpec], task: str) -> ModelSpec | None:
        found = [s for s in specs.values() if s.task == task]
        if len(found) > 1:
            raise BackendError(
                f"{len(found)} models declare task '{task}' "
                f"({', '.join(s.key for s in found)}); exactly one is expected")
        return found[0] if found else None

    def _runner(self, spec: ModelSpec):
        """Load on first use, so an unused model never costs load time."""
        if spec.key not in self._runners:
            runner_cls = _RUNNERS.get(spec.runtime)
            if runner_cls is None:
                raise BackendError(
                    f"model '{spec.key}' declares unknown runtime "
                    f"'{spec.runtime}'; known: {', '.join(sorted(_RUNNERS))}")
            self._runners[spec.key] = runner_cls(spec)
        return self._runners[spec.key]

    def _all_classes(self) -> dict[str, ClassMeaning]:
        out: dict[str, ClassMeaning] = {}
        for spec in (self._status, self._stain):
            if spec is not None:
                out.update(spec.classes)
        return out

    # -- capabilities --------------------------------------------------
    # Read off the manifest rather than guessed from class-name substrings. The
    # question each answers is "can this model ever say that", which the manifest
    # knows exactly and a pattern list only approximates.

    @property
    def supports_state_classification(self) -> bool:
        return any(m.kind == KIND_STATUS for m in self._all_classes().values())

    @property
    def supports_escalation(self) -> bool:
        return any(m.kind == KIND_ESCALATE for m in self._all_classes().values())

    @property
    def supports_permanent_detection(self) -> bool:
        return any(m.kind == KIND_PERMANENT for m in self._all_classes().values())

    def capabilities(self) -> dict:
        """Everything a caller might want to log once at startup."""
        return {
            "backend": self.name,
            "manifest": str(self._manifest_path),
            "status_model": self._status.key if self._status else None,
            "stain_model": self._stain.key if self._stain else None,
            "state_classification": self.supports_state_classification,
            "escalation": self.supports_escalation,
            "permanent_detection": self.supports_permanent_detection,
            "fixture_classes": sorted(
                n for n, m in self._all_classes().items() if m.locates_fixture),
            "min_stain_mm": self.min_stain_mm,
            "require_fixture_bbox": self.require_fixture_bbox,
        }

    # -- Backend -------------------------------------------------------
    def check_status(self, image, fixture_hint: str = "") -> FixtureStatus:
        if self._status is None:
            raise BackendError(
                f"{self._manifest_path} declares no model with task 'status'")

        detections = self._runner(self._status).infer(image)
        if not detections:
            # No fixture found is a legitimate answer and must be distinguishable
            # from a confident one. butterfli's classifier had no negative class,
            # so it named a fixture in an empty room.
            return FixtureStatus()

        classes = self._status.classes
        unknown = {name for name, _, _ in detections if name not in classes}
        if unknown:
            raise BackendError(
                f"model '{self._status.key}' returned class(es) "
                f"{', '.join(sorted(unknown))}, which its manifest does not map. "
                f"Add them under classes: with a kind, and fixture_type / "
                f"fixture_state where they apply.")

        fixture_type, type_confidence = self._resolve_fixture_type(
            detections, classes, fixture_hint)

        # The fixture box: the highest-confidence detection of a class that
        # LOCATES the fixture, and that agrees with the resolved type. Boxes from
        # classes that only grade dirt would put the region denominator on a
        # stain rather than on the commode.
        bbox, bbox_conf = None, 0.0
        for name, conf, box in detections:
            meaning = classes[name]
            if not meaning.locates_fixture:
                continue
            if meaning.fixture_type and fixture_type and meaning.fixture_type != fixture_type:
                continue
            if conf > bbox_conf:
                bbox, bbox_conf = box, conf

        state, state_conf, is_default = self._resolve_state(
            detections, classes, fixture_type)

        return FixtureStatus(fixture_type=fixture_type, fixture_state=state,
                             confidence=state_conf if not is_default else type_confidence,
                             bbox=bbox, state_is_default=is_default)

    def _resolve_fixture_type(self, detections, classes,
                              fixture_hint: str) -> tuple[str, float]:
        """Which fixture is this? The caller's hint wins when it is a real type.

        butterfli's `_detect_fixture` (ai_client.py:1017-1045). The hint takes
        priority because the caller is the task, and the task names its own
        fixture: making the model re-derive from pixels something the orchestrator
        already knows adds a way to be wrong and no way to be more right. The
        model is still consulted — if it corroborates the hint, its confidence is
        used instead of the flat `HINT_CONFIDENCE`.

        One difference from butterfli, deliberate: there is no `best_fixture =
        "commode"` fallback (ai_client.py:1038). Defaulting to the commonest
        fixture means an unrecognised frame is reported as a commode seen at 0.5
        confidence, and 0.5 is above several thresholds. Here an unrecognised
        frame reports no fixture, which `analysis.py` already handles as
        "no fixture recognised in the frame".
        """
        if fixture_hint:
            if fixture_hint not in FIXTURE_TYPES:
                logger.warning("fixture_hint %r is not a known fixture type (%s); "
                               "ignored", fixture_hint, ", ".join(FIXTURE_TYPES))
            else:
                for name, conf, _ in detections:
                    if classes[name].fixture_type == fixture_hint:
                        return fixture_hint, conf
                return fixture_hint, HINT_CONFIDENCE

        best_type, best_conf = "", 0.0
        for name, conf, _ in detections:
            meaning = classes[name]
            if meaning.locates_fixture and meaning.fixture_type and conf > best_conf:
                best_type, best_conf = meaning.fixture_type, conf
        return best_type, best_conf

    def _resolve_state(self, detections, classes,
                       fixture_type: str) -> tuple[str, float, bool]:
        """-> (state, confidence, is_default). butterfli's `_detect_state`.

        Votes only among classes that actually BEAR a state, so a high-confidence
        fixture-presence detection cannot outvote the one class that read the lid.
        """
        if not self.supports_state_classification:
            return self._default_state.get(fixture_type, ""), 0.0, True

        best_state, best_conf = "", 0.0
        for name, conf, _ in detections:
            meaning = classes[name]
            if meaning.kind != KIND_STATUS:
                continue
            if fixture_type and meaning.fixture_type and meaning.fixture_type != fixture_type:
                continue
            if conf > best_conf:
                best_state, best_conf = meaning.fixture_state, conf

        if not best_state:
            return self._default_state.get(fixture_type, ""), 0.0, True
        return best_state, best_conf, False

    def detect_stains(self, image, status: FixtureStatus | None = None
                      ) -> list[Detection]:
        if self._stain is None:
            raise BackendError(
                f"{self._manifest_path} declares no model with task 'stain'")

        status = status or FixtureStatus()
        raw = self._runner(self._stain).infer(image)

        # THE 'FRAME NOT AIMED AT THE FIXTURE' REFUSAL (butterfli ai_client.py:
        # 1066-1071). Only when the stack CAN find fixtures: a model with no
        # fixture-locating class anywhere would otherwise refuse every frame.
        if (self.require_fixture_bbox and status.bbox is None
                and self._can_locate_fixtures()):
            logger.warning(
                "no fixture located in this frame; %d stain detection(s) "
                "discarded rather than mapped against the image, which would "
                "move the regions with the camera", len(raw))
            return []

        out: list[Detection] = []
        for name, conf, (x0, y0, x1, y1) in raw:
            meaning = self._stain.classes.get(name)
            if meaning is None:
                # An unmapped class on the stain model is skipped, not fatal:
                # these models routinely carry auxiliary classes (butterfli's
                # carried `toilet-labelling`) that are not stains.
                logger.debug("stain model class '%s' unmapped; skipped", name)
                continue
            if meaning.kind in (KIND_IGNORE, KIND_FIXTURE, KIND_STATUS):
                continue

            det = Detection(
                bbox=(x0, y0, x1, y1), confidence=conf, label=name,
                kind=meaning.kind,
                severity=(meaning.severity or
                          (PERMANENT if meaning.kind == KIND_PERMANENT
                           else severity_for(conf))),
                region=meaning.region)
            if not det.region:
                det.region = self._region_for(det, status)
            out.append(det)
        return out

    def _can_locate_fixtures(self) -> bool:
        return any(m.locates_fixture for m in self._all_classes().values())

    def _region_for(self, det: Detection, status: FixtureStatus) -> str:
        """Region for a detection: from its own class name first, then geometry.

        butterfli's two-tier rule (ai_client.py:1148-1163). The class-name tier
        exists so a future model can carry the answer in its labels — a class
        called `stain_seat_ring` is a direct statement about which surface the
        mark is on, and it beats any band table because it came from the pixels
        rather than from an assumption about the viewing angle.

        Matched against THIS FIXTURE MODEL's declared regions, not against a
        hand-written list: butterfli checked a 10-entry `VALID_REGIONS` tuple,
        where alix's commode_c declares 21.
        """
        from_name = self._vocabulary.best_match(det.label)
        if from_name:
            return from_name
        return self._mapper.region_for(
            status.fixture_type, status.fixture_state, det.center, status.bbox)


def _factory(**kwargs) -> Backend:
    return LocalBackend(**kwargs)


register("local", _factory)
