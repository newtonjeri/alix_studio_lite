"""A vision-language backend — an ALTERNATIVE, never a fallback.

Newton, 2026-10-03: *"the current approach default is our developed models, not
the VLA, please do not put much focus on that end but it is good to have it as an
alternative"*. So this exists, it is registered, and it is reachable only by
asking for it by name:

    inference_backend: "cloud"          # node parameter, or
    create("cloud", api_key=...)        # in process

**There is no automatic fall-through to it.** butterfli's node fell back to
Gemini whenever the local weights were missing, which turned a broken model path
into a silent network dependency on a robot standing in a customer washroom with
no connectivity guarantee. `backends/base.create()` raises on an unknown name and
`DEFAULT_BACKEND` is `"local"`; nothing here changes either.

WHY IT IS WORTH HAVING AT ALL, stated plainly because the default says otherwise:
this is still the only recogniser with a field record. DESIGN.md §2.2 — 21 runs,
zero classification errors — against local models scoring 7/14 on state and 76
false positives over 13 clean images. The local path is the priority because of
where the robot runs, not because it is currently better. Keeping a working
alternative is what makes that an engineering choice rather than a corner.

KEPT SMALL ON PURPOSE. butterfli's `GeminiClient` is ~500 lines; this is the
subset that answers the `Backend` protocol. Dropped, with reasons:

* **the pricing table and USD cost estimate** — a hard-coded per-model price list
  is wrong the week the vendor changes it, and a wrong cost figure is worse than
  none. Token counts are reported; multiply them yourself against today's prices.
* **`difflib` region correction** — a fuzzy match on a 21-name vocabulary full of
  shared stems maps `lid_side` onto `lidsides_back` convincingly, and a wrong
  region sends an arm to the wrong surface. `RegionVocabulary.best_match` is used
  instead, and it refuses an ambiguous name.
* **the baseline before/after image pair** — real, and it needs a baseline store
  keyed by fixture, state and pose, which does not exist here yet. The prompt
  side is ported (`prompts.stain_prompt(baseline=True)`) so wiring a store is the
  only remaining work.

`google-genai` is imported lazily and declared as the `cloud` extra, so the core
installs and the local path runs with it absent.
"""

from __future__ import annotations

import json
import logging
import os
import re
import time

from ..prompts import stain_prompt, status_prompt
from ..regions import RegionVocabulary, load_vocabulary
from ..types import (FIXTURE_STATES, FIXTURE_TYPES, HEAVY, KIND_STAIN, LIGHT,
                     MODERATE, SEVERITIES, Detection, FixtureStatus)
from .base import Backend, BackendError, register

logger = logging.getLogger("alix_perception_core.cloud")

DEFAULT_MODEL = "gemini-2.5-flash"

#: Retries live HERE, not in the node. butterfli retried the whole analysis in
#: the node (perception_node.py:471-491), which also retried deterministic local
#: inference — 4 s of waiting for the identical failure. A network call is the
#: only thing on either path worth retrying, and only on a transport error: a
#: refusal or a malformed reply will be refused and malformed again.
MAX_ATTEMPTS = 3
BACKOFF_S = (1.0, 3.0)


class CloudBackend(Backend):
    """Gemini, asked for by name."""

    name = "cloud"

    #: This backend DOES take `custom_prompt_appendix`; the node checks this flag
    #: before telling a caller their instruction was ignored.
    accepts_prompt_appendix = True

    def __init__(self, api_key: str | None = None, model: str = DEFAULT_MODEL,
                 fixture_model: str = "commode_c", slim: bool = True,
                 min_stain_mm: float = 15.0, timeout_s: float = 30.0):
        key = api_key or os.environ.get("GEMINI_API_KEY") or os.environ.get("GOOGLE_API_KEY")
        if not key:
            raise BackendError(
                "the cloud backend needs an API key: set GEMINI_API_KEY, or pass "
                "api_key. It is never selected automatically, so if you are "
                "seeing this you asked for inference_backend='cloud'.")
        try:
            from google import genai
        except ImportError as exc:
            raise BackendError(
                "the cloud backend needs google-genai: "
                "pip install 'alix_perception_core[cloud]'") from exc

        self._genai = genai
        self._client = genai.Client(api_key=key)
        self._model = model
        self._slim = bool(slim)
        self._timeout_s = float(timeout_s)
        self.min_stain_mm = float(min_stain_mm)
        self._vocabulary: RegionVocabulary = load_vocabulary(model=fixture_model)
        #: Cumulative token counts. Not money — see the module docstring.
        self.tokens = {"prompt": 0, "response": 0, "calls": 0, "retries": 0}
        logger.info("cloud backend: model=%s, %d regions, slim=%s",
                    model, len(self._vocabulary), self._slim)

    # -- capabilities --------------------------------------------------
    # A VLM is asked for all three in the prompt and answers in prose, so it has
    # every capability in the sense that matters: it can say the thing.

    @property
    def supports_state_classification(self) -> bool:
        return True

    @property
    def supports_escalation(self) -> bool:
        return True

    @property
    def supports_permanent_detection(self) -> bool:
        return True

    # -- Backend -------------------------------------------------------
    def check_status(self, image, fixture_hint: str = "",
                     prompt_appendix: str = "") -> FixtureStatus:
        reply = self._ask(status_prompt(prompt_appendix), image, max_tokens=1024,
                          temperature=0.0)
        fixture_type = str(reply.get("fixture_type") or "")
        if fixture_hint and fixture_hint in FIXTURE_TYPES:
            # Same rule as the local path: the caller is the task, and the task
            # knows its own fixture.
            fixture_type = fixture_hint
        elif fixture_type and fixture_type not in FIXTURE_TYPES:
            logger.warning("model returned fixture_type %r; not a known type",
                           fixture_type)
            fixture_type = ""

        state = str(reply.get("fixture_state") or "")
        allowed = FIXTURE_STATES.get(fixture_type, ())
        if state and allowed and state not in allowed:
            logger.warning("model returned state %r for %s; allowed: %s",
                           state, fixture_type, ", ".join(allowed))
            state = ""

        # NO BBOX. A VLM asked for a fixture box returns one that is plausible
        # and unmeasured, and that box would become the denominator for every
        # region band. The cloud path names its regions directly in the reply
        # instead, so it does not need one — and reporting a made-up box would
        # hand the geometric router a confident wrong answer.
        return FixtureStatus(fixture_type=fixture_type, fixture_state=state,
                             confidence=float(reply.get("state_confidence") or 0.0),
                             bbox=None, state_is_default=not state)

    def detect_stains(self, image, status: FixtureStatus | None = None,
                      prompt_appendix: str = "") -> list[Detection]:
        status = status or FixtureStatus()
        prompt = stain_prompt(self._vocabulary, baseline=False, slim=self._slim,
                              appendix=prompt_appendix)
        reply = self._ask(prompt, image, max_tokens=4096, temperature=0.2)

        height, width = image.shape[:2]
        out: list[Detection] = []
        for raw in reply.get("stains") or []:
            det = self._detection(raw, width, height)
            if det is not None:
                out.append(det)

        permanent = str(reply.get("permanent_discoloration") or "").strip()
        if permanent:
            # Prose, not a located mark. butterfli carried the same string in
            # `permanent_discoloration_noted`. It is logged rather than invented
            # into a `permanent` ROI, because an ROI carries a position and this
            # has none.
            logger.info("permanent discolouration reported: %s", permanent[:200])
        return out

    def _detection(self, raw: dict, width: int, height: int) -> Detection | None:
        """One reply entry -> a Detection in THIS image's pixels, or None.

        The denormalisation butterfli got wrong often enough to be worth naming:
        the model is asked for 0-1000 normalised `[y_min, x_min, y_max, x_max]`,
        and that has to be scaled against the image the model actually saw. The
        caller passes that image, so the scaling happens here, once, beside the
        prompt that set the convention — not in a visualiser three modules away
        working from a different canvas.
        """
        box = raw.get("bbox")
        if not (isinstance(box, (list, tuple)) and len(box) == 4):
            logger.debug("stain entry with no usable bbox: %r", raw)
            return None
        try:
            y0, x0, y1, x1 = (float(v) for v in box)
        except (TypeError, ValueError):
            return None

        pixels = (x0 / 1000.0 * width, y0 / 1000.0 * height,
                  x1 / 1000.0 * width, y1 / 1000.0 * height)
        confidence = float(raw.get("confidence") or 0.0)
        severity = str(raw.get("severity") or "").lower()
        if severity not in SEVERITIES:
            severity = MODERATE if confidence >= 0.60 else LIGHT

        region = self._vocabulary.best_match(str(raw.get("region") or ""))
        if raw.get("region") and not region:
            # Reported, not silently dropped: a model naming a region nobody
            # recognises is a prompt problem, and `analysis.py` will count the
            # detection as unrouted so it stays visible.
            logger.warning("model named region %r, which %s does not declare",
                           raw.get("region"), self._vocabulary.model)

        return Detection(bbox=pixels, confidence=confidence,
                         label=str(raw.get("label") or "stain"),
                         region=region, kind=KIND_STAIN,
                         severity=HEAVY if severity == HEAVY else severity)

    # -- transport -----------------------------------------------------
    def _ask(self, prompt: str, image, *, max_tokens: int, temperature: float) -> dict:
        """One call, retried on TRANSPORT failure only. -> the parsed reply."""
        from google.genai import types

        payload = types.Part.from_bytes(data=_encode_jpeg(image), mime_type="image/jpeg")
        config = types.GenerateContentConfig(
            temperature=temperature,
            max_output_tokens=max_tokens,
            # Thinking off: this is a perception call on a robot waiting with an
            # arm in the air, and butterfli's 21-run field record was set with it
            # off. Latency is the budget.
            thinking_config=types.ThinkingConfig(thinking_budget=0),
        )

        last = None
        for attempt in range(MAX_ATTEMPTS):
            try:
                started = time.monotonic()
                response = self._client.models.generate_content(
                    model=self._model, contents=[payload, prompt], config=config)
                self._account(response, time.monotonic() - started)
                return _parse_json(response.text or "")
            except BackendError:
                # A malformed reply will be malformed again. Do not spend 4 s
                # proving it.
                raise
            except Exception as exc:                       # noqa: BLE001 — transport
                last = exc
                if attempt == MAX_ATTEMPTS - 1:
                    break
                delay = BACKOFF_S[min(attempt, len(BACKOFF_S) - 1)]
                self.tokens["retries"] += 1
                logger.warning("cloud call failed (%s); retrying in %.0f s "
                               "[%d/%d]", exc, delay, attempt + 2, MAX_ATTEMPTS)
                time.sleep(delay)
        raise BackendError(
            f"cloud backend ({self._model}) failed after {MAX_ATTEMPTS} attempts: "
            f"{last}. The local backend is the default and needs no network.")

    def _account(self, response, elapsed_s: float) -> None:
        usage = getattr(response, "usage_metadata", None)
        self.tokens["calls"] += 1
        if usage is not None:
            self.tokens["prompt"] += int(getattr(usage, "prompt_token_count", 0) or 0)
            self.tokens["response"] += int(
                getattr(usage, "candidates_token_count", 0) or 0)
        logger.info("cloud call %.2f s, %d prompt / %d response tokens cumulative",
                    elapsed_s, self.tokens["prompt"], self.tokens["response"])


def _encode_jpeg(image, quality: int = 90) -> bytes:
    """ndarray -> JPEG bytes. The model is charged by the image, so send one."""
    import io

    from PIL import Image as PILImage

    buffer = io.BytesIO()
    PILImage.fromarray(image[:, :, :3]).save(buffer, format="JPEG", quality=quality)
    return buffer.getvalue()


_FENCE = re.compile(r"^\s*```(?:json)?\s*|\s*```\s*$", re.MULTILINE)


def _parse_json(text: str) -> dict:
    """Pull the JSON object out of a reply. Raises `BackendError` if there is none.

    Models wrap JSON in markdown fences, prefix it with "Here is the analysis:",
    or both, despite being told not to. butterfli stripped fences then fell back
    to a greedy `{.*}` regex; both tiers are kept because both fire in practice.
    What is NOT kept is returning `{}` on a parse failure — an empty reply and a
    clean fixture then look identical, which is the difference between "nothing
    is wrong" and "I could not tell".
    """
    stripped = _FENCE.sub("", text).strip()
    try:
        parsed = json.loads(stripped)
    except json.JSONDecodeError:
        match = re.search(r"\{.*\}", stripped, re.DOTALL)
        if not match:
            raise BackendError(
                f"cloud reply contained no JSON object: {text[:200]!r}") from None
        try:
            parsed = json.loads(match.group(0))
        except json.JSONDecodeError as exc:
            raise BackendError(
                f"cloud reply was not valid JSON ({exc}): {text[:200]!r}") from exc
    if not isinstance(parsed, dict):
        raise BackendError(f"cloud reply was {type(parsed).__name__}, not an object")
    return parsed


def _factory(**kwargs) -> Backend:
    return CloudBackend(**kwargs)


register("cloud", _factory)
