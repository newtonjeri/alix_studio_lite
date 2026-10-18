"""The prompt system for a vision-language backend.

Ported from `butterfli_perception/scripts/ai_prompts.py`. It is the interface
contract between alix and a VLM: the fixture taxonomy, the state visual guide,
the region rules, the recommendation rules and the JSON response template.

ONE THING IS GENERATED RATHER THAN COPIED, and it is the reason this file is
worth having at all. butterfli's `_REGION_RULES` was a hand-written block naming
ten commode regions, built from a hand-written `VALID_REGIONS` tuple. On alix
that would be a second copy of a list whose first copy is annotated SINGLE SOURCE
OF TRUTH, and the two would drift the first time a region was added in the
Studio. `region_rules()` derives the block from `RegionVocabulary` instead, which
has three consequences:

* **21 commode regions are offered where butterfli offered 10.** `hinge_area`,
  `bowl_inside_rim`, `lid_side`, `lid_seat_front`, `seat_body_hinge_area` and the
  rest were simply unsayable before — the model could not name them, so a stain
  on one was reported as something else or not at all.
* **Each region arrives with its own description**, straight from `regions.yaml`.
  A model has no visual grounding for `lidsides_back` against `lid_side`, and the
  description is the only thing that can give it one. A hand-written tuple can
  never carry this.
* **Non-servable regions are still offered.** A region the robot cannot clean
  today is still a real place a stain can be, and `analysis.py` already reports
  those as `unservable_regions` — a gap someone should see. Hiding them from the
  prompt would convert a visible gap into a silent misclassification.

ROS-free, model-free: this module imports `regions` and `types` and nothing else,
so the prompts are testable without a network, an API key, or a backend.
"""

from __future__ import annotations

from .regions import RegionVocabulary
from .types import ESCALATE, FIXTURE_STATES, FIXTURE_TYPES, PASS, RE_CLEAN, TARGET_CLEAN

#: How much caller-supplied text may be appended to a prompt. `AnalyzeScene.srv`'s
#: `custom_prompt_appendix` is caller text going to a model, so it is treated as
#: untrusted: capped, and appended AFTER the response-format instruction so it
#: cannot displace it.
MAX_APPENDIX_CHARS = 400

COMMODE_STATE_VISUAL_GUIDE = (
    "COMMODE STATE GUIDE — compare the input image against the labelled "
    "REFERENCE photos provided.\n"
    "  closed — Flat continuous lid, no hole visible. (reference: lid_closed)\n"
    "  lid_open — Seat ring (thick horseshoe/U-shape) frames a narrow oval "
    "opening into the bowl. Lid folded back. (reference: lid_open)\n"
    "  lid_seat_open — Both lid and seat ring raised. Bare porcelain rim with "
    "wider opening, no seat ring in view. (reference: lid_seat_open)\n"
    "KEY: lid_open has a soft oval frame (seat ring); lid_seat_open has a bare "
    "hard porcelain rim.\n"
)

STATUS_PROMPT = (
    "Vision model for a restroom-cleaning robot.\n"
    "Analyze the image and return JSON with:\n"
    f"1. fixture_type — {' | '.join(FIXTURE_TYPES)}\n"
    "2. fixture_state:\n"
    f"   - commode: {' | '.join(FIXTURE_STATES['commode'])}\n"
    + COMMODE_STATE_VISUAL_GUIDE +
    "   - urinal/washtab: default\n"
    "3. state_confidence — float [0.0, 1.0]\n"
    "Return ONLY valid JSON, no markdown fences or extra text."
)

RECOMMENDATION_RULES = (
    "RECOMMENDATION RULES — choose the right action:\n"
    f"  {PASS} — surface is clean, no actionable stains\n"
    f"  {RE_CLEAN} — widespread dirt across the region, full re-clean needed\n"
    f"  {TARGET_CLEAN} — isolated stains addressable with precision spot-cleaning\n"
    f"  {ESCALATE} — damage, permanent issues, or a situation the robot cannot "
    "handle\n\n"
)

_PREAMBLE_NO_BASELINE = (
    "You are the post-cleaning QC vision system of an autonomous "
    "restroom-cleaning robot.\n\n"
    "First identify the fixture type and its current state.\n"
    + COMMODE_STATE_VISUAL_GUIDE +
    "Then detect stains.\n\n"
    "CRITICAL — distinguish between:\n"
    "  REMOVABLE STAINS (flag): fresh urine stains, fecal marks, new "
    "mold/mildew, rust streaks, new soap scum deposits.\n"
    "  PERMANENT DISCOLORATION (DO NOT flag as a stain; report it in "
    "permanent_discoloration): overall porcelain yellowing from age, hard-water "
    "mineral ring at the waterline, uniform chalky limescale haze, shadows, "
    "reflections, hardware.\n\n"
    "RULE: uniform large-area discoloration = permanent. Localised patches = "
    "stain.\n\n"
)

_PREAMBLE_WITH_BASELINE = (
    "You are the post-cleaning QC vision system of an autonomous "
    "restroom-cleaning robot.\n\n"
    "IMAGE 1 -> BASELINE (reference clean state — may have permanent "
    "discoloration).\n"
    "IMAGE 2 -> CURRENT  (after today's cleaning cycle).\n\n"
    "First identify the fixture type and its current state.\n"
    + COMMODE_STATE_VISUAL_GUIDE +
    "Then only flag stains PRESENT in IMAGE 2 but ABSENT in IMAGE 1.\n"
    "DO NOT flag: porcelain yellowing, water-line mineral rings, limescale "
    "haze, shadows, reflections, or hardware — if any of these existed in the "
    "baseline.\n\n"
)


def region_rules(vocabulary: RegionVocabulary) -> str:
    """The allowed-region block, DERIVED from the fixture model's regions.yaml.

    One list per fixture state, because a region is only visible in some states
    and offering `toplid` for an open lid invites exactly the error the geometry
    gate then has to catch. Descriptions are included because they are the only
    visual grounding a model gets for names like `lidsides_back`.
    """
    if not len(vocabulary):
        return ("REGION RULES — the fixture model declares no regions, so do not "
                "name one: leave `region` empty rather than inventing a word.\n\n")

    lines = [
        "REGION RULES — each stain MUST use ONE region name from the list for "
        "the state you classified. The allowed set depends on BOTH the "
        "fixture_type AND the fixture_state.\n",
        f"  {vocabulary.fixture_type} ({vocabulary.model}):\n",
    ]
    for state in FIXTURE_STATES.get(vocabulary.fixture_type, ("default",)):
        visible = vocabulary.visible_in(state)
        if not visible:
            continue
        lines.append(f"    state = {state}:\n")
        for name in visible:
            region = vocabulary.get(name)
            text = (region.description or "").strip().replace("\n", " ")
            lines.append(f"      {name}" + (f" — {text[:110]}" if text else "") + "\n")
    lines.append(
        "\nNEVER use a region that is not listed for the detected state, and "
        "never invent a name: an unrecognised region is discarded, so a stain "
        "reported under one is a stain nobody cleans.\n\n")
    return "".join(lines)


def response_template(*, baseline_used: bool, slim: bool = True) -> str:
    """The JSON shape the model must return.

    `slim` is the alix DEFAULT, where butterfli defaulted to the full form. The
    fields the full form adds — `zone`, `cleaning_instruction`, a prose
    `description` — are either unused by this stack or actively unwanted:
    `cleaning_instruction` invites a VLM to prescribe robot motion, which is
    `alix_taskgen`'s job and not something to take dictation on from a model that
    cannot see the arm. Fewer output tokens is the smaller reason.
    """
    stain_fields = [
        '"bbox": [y_min, x_min, y_max, x_max]   // 0-1000, normalised to the image',
        '"region": "<one name from REGION RULES>"',
        '"severity": "light | moderate | heavy"',
        '"confidence": 0.0',
    ]
    if not slim:
        stain_fields += [
            '"label": "<short description of what it is>"',
            '"is_new_vs_baseline": false',
        ]
    body = ",\n          ".join(stain_fields)
    extra = "" if slim else '  "description": "",\n'
    return (
        "Return ONLY valid JSON, no markdown fences and no text around it:\n"
        "{\n"
        '  "fixture_type": "",\n'
        '  "fixture_state": "",\n'
        '  "state_confidence": 0.0,\n'
        f'  "recommendation": "{PASS} | {RE_CLEAN} | {TARGET_CLEAN} | {ESCALATE}",\n'
        f'  "baseline_used": {str(baseline_used).lower()},\n'
        '  "permanent_discoloration": "",   // describe it; do NOT list it as a stain\n'
        '  "stains": [\n'
        "        {\n          " + body + "\n        }\n"
        "  ],\n"
        '  "summary": ""' + (",\n" + extra.rstrip(",\n") if extra else "") + "\n"
        "}\n")


def stain_prompt(vocabulary: RegionVocabulary, *, baseline: bool = False,
                 slim: bool = True, appendix: str = "") -> str:
    """Assemble the full stain-detection prompt.

    `appendix` is `AnalyzeScene.srv`'s `custom_prompt_appendix`, which had no
    consumer anywhere in alix until this module existed. It is appended AFTER the
    response template, never before: caller text placed ahead of the format
    instruction can displace it, and a model that then returns prose instead of
    JSON fails the whole request.
    """
    preamble = _PREAMBLE_WITH_BASELINE if baseline else _PREAMBLE_NO_BASELINE
    prompt = (preamble + region_rules(vocabulary) + RECOMMENDATION_RULES
              + response_template(baseline_used=baseline, slim=slim))
    extra = (appendix or "").strip()
    if extra:
        prompt += ("\nADDITIONAL INSTRUCTION FROM THE CALLER (does not override "
                   "the response format above):\n" + extra[:MAX_APPENDIX_CHARS] + "\n")
    return prompt


def status_prompt(appendix: str = "") -> str:
    extra = (appendix or "").strip()
    if not extra:
        return STATUS_PROMPT
    return (STATUS_PROMPT + "\nADDITIONAL INSTRUCTION FROM THE CALLER (does not "
            "override the response format above):\n"
            + extra[:MAX_APPENDIX_CHARS] + "\n")
