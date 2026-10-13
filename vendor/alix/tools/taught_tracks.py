#!/usr/bin/env python3
"""taught_tracks.py — the taught-motion lane: record -> screen -> fastest legal replay.

WHAT THIS IS (Newton, 2026-09-14: "a replay with the fastest speed within
margin, and with deviation of less than 10 mm from the path (5 mm advisable)").
A taught `.txt` is ALREADY the upload channel's native format — NDJSON integer
millidegrees, CRLF — so nothing here converts anything. The controller replays
points verbatim at its own fixed clock, so a track's point SPACING is its whole
velocity profile. The lane decides WHICH BYTES to hand it:

    tools/taught_tracks.py certify <file.txt>            # .txt in -> certified .proj out (~3 s)
    tools/taught_tracks.py upload  <file.proj>           # only a certified .proj is sent
    tools/taught_tracks.py run     <file.proj>           # watch it, record it
    tools/taught_tracks.py verify  <file.proj>           # re-check a certificate from the files

    tools/taught_tracks.py screen  <dir or files>        # triage a batch in seconds, nothing written
    tools/taught_tracks.py batch   <dir or files> --side left --taught-here --pole-mm 20 \
                                   --hand-state release --upload --run      # to the arm, one session
    tools/taught_tracks.py flow    <a.proj b.proj …>     # several certified rows, one session

`certify` is admit + shape + a certificate, with the slot and the provenance
resolved for you (asked once for a new recording, then saved beside it). The
steps stay available on their own:

    tools/taught_tracks.py admit  <file.txt> --side right --slot 45
    tools/taught_tracks.py shape  <file.proj | slot --side S>   # the speed ladder
    tools/taught_tracks.py status                              # rows + certificate state
    tools/taught_tracks.py preflight --side right              # = cleaning_tracks preflight --taught

THE CERTIFICATE. `<track>.json` beside the admitted bytes carries every gate's
measured value and bar, bound to the .proj and .txt by sha256. `upload` and
`run` re-check it against the files (and refuse without `--force`); `verify`
re-derives it from the artifacts alone, G4 included.

WHAT IT REUSES, UNCHANGED. `alix_dispatch.track` (parse / build_project_file /
send_project / program_list / preflight_slots), `cleaning_tracks` (pins,
`check_row_provenance`, `provenance_gate`, `build_proj_for`,
`controller_slot_check`, `trajectory_name_check`, `preflight_report`, `run_one`,
the ledger), `plan_min_time` (LSQ B-spline + TOPP-RA + player-clock resample,
through exp01's `teach_as_plan` wrapper), and exp01's calibrated instruments
(`predict_track_replay`: 20 Hz band limit, position margins, step vs the 10 deg
0x4000 threshold; `teachlab.kinematics` for FK at the declared frame). Rows are
MANIFEST-SHAPED and pinned like every other row, so `--taught` on the
`cleaning_tracks` verbs is the whole seam (plan section 8).

THE GATES. Enforced: format; `hardware: ok`; joint-position margin; 0 ticks over
a joint RATING at the FAST clock; step <= 10 deg/frame; pins/identity/slots; and
deviation <= 10 mm at the declared contact frame. Advisory (measured, recorded,
never refused): acceleration, jerk, Cartesian speed, recorder beat — the track
accel gate has been ADVISORY since Newton's ruling of 2026-09-07, and the
files that run clean read 3-10x the vendor's planning figure.

THE CLOCK IS A BAND, NOT A CONSTANT (Newton, 2026-09-14: "the period sometimes
fluctuates between 5.4 ms and 6.3 ms"). Rate = spacing / period, so every rate
and step figure is judged at the FAST end (default 5.4 ms) and durations are
reported as a band whose ramp and per-point rate come from the recorded ledger.

THE FASTEST LEGAL RUNG. `shape` prices rung 0 (verbatim: the recording as
taught, zero deviation) against the re-parameterised ladder, keeps every rung
whose gates pass AND whose deviation is inside the cap, and installs the fastest
one — preferring a rung inside the 5 mm advisory where one exists. Nothing is
uploaded by `shape`; the admitted bytes are what `upload` sends.
"""
from __future__ import annotations

import argparse
import datetime
import hashlib
import itertools
import json
import pathlib
import re
import shutil
import sys

import numpy as np

REPO = pathlib.Path(__file__).resolve().parents[1]
for _p in ("tools", "core/python/alix_dispatch/src"):
    _q = str(REPO / _p)
    if _q not in sys.path:
        sys.path.insert(0, _q)
#: exp01 owns the calibrated instruments this lane reads (band limit, margins,
#: step, FK, the teach_as_plan wrapper). Imported, never copied — one source.
EXP01 = REPO / "experiments/exp01_teach_trajectory_reverse_engineering"
EXP01_SRC = EXP01 / "src"
EXP01_TEACH = EXP01 / "data/teach_files"
if str(EXP01_SRC) not in sys.path:
    sys.path.insert(0, str(EXP01_SRC))

import cleaning_tracks as ct                                          # noqa: E402
from alix_dispatch import track as tr                                 # noqa: E402

#: THE CLOCK EVERY RATE AND STEP GATE IS JUDGED AT: the fastest the upload player ever plays a
#: line. CORRECTED 2026-09-17 from 5.4 ms. 5.4 ms was the fast end of the WHOLE-RUN average
#: (181 complete runs, replay_s/(N-1) 5.4647..6.6542 ms/pt), but the player advances at most ONE
#: point per 5 ms controller cycle and holds otherwise (Resource/reports/2026-09-17_upload_player_clock),
#: so between its holds it plays at 5.0 ms/pt. A file judged at 5.4 ms rides up to 8 % over a
#: rating there: 22 of the 24 production tracks and 6 taught ones did, and the arm clamps at the
#: rating (logged joint_speed peaks 183-185 on J1, 228-231 on J4), where the median departure is
#: ~1.7x the unclamped one.
CLOCK_GATE_S = 0.005
#: The clock the EXECUTION MODEL's predictions are evaluated at (G1 predicted margins, G5 departure,
#: the emitter's departure cap). It stays at the 5.4 ms the model was fitted and validated with;
#: moving it would change every prediction without a refit. Certificates record both clocks.
CLOCK_PREDICT_S = 0.0054
#: The nominal clock, for the human-facing `track_s` of a row only.
CLOCK_NOMINAL_S = 0.0055545
#: THE CLOCK A DRAG-TEACH RECORDING WAS SAMPLED AT: 5 ms exactly. Each of the 14 J1-J7 single-joint
#: recordings names its own duration, and every one equals points x 5 ms to the millisecond, so the
#: pendant's recorder ticks at the controller cycle. Any filter designed for a recording must use THIS,
#: not `CLOCK_NOMINAL_S` (which is a PLAYBACK average): designing a 5 Hz low-pass at 5.5545 ms and
#: applying it to 5 ms samples puts the real cutoff at 5 x 5.5545/5.0 = 5.55 Hz — which is what slots 50
#: and 51 were actually smoothed at (2026-09-17 review, C1).
CLOCK_RECORD_S = 0.005
#: The length-aware duration envelope: replay_s = ramp + rate x (N-1). The
#: constants are the ledger's own (dual_arm_verdicts.py refit 2026-09-10, which
#: contains all 166 complete runs); the same convention is used here so a
#: taught row's band means what a cleaning row's band means.
RAMP_LO_S, RATE_LO_S = 0.057, 0.005515
RAMP_HI_S, RATE_HI_S = 0.750, 0.005810
#: Joint-position allowance, degrees. plan_min_time refuses below 5.0 by
#: default and the fastpath refuses below 5.0, so the lane uses the same number
#: and the rung search and the gate cannot disagree. The channel's measured
#: replay overshoot (1.9 deg at an apex, up to 4.5 max) sits under it.
POSITION_ALLOWANCE_DEG = 5.0
#: Error 0x4000 "Position Command Step Jump" — a machine threshold, not a taste.
STEP_FAULT_DEG = 10.0
#: The path-deviation budget at the declared contact frame (Newton 2026-09-14) — G4, the BYTES
#: against the recording.
MAX_DEVIATION_MM = 10.0
PREFER_DEVIATION_MM = 5.0
#: The budget for the PREDICTED EXECUTED deviation at the reversals (G5) — the overrun past a
#: stroke end that the upload channel produces and slowing the whole approach is the only
#: cure for (Newton, 2026-09-15 evening: "we can allow the deviations ... it cannot be corrected
#: by only slowing down at the turns, so we can only buy speed for now"). 20 mm buys the hand's
#: speed to 1.2x on the stroke wipes; `--exec-budget-mm` per file, per region's room.
EXEC_BUDGET_MM = 20.0
#: Rungs priced by `shape`. 1.0 is the time-optimal re-parameterisation; the
#: rest trade speed for velocity margin (the report's own ladder: the left file
#: needs 0.70 to reach 0 ticks over a rating, and that costs 2.8 % of duration).
VEL_SCALES_DEFAULT = [1.0, 0.9, 0.8, 0.7, 0.6]
#: DIAL 1 — POINT SPACING, the uniform re-time ladder (λ x the recorded speed).
#: The new points are linear interpolations between taught samples, so they lie
#: ON the taught polyline and the path CANNOT move (measured: 0.01 mm by exact
#: global search) — the exact option, priced against the solver's, and cheap.
#: Velocity scales λ, acceleration λ² (advisory), so the velocity rating at the
#: fast clock is what stops the ladder.
TIME_SCALES_DEFAULT = [1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.8, 2.0, 2.2, 2.4, 2.6, 2.8, 3.0, 3.5, 4.0, 5.0]
#: plan_min_time's acceleration readback ceiling, as a multiple of the 10.472
#: rad/s^2 planning figure. 1.5 is the measured 15.7 rad/s^2 bound Newton ruled
#: advisory against on 2026-09-07 — the same default exp01's predictor uses.
MAX_ACC_UTIL_DEFAULT = 1.5
#: DIAL 1 SLOWER than taught — a LAST RESORT, priced only when no other rung
#: passes (or --max-speed excludes the rest). The uniform slow-down keeps every
#: new point on the taught polyline, so it rescues a recording the solver cannot
#: re-parameterise. It is NOT offered alongside a passing solver rung: under the
#: 5 mm preference a 0.6x uniform rung (73.5 s, 1.75 mm) would beat the solver's
#: 0.82x (53.7 s, 6.13 mm) on Left_High_Speed — 20 s slower for 4 mm of an offline
#: figure (measured 2026-09-14). A POSITION failure is geometry; no rung fixes it.
SLOW_SCALES_DEFAULT = [0.9, 0.8, 0.7, 0.6, 0.5]
#: NEAR A LIMIT, RATE IS THE GATE (Newton, 2026-09-15: "make the motion slower where the
#: margin is small instead of rejecting the whole taught file"). What stops a run is the
#: EXECUTED overshoot past a commanded extreme, and that grows with approach speed: F103
#: measured the controller tolerating limit crossings at <= 13.4 deg/s and tripping at
#: >= 32.9; the 2026-09-15 turn analysis put the p90 overshoot at 1.6 deg when a joint
#: arrives below 25 deg/s. So a joint may come inside the 5 deg allowance if it stays
#: outside the HARD FLOOR and its band-limited rate, wherever it is inside the allowance,
#: is at most the CAP — and the "guarded" rung is the uniform re-time with exactly that cap
#: applied, so a file fast everywhere else crawls only where it is close. Both numbers
#: are provisional until Newton rules on them (flags --near-limit-rate-deg-s, --hard-margin-deg).
NEAR_LIMIT_RATE_DEG_S = 10.0
HARD_MARGIN_FLOOR_DEG = 2.0
#: how fast lambda may change per taught line around a guarded stretch (0.01 = from 2.0x
#: down to 0.2x over ~180 lines, ~1 s of replay), so the slow-down is a ramp, not a step
GUARD_RAMP = 0.01
#: the guard is DESIGNED below the bar it is JUDGED against: at the fast gate clock, at
#: this fraction of the cap, over a zone this much wider than the allowance — because
#: the 20 Hz band limit smears a little of the faster outside rate into the first
#: samples inside the allowance (a guard designed exactly at 10 deg/s read 10.8)
GUARD_DESIGN_FRACTION = 0.9
GUARD_ZONE_EXTRA_DEG = 1.0
#: THE EXECUTION MODEL (Newton, 2026-09-15: "no rate factors, we should be using the rate from
#: the manufacturers ... the only thing we should be checking ... is the deviation, we can
#: reproduce a deviation on the hardware execution ... right lid hin.txt is a proof of that").
#: The executed joint lags its command and overshoots every commanded extreme by tau x its
#: arrival rate: tau reads 0.050 s at the 90th percentile and 0.135 s at the maximum over
#: 27,392 planned and 422 taught turns (the 2026-09-15 analyses), with a floor of 0.6 deg (p90)
#: below 25 deg/s; right-lid-hin's first run measured 0.42 deg where the law predicted 0.46.
#: The timing tool inverts the law (`model_caps`: the arrival rate at every reversal and every
#: limit is the budget over tau, the joint rating everywhere else, no factor on it) and the
#: gates apply it to the emitted bytes (G1 at the limits at the maximum, G5 on the path at the
#: 90th percentile). The live values are read from EXECUTION_MODEL_FILE when it exists
#: (`taught_tracks.py model` prints and refits it); these defaults are what it was written
#: with. `--timing legacy` keeps the pre-model ladder (uniform / guarded rungs).
EXECUTION_MODEL_FILE = REPO / "Resource/tracks/taught/EXECUTION_MODEL.json"
DEFAULT_EXECUTION_MODEL = dict(
    version=1, fitted="2026-09-15",
    tau_path_s=0.050,          # overshoot / arrival rate, 90th percentile (the path budget)
    tau_path_p50_s=0.010,      # the same at the median (`--budget-percentile p50` judges with it)
    percentile="p90",          # which of the two the path budget is held at
    tau_limit_s=0.135,         # overshoot / arrival rate, maximum above 75 deg/s (the limits)
    floor_deg=0.6,             # overshoot below 25 deg/s, 90th percentile (2026-09-15 runs: 0.58)
    limit_eps_deg=0.5,         # the executed extreme must stay this far from a joint limit
    pad_factor=1.0,            # calibration of the pad-level prediction (R7 refits it)
    #: THE SEVEN LAGS, in joint order. The departure is made by the joints lagging by
    #: DIFFERENT amounts: a lag common to all seven lands ALONG the commanded path (a pure
    #: time delay, measured 0.00 mm perpendicular) and leaves it not at all, so a law that
    #: sums the per-joint offsets into one pad velocity before applying one tau is blind to
    #: the whole effect. Fitted from the turn overshoots (2026-09-16); the corpus test of
    #: 2026-09-16 is what promoted them from a diagnostic to the predictor.
    tau_joint_s=[0.0678, 0.0324, 0.0536, 0.0378, 0.0533, 0.0603, 0.0654],
    #: the measured ratio of real p95 departure to what the joint-space law predicts,
    #: 1.99x over 148 planned runs and 1.56x over 10 taught ones -- the law is a SHAPE
    #: that ranks and scales correctly across both families and under-reads by about two,
    #: so the budget is held against the prediction times this number
    departure_calibration=2.0,
    prominence_deg=0.5,        # a reversal is an extreme of the 20 Hz trace this prominent
    approach_window_s=0.30,    # the arrival rate is the max band-limited rate this long before
    window_after_s=0.05,       # the cap holds this long after the extreme (the departure)
    fitted_from=["Resource/reports/2026-09-15_planned_task_runs/summary.json (27,392 turns)",
                 "Resource/reports/2026-09-14_taught_track_lane/runs_2026-09-15/results.json (422 turns)",
                 "Resource/reports/2026-09-14_taught_track_lane/runs_2026-09-14/results.json"])
#: how fast lambda may change per source line around a model-capped stretch: from 5.0x to 0.2x
#: in ~100 lines (0.1-0.5 s of replay) — the arms followed 3,830 deg/s^2 in the captures
MODEL_RAMP = 0.05
#: the rating cap is designed this far under the rating so that the 20 Hz band-limited rate
#: G2 reads (which overshoots a change of rate by up to 2 %) is at most the rating itself
RATING_FILTER_ALLOWANCE = 0.97
#: how much harder than the ratio asks the emitter pulls its ceiling down each pass (1.0 =
#: exactly the ratio, which converges too slowly to land inside on a reversal-dense file)
DEPARTURE_RELAX = 1.25
#: the emitter aims this far inside the departure budget, so the file it writes is INSIDE
#: the bar the gate then reads it at rather than exactly on it (the same "design below the
#: bar" rule the near-limit guard uses)
DEPARTURE_DESIGN_FRACTION = 0.98
#: A PAUSE is a run of >= PAUSE_MIN_LINES consecutive lines all WITHIN PAUSE_TOL_DEG of the run's
#: FIRST line on every joint (the operator holding still) — not a run of small steps, which would
#: swallow a slow apex; `strip_pauses` is the rule. Removed by default (R4), `--keep-pauses` keeps them
PAUSE_TOL_DEG = 0.05
PAUSE_MIN_LINES = 3
#: The certificate format written beside every installed rung. `verify` re-checks version 1
#: (the pre-model certificates on the shelf) and version 2 (with the model and the timing
#: record) each by its own rules, and refuses any other version rather than guess.
CERT_VERSION = 2
CERT_VERSIONS_VERIFIABLE = (1, 2)
#: `--solver auto` (the default): DIAL 2 — ~11 s of TOPP-RA per velocity scale —
#: runs only when the uniform ladder cannot reach this speed-up, i.e. when the
#: recording itself is too fast or too jittery for point spacing alone (slot 45's
#: fluid-movement file: no uniform rung >= 1.0x passes, the solver's sigma 0.9
#: does). On the 2026-09-14 shelf the uniform ladder won every time it reached
#: 1.5x or more, so stopping there loses no speed; `--solver always` prices it.
SOLVER_AUTO_BELOW = 1.3


# ─────────────────────────────────────────────────────────────── helpers
def _ptr():
    """exp01's calibrated instruments, imported lazily (they pull scipy and the
    URDF stack, which no admit/status path needs)."""
    import predict_track_replay as ptr
    return ptr


def side_char(side: str) -> str:
    return "R" if side == "right" else "L"


def slug(text: str) -> str:
    """A short, filename-safe token. A leading or trailing side word is dropped
    (the row name appends the side itself, so `Left_High_Speed_...` must not
    become `..._p_left_left`)."""
    s = re.sub(r"[^A-Za-z0-9]+", "_", str(text)).strip("_")
    s = re.sub(r"^(left|right)_", "", s, flags=re.I)
    s = re.sub(r"_(left|right)$", "", s, flags=re.I)
    return (s.lower() or "motion")[:40]


def duration_band_s(points: int) -> list:
    """[lo, hi] seconds for an N-point track, over the measured envelope."""
    n1 = max(int(points) - 1, 1)
    return [round(RAMP_LO_S + RATE_LO_S * n1, 4), round(RAMP_HI_S + RATE_HI_S * n1, 4)]


def read_q_deg(path) -> np.ndarray:
    """The track as degrees [deg], through the SAME parser `upload` uses, so a
    file this lane can measure is a file the channel can carry."""
    rows = tr.parse_track(pathlib.Path(path))
    return np.array(rows, dtype=float) / 1000.0


def sha256_of(path) -> str:
    return hashlib.sha256(pathlib.Path(path).read_bytes()).hexdigest()


def load_teach_manifest() -> dict:
    """exp01's per-file provenance (side, frame, pole_m, hand_state, hardware,
    taught_by, ...). Absent is fine — `SOURCE.json` beside a recording covers
    it, and neither is required to admit (the lane then refuses to REPLAY
    without an explicit `hardware: ok`, which is the honest default)."""
    p = EXP01_TEACH / "MANIFEST.yaml"
    if not p.is_file():
        return {}
    try:
        import yaml
        return yaml.safe_load(p.read_text()) or {}
    except Exception:                                                  # noqa: BLE001
        return {}


def source_provenance(src: pathlib.Path) -> dict:
    """What is known about ONE recording: `<stem>.SOURCE.json` beside it, with
    exp01's MANIFEST.yaml entry (keyed by filename) overlaid on top — the
    curated record wins over the convenience one."""
    out = dict(side=None, frame=None, pole_m=None, hand_state=None,
               hand_counts=None, hardware="unknown", taught_by=None, date=None,
               units="millideg", note=None, source_of_provenance=None)
    cand = [src.with_suffix(".SOURCE.json"), src.with_suffix(".json")]
    for c in cand:
        if c.is_file() and c.name != src.name:
            try:
                d = json.loads(c.read_text())
            except Exception:                                          # noqa: BLE001
                continue
            if isinstance(d, dict):
                out.update({k: v for k, v in d.items() if k in out})
                out["source_of_provenance"] = ct.repo_rel(c, REPO)
                break
    entry = (load_teach_manifest().get("files") or {}).get(src.name)
    if isinstance(entry, dict):
        for k in ("side", "frame", "pole_m", "hand_state", "hand_counts", "hardware",
                  "units", "note"):
            if entry.get(k) is not None:
                out[k] = entry[k]
        out["taught_by"] = entry.get("taught_by")
        out["date"] = entry.get("exported") or entry.get("date")
        out["source_of_provenance"] = "exp01 data/teach_files/MANIFEST.yaml"
        # exp01 spells the side L/R and stores an arm name besides.
        if out["side"] in ("L", "R"):
            out["side"] = "left" if out["side"] == "L" else "right"
    return out


#: THE FLOW SHEETS' OWN SPEC (Newton, 2026-09-16: the sheets "are what should be used, in
#: addition to the percentages"). Every hand step on both sheets reads "hand open 100 %
#: (first 4 fingers)" or "hand close 50 % (first 4 fingers)": a percentage of the OPEN
#: position on the four finger channels, and nothing at all about the thumb. So the counts
#: are pct x 10 (1000 = open, 0 = closed) on channels 1-4 (little, ring, middle, index) and
#: None — HOLD — on channels 5-6 (thumb rotate, thumb bend). The SRDF states are a different
#: thing: `release` is 993 on the fingers and commands the thumb to 981/992, `half_grasp`
#: bends it to 133/944. Until 2026-09-16 the lane used those, which is why both right-arm
#: runs of 2026-09-15 logged "channels [5, 6] were commanded to move and did NOT".
FINGER_CHANNELS = 4


def fingers_pct_counts(pct: float) -> list:
    """The sheets' `hand <pct> % (first 4 fingers)` as a counts vector: the four
    finger channels at that percentage of open, the two thumb channels HELD."""
    if not 0 <= float(pct) <= 100:
        raise SystemExit(f"a hand percentage is 0-100, got {pct!r}")
    return [int(round(float(pct) * 10))] * FINGER_CHANNELS + [None] * 2


def fingers_pct_label(pct: float) -> str:
    return f"fingers_{int(round(float(pct)))}pct"


def hand_counts_for(state, side: str):
    """Six ANGLE_SET counts for a hand state: `fingers_<pct>pct` (the flow sheets'
    own spec — the four fingers at that percentage, the thumb held) or an SRDF
    hand group state (the resolution the cleaning tasks use, via
    glove_frames.yaml's state names). `None` in a slot means HOLD that channel."""
    if not state:
        return None
    m = re.fullmatch(r"fingers_(\d{1,3})pct", str(state))
    if m:
        return fingers_pct_counts(int(m.group(1)))
    try:
        return ct._srdf_hand_state(str(state), side)[1]     # noqa: SLF001
    except Exception:                                                  # noqa: BLE001
        return None


_TOOL_FRAME_CACHE: dict = {}


def inherited_tool_frame(side: str, evidence: dict = None):
    """The frame THIS side's controller was last observed holding, taken from
    the run ledger — the dry-row rule (H35), so a taught run's record can say
    whether the controller still held it. Since 2026-09-15 a different frame
    is RECORDED with the run (`tool_frame_check`), never refused (Newton: the
    frame must not block a run). Most-recorded name wins; None when the ledger
    has nothing for this side, and the row then says so. The ledger scan costs
    ~0.7 s and the answer cannot change within one process, so it is kept
    per side (a `batch` admits many rows in a row)."""
    if evidence is None and side in _TOOL_FRAME_CACHE:
        return json.loads(json.dumps(_TOOL_FRAME_CACHE[side]))
    ev = evidence if evidence is not None else ct.run_record_evidence()
    best, best_runs, best_src = None, 0, None
    for name, e in ev.items():
        if not (e.get("sides") or {}).get(side):
            continue
        for _key, t in (e.get("tool_frames") or {}).items():
            if int(t.get("runs") or 0) > best_runs:
                best, best_runs, best_src = t["frame"], int(t["runs"]), name
    if not best:
        if evidence is None:
            _TOOL_FRAME_CACHE[side] = None
        return None
    out = json.loads(json.dumps(best))
    out["provenance"] = {
        "from": f"the run ledger: {best_runs} recorded {side} run(s) of {best_src}",
        "note": ("inherited, like a dry row's: a taught motion does not choose a TCP, it "
                 "must simply not run under a change from what this controller has been "
                 "run under")}
    if evidence is None:
        _TOOL_FRAME_CACHE[side] = json.loads(json.dumps(out))
    return out


def gate_report(q_deg: np.ndarray, side: str, clock_s: float = CLOCK_GATE_S,
                allowance_deg: float = POSITION_ALLOWANCE_DEG,
                v_near: float = NEAR_LIMIT_RATE_DEG_S, hard_floor: float = HARD_MARGIN_FLOOR_DEG,
                model: dict = None, frame: str = None, budget_mm: float = MAX_DEVIATION_MM, *,
                cal_lines: np.ndarray = None, pred_clock_s: float = CLOCK_PREDICT_S) -> dict:
    """The enforced and advisory readings for ONE point list, at the fast clock.

    The instruments are exp01's (`predict_track_replay`), so the numbers a
    taught file is judged by are the numbers its forecast was published with —
    including the 20 Hz band limit the house rule requires before any rate or
    acceleration figure means anything (one LSB of jitter at this clock is
    2.26 rad/s^2 raw).

    With `model` (the execution model, 2026-09-15) G1 becomes the PREDICTED
    executed margin at every limit and G5 the PREDICTED pad deviation at every
    reversal (`model_predictions`); without it the pre-model rules apply (the
    5 deg allowance with the guarded rate cap), which is how a version-1
    certificate is re-checked."""
    ptr = _ptr()
    import plan_min_time as pmt
    vmax = np.array(ptr.K.QDOT_MAX_DEG_S, dtype=float)
    rate = ptr.rate_report(q_deg, clock_s, vmax)
    pos = ptr.position_report(q_deg, np.array(ptr.K.JOINT_LIMITS_DEG, dtype=float))
    step = ptr.step_report(q_deg)
    bl, raw = rate["band_limited_20hz"], rate["raw"]
    margins = [j["margin_deg"] for j in pos["per_joint"]]
    after = [m - float(allowance_deg) for m in margins]
    # near a limit, rate is the gate (see NEAR_LIMIT_RATE_DEG_S): the band-limited rate of
    # every joint over the samples where it sits inside the allowance
    from scipy.signal import butter, filtfilt
    q = np.asarray(q_deg, dtype=float)
    lim = np.array(ptr.K.JOINT_LIMITS_DEG, dtype=float)
    marg = np.minimum(q - lim[:, 0], lim[:, 1] - q)
    if q.shape[0] > 30:
        b_, a_ = butter(4, min(20.0 / (0.5 / float(clock_s)), 0.99), "low")
        qbl = filtfilt(b_, a_, q, axis=0)
    else:
        qbl = q
    rate_bl = np.abs(np.gradient(qbl, axis=0)) / float(clock_s)
    inside = marg < float(allowance_deg)
    near_rate = [float(rate_bl[inside[:, j], j].max()) if inside[:, j].any() else None
                 for j in range(q.shape[1])]
    near_joints = [j + 1 for j in range(q.shape[1]) if inside[:, j].any()]
    worst_near = max((r for r in near_rate if r is not None), default=None)
    min_margin = float(min(margins))
    mp = None
    if model is not None:
        # THE EXECUTION MODEL decides G1 (and G5 below): the predicted executed extreme
        # of every joint must stay `limit_eps_deg` outside its limit, whatever the
        # commanded margin. "geometry" = no rate fixes it (even the floor overshoot
        # reaches inside eps); "rate" = this file's arrival rates do, a slower rung will not.
        mp = model_predictions(q_deg, side, frame or f"{side_char(side)}_glove_frame_2", model,
                               pred_clock_s, budget_mm, cal_lines=cal_lines)
        eps = float(model["limit_eps_deg"])
        em = mp["pred_exec_margin_min_deg"]
        if not mp["geometry_ok"]:
            pos_ok, pos_rule = False, "geometry"
        elif em is None or em >= eps:
            pos_ok, pos_rule = True, ("margin" if min_margin >= float(allowance_deg) else "model")
        else:
            pos_ok, pos_rule = False, "rate"
    elif min_margin >= float(allowance_deg):
        pos_ok, pos_rule = True, "margin"
    elif min_margin < float(hard_floor):
        pos_ok, pos_rule = False, "floor"
    else:
        pos_ok = bool(worst_near is not None and worst_near <= float(v_near))
        pos_rule = "guarded" if pos_ok else "rate"
    try:
        _n, vlim, alim, jlim = pmt.load_limits(side_char(side))
        uv, ua, uj = pmt.band_limited_util(np.radians(q_deg), clock_s, vlim, alim,
                                           float(jlim.min()), 20.0)
    except Exception:                                                  # noqa: BLE001
        uv = ua = uj = None
    enforced = {
        "clock_gate_s": float(clock_s),
        "clock_predict_s": float(pred_clock_s),
        "position_allowance_deg": float(allowance_deg),
        "position_margin_min_deg": round(min(margins), 4),
        "position_margin_after_allowance_deg": round(min(after), 4),
        "position_binding_joint": pos["binding_joint"],
        "position_ok": bool(pos_ok),
        "position_rule": pos_rule,
        "hard_margin_floor_deg": float(hard_floor),
        "near_limit_rate_cap_deg_s": float(v_near),
        "near_limit_rate_deg_s": (None if worst_near is None else round(worst_near, 3)),
        "near_limit_joints": near_joints,
        "ticks_over_rating_band_limited": bl["ticks_over_rating"],
        "longest_over_run_ms_band_limited": bl["longest_over_run_ms"],
        "ticks_over_rating_raw": raw["ticks_over_rating"],
        "peak_rate_util_band_limited": bl["peak_util"],
        "peak_rate_util_raw": raw["peak_util"],
        "peak_rate_deg_s": max(bl["per_joint_peak_deg_s"]),
        "peak_joint": bl["peak_joint"],
        "max_step_deg": step["max_step_deg"],
        "step_fault_deg": STEP_FAULT_DEG,
    }
    advisory = {
        "accel_util_band_limited": None if ua is None else round(ua, 3),
        "jerk_util_band_limited": None if uj is None else round(uj, 3),
        "vel_util_band_limited": None if uv is None else round(uv, 3),
        "rate_p50_util": bl["util_p50"], "rate_p90_util": bl["util_p90"],
        "slowness": slowness_census(q_deg, side, frame, clock_s),
        "note": ("acceleration and jerk are ADVISORY on this channel (Newton 2026-09-07: the "
                 "track accel gate is advisory against a measured 15.7 rad/s^2 bound) and are "
                 "reported, never refused"),
    }
    if mp is not None:
        enforced.update({
            "model_version": model.get("version"), "model_fitted": model.get("fitted"),
            "tau_path_s": float(model["tau_path_s"]), "tau_limit_s": float(model["tau_limit_s"]),
            "floor_deg": float(model["floor_deg"]), "limit_eps_deg": float(model["limit_eps_deg"]),
            "pad_factor": float(model["pad_factor"]), "budget_mm": float(budget_mm),
            "tau_joint_s": list(model.get("tau_joint_s") or []),
            "reversals": mp["reversals"],
            "pred_dev_max_mm": mp.get("pred_line_dev_max_mm"),
            "pred_dev_p95_mm": mp.get("pred_line_dev_p95_mm"),
            "pred_dev_p50_mm": mp.get("pred_line_dev_p50_mm"),
            "pred_dev_worst": mp.get("pred_line_dev_worst"),
            "pred_dev_bound_mm": mp.get("pred_line_dev_bound_mm"),
            "pred_dev_over_budget": mp.get("pred_line_over_budget"),
            # the law G5 was read with, and the pad-space number it replaced: the GAP
            # between the two is the differential-lag term this lane was blind to
            "pred_dev_law": mp.get("pred_line_law"),
            "pred_dev_calibration": mp.get("pred_line_dev_calibration"),
            "pred_dev_padspace_max_mm": mp.get("pred_line_dev_padspace_max_mm"),
            "pred_dev_padspace_p95_mm": mp.get("pred_line_dev_padspace_p95_mm"),
            "pred_dev_attribution": mp.get("pred_line_dev_attribution"),
            "pred_dev_at_reversals_max_mm": mp["pred_dev_max_mm"],
            "pred_dev_at_reversals_worst": mp["pred_dev_worst"],
            "pred_dev_over_prefer": mp["over_prefer"],
            "pred_exec_margin_min_deg": mp["pred_exec_margin_min_deg"],
            "pred_exec_margin_worst": mp["pred_exec_margin_worst"],
            "g5_ok": bool(mp.get("g5_ok")),
            "pred_dev_measured_calibrated": bool(mp.get("pred_line_measured_calibrated")),
        })
    reasons = []
    if not pos_ok:
        if pos_rule == "geometry":
            w = mp["pred_exec_margin_worst"] or {}
            reasons.append(
                f"GEOMETRY, no speed fixes it: J{w.get('joint')} is commanded "
                f"{w.get('commanded_margin_deg')} deg from its limit at line {w.get('line')}, and even "
                f"at a crawl the arm overshoots by the model's {float(model['floor_deg']):g} deg floor "
                f"and comes inside the {float(model['limit_eps_deg']):g} deg it must keep; re-teach")
        elif mp is not None:
            w = mp["pred_exec_margin_worst"] or {}
            reasons.append(
                f"J{w.get('joint')} arrives at its extreme at line {w.get('line')} at "
                f"{w.get('arrival_deg_s')} deg/s with {w.get('commanded_margin_deg')} deg of commanded "
                f"margin: the model predicts {w.get('pred_overshoot_deg')} deg of overshoot "
                f"(max({float(model['floor_deg']):g}, {float(model['tau_limit_s']):g} s x rate)), leaving "
                f"{w.get('pred_exec_margin_deg')} deg, under the {float(model['limit_eps_deg']):g} deg "
                f"it must keep — a slower arrival there fixes it")
        elif pos_rule == "floor":
            reasons.append(
                f"joint-position margin {min_margin:.3f} deg on J{pos['binding_joint']} is inside "
                f"the {float(hard_floor):.1f} deg hard floor — this is GEOMETRY, no speed change "
                f"fixes it (the arm overshoots a commanded extreme by 1-4 deg even when it arrives "
                f"slowly); re-teach")
        else:
            reasons.append(
                f"joint-position margin {min_margin:.3f} deg on J{pos['binding_joint']} is inside "
                f"the {float(allowance_deg):.1f} deg allowance and the joint moves at up to "
                f"{worst_near:.1f} deg/s there (cap {float(v_near):g}) — a guarded rung slows it "
                f"inside the allowance")
    if mp is not None and not enforced["g5_ok"]:
        w = mp.get("pred_line_dev_worst") or {}
        reasons.append(
            f"predicted executed departure {mp.get('pred_line_dev_max_mm'):.2f} mm at line "
            f"{w.get('line')} (the pad travels {w.get('pad_speed_m_s')} m/s there, so it carries on "
            f"{1000 * float(model['tau_path_s']) * float(w.get('pad_speed_m_s') or 0):.1f} mm past "
            f"where the command turns) is over the {float(budget_mm):g} mm budget — "
            f"{mp.get('pred_line_over_budget')} line(s) over it")
    if bl["ticks_over_rating"]:
        reasons.append(
            f"{bl['ticks_over_rating']} tick(s) command a joint over its RATING at the "
            f"{float(clock_s) * 1e3:.2f} ms fast clock (band-limited peak "
            f"{bl['peak_util']:.3f} on J{bl['peak_joint']}, longest excursion "
            f"{bl['longest_over_run_ms']:.0f} ms)")
    if step["max_step_deg"] > STEP_FAULT_DEG:
        reasons.append(f"largest commanded step {step['max_step_deg']:.3f} deg exceeds the "
                       f"{STEP_FAULT_DEG:.0f} deg 0x4000 threshold")
    return {"enforced": enforced, "advisory": advisory, "reasons": reasons,
            "admissible": not reasons}


def deviation_mm(q_ref_deg: np.ndarray, q_new_deg: np.ndarray, side: str, frame: str) -> dict:
    """How far the re-parameterised path sits from the TAUGHT path, in mm at the
    declared contact frame, both directions (new->taught and taught->new).

    COMPARED BY ARC-LENGTH STATION, NOT BY TIME AND NOT BY NEAREST POINT. A
    re-timed track reaches the same place at a different moment by design, so a
    same-index comparison says nothing; and a free nearest-point search is
    unusable on a scrub, which revisits its own path (teachlab.tracking says so
    itself: "a free nearest-point search is unusable: a scrub revisits its own
    path"). The first version of this function did use the forward-monotone
    projection and reported **320 mm** on a rim-top scrub whose LSQ fit is
    within **0.26 deg** of every taught point — a metric artefact that would
    have refused every rung.

    THE MEASUREMENT IS AN EXACT GLOBAL SEARCH, after two wrong attempts. A
    re-timed track reaches the same place at a different moment by design, so a
    same-index comparison is meaningless; a windowed monotone search can lag
    behind a faster track (it reported 320 mm on a rung that is within 2 mm);
    an arc-length anchor made it *worse* (253 mm on a uniform rung that is
    provably ON the taught polyline, 0.01 mm by brute force); equal-fraction
    comparison folds in a longitudinal offset and read 67 mm.

    So: every point of one path against every segment of the other, in chunks —
    the definition, with no heuristic to be wrong. Cost is real (a few seconds
    per rung per direction) and it is paid only on rungs that already cleared
    the enforced gates.

    The reported number is the PERPENDICULAR distance — the nearest point on
    the taught path — which is what the cap asks: does the pad stay on the
    taught path? The arc-length ratio is reported alongside, because it is the
    thing that moves when a refit removes recorder dither (the left file's refit
    came out 0.940x, "the missing 6 % is the beat jitter"), and it is NOT a
    path deviation."""
    q_ref = np.asarray(q_ref_deg, dtype=float)
    q_new = np.asarray(q_new_deg, dtype=float)
    if q_ref.shape == q_new.shape and np.array_equal(q_ref, q_new):
        # rung 0: the recording's own points ARE the track, so the deviation is
        # zero BY CONSTRUCTION and needs no FK to say so.
        z = {"p50_mm": 0.0, "p95_mm": 0.0, "max_mm": 0.0}
        return {"frame": frame, "frame_source": "n/a (identical points)",
                "method": "identical points", "arc_ratio": 1.0,
                "new_to_taught_mm": dict(z), "taught_to_new_mm": dict(z), "max_mm": 0.0}
    from teachlab.kinematics import ArmKinematics
    ak = ArmKinematics(side=side_char(side), frame=frame)
    p_ref = ak.fk_batch(np.radians(q_ref))[0]
    p_new = ak.fk_batch(np.radians(q_new))[0]

    def _stats(v):
        return {"p50_mm": round(float(np.percentile(v, 50)), 3),
                "p95_mm": round(float(np.percentile(v, 95)), 3),
                "max_mm": round(float(v.max()), 3)}

    fwd = _stats(_point_to_polyline_mm(p_new, p_ref))
    rev = _stats(_point_to_polyline_mm(p_ref, p_new))
    lens = float(np.linalg.norm(np.diff(p_new, axis=0), axis=1).sum())
    lenr = float(np.linalg.norm(np.diff(p_ref, axis=0), axis=1).sum())
    return {"frame": ak.frame, "frame_source": ak.frame_source,
            "method": "exact global point-to-polyline distance at the declared frame",
            "arc_ratio": round(lens / lenr, 4) if lenr else None,
            "new_to_taught_mm": fwd, "taught_to_new_mm": rev,
            "max_mm": max(fwd["max_mm"], rev["max_mm"])}


def _point_to_polyline_mm(P: np.ndarray, C: np.ndarray) -> np.ndarray:
    """Distance [mm] from every point of P to the nearest point of the polyline
    C, EXACTLY — the number the all-pairs search returns, found without
    visiting every segment.

    The nearest VERTEX of C to a point (one KD-tree query) is an upper bound
    d_up on its distance to the polyline. The true nearest point X lies on some
    segment s of length L_s with |P - X| <= d_up, so the MIDPOINT of s is within
    d_up + L_s/2 <= d_up + L_max/2 of P (L_max the longest segment of C). Only
    the segments whose midpoint lies in that ball (a second KD-tree, over the
    segment midpoints) can hold it; those are enumerated and measured exactly,
    in chunks.
    On the 9.6k-line bowl recording this is 5.7 s -> ~0.2 s, and the values
    agree with `_point_to_polyline_mm_brute` to floating-point precision
    (test_taught_certify); the certificates issued before 2026-09-15 still
    verify because the number did not change, only how it is found."""
    P = np.asarray(P, dtype=float)
    C = np.asarray(C, dtype=float)
    A, D = C[:-1], np.diff(C, axis=0)
    L = np.linalg.norm(D, axis=1)
    L2 = np.where(L > 0, L * L, 1e-18)
    nseg = A.shape[0]
    if nseg < 64 or P.shape[0] < 64:
        return _point_to_polyline_mm_brute(P, C)
    from scipy.spatial import cKDTree
    # SEGMENTS BY THEIR MIDPOINTS (2026-09-22): a segment of length L holding the nearest point
    # (at distance <= d_up) has its midpoint within d_up + L/2 <= d_up + L_max/2 of the point —
    # half the radius of the vertex ball it replaces, one candidate per segment instead of two per
    # vertex, and the list flattening done by numpy rather than a per-element generator. Exact as
    # before; 3-6x fewer candidates where a dense, self-retracing path meets far points.
    vtree, stree = cKDTree(C), cKDTree(A + 0.5 * D)
    half = 0.5 * float(L.max())
    out = np.empty(P.shape[0])
    step = 1024
    for i0 in range(0, P.shape[0], step):
        Pc = P[i0:i0 + step]
        d_up, _ = vtree.query(Pc)
        balls = stree.query_ball_point(Pc, d_up + half + 1e-12)
        counts = np.fromiter((len(b) for b in balls), dtype=np.intp, count=len(balls))
        si = np.concatenate([np.asarray(b, dtype=np.intp) for b in balls])
        pi = np.repeat(np.arange(Pc.shape[0]), counts)
        w = Pc[pi] - A[si]
        t = np.clip(np.einsum("ij,ij->i", w, D[si]) / L2[si], 0.0, 1.0)
        dist = np.linalg.norm(w - t[:, None] * D[si], axis=1)
        starts = np.concatenate([[0], np.cumsum(counts)[:-1]])
        out[i0:i0 + step] = np.minimum.reduceat(dist, starts)
    return out * 1000.0


def _point_to_polyline_mm_brute(P: np.ndarray, C: np.ndarray) -> np.ndarray:
    """The definition, unabridged: every point of P against every segment of C,
    chunked so the temporaries stay bounded. Kept as the reference the fast
    search is tested against."""
    A, D = C[:-1], np.diff(C, axis=0)
    L2 = np.einsum("ij,ij->i", D, D)
    L2 = np.where(L2 > 0, L2, 1e-18)
    nseg = A.shape[0]
    chunk = max(1, min(P.shape[0], int(2_000_000 / max(nseg, 1))))
    out = np.empty(P.shape[0])
    for i in range(0, P.shape[0], chunk):
        w = P[i:i + chunk, None, :] - A[None, :, :]
        t = np.clip(np.einsum("mnj,nj->mn", w, D) / L2[None, :], 0.0, 1.0)
        foot = A[None, :, :] + t[:, :, None] * D[None, :, :]
        out[i:i + chunk] = np.linalg.norm(P[i:i + chunk, None, :] - foot, axis=2).min(axis=1)
    return out * 1000.0


def write_track(path: pathlib.Path, md: np.ndarray) -> pathlib.Path:
    """NDJSON integer millidegrees, LF on disk (the .proj builder normalises the
    terminators to CRLF, which is what the controller requires)."""
    path = pathlib.Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", newline="") as fh:
        for row in md:
            fh.write('{"point":[' + ",".join(str(int(v)) for v in row) + "]}\n")
    return path


def uniform_millideg(q_ref_deg: np.ndarray, lam: float) -> np.ndarray:
    """DIAL 1 — the taught path played λ times faster, by spacing alone.

    The player's clock is fixed, so speed IS point spacing: reading the taught
    polyline at parameter i·λ puts each new point ON a taught chord (linear
    interpolation in joint space), which is why this cannot move the path —
    unlike the solver's fit, which can shortcut a self-revisiting scrub. λ is
    the only degree of freedom; velocity scales with it, acceleration with λ²
    (advisory)."""
    q_ref = np.asarray(q_ref_deg, dtype=float)
    n = q_ref.shape[0]
    u = np.arange(0.0, n - 1 + 1e-9, float(lam))
    if u.size < 2:
        u = np.array([0.0, float(n - 1)])
    out = np.column_stack([np.interp(u, np.arange(n, dtype=float), q_ref[:, j])
                           for j in range(q_ref.shape[1])])
    return np.round(out * 1000.0).astype(np.int64)


def capped_millideg(q_ref_deg: np.ndarray, lam_fast: float, rate_cap_deg_s: np.ndarray,
                    ramp: float = GUARD_RAMP, clock_s: float = CLOCK_NOMINAL_S,
                    lam_cap_line: np.ndarray = None, return_u: bool = False,
                    smooth_lines: int = 0):
    """The taught path played lambda_fast times faster by spacing alone, except
    where `rate_cap_deg_s` (one value per taught line and joint, inf = no cap)
    says a joint may not move faster than that: there the spacing is capped so
    the joint's rate at `clock_s` stays at or below the cap, with a ramp into
    and out of every capped stretch. Points still lie on taught chords (linear
    interpolation in joint space), so the path cannot move; only the clock
    along it does. `guarded_millideg` builds the cap for the near-limit zones;
    a turn guard would build it around reversals — the machinery is the same.

    Per taught segment k the largest admissible advance is lambda_k =
    cap_{k,j} * clock / |dq_{k,j}| over the joints capped at either end of the
    segment, clipped to lambda_fast; a backward then a forward pass bound the
    change of lambda per taught line to `ramp`, so a slow-down begins before
    the stretch and ends after it. The output parameter then advances by
    lambda(u) per player tick, and the last taught point is always reached."""
    q = np.asarray(q_ref_deg, dtype=float)
    n = q.shape[0]
    cap = np.asarray(rate_cap_deg_s, dtype=float)
    assert cap.shape == q.shape, "one rate cap per taught line and joint"
    step = np.abs(np.diff(q, axis=0))
    seg_cap = np.minimum(cap[:-1], cap[1:])                          # capped at either end
    with np.errstate(divide="ignore", invalid="ignore"):
        lam_seg = np.where(np.isfinite(seg_cap), seg_cap * float(clock_s) / np.maximum(step, 1e-9), np.inf)
    lam = np.minimum(float(lam_fast), lam_seg.min(axis=1))
    if lam_cap_line is not None:
        # a SECTION override (R5): lambda may not exceed this on the segments it names
        lc = np.asarray(lam_cap_line, dtype=float)
        assert lc.shape[0] == n, "one lambda cap per source line"
        lam = np.minimum(lam, np.minimum(lc[:-1], lc[1:]))
    for k in range(len(lam) - 2, -1, -1):
        lam[k] = min(lam[k], lam[k + 1] + ramp)
    for k in range(1, len(lam)):
        lam[k] = min(lam[k], lam[k - 1] + ramp)
    if smooth_lines and smooth_lines > 1 and len(lam) > smooth_lines:
        # ROUND THE FAST CORNERS. A rate profile that drops from the rating straight into a
        # cap has a kink there, and the 20 Hz zero-phase filter G2 reads the rate through
        # rings past the rating on the fast side of it (1.7 % over on a sweep whose raw rate
        # never exceeded 0.97 x rating). min(lambda, moving average) lowers the profile
        # into every corner from the fast side only: no cap is ever exceeded, the slow
        # stretches are untouched, and the emitted rate is itself band-limited.
        w = int(smooth_lines)
        kern = np.ones(w) / w
        padded = np.concatenate([np.full(w // 2, lam[0]), lam, np.full(w - 1 - w // 2, lam[-1])])
        lam = np.minimum(lam, np.convolve(padded, kern, mode="valid"))
    us = [0.0]
    while us[-1] < n - 1:
        k = min(int(us[-1]), len(lam) - 1)
        step_k = float(lam[k])
        # the advance crosses every line between here and where it lands, and a big
        # lambda must not jump over a stretch that is capped tighter (found by the
        # 5 mm-budget test: 756 emitted lines sat over a budget the cap had honoured
        # line by line)
        for _ in range(2):
            k1 = min(len(lam) - 1, int(us[-1] + step_k) + 1)
            step_k = min(step_k, float(lam[k:k1 + 1].min()))
        us.append(min(us[-1] + max(step_k, 1e-6), float(n - 1)))
    u = np.array(us)
    if u.size < 2:
        u = np.array([0.0, float(n - 1)])
    out = np.column_stack([np.interp(u, np.arange(n, dtype=float), q[:, j]) for j in range(q.shape[1])])
    md = np.round(out * 1000.0).astype(np.int64)
    return (md, u) if return_u else md


def near_limit_caps(q_ref_deg: np.ndarray, near_deg: float = POSITION_ALLOWANCE_DEG,
                    v_near: float = NEAR_LIMIT_RATE_DEG_S) -> np.ndarray:
    """The rate-cap array for the near-limit guard: GUARD_DESIGN_FRACTION x v_near
    wherever a joint is inside near_deg + GUARD_ZONE_EXTRA_DEG of its limit,
    inf elsewhere (designed below the bar G1 judges at; see the constants)."""
    q = np.asarray(q_ref_deg, dtype=float)
    lim = np.array(_ptr().K.JOINT_LIMITS_DEG, dtype=float)
    marg = np.minimum(q - lim[:, 0], lim[:, 1] - q)
    near = marg < float(near_deg) + GUARD_ZONE_EXTRA_DEG
    return np.where(near, GUARD_DESIGN_FRACTION * float(v_near), np.inf)


def guarded_millideg(q_ref_deg: np.ndarray, lam_fast: float, near_deg: float = POSITION_ALLOWANCE_DEG,
                     v_near: float = NEAR_LIMIT_RATE_DEG_S, ramp: float = GUARD_RAMP,
                     clock_s: float = CLOCK_NOMINAL_S) -> np.ndarray:
    """DIAL 1 with the near-limit guard (`--timing legacy`): `capped_millideg` with
    `near_limit_caps`. Superseded by the execution model on 2026-09-15."""
    return capped_millideg(q_ref_deg, lam_fast, near_limit_caps(q_ref_deg, near_deg, v_near),
                           ramp=ramp, clock_s=clock_s)


# ─────────────────────────────────────────────── the execution model (2026-09-15)
def execution_model(path=None) -> dict:
    """The live model: EXECUTION_MODEL_FILE (or `path`) over the defaults, so a
    refit changes every later certificate and none of the earlier ones (each
    certificate carries the values it was judged with)."""
    m = dict(DEFAULT_EXECUTION_MODEL)
    p = pathlib.Path(path) if path else EXECUTION_MODEL_FILE
    if p.is_file():
        try:
            m.update({k: v for k, v in json.loads(p.read_text()).items() if k in m})
            m["file"] = ct.repo_rel(p, REPO) if str(p).startswith(str(REPO)) else str(p)
        except Exception as exc:                                       # noqa: BLE001
            raise SystemExit(f"the execution model {p} does not read: {exc!r}")
    return m


def _model_params(m: dict) -> dict:
    """The numbers a certificate records and `verify` re-applies."""
    return {k: m.get(k) for k in ("version", "fitted", "percentile", "tau_path_s", "tau_path_p50_s",
                                  "tau_limit_s", "floor_deg", "limit_eps_deg", "pad_factor",
                                  "tau_joint_s", "departure_calibration",
                                  "prominence_deg", "approach_window_s", "window_after_s")}


#: SMOOTHING THE RECORDING (Newton, 2026-09-17: "an allowance of 10mm allowed deviation on
#: smoothing and jitter removal", 5 Hz zero-phase low-pass). A drag-teach recording advances each
#: joint by one or two encoder quanta per sample, so its commanded speed ripples 21-36 % line to
#: line; CANFD passes that ripple to the motors (current fluctuation 2.4-2.7x the upload channel's
#: on slots 43/49, 1.01x on a smooth plan) and the arm visibly "hits each point". An interpolating
#: spline cannot remove it — RealMan's rm_control spline through every line only takes slot 49 from
#: 22 % to 14 % — so the recording is LOW-PASSED at its own clock, BEFORE pauses and timing: 5 Hz
#: there takes the ripple to ~5 % while moving the pad 0.30 mm p95 / 1.11 mm max, where the same
#: filter on the emitted file (playback clock, 2-2.75x faster motion) moves it ~6 mm. The path
#: change is not a new gate: G4 already holds the bytes to the taught RECORDING within
#: `--max-deviation-mm` (10 mm), and with smoothing it measures against the raw, unsmoothed one.
SMOOTH_ORDER = 2


def smooth_taught(q_deg: np.ndarray, hz: float, clock_s: float = CLOCK_RECORD_S) -> np.ndarray:
    """Zero-phase Butterworth low-pass (order SMOOTH_ORDER, run forward and back) of a joint
    series sampled at *clock_s*. Zero-phase so the smoothed path is not shifted along itself.

    THE DEFAULT IS THE RECORD CLOCK, 5 ms. It was `CLOCK_NOMINAL_S` (5.5545 ms, a playback average),
    which makes the filter 11 % wider than asked: slots 50 and 51 carry a 5 Hz label and a 5.55 Hz
    cutoff (2026-09-17 review, C1). Their measured result stands — the ripple went and the path moved
    <= 0.9 mm — but the number to quote for them is 5.55 Hz, and a new rung at `--smooth-hz 5` now gets
    5 Hz.
    """
    from scipy.signal import butter, filtfilt
    q = np.asarray(q_deg, float)
    nyq = 0.5 / float(clock_s)
    if not 0.0 < float(hz) < nyq:
        raise SystemExit(f"--smooth-hz {hz} must be inside (0, {nyq:.1f}) Hz at a {clock_s * 1e3:.4f} ms clock")
    if q.shape[0] < 16:
        return q.copy()
    b, a = butter(SMOOTH_ORDER, float(hz) / nyq)
    return filtfilt(b, a, q, axis=0, padlen=min(3 * max(len(a), len(b)) * 50, q.shape[0] - 1))


def strip_pauses(q_deg: np.ndarray, tol_deg: float = PAUSE_TOL_DEG,
                 min_lines: int = PAUSE_MIN_LINES) -> tuple:
    """R4: a recording's pauses — the operator holding still, at the ends and at
    reversals — are not part of the path. A run of `min_lines` or more lines
    whose every step is within `tol_deg` on every joint collapses to its first
    line. The polyline is unchanged (identical points removed), so G4 against
    the recording cannot move. Returns (q, info)."""
    q = np.asarray(q_deg, dtype=float)
    n = q.shape[0]
    if n < 3:
        return q, dict(removed_lines=0, removed_s=0.0, runs=0, longest_lines=0)
    # STILL MEANS WITHIN tol OF THE RUN'S FIRST LINE, not of the previous line: a slow apex
    # whose every step is under tol but drifts degrees over the run is path, not a pause
    # (judged step by step it was collapsed, cutting the apex by up to 9 mm at the pad)
    keep = np.ones(n, dtype=bool)
    tol = float(tol_deg)
    runs, longest = 0, 0
    k = 0
    while k < n - 1:
        anchor = q[k]
        j = k + 1
        while j < n and np.abs(q[j] - anchor).max() <= tol:
            j += 1
        length = j - k                                # lines k .. j-1 sit within tol of line k
        if length >= int(min_lines):
            keep[k + 1:j] = False
            runs += 1
            longest = max(longest, length)
        k = j if length >= int(min_lines) else k + 1
    keep[-1] = True                                   # the end point is part of the path
    removed = int((~keep).sum())
    # PRICED AT THE RECORDING'S OWN CLOCK (2026-09-21). It was CLOCK_NOMINAL_S, the upload
    # player's average, which overstated the removed time by 11 %: slots 43/49 read 36.449 s for
    # 32.81 s of recording, 50/51 34.377 s for 30.95 s. Certificates issued before carry the old
    # figure; `removed_s_clock_s` says which clock a record used.
    return q[keep], dict(removed_lines=removed, removed_s=round(removed * CLOCK_RECORD_S, 3),
                         removed_s_clock_s=CLOCK_RECORD_S, runs=runs, longest_lines=int(longest))


#: The pad speeds the slowness census reports time under (mm/s).
SLOW_PAD_MM_S = (20.0, 50.0)


def slowness_census(q_deg: np.ndarray, side: str, frame: str = None,
                    clock_s: float = CLOCK_GATE_S) -> dict:
    """HOW MUCH OF A FILE IS SLOW OR STILL, measured on its bytes at *clock_s*. Recorded, never
    refused (2026-09-21, Newton: "what is the efficiency in removing the too slow and close to stop
    points ... I think some persist"). No gate read this before: a certificate could not say
    whether a file still crawled, and admitted-only rows recorded nothing about it at all.

    Pad = the declared contact frame. Its path is band-limited at 20 Hz before differencing, the
    house rule for any rate figure. `r4_still` is what R4 would STILL remove from these bytes — on
    a time-optimal file those are the stops at reversals, not recorded pauses."""
    q = np.asarray(q_deg, dtype=float)
    frame = frame or f"{side_char(side)}_glove_frame_2"
    out = dict(clock_s=float(clock_s), frame=frame,
               note="recorded, never refused: time under a pad speed, the longest such stretch, "
                    "and the still runs R4 would still remove from these bytes")
    if q.shape[0] < 32:
        return out
    P = _bandlimit_q(pad_path_m(q, side, frame) * 1000.0, clock_s)
    v = np.linalg.norm(np.diff(P, axis=0), axis=1) / float(clock_s)
    out.update(pad_speed_p10_mm_s=round(float(np.percentile(v, 10)), 1),
               pad_speed_p50_mm_s=round(float(np.percentile(v, 50)), 1),
               pad_speed_p90_mm_s=round(float(np.percentile(v, 90)), 1))
    for thr in SLOW_PAD_MM_S:
        slow = v < thr
        longest, run = 0, 0
        for x in slow:
            run = run + 1 if x else 0
            longest = max(longest, run)
        out[f"s_below_{thr:g}_mm_s"] = round(float(slow.sum()) * float(clock_s), 3)
        out[f"longest_below_{thr:g}_mm_s_s"] = round(longest * float(clock_s), 3)
    _, info = strip_pauses(q)
    out["r4_still"] = dict(lines=info["removed_lines"], runs=info["runs"],
                           longest_lines=info["longest_lines"])
    return out


def _bandlimit_q(q: np.ndarray, clock_s: float, fc_hz: float = 20.0) -> np.ndarray:
    from scipy.signal import butter, filtfilt
    if q.shape[0] <= 30:
        return np.asarray(q, dtype=float)
    b_, a_ = butter(4, min(fc_hz / (0.5 / float(clock_s)), 0.99), "low")
    return filtfilt(b_, a_, np.asarray(q, dtype=float), axis=0)


def path_reversals(q_deg: np.ndarray, clock_s: float, model: dict, include_end: bool = False) -> list:
    """Every commanded extreme of every joint — a peak or trough of the 20 Hz
    band-limited trace at least `prominence_deg` prominent. Each carries the
    arrival rate (the largest band-limited rate in the `approach_window_s`
    before it, the definition the law was fitted with), the direction of travel
    into it and the margin to the limit ahead of it. The END of a file is not
    one of them: the command itself stops there and the arm settles onto the
    last line (0.01-0.09 deg off it in every 2026-09-15 capture); the overshoot
    law is about a command that turns back under a joint still moving."""
    from scipy.signal import find_peaks
    q = np.asarray(q_deg, dtype=float)
    n = q.shape[0]
    qbl = _bandlimit_q(q, clock_s)
    rate = np.abs(np.gradient(qbl, axis=0)) / float(clock_s)
    lim = np.array(_ptr().K.JOINT_LIMITS_DEG, dtype=float)
    n_w = max(1, int(np.ceil(float(model["approach_window_s"]) / float(clock_s))))
    out = []
    for j in range(q.shape[1]):
        idx = []
        for sgn in (1.0, -1.0):
            pk, _ = find_peaks(sgn * qbl[:, j], prominence=float(model["prominence_deg"]))
            idx += [(int(k), sgn) for k in pk]
        if include_end and n >= 2:
            step = qbl[-1, j] - qbl[-2, j]
            if abs(step) > 1e-9:
                idx.append((n - 1, 1.0 if step > 0 else -1.0))
        for k, sgn in sorted(idx):
            v = float(rate[max(0, k - n_w):k + 1, j].max())
            margin = float(lim[j, 1] - q[k, j]) if sgn > 0 else float(q[k, j] - lim[j, 0])
            out.append(dict(k=k, j=j, sign=sgn, v_deg_s=v, margin_deg=margin,
                            kind=("end" if k == n - 1 else "reversal")))
    return sorted(out, key=lambda r: (r["k"], r["j"]))


_PAD_FK_CACHE: dict = {}
_DEPARTURE_CACHE: dict = {}


def pad_path_m(q_deg: np.ndarray, side: str, frame: str) -> np.ndarray:
    """(N,3) pad positions in metres at the declared frame, memoised on the
    joint bytes (the ladder asks for the same rung's path twice — once to gate
    it, once to price its deviation — and FK is the expensive part)."""
    q = np.asarray(q_deg, dtype=float)
    key = (hashlib.sha256(np.ascontiguousarray(q).tobytes()).hexdigest(), side_char(side), frame)
    if key not in _PAD_FK_CACHE:
        from teachlab.kinematics import ArmKinematics
        if len(_PAD_FK_CACHE) > 8:
            _PAD_FK_CACHE.clear()
        _PAD_FK_CACHE[key] = ArmKinematics(side=side_char(side), frame=frame).fk_batch(np.radians(q))[0]
    return _PAD_FK_CACHE[key]


def pad_departure_mm(P: np.ndarray, tau_s: float, clock_s: float = CLOCK_NOMINAL_S) -> np.ndarray:
    """THE DEPARTURE LAW, at every line (2026-09-16). The arm is `tau_s` behind
    its command and still doing what the command said then, so where it is is
    where that motion CARRIES ON TO:

        p_pred(k) = p(k) + tau x v(k),   v(k) = (p(k+1) - p(k)) / clock

    and the departure is that point's distance to the commanded path itself.
    One constant, no turn detector, and it is the right number in all three
    regimes by construction: on a straight run the carry-on stays ON the path
    (zero), past a cusp it leaves the path by the overshoot, and around a curve
    it leaves by the chord it cuts. Measured against the two slot-43 runs of
    2026-09-16 at tau = 0.054 s it reads p50 1.44 mm against a measured 1.36
    and p95 6.10 against 7.70, and it ranks the file's rough stretches 3-5x
    better than the reversals-only law it replaces (Spearman 0.19-0.42 per
    200-line bin against 0.07). The last line has no successor, so it carries
    its neighbour's value."""
    P = np.asarray(P, dtype=float)
    if P.shape[0] < 3:
        return np.zeros(P.shape[0])
    v = (P[1:] - P[:-1]) / float(clock_s)
    L = float(tau_s) * np.linalg.norm(v, axis=1)
    L = np.minimum(L, _remaining_arc_m(P))          # never past the file's last point
    with np.errstate(invalid="ignore", divide="ignore"):
        vh = v / np.maximum(np.linalg.norm(v, axis=1), 1e-12)[:, None]
    d = _point_to_polyline_mm(P[:-1] + L[:, None] * vh, P)
    return np.concatenate([d, d[-1:]])


def _departure_attribution(q_deg: np.ndarray, side: str, frame: str, tau_j,
                           clock_s: float, line: int) -> dict:
    """WHICH JOINT SPENDS THE BUDGET at `line`. The offset is the sum of the
    per-joint carry-ons, so dropping one joint's lag to the mean and re-measuring
    says what that joint contributed. This is the number the C-space authoring
    score wants: not "18 mm here" but "J1 and J6 own this stretch"."""
    q = np.asarray(q_deg, float)
    tau = np.asarray(tau_j, float)
    k = int(np.clip(line, 0, q.shape[0] - 2))
    lo, hi = max(0, k - 2), min(q.shape[0], k + 3)
    seg = q[lo:hi]
    if seg.shape[0] < 3:
        return {}
    frame = frame or f"{side_char(side)}_glove_frame_2"
    base = float(pad_departure_joint_mm(seg, side, frame, tau, clock_s).max())
    share = {}
    for j in range(7):
        t2 = tau.copy(); t2[j] = tau.mean()
        share[f"J{j + 1}"] = round(base - float(
            pad_departure_joint_mm(seg, side, frame, t2, clock_s).max()), 3)
    top = sorted(share, key=lambda n: -abs(share[n]))[:3]
    return dict(line=k, mm=round(base, 3), per_joint_mm=share, owns=top)


def pad_departure_joint_mm(q_deg: np.ndarray, side: str, frame: str, tau_j,
                           clock_s: float = CLOCK_NOMINAL_S,
                           P: np.ndarray = None, model: dict = None) -> np.ndarray:
    """THE DEPARTURE LAW IN JOINT SPACE, per joint (2026-09-16).

        p_pred(k) = FK( q(k) + tau_j (*) qdot(k) ),   qdot = (q(k+1) - q(k)) / clock

    and the departure is that point's distance to the commanded pad path. The
    difference from `pad_departure_mm` is WHERE the carry-on is formed. Summing
    the joints into one pad velocity first, then carrying that forward by one
    tau, throws away which joint is late -- and that is the whole effect: at one
    line of a commode raster the common-lag offset is 2.50 mm and lies 2.50 mm
    ALONG the path with 0.00 mm perpendicular (a pure time delay leaves the path
    not at all), while the per-joint offset is 4.95 mm with 3.98 mm PERPENDICULAR.

    THE SEVEN OFFSETS MUST BE APPLIED TOGETHER. Measuring each joint's offset
    against the path on its own and combining the seven as a root-sum-square --
    tried 2026-09-16 to stop the p90 lags being priced as if they coincided --
    is WRONG, and measurably so: it reads p50 22 mm where the arm measures 1.5.
    Each joint's individual offset carries an ALONG-path component, and those are
    exactly what cancel between joints when the lag is common. An RSS cannot
    cancel, so it charges the common mode seven times over. The cancellation is
    the law.

    AND qdot IS THE ONE-LINE DIFFERENCE, not a windowed rate. Taking the fastest
    rate over `approach_window_s` was tried the same day, on the strength of ONE
    observation (line 11170 of slot 43, where tau_J6 x the 0.3 s peak rate
    happened to match the measured 5.51 deg error). Applied at every line it
    reads p50 38 mm against a measured 1.5: the max over a window is a REVERSAL
    quantity -- the arm arrives at an extreme at its peak rate -- and a scrub
    that reverses every 0.3 s is at its peak somewhere in every window. That
    single match was an over-fit.

    Measured against the two complete slot-43 runs of 2026-09-16 (the first files
    this law shaped, run to completion): p50 6.4 mm against a measured 1.4-1.5,
    p95 14.9 against 9.3-10.7, max 19.5 against 40.2-43.2, Spearman -0.21. So it
    is conservative through the bulk, it does NOT bound the worst point, and on a
    taught file it ranks worse than the pad-space law it replaced (+0.31). What it
    does hold is out-of-sample magnitude: 1.99x on 148 planned rasters where the
    pad-space law read 9.08x. Both statements are in
    Resource/reports/2026-09-16_predictor_generalisation."""
    q = np.asarray(q_deg, dtype=float)
    if q.shape[0] < 3:
        return np.zeros(q.shape[0])
    frame = frame or f"{side_char(side)}_glove_frame_2"
    if P is None:
        P = pad_path_m(q, side, frame)
    tau = np.asarray(tau_j, dtype=float).reshape(1, -1)
    qd = (q[1:] - q[:-1]) / float(clock_s)
    Pp = pad_path_m_uncached(q[:-1] + tau * qd, side, frame)
    reach = np.linalg.norm(Pp - P[:-1], axis=1)
    rem = _remaining_arc_m(P)
    over = reach > rem
    if over.any():
        sc = np.ones_like(reach)
        sc[over] = rem[over] / np.maximum(reach[over], 1e-12)
        Pp = P[:-1] + (Pp - P[:-1]) * sc[:, None]
    d = _point_to_polyline_mm(Pp, P)
    return np.concatenate([d, d[-1:]])


def pad_path_m_uncached(q_deg: np.ndarray, side: str, frame: str) -> np.ndarray:
    """FK without the memo: the carry-on path is asked for once per call and
    would evict the recording's own pad path from `_PAD_FK_CACHE`."""
    from teachlab.kinematics import ArmKinematics
    return ArmKinematics(side=side_char(side), frame=frame).fk_batch(np.radians(np.asarray(q_deg, float)))[0]


def _remaining_arc_m(P: np.ndarray) -> np.ndarray:
    """Arc length from each line to the end of the path (per segment start)."""
    seg = np.linalg.norm(np.diff(P, axis=0), axis=1)
    return np.concatenate([np.cumsum(seg[::-1])[::-1][1:], [0.0]]) + seg


def pad_speed_budget_m_s(P: np.ndarray, tau_s: float, budget_mm: float,
                         n_probe: int = 24) -> np.ndarray:
    """THE DEPARTURE LAW INVERTED, for the emitter: the fastest the pad may
    travel at each line and still stay inside `budget_mm`.

    The law says the arm carries on tangentially for a length L = tau x speed,
    so the question is how far the carry-on can reach before it is `budget_mm`
    from the path. That is pure geometry of the path — it does not depend on
    the timing at all — so it is computed once per recording: walk a set of
    candidate lengths along each line's own tangent, measure each landing point
    against the path, and keep the largest length every shorter candidate also
    cleared (a path that doubles back can bring a longer reach near another of
    its own branches, so monotonicity is enforced rather than assumed)."""
    P = np.asarray(P, dtype=float)
    n = P.shape[0]
    if n < 3:
        return np.full(n, np.inf)
    t = P[1:] - P[:-1]
    L0 = np.linalg.norm(t, axis=1)
    that = t / np.maximum(L0, 1e-12)[:, None]
    # A MONOTONE SCAN TO BRACKET, THEN BISECTION TO RESOLVE. The scan alone
    # resolves the allowed length only to its own step (a x1.5 grid answers within
    # 40 %, which slowed a file that needed no slowing at all), and bisection alone
    # would assume the distance grows with the length — false on a path that doubles
    # back near itself. So: scan coarsely and keep the largest length every shorter
    # one also cleared (that AND is the guard), then bisect inside the bracket it
    # leaves. 16 + 8 passes resolve the length to ~0.4 %.
    scan = np.geomspace(0.2e-3, 0.6, max(8, n_probe))
    ok = np.ones(n - 1, dtype=bool)
    lo = np.zeros(n - 1)
    hi = np.full(n - 1, np.inf)
    for L in scan:
        # NOT clamped to the end of the path here, deliberately: the departure STATISTIC
        # exempts the file's last lines (the program stops and the arm settles onto the
        # last point, measured 0.04-0.06 deg away on both slot-43 runs), but the CAP must
        # still pace them — exempting them handed the emitter a free hand at the end and
        # it emitted a 157 mm pad step at 2.8 m/s, past the robot's own 1.8 m/s TCP cap.
        d = _point_to_polyline_mm(P[:-1] + L * that, P)
        passes = ok & (d <= float(budget_mm))
        hi = np.where(ok & ~passes & np.isinf(hi), L, hi)          # the first failure
        lo = np.where(passes, L, lo)
        ok = passes
    still_ok = np.isinf(hi)                                        # never failed: no cap
    hi = np.where(still_ok, lo, hi)
    for _ in range(8):
        mid = 0.5 * (lo + hi)
        d = _point_to_polyline_mm(P[:-1] + mid[:, None] * that, P)
        good = d <= float(budget_mm)
        lo = np.where(good, mid, lo)
        hi = np.where(good, hi, mid)
    v = lo / max(float(tau_s), 1e-9)
    v = np.where(still_ok, np.inf, v)                              # never binding
    return np.concatenate([v, v[-1:]])


def pad_levers_mm_per_deg(q_rows_deg: np.ndarray, side: str, frame: str) -> np.ndarray:
    """(rows, 7, 3): how far the pad moves (mm) per degree of each joint at each
    row, by central differences of the declared frame's FK — the lever arm that
    turns a joint overshoot into a path deviation."""
    from teachlab.kinematics import ArmKinematics
    ak = ArmKinematics(side=side_char(side), frame=frame)
    q = np.radians(np.asarray(q_rows_deg, dtype=float))
    eps = np.radians(0.25)
    rows, nj = q.shape
    stack = np.empty((rows * nj * 2, nj))
    for i in range(nj):
        e = np.zeros(nj)
        e[i] = eps
        stack[(2 * i) * rows:(2 * i + 1) * rows] = q + e
        stack[(2 * i + 1) * rows:(2 * i + 2) * rows] = q - e
    pos = ak.fk_batch(stack)[0]
    J = np.empty((rows, nj, 3))
    for i in range(nj):
        J[:, i, :] = (pos[(2 * i) * rows:(2 * i + 1) * rows] - pos[(2 * i + 1) * rows:(2 * i + 2) * rows]) / (2 * eps)
    return J * 1000.0 * np.pi / 180.0


def model_predictions(q_deg: np.ndarray, side: str, frame: str, model: dict, clock_s: float,
                      budget_mm: float = MAX_DEVIATION_MM, *,
                      cal_lines: np.ndarray = None) -> dict:
    """R3, applied to ONE point list: what the arm will do with these bytes.

    At every extreme the joint overshoots by max(floor, tau x arrival rate) —
    tau_path (p90) for the path, tau_limit (the maximum) at a limit. The pad
    leaves the path at that extreme by the overshoots of every joint reversing
    within `window_after_s` of it, through their lever arms, added as vectors.
    Reported: the worst predicted deviation (G5) and the smallest predicted
    executed margin at a limit (G1), each with where and why."""
    q = np.asarray(q_deg, dtype=float)
    revs = path_reversals(q, clock_s, model)
    out = dict(reversals=len(revs), pred_dev_max_mm=0.0, pred_dev_worst=None,
               pred_exec_margin_min_deg=None, pred_exec_margin_worst=None, geometry_ok=True,
               over_budget=0, over_prefer=0)
    # THE DEPARTURE LAW AT EVERY LINE (2026-09-16) — this is G5. The reversals-only
    # prediction below is kept as the diagnostic it has earned: on the two slot-43 runs
    # it put its worst number (18.1 mm) where the arm measured 10.0 and 2.8 mm, while
    # both runs' real worst (32.0 / 41.5 mm) fell 0.7 s from any reversal, in a stretch
    # the model was letting run at the ratings. The line law prices that stretch.
    frame = frame or f"{side_char(side)}_glove_frame_2"
    P = pad_path_m(q, side, frame)
    # G5 IS THE JOINT-SPACE LAW NOW (2026-09-16, the 158-run corpus test). The pad-space
    # number is kept beside it because it is what every certificate before this was judged
    # with, and because the gap between them IS the differential-lag term: where the two
    # agree the joints are lagging together, where the joint-space number runs away they
    # are not. `departure_calibration` carries the measured 2x under-read.
    cal = float(model.get("departure_calibration") or 1.0)
    tau_j = np.asarray(model.get("tau_joint_s") or [model["tau_path_s"]] * 7, float)
    dep = cal * pad_departure_joint_mm(q, side, frame, tau_j, clock_s, P=P, model=model)
    # THE MEASURED PER-LINE CORRECTION, when the file was shaped with one. The model
    # gets the departure's magnitude right across path families and its LOCATION wrong
    # (Spearman -0.22 against the two complete slot-43 runs) while two runs of the same
    # bytes agree at +0.64, so where the correction exists it is the better number: on
    # the installed slot-43 file it reads 41.5 mm against a MEASURED 40.2-43.2, where
    # the raw prediction says 19.8. The certificate carries it (`cal_pack`), so this
    # still recomputes from the files alone.
    dep_pad = pad_departure_mm(P, float(model["tau_path_s"]), clock_s)
    dep_max = cal * pad_departure_joint_mm(
        q, side, frame, tau_j * float(model["tau_limit_s"]) / max(float(tau_j.mean()), 1e-9),
        clock_s, P=P, model=model)
    if cal_lines is not None and len(cal_lines) == q.shape[0]:
        cl = np.asarray(cal_lines, float)
        dep = dep * cl
        dep_max = dep_max * cl
    k = int(np.argmax(dep))
    # THE GATE COMPARES THE NUMBER IT PUBLISHES. The emitter honours the budget on the
    # source's geometry and the gate reads the emitted file's, so the two agree to about
    # a tenth of a percent and never exactly; rounding the published figure to the
    # hundredth of a millimetre it is stated in — and gating on that — keeps the
    # certificate's own claim and its bar consistent instead of failing a file by 4 um.
    worst_mm = round(float(dep.max()), 2)
    out.update(
        pred_line_dev_max_mm=worst_mm,
        pred_line_dev_p95_mm=round(float(np.percentile(dep, 95)), 3),
        pred_line_dev_p50_mm=round(float(np.percentile(dep, 50)), 3),
        pred_line_dev_worst=dict(line=k, mm=round(float(dep[k]), 3),
                                 pad_speed_m_s=round(float(np.linalg.norm(
                                     (P[min(k + 1, len(P) - 1)] - P[k]) / clock_s)), 4)),
        pred_line_dev_bound_mm=round(float(dep_max.max()), 3),
        pred_line_over_budget=int((np.round(dep, 2) > float(budget_mm)).sum()),
        pred_line_law=("joint-space per-joint carry-on x departure_calibration"
                       + (" x the measured per-line correction" if cal_lines is not None else "")),
        pred_line_measured_calibrated=bool(cal_lines is not None),
        pred_line_dev_calibration=cal,
        pred_line_dev_padspace_max_mm=round(float(dep_pad.max()), 3),
        pred_line_dev_padspace_p95_mm=round(float(np.percentile(dep_pad, 95)), 3),
        pred_line_dev_attribution=_departure_attribution(q, side, frame, tau_j, clock_s, k),
        g5_ok=bool(worst_mm <= float(budget_mm)))
    if not revs:
        return out
    tau_p, tau_l = float(model["tau_path_s"]), float(model["tau_limit_s"])
    floor, eps, pf = float(model["floor_deg"]), float(model["limit_eps_deg"]), float(model["pad_factor"])
    n_grp = max(1, int(np.ceil(float(model["window_after_s"]) / float(clock_s))))
    rows = sorted({r["k"] for r in revs})
    J = pad_levers_mm_per_deg(q[rows], side, frame)
    jrow = {k: i for i, k in enumerate(rows)}
    worst_dev, worst_margin = None, None
    for r in revs:
        group = [s for s in revs if abs(s["k"] - r["k"]) <= n_grp]
        # the law's part: every reversing joint's tau x rate through its lever arm, combined as
        # a ROOT-SUM-SQUARE — the overshoots are erratic (two thirds of their variance is not
        # in the command) and not in step with each other, so the p90 of the group is the RSS
        # of the joints' p90s, not their sum (a sum priced a wrist scrub's every reversal as
        # if all seven joints peaked together, and read 54 mm on a file that measured 18 mm
        # at p95); the floor's part: ONCE per reversal, on the longest lever in the group
        lever_max, rss = 0.0, 0.0
        for s in group:
            lever = float(np.linalg.norm(J[jrow[r["k"]], s["j"], :]))
            lever_max = max(lever_max, lever)
            rss += (lever * tau_p * s["v_deg_s"]) ** 2
        dev = pf * max(float(np.sqrt(rss)), floor * lever_max)
        r["pred_dev_mm"] = round(dev, 3)
        r["lever_mm_per_deg"] = round(float(np.linalg.norm(J[jrow[r["k"]], r["j"], :])), 3)
        if worst_dev is None or dev > worst_dev["pred_dev_mm"]:
            worst_dev = dict(line=r["k"], joints=[s["j"] + 1 for s in group],
                             arrival_deg_s=[round(s["v_deg_s"], 1) for s in group],
                             pred_dev_mm=round(dev, 3), kind=r["kind"])
        o_l = max(floor, tau_l * r["v_deg_s"])
        em = r["margin_deg"] - o_l
        r["pred_exec_margin_deg"] = round(em, 3)
        if worst_margin is None or em < worst_margin["pred_exec_margin_deg"]:
            worst_margin = dict(line=r["k"], joint=r["j"] + 1, arrival_deg_s=round(r["v_deg_s"], 1),
                                commanded_margin_deg=round(r["margin_deg"], 3),
                                pred_overshoot_deg=round(o_l, 3), pred_exec_margin_deg=round(em, 3))
        if r["margin_deg"] - floor < eps:
            out["geometry_ok"] = False
    devs = np.array([r["pred_dev_mm"] for r in revs])
    out.update(pred_dev_max_mm=round(float(devs.max()), 3), pred_dev_worst=worst_dev,
               pred_dev_p95_mm=round(float(np.percentile(devs, 95)), 3),
               over_budget=int((devs > float(budget_mm)).sum()),
               over_prefer=int((devs > PREFER_DEVIATION_MM).sum()),
               pred_exec_margin_min_deg=worst_margin["pred_exec_margin_deg"],
               pred_exec_margin_worst=worst_margin,
               reversal_lines=[r["k"] for r in revs])
    return out


def model_caps(q_src_deg: np.ndarray, side: str, frame: str, model: dict, lam_fast: float,
               budget_mm: float = MAX_DEVIATION_MM, clock_s: float = CLOCK_GATE_S) -> tuple:
    """R3 inverted, for the emitter: the rate-cap array (source lines x joints)
    that keeps the predicted deviation inside the budget and the predicted
    executed extreme outside `limit_eps_deg` of every limit — the joint RATING
    everywhere else (no factor on it). Returns (caps, notes)."""
    q = np.asarray(q_src_deg, dtype=float)
    n = q.shape[0]
    # the rating on every line — designed at RATING_FILTER_ALLOWANCE x it, because G2 reads the
    # rate through the 20 Hz zero-phase filter, which overshoots a change of rate by up to 2 %
    # (a file designed exactly at the rating read 1.020 x it): the instrument's own bias, so the
    # band-limited number the gate reads is the rating, not a factor on the manufacturer's limit
    rating = np.array(_ptr().K.QDOT_MAX_DEG_S, dtype=float)
    caps = np.tile(RATING_FILTER_ALLOWANCE * rating, (n, 1))
    tau_p, tau_l = float(model["tau_path_s"]), float(model["tau_limit_s"])
    floor, eps, pf = float(model["floor_deg"]), float(model["limit_eps_deg"]), float(model["pad_factor"])
    revs = path_reversals(q, CLOCK_NOMINAL_S, model)
    notes = dict(reversals=len(revs), capped=0, geometry=[], unreachable=[],
                 what="the joint LIMITS only; the path budget is departure_lambda_cap's")
    if not revs:
        return caps, notes
    n_grp = max(1, int(np.ceil(float(model["window_after_s"]) / CLOCK_NOMINAL_S)))
    # THE WINDOW IS A DISTANCE, NOT A LINE COUNT: the arrival rate the gate reads is the
    # largest band-limited rate in `approach_window_s` of EMITTED time before the extreme,
    # and at the cap the joint covers v_cap x that time in degrees — so the cap holds over
    # that much travel of the joint (plus 0.1 s of it, so the 20 Hz filter's smear of the
    # faster stretch before it stays outside the window), whatever lambda_fast is.
    smear_s = 0.10
    win_before_s = float(model["approach_window_s"]) + smear_s
    win_after_s = float(model["window_after_s"]) + 0.5 * smear_s
    cum = np.vstack([np.zeros((1, q.shape[1])), np.cumsum(np.abs(np.diff(q, axis=0)), axis=0)])
    rows = sorted({r["k"] for r in revs})
    J = pad_levers_mm_per_deg(q[rows], side, frame)
    jrow = {k: i for i, k in enumerate(rows)}
    for r in revs:
        # THE PATH BUDGET IS NOT PRICED HERE ANY MORE (2026-09-16). It used to be:
        # the arrival rate at each reversal was held to budget / (tau x lever arms).
        # `departure_lambda_cap` now owns it, at every line and from the geometry
        # itself, which is both more general (a corner the pad turns without any
        # joint reversing is priced too) and measured better (the reversal-only
        # prediction ranked the two slot-43 runs' rough stretches at Spearman 0.07).
        # What stays here is the JOINT LIMITS — G1 — where the law is validated:
        # measured over predicted overshoot p90 1.04 over 362 turns.
        v_path = np.inf
        if r["margin_deg"] - floor < eps:
            notes["geometry"].append(dict(line=r["k"], joint=r["j"] + 1,
                                          margin_deg=round(r["margin_deg"], 3)))
            v_limit = floor / tau_l
        else:
            v_limit = max(r["margin_deg"] - eps, floor) / tau_l
        v_cap = float(min(v_path, v_limit))
        if not np.isfinite(v_cap) or v_cap >= rating[r["j"]]:
            continue
        j = r["j"]
        lo = int(np.searchsorted(cum[:, j], cum[r["k"], j] - v_cap * win_before_s, side="left"))
        hi = int(np.searchsorted(cum[:, j], cum[r["k"], j] + v_cap * win_after_s, side="right"))
        lo, hi = max(0, min(lo, r["k"])), min(n, max(hi, r["k"] + 1))
        caps[lo:hi, j] = np.minimum(caps[lo:hi, j], v_cap)
        notes["capped"] += 1
    return caps, notes


def parse_sections(tokens) -> list:
    """`--section A:B:F` -> [(a, b, f)]: a, b fractions of the source path (0-1) or
    line numbers (>= 2), f the factor on the fastest speed there (0 < f <= 1)."""
    out = []
    for t in tokens or []:
        try:
            a, b, f = (float(x) for x in str(t).split(":"))
        except Exception:                                              # noqa: BLE001
            raise SystemExit(f"--section wants A:B:F, got {t!r}")
        if not (0 < f <= 1.0) or not (a < b):
            raise SystemExit(f"--section {t!r}: need A < B and 0 < F <= 1")
        out.append((a, b, f))
    return out


def parse_dwells(tokens) -> list:
    """`--dwell S:SEC` -> [(s, sec)]: s a fraction of the source path (0-1) or a
    line number (>= 2), sec the dwell in seconds."""
    out = []
    for t in tokens or []:
        try:
            s, sec = (float(x) for x in str(t).split(":"))
        except Exception:                                              # noqa: BLE001
            raise SystemExit(f"--dwell wants S:SEC, got {t!r}")
        if sec <= 0:
            raise SystemExit(f"--dwell {t!r}: the dwell must be positive")
        out.append((s, sec))
    return out


def _as_line(x: float, n: int) -> int:
    return int(round(x * (n - 1))) if x <= 1.0 else int(round(x))


def insert_dwells(md: np.ndarray, u: np.ndarray, dwells: list, n_src: int,
                  clock_s: float = CLOCK_NOMINAL_S) -> np.ndarray:
    """R4/R5: the only stops a file contains are the ones the task lists — the
    line nearest each requested station repeated for the requested time."""
    if not dwells:
        return md
    reps = np.ones(md.shape[0], dtype=int)
    for s, sec in dwells:
        k = int(np.argmin(np.abs(u - _as_line(s, n_src))))
        reps[k] += int(round(float(sec) / float(clock_s)))
    return np.repeat(md, reps, axis=0)


def _capture_joints(frames_path) -> tuple:
    """(t, q_deg, status) for the frames of one capture."""
    T, Q, S = [], [], []
    with open(frames_path) as fh:
        for line in fh:
            line = line.strip()
            if not line:
                continue
            d = json.loads(line)
            fr = d.get("frame") or {}
            jp = (fr.get("joint_status") or {}).get("joint_position")
            if jp is None:
                continue
            T.append(float(d.get("t_mono", 0.0)))
            Q.append(jp)
            S.append(fr.get("arm_current_status"))
    return np.asarray(T), np.asarray(Q, float) / 1000.0, np.asarray(S, dtype=object)


def _align_monotone(PE: np.ndarray, PC: np.ndarray, adv: int = 2) -> np.ndarray:
    """Banded monotone alignment of an executed pad path onto a commanded one.
    The index may advance 0..`adv` lines per frame and never go back: a plain
    nearest-vertex search puts an off-path point on whichever pass of a
    self-approaching path is closest, and a time-linear index cannot survive the
    player's clock drifting from nominal."""
    n_f, n_l = PE.shape[0], PC.shape[0]
    BIG = np.float32(1e9)
    prev = np.full(n_l, BIG, dtype=np.float32)
    prev[0] = np.float32(np.linalg.norm(PE[0] - PC[0]))
    choice = np.empty((n_f, n_l), dtype=np.uint8)
    choice[0] = 0
    for i in range(1, n_f):
        d = np.linalg.norm(PC - PE[i], axis=1).astype(np.float32)
        cands = [prev]
        for a in range(1, adv + 1):
            sh = np.full(n_l, BIG, dtype=np.float32)
            sh[a:] = prev[:-a]
            cands.append(sh)
        st = np.stack(cands)
        ch = np.argmin(st, axis=0).astype(np.uint8)
        prev = np.take_along_axis(st, ch[None, :].astype(np.intp), axis=0)[0] + d
        choice[i] = ch
    k = np.empty(n_f, dtype=int)
    k[-1] = int(np.argmin(prev))
    for i in range(n_f - 1, 0, -1):
        k[i - 1] = k[i] - int(choice[i][k[i]])
    return np.clip(k, 0, n_l - 1)


def _windowed_dist_mm(P, C, k_idx, half=120) -> np.ndarray:
    """Distance [mm] from each executed point to the commanded segments within
    +/- `half` lines of its aligned index -- so a neighbouring pass of the same
    path cannot answer for the pass being measured."""
    P = np.asarray(P, float); C = np.asarray(C, float)
    nseg = C.shape[0] - 1
    A = C[:-1]; D = np.diff(C, axis=0)
    L2 = np.einsum("ij,ij->i", D, D); L2 = np.where(L2 > 0, L2, 1e-18)
    offs = np.arange(-half, half + 1)
    out = np.empty(P.shape[0])
    chunk = max(1, int(1_500_000 / (2 * half + 1)))
    for i in range(0, P.shape[0], chunk):
        sidx = np.clip(k_idx[i:i + chunk, None] + offs[None, :], 0, nseg - 1)
        w = P[i:i + chunk, None, :] - A[sidx]
        t = np.clip(np.einsum("mkj,mkj->mk", w, D[sidx]) / L2[sidx], 0.0, 1.0)
        out[i:i + chunk] = np.linalg.norm(w - t[..., None] * D[sidx], axis=2).min(axis=1)
    return out * 1000.0


def _spearman(a, b) -> float:
    a = np.asarray(a, float); b = np.asarray(b, float)
    m = np.isfinite(a) & np.isfinite(b)
    if m.sum() < 4:
        return float("nan")
    ra = np.argsort(np.argsort(a[m])).astype(float)
    rb = np.argsort(np.argsort(b[m])).astype(float)
    ra -= ra.mean(); rb -= rb.mean()
    den = np.sqrt((ra * ra).sum() * (rb * rb).sum())
    return float((ra * rb).sum() / den) if den > 0 else float("nan")


#: how far the measured profile may RELAX the model's ceiling where the arm demonstrably
#: stayed well inside the budget. The model over-reads the bulk (predicted p50 6.4 mm on
#: slot 43 against a measured 1.4), so this is where the speed comes back; bounded because
#: two runs are two runs.
MEASURED_RELAX_MAX = 2.5
#: a measured profile is only used to place the slowing when the runs AGREE about
#: where the arm leaves the path. At 2.02x two slot-43 runs agree at Spearman 0.64;
#: at 6.29x two slot-44 runs read 0.04 to -0.17 and the hot spots move between runs
#: (one bin 59.5 then 10.4 mm, another 8.6 then 52.0), so there is nothing to aim at.
MEASURED_PROFILE_MIN_RHO = 0.35
#: bins used to judge that agreement (lines)
MEASURED_PROFILE_BIN = 200


def measured_departure_profile(track_path, side: str, frame: str,
                               repo: pathlib.Path = None, min_runs: int = 2,
                               source_path=None) -> tuple:
    """WHERE THE ARM ACTUALLY LEFT THE PATH, per commanded line, from every
    COMPLETE run of these exact bytes.

    The model predicts the departure's magnitude across path families (1.99x on
    148 planned rasters where the pad-space law read 9.08x) but it does NOT
    predict WHERE: against the two complete slot-43 runs it scores Spearman
    -0.22, slightly anti-correlated. Measurement does: those same two runs agree
    with each other at +0.64. So a file that has RUN can be re-timed against what
    it did, which is the only targeting signal that has survived a test.

    Returns (profile, notes). `profile` is the worst measured departure [mm] seen
    at each commanded line, over the runs, or None when there are fewer than
    `min_runs` complete runs or they do not agree (`MEASURED_PROFILE_MIN_RHO`)."""
    repo = repo or REPO
    track_path = pathlib.Path(track_path)
    q = read_q_deg(track_path)
    N = q.shape[0]
    sha = sha256_of(track_path)
    frame = frame or f"{side_char(side)}_glove_frame_2"
    PC = pad_path_m(q, side, frame)
    profs, used = [], []
    for rp in sorted(repo.glob("Resource/runs/tracks/*/run_*.json")):
        try:
            rec = json.loads(rp.read_text())
        except Exception:                                              # noqa: BLE001
            continue
        if not isinstance(rec, dict) or rec.get("track_sha256") != sha:
            continue
        if not rec.get("complete") or rec.get("side") != side:
            continue
        fp = rec.get("frames_file")
        if not fp or not (repo / fp).is_file():
            continue
        T, Q, S = _capture_joints(repo / fp)
        sel = np.flatnonzero(S == "move_through_joint")
        if sel.size < 50:
            continue
        Qw = Q[sel[0]:sel[-1] + 1]
        PE = pad_path_m_uncached(Qw, side, frame)
        k = _align_monotone(PE, PC)
        d = _windowed_dist_mm(PE, PC, np.clip(k, 0, N - 2))
        prof = np.zeros(N)
        np.maximum.at(prof, k, d)
        profs.append(prof)
        used.append(dict(run=rp.name, frames=int(Qw.shape[0]),
                         p95_mm=round(float(np.percentile(d, 95)), 2),
                         max_mm=round(float(d.max()), 2)))
    notes = dict(runs=used, n_runs=len(profs), min_runs=int(min_runs))
    if len(profs) < min_runs:
        notes["usable"] = False
        notes["why"] = f"{len(profs)} complete run(s) of these bytes, {min_runs} needed"
        return None, notes
    W = MEASURED_PROFILE_BIN
    nb = int(np.ceil(N / W))
    binned = [np.array([p[i * W:min((i + 1) * W, N)].max() for i in range(nb)]) for p in profs]
    rhos = [_spearman(binned[i], binned[j])
            for i in range(len(binned)) for j in range(i + 1, len(binned))]
    rho = float(np.nanmedian(rhos)) if rhos else float("nan")
    notes["run_to_run_rho"] = round(rho, 3)
    notes["usable"] = bool(np.isfinite(rho) and rho >= MEASURED_PROFILE_MIN_RHO)
    if not notes["usable"]:
        notes["why"] = (f"the runs do not agree about WHERE (Spearman {rho:.3f} < "
                        f"{MEASURED_PROFILE_MIN_RHO}); there is nothing to aim at")
        return None, notes
    prof = np.maximum.reduce(profs)
    if source_path is not None:
        # the runs played the EMITTED file; the emitter works in SOURCE lines. Every
        # emitted point lies on a source chord, so the nearest source configuration
        # is its line, and a source line carries the worst departure of the emitted
        # lines that map to it.
        from scipy.spatial import cKDTree
        q_src = read_q_deg(pathlib.Path(source_path))
        _d, j = cKDTree(q_src).query(q)
        src_prof = np.zeros(q_src.shape[0])
        np.maximum.at(src_prof, j, prof)
        notes["mapped_to_source"] = dict(source_lines=int(q_src.shape[0]),
                                         emitted_lines=int(N),
                                         match_deg_p95=round(float(np.percentile(_d, 95)), 4))
        prof = src_prof
        N = q_src.shape[0]
    notes["profile_p95_mm"] = round(float(np.percentile(prof, 95)), 2)
    notes["profile_max_mm"] = round(float(prof.max()), 2)
    return prof, notes


def _measured_sibling_track(row: dict, repo: pathlib.Path = None):
    """An installed file emitted from THIS row's source that has complete runs on
    record -- the row's own first, else any sibling row of the same recording."""
    repo = repo or REPO
    src_sha = ((row.get("source") or {}).get("sha256"))
    cands = []
    own = row.get("track")
    if own and (repo / own).is_file():
        cands.append(repo / own)
    if src_sha:
        try:
            reg = json.loads((repo / "Resource/tracks/taught/TAUGHT_TRACKS.json").read_text())
        except Exception:                                              # noqa: BLE001
            reg = {}
        for r in (reg.get("tasks") or []):
            if not isinstance(r, dict):
                continue
            if ((r.get("source") or {}).get("sha256")) != src_sha:
                continue
            t = r.get("track")
            if t and (repo / t).is_file() and (repo / t) not in cands:
                cands.append(repo / t)
    for c in cands:
        sha = sha256_of(c)
        for rp in sorted(repo.glob("Resource/runs/tracks/*/run_*.json")):
            try:
                rec = json.loads(rp.read_text())
            except Exception:                                          # noqa: BLE001
                continue
            if isinstance(rec, dict) and rec.get("track_sha256") == sha and rec.get("complete"):
                return c
    return None


def _measured_clock_s(row: dict, repo: pathlib.Path = None) -> float:
    """The clock the row's installed file actually ran at, from its complete runs."""
    repo = repo or REPO
    inst = row.get("track")
    if not inst or not (repo / inst).is_file():
        return None
    sha = sha256_of(repo / inst)
    n = row.get("points") or read_q_deg(repo / inst).shape[0]
    vals = []
    for rp in sorted(repo.glob("Resource/runs/tracks/*/run_*.json")):
        try:
            rec = json.loads(rp.read_text())
        except Exception:                                              # noqa: BLE001
            continue
        if not isinstance(rec, dict) or rec.get("track_sha256") != sha or not rec.get("complete"):
            continue
        ms = rec.get("clock_ms_per_pt") or rec.get("implied_ms_per_pt")
        if ms:
            vals.append(float(ms) / 1000.0)
    return float(np.median(vals)) if vals else None


def cal_on_lines(cal_src, q_src_deg: np.ndarray, q_lines_deg: np.ndarray) -> np.ndarray:
    """Carry a SOURCE-line calibration onto any point list's own lines, by nearest
    commanded configuration.

    There must be exactly ONE of these. The emitter first mapped through its own `u`
    array while the gate mapped by nearest configuration, so the two read 19.56 and
    25.69 mm for the same file: the emitter converged on a number the gate never
    agreed with, every fast rung was refused, and the ladder walked down to 0.8x the
    taught speed. Same function everywhere, or the same bug comes back."""
    if cal_src is None:
        return None
    from scipy.spatial import cKDTree
    cal_src = np.asarray(cal_src, float)
    _d, j = cKDTree(np.asarray(q_src_deg, float)).query(np.asarray(q_lines_deg, float))
    return cal_src[np.clip(j, 0, cal_src.size - 1)]


def cal_pack(cal: np.ndarray, stride: int = 1) -> dict:
    """The calibration, stored IN the certificate so a calibrated certificate still
    re-checks FROM THE FILES ALONE — the property `verify` exists for.

    STRIDE 1, i.e. every value, deliberately. Sub-sampling it (stride 8, on the
    reasoning that a 21-line-smoothed curve interpolates harmlessly) made `verify`
    recompute 24.19 mm against a certificate claiming 19.42: G5 gates on the MAX, and
    a peak is precisely what an interpolation removes. The array costs ~65 KB of JSON
    beside a 527 KB .proj, which is no reason to make the number irreproducible."""
    cal = np.asarray(cal, float)
    idx = np.arange(0, cal.size, stride)
    if idx.size and idx[-1] != cal.size - 1:
        idx = np.append(idx, cal.size - 1)
    return dict(n=int(cal.size), stride=int(stride),
                values=[round(float(x), 3) for x in cal[idx]])


def cal_unpack(packed: dict) -> np.ndarray:
    """`cal_pack` inverted, for `verify`."""
    if not packed:
        return None
    n = int(packed["n"]); st = int(packed.get("stride", 8))
    v = np.asarray(packed["values"], float)
    idx = np.arange(0, n, st)
    if idx.size and idx[-1] != n - 1:
        idx = np.append(idx, n - 1)
    return np.interp(np.arange(n), idx[:v.size], v)


def measured_calibration(emitted_track, source_q_deg: np.ndarray, side: str, frame: str,
                         model: dict, clock_ran_s: float, repo: pathlib.Path = None,
                         min_runs: int = 2) -> tuple:
    """THE PER-LINE CORRECTION the measurement earns: measured / predicted, on the
    file that ran, carried onto the source's lines.

    The model gets the departure's MAGNITUDE right across path families and its
    LOCATION wrong (Spearman -0.22 against the two complete slot-43 runs), while
    those runs agree with each other at +0.64. This takes WHERE from the
    measurement and HOW-IT-SCALES-WITH-SPEED from the model, which is the only
    division of labour that survived a test: the shipped slot-43 file's calibrated
    worst estimate is 41.5 mm against a measured 40.2-43.2, where the raw
    prediction said 19.8.

    Returns (cal, notes), or (None, notes) when the runs do not agree."""
    repo = repo or REPO
    prof, notes = measured_departure_profile(emitted_track, side, frame, repo=repo,
                                             min_runs=min_runs)
    if prof is None:
        return None, notes
    q_e = read_q_deg(pathlib.Path(emitted_track))
    tau_j = np.asarray(model.get("tau_joint_s") or [model["tau_path_s"]] * 7, float)
    cal_f = float(model.get("departure_calibration") or 1.0)
    pred = cal_f * pad_departure_joint_mm(q_e, side, frame, tau_j, clock_ran_s, model=model)
    r = np.clip(prof / np.maximum(pred, 0.5), 0.25, 4.0)
    from scipy.spatial import cKDTree
    q_s = np.asarray(source_q_deg, float)
    _d, j = cKDTree(q_s).query(q_e)
    acc = np.zeros(q_s.shape[0])
    np.maximum.at(acc, j, r)
    seen = np.flatnonzero(acc > 0)
    if seen.size < 8:
        notes["usable"] = False
        notes["why"] = "the emitted file's lines do not map onto the source"
        return None, notes
    cal = np.interp(np.arange(q_s.shape[0]), seen, acc[seen])
    k = 21                                      # a field, not a comb
    cal = np.convolve(np.pad(cal, (k // 2, k // 2), mode="edge"), np.ones(k) / k, mode="valid")
    notes.update(calibration=dict(p05=round(float(np.percentile(cal, 5)), 3),
                                  p50=round(float(np.percentile(cal, 50)), 3),
                                  p95=round(float(np.percentile(cal, 95)), 3),
                                  max=round(float(cal.max()), 3),
                                  source_lines=int(cal.size),
                                  mapped_lines=int(seen.size)))
    return cal, notes


def departure_lambda_cap(q_src_deg: np.ndarray, side: str, frame: str, model: dict,
                         budget_mm: float, clock_s: float = CLOCK_GATE_S) -> tuple:
    """The per-source-line ceiling on lambda that keeps the predicted departure
    (`pad_departure_mm`) inside `budget_mm`.

    The allowed pad speed at each line is geometry alone (`pad_speed_budget_m_s`),
    and one source line of the recording is |dP| of pad travel, so at lambda the
    pad covers lambda x |dP| per player tick: lambda <= v_allowed x clock / |dP|.
    Nothing here looks at a reversal — a cusp is simply where the geometry allows
    the least carry-on."""
    # memoised: it is geometry and a budget, so it is the same for every rung of
    # the ladder (16 of them) and costs ~6 s on a 19k-line recording
    key = (hashlib.sha256(np.ascontiguousarray(np.asarray(q_src_deg, float)).tobytes()).hexdigest(),
           side_char(side), frame, float(model["tau_path_s"]), float(budget_mm), float(clock_s),
           tuple(model.get("tau_joint_s") or ()), float(model.get("departure_calibration") or 1.0))
    if key in _DEPARTURE_CACHE:
        return _DEPARTURE_CACHE[key]
    P = pad_path_m(q_src_deg, side, frame)
    cal = float(model.get("departure_calibration") or 1.0)
    tau_j = np.asarray(model.get("tau_joint_s") or [model["tau_path_s"]] * 7, float)
    # THE JOINT-SPACE LAW INVERTS IN CLOSED FORM, which the pad-space one did not.
    # At lambda the joint step per tick is lambda x the source's, so the carry-on
    # offset -- and hence the departure -- scales linearly in lambda: no monotone
    # scan and no bisection, just the departure measured once at lambda = 1.
    # (`pad_speed_budget_m_s` stays for the pad-space law and its tests.)
    dep1 = cal * pad_departure_joint_mm(q_src_deg, side, frame, tau_j, clock_s, P=P, model=model)
    with np.errstate(divide="ignore", invalid="ignore"):
        lam = np.where(dep1 > 1e-9, float(budget_mm) / dep1, np.inf)
    binding = int(np.isfinite(lam).sum() - (lam >= 1e6).sum())
    out = (lam, dict(lines=int(len(lam)), binding=binding,
                     slowest_lambda=(round(float(np.nanmin(lam)), 3) if binding else None),
                     budget_mm=float(budget_mm)))
    if len(_DEPARTURE_CACHE) > 4:
        _DEPARTURE_CACHE.clear()
    _DEPARTURE_CACHE[key] = out
    return out


def model_millideg(q_src_deg: np.ndarray, lam_fast: float, side: str, frame: str, model: dict,
                   budget_mm: float = MAX_DEVIATION_MM, clock_s: float = CLOCK_GATE_S,
                   sections: list = None, dwells: list = None,
                   measured_prof: np.ndarray = None, cal_src: np.ndarray = None, *,
                   pred_clock_s: float = CLOCK_PREDICT_S) -> tuple:
    """The model rung: `capped_millideg` with `model_caps` (the rating everywhere,
    the model's arrival rate at every extreme), the section factors, then the
    dwells. Points lie on the source chords; only the clock along them changes.
    Returns (millidegrees, notes)."""
    q = np.asarray(q_src_deg, dtype=float)
    n = q.shape[0]
    caps, notes = model_caps(q, side, frame, model, lam_fast, budget_mm, clock_s)
    # THE DEPARTURE BUDGET, EVERYWHERE (2026-09-16): a ceiling on lambda per source
    # line from the geometry, so the file is slow wherever carrying on for tau would
    # leave the path by more than the budget — at a cusp, around a curve, anywhere.
    lam_cap, dep_notes = departure_lambda_cap(q, side, frame, model, budget_mm, pred_clock_s)
    notes["departure"] = dep_notes
    lam_cap = np.array(lam_cap, dtype=float, copy=True)          # the cache's array is shared
    if sections:
        for a, b, f in sections:
            lo, hi = max(0, _as_line(a, n)), min(n, _as_line(b, n) + 1)
            lam_cap[lo:hi] = np.minimum(lam_cap[lo:hi], float(lam_fast) * float(f))
    # THE MEASURED PROFILE PLACES THE SLOWING, where it has earned the right to.
    # The model predicts the departure's MAGNITUDE across path families but not its
    # LOCATION (Spearman -0.22 against the two complete slot-43 runs, slightly
    # anti-correlated), while those two runs agree with each other at +0.64. So where
    # this file has run and the runs agree, the ceiling is pulled down at the lines
    # that really departed and let back up at the lines that really did not -- which
    # is where the speed comes from, the model reading p50 6.4 mm against a measured
    # 1.4. `measured_departure_profile` returns None unless the runs agree.
    if cal_src is not None and len(cal_src) == n:
        # PLACEMENT from the measurement: pull the ceiling down where the arm really
        # departed more than predicted, let it up where it really departed less. The
        # convergence below then judges the CALIBRATED estimate, so this is a
        # placement hint and never the bar itself.
        sc = 1.0 / np.clip(np.asarray(cal_src, float), 1.0 / MEASURED_RELAX_MAX,
                           MEASURED_RELAX_MAX)
        w = max(2, int(np.ceil(float(lam_fast))) + 1)
        sc = np.minimum.reduce([np.roll(sc, d) for d in range(-w, w + 1)])
        lam_cap = np.maximum(lam_cap * sc, 1e-3)
        notes["measured"] = dict(lines=int(n), tightened=int((sc < 1).sum()),
                                 relaxed=int((sc > 1).sum()),
                                 min_scale=round(float(sc.min()), 3),
                                 max_scale=round(float(sc.max()), 3))
    if measured_prof is not None and len(measured_prof) == n:
        aim = DEPARTURE_DESIGN_FRACTION * float(budget_mm)
        mp = np.asarray(measured_prof, float)
        floor = aim / MEASURED_RELAX_MAX
        sc = aim / np.clip(mp, floor, None)
        sc = np.clip(sc, 1.0 / MEASURED_RELAX_MAX, MEASURED_RELAX_MAX)
        # a correction belongs to the stretch, not to one line
        w = max(2, int(np.ceil(float(lam_fast))) + 1)
        sc = np.minimum.reduce([np.roll(sc, d) for d in range(-w, w + 1)])
        lam_cap = np.maximum(lam_cap * sc, 1e-3)
        notes["measured"] = dict(lines=int(n), tightened=int((sc < 1).sum()),
                                 relaxed=int((sc > 1).sum()),
                                 min_scale=round(float(sc.min()), 3),
                                 max_scale=round(float(sc.max()), 3))
    # the corner rounding spans ~0.1 s of emitted time at the fastest stretches: at the
    # rating a source line lasts clock/lambda, so that many source lines
    smooth = int(np.ceil(0.10 * float(lam_fast) / float(clock_s)))
    # THE CAP IS HONOURED ON THE SOURCE'S GEOMETRY, THE GATE READS THE EMITTED FILE'S.
    # They differ: an emitted line spans lambda source lines, so its own tangent is a
    # chord of theirs and its carry-on can reach a little further (measured: 10 % over a
    # 5 mm budget). Rather than trust a design margin, the emitter measures what it just
    # wrote and pulls the ceiling down where it went over, until it is inside.
    # THE EMITTER MEASURES THE LAW THE GATE READS. It measured the pad-space law while
    # G5 moved to the joint-space one, so it converged on a number nobody was checking
    # and left the file 10 % over a 5 mm budget (386 lines over it).
    cal = float(model.get("departure_calibration") or 1.0)
    tau_j = np.asarray(model.get("tau_joint_s") or [model["tau_path_s"]] * 7, float)
    # KEEP THE BEST PASS, not the last one. Pulling the ceiling down at one place
    # changes the emitted geometry elsewhere, so the sequence of worst departures is
    # not monotone: on the bowl-outside source it read 34.8, 22.0, 22.4, 21.0, 20.6,
    # 22.0, 20.3, 20.7, 20.1, 19.7, 20.0, 20.8 mm against a 20 mm budget — pass ten
    # was INSIDE and the loop returned pass twelve, which was not. The best pass is
    # the file, and the loop stops as soon as a pass lands inside.
    attempts = []
    best = None
    for _ in range(12):
        md, u = capped_millideg(q, lam_fast, caps, ramp=MODEL_RAMP, clock_s=clock_s,
                                lam_cap_line=lam_cap, return_u=True, smooth_lines=smooth)
        dep = cal * pad_departure_joint_mm(md / 1000.0, side, frame, tau_j, pred_clock_s, model=model)
        if cal_src is not None and len(cal_src) == n:
            # judge what the ARM will do, through the SAME mapping the gate uses --
            # `cal_on_lines`, not this emitter's own `u`. Mapping two different ways is
            # what made the emitter converge on 19.56 while the gate read 25.69.
            dep = dep * cal_on_lines(cal_src, q, md / 1000.0)
        worst = float(dep.max())
        attempts.append(round(worst, 3))
        if best is None or worst < best[0]:
            best = (worst, md, u)
        if worst <= DEPARTURE_DESIGN_FRACTION * float(budget_mm) or md.shape[0] < 4:
            break
        # DON'T PAY FOR A HOPELESS RUNG. The ladder prices every rung and refuses the
        # ones that cannot get inside the budget; a rung still half again over the aim
        # after four passes is one of those, and the remaining eight passes are spent
        # to reach the same refusal.
        if len(attempts) >= 4 and best[0] > 1.5 * DEPARTURE_DESIGN_FRACTION * float(budget_mm):
            break
        # aim the correction at the SAME number the exit test uses. Correcting to the
        # budget exactly left the file 0.6 % over it (11 lines), because the departure is
        # only approximately linear in the ceiling and the last step lands on the far side.
        aim = DEPARTURE_DESIGN_FRACTION * float(budget_mm)
        over = dep > aim
        if not over.any():
            break
        src = np.clip(u.astype(int), 0, n - 1)
        scale = np.ones(n)
        # OVER-RELAX. A plain budget/departure correction converges geometrically and
        # slowly once it is close (measured on the bowl-outside source: 34.8, 23.4,
        # 21.8, 21.3, 20.9 mm against a 20 mm budget — still outside after five passes,
        # so the emitter handed its own gate a file the gate refused). The ceiling is
        # pulled DOWN by half again as much as the ratio asks, floored so one pass can
        # never more than halve it, and the loop still exits only on a measurement.
        ratio = np.minimum(aim / dep[over], 1.0)
        np.minimum.at(scale, src[over], np.maximum(1.0 - DEPARTURE_RELAX * (1.0 - ratio), 0.5))
        # a correction belongs to the stretch, not to one line: carry it to the
        # neighbours the same emitted step covers
        w = max(2, int(np.ceil(float(lam_fast))) + 1)
        scale = np.minimum.reduce([np.roll(scale, d) for d in range(-w, w + 1)])
        lam_cap = np.maximum(lam_cap * scale, 1e-3)
    if best is not None:
        worst_best, md, u = best
        notes["departure"]["emitted_worst_mm_best"] = round(worst_best, 3)
    notes["departure"]["emitted_worst_mm"] = attempts
    md = insert_dwells(md, u, dwells or [], n)
    notes["sections"] = [list(s) for s in (sections or [])]
    notes["dwells"] = [list(d) for d in (dwells or [])]
    return md, notes


def resolve_solver_frame(side: str, frame: str) -> tuple:
    """(the frame to hand `plan_min_time`, a note).

    plan_min_time resolves `--ik-frame` as a LINK of
    core/python/alix_workspace/src/alix_workspace/urdf/alix.urdf (its tool-speed
    report walks a chain), and `Arm_Tip` is NOT one — in teachlab and on the
    controller it is `ConnectorLink + (0, 0, -0.0153)`, an offset, not a link.
    Handed a non-link it raised and every rung came back without a track
    (measured 2026-09-14, the first `shape` run: five rungs, no tracks).

    So the SOLVER gets a real link of the same side, and the number that feeds
    is its per-stage tool-speed REPORT — advisory on this channel. The DEVIATION
    gate is measured at the declared frame either way (teachlab resolves Arm_Tip
    and the glove frames properly, offsets included)."""
    cand = str(frame).strip()
    if not cand.startswith(("L_", "R_")):
        cand = f"{side_char(side)}_{cand}"
    urdf = REPO / "core/python/alix_workspace/src/alix_workspace/urdf/alix.urdf"
    try:
        if f'link name="{cand}"' in urdf.read_text():
            return cand, ""
    except OSError:
        pass
    fb = f"{side_char(side)}_glove_frame_2"
    return fb, (f"\n  (the solver's --ik-frame is {fb}: {cand} is not a link of its URDF. "
                f"Its tool-speed report is the only thing it feeds — advisory here — and the "
                f"deviation gate is measured at the declared frame {frame}.)")


def _path_token(token) -> pathlib.Path:
    """The file a token names, when it names one: a `.proj`, a `.txt` or a
    certificate `.json`, given relative to the cwd or to the repo."""
    s = str(token)
    if not s.endswith((".proj", ".txt", ".json")):
        return None
    for base in (pathlib.Path(s).expanduser(), REPO / s):
        if base.is_file():
            return base.resolve()
    raise SystemExit(f"no such file: {s}")


def find_row(reg: dict, token, side: str = None) -> dict:
    """ONE taught row by slot, full name, unambiguous substring — or by the
    admitted `.proj` / `.txt` / certificate file itself, which names its own
    arm, so `side` may then be omitted. A slot or a name needs the side: slots
    repeat per arm."""
    path = _path_token(token)
    if path is not None:
        stem = path.with_suffix("")
        hits = [r for r in reg.get("tasks", [])
                if (REPO / str(r.get("track") or "")).with_suffix("").resolve() == stem]
        if len(hits) != 1:
            raise SystemExit(f"{path.name} is not an admitted taught track (the file an "
                             f"`upload` sends lives beside its registry row; `certify` makes one)")
        if side is not None and hits[0].get("side") != side:
            raise SystemExit(f"{path.name} belongs to the {hits[0].get('side')} arm, but "
                             f"--side {side} was given — refusing")
        return hits[0]
    if side is None:
        raise SystemExit(f"'{token}': a slot or a name needs --side (slots repeat per arm); "
                         f"or pass the .proj file itself")
    rows = ct.rows_for(reg, side)
    if not rows:
        raise SystemExit(f"no taught rows for the {side} arm in "
                         f"{ct.repo_rel(ct.TAUGHT_REGISTRY, REPO)} — `taught_tracks.py certify` "
                         f"a recording first")
    if str(token).isdigit():
        hits = [r for r in rows if r["slot"] == int(token)]
    else:
        hits = [r for r in rows if r["name"] == token or str(token) in r["name"]]
    if len(hits) != 1:
        raise SystemExit(f"'{token}' matches {len(hits)} {side} taught row(s) — use the slot "
                         f"number or the full name (see: taught_tracks.py status)")
    return hits[0]


def rung_label(c: dict) -> str:
    """How a rung is named on the command line (`--pick`) and in records.

    "verbatim" means THE RECORDING'S OWN POINTS. With `--smooth-hz` rung 0 is the smoothed recording,
    which is not verbatim — calling it that put "chosen: verbatim" in the records of slots 50 and 51,
    whose bytes are a filtered path (2026-09-17 review, C2). It is `smooth<F>hz` there, and the label is
    what `--pick` takes, so a smoothed rung cannot be picked by the unsmoothed name either.
    """
    if c["mode"] == "verbatim":
        return f"smooth{c['smooth_hz']:g}hz" if c.get("smooth_hz") else "verbatim"
    if c["mode"] == "uniform":
        return f"uniform:{c['time_scale']:g}"
    if c["mode"] == "guarded":
        return f"guard:{c['time_scale']:g}"
    if c["mode"] == "model":
        return f"model:{c['time_scale']:g}"
    return f"vel:{c['vel_scale']:g}"


def rung_title(c: dict) -> str:
    """How a rung reads in a table."""
    if c["mode"] == "verbatim":
        return (f"smoothed {c['smooth_hz']:g} Hz" if c.get("smooth_hz") else "verbatim")
    if c["mode"] == "uniform":
        return f"uniform {c['time_scale']:.1f}x"
    if c["mode"] == "guarded":
        return f"guard {c['time_scale']:.1f}x"
    if c["mode"] == "model":
        return f"model {c['time_scale']:.1f}x"
    return f"vel {c['vel_scale']:.2f}"


def _row_admissible(row: dict) -> tuple:
    g = row.get("gates") or {}
    if g.get("admissible"):
        return True, ""
    why = "; ".join(g.get("reasons") or ["never screened"])
    return False, (f"this row is NOT admissible yet: {why}. Run `taught_tracks.py certify "
                   f"<recording.txt>` — it finds the fastest rung that passes, and `--force` is "
                   f"the recorded override when you mean to send it anyway.")


# ─────────────────────────────────────────────────────────────── certificate
def certificate_for(row: dict, best: dict, *, verb: str, clock_s: float, allowance_deg: float,
                    max_dev_mm: float, prefer_dev_mm: float, n_ref: int, frame: str,
                    model: dict = None, timing: dict = None,
                    cal_lines: np.ndarray = None) -> dict:
    """What `shape`/`certify` promise about the installed bytes, bound to them by sha256.

    Every gate is listed with the number it measured and the bar it cleared, so a
    certificate can be re-checked from the artifact alone (`verify`). The verdict
    is CERTIFIED only when EVERY entry passed — a rung shaped with `--force` from
    a recording that is not `hardware: ok` is installed but NOT certified, and
    `upload`, `run` and `flow` refuse it."""
    enf = best["gates"]["enforced"]
    adv = best["gates"]["advisory"]
    dev = best["dev"] or {}
    # THE MEASURED CORRECTION TRAVELS WITH THE CERTIFICATE. G5 was judged with it, so
    # `verify` must apply the same number -- and must not need the run CAPTURES to do
    # it, because a certificate is re-checkable from the artifact alone. Storing the
    # field (every 8th value of an already-21-line-smoothed curve) keeps that true for
    # a calibrated certificate too.
    cal_packed = cal_pack(cal_lines) if cal_lines is not None else None
    src = row.get("source") or {}
    slot = row.get("slot")
    lo, hi = ct.TAUGHT_SLOT_RANGE
    gates = [
        dict(id="format", name="7 integer millidegrees per line, read by the upload parser",
             value=f"{row['points']} lines", passed=row.get("points_on_disk") == row.get("points")),
        dict(id="provenance", name="taught on this robot's arm",
             value=f"hardware: {src.get('hardware')}", passed=src.get("hardware") == "ok"),
        dict(id="slot", name="taught slot range", value=slot,
             limit=f"{lo}-{hi}, or >= {ct.TAUGHT_CANFD_SLOT_MIN} CANFD-only (nothing uploaded); "
                   f"never {ct.SLOT_BASE}-{ct.SLOT_MAX}",
             passed=bool(ct.taught_slot_ok(slot))),
        (dict(id="G1", name="joint-position margin, predicted executed", value=round(enf["position_margin_min_deg"], 3),
              unit="deg",
              limit=(f"predicted executed margin >= {enf['limit_eps_deg']:g} at every limit "
                     f"(overshoot = max({enf['floor_deg']:g}, {enf['tau_limit_s']:g} s x arrival rate))"),
              joint=enf["position_binding_joint"], rule=enf.get("position_rule", "margin"),
              pred_exec_margin_min_deg=enf.get("pred_exec_margin_min_deg"),
              pred_exec_margin_worst=enf.get("pred_exec_margin_worst"),
              passed=bool(enf.get("position_ok")))
         if "pred_exec_margin_min_deg" in enf else
         dict(id="G1", name="joint-position margin", value=round(enf["position_margin_min_deg"], 3),
              unit="deg",
              limit=(f">= {allowance_deg:g}, or >= {enf.get('hard_margin_floor_deg', HARD_MARGIN_FLOOR_DEG):g} "
                     f"with the joint at <= {enf.get('near_limit_rate_cap_deg_s', NEAR_LIMIT_RATE_DEG_S):g} "
                     f"deg/s inside the allowance"),
              joint=enf["position_binding_joint"], rule=enf.get("position_rule", "margin"),
              near_limit_rate_deg_s=enf.get("near_limit_rate_deg_s"),
              passed=bool(enf.get("position_ok", enf["position_margin_min_deg"] >= allowance_deg)))),
        dict(id="G2", name="ticks over a joint rating", value=int(enf["ticks_over_rating_band_limited"]),
             unit="ticks", limit=f"0 at {clock_s * 1e3:.2f} ms, 20 Hz band-limited",
             peak_util=round(enf["peak_rate_util_band_limited"], 4), joint=enf["peak_joint"],
             passed=int(enf["ticks_over_rating_band_limited"]) == 0),
        dict(id="G3", name="largest step between lines", value=round(enf["max_step_deg"], 3),
             unit="deg", limit=f"<= {STEP_FAULT_DEG:g}", passed=enf["max_step_deg"] <= STEP_FAULT_DEG),
        dict(id="G4", name="pad deviation from the taught path", value=dev.get("max_mm"), unit="mm",
             limit=f"<= {max_dev_mm:g} ({prefer_dev_mm:g} preferred)", frame=dev.get("frame", frame),
             passed=dev.get("max_mm") is not None and dev["max_mm"] <= max_dev_mm),
    ]
    if "pred_dev_max_mm" in enf:
        gates.append(dict(
            id="G5", name="predicted executed deviation at the reversals (overrun past a stroke end)",
            value=enf["pred_dev_max_mm"],
            unit="mm", limit=f"<= {enf['budget_mm']:g} ({prefer_dev_mm:g} preferred) at the "
                             f"{'median' if (model or {}).get('percentile') == 'p50' else '90th percentile'} "
                             f"(overshoot = max({enf['floor_deg']:g}, {enf['tau_path_s']:g} s x arrival rate) "
                             f"through the lever arms at {frame})",
            percentile=(model or {}).get("percentile", "p90"),
            reversals=enf.get("reversals"), worst=enf.get("pred_dev_worst"),
            p95_mm=enf.get("pred_dev_p95_mm"), over_prefer=enf.get("pred_dev_over_prefer"),
            passed=bool(enf.get("g5_ok"))))
    taught_s = (n_ref - 1) * CLOCK_NOMINAL_S
    return dict(
        version=CERT_VERSION if model is not None else 1,
        verdict="CERTIFIED" if all(g["passed"] for g in gates) else "NOT CERTIFIED",
        issued=datetime.datetime.now().isoformat(timespec="seconds"),
        issued_by=f"tools/taught_tracks.py {verb}",
        side=row["side"], slot=slot, name=row["name"], frame=frame,
        clock_gate_s=float(clock_s), clock_predict_s=float(enf.get("clock_predict_s", clock_s)),
        position_allowance_deg=float(allowance_deg),
        near_limit_rate_cap_deg_s=float(enf.get("near_limit_rate_cap_deg_s", NEAR_LIMIT_RATE_DEG_S)),
        hard_margin_floor_deg=float(enf.get("hard_margin_floor_deg", HARD_MARGIN_FLOOR_DEG)),
        model=(_model_params(model) if model is not None else None),
        measured_calibration=cal_packed,
        timing=timing,
        max_deviation_mm=float(max_dev_mm), prefer_deviation_mm=float(prefer_dev_mm),
        proj=dict(path=row["proj"], sha256=row["proj_sha256"], bytes=row["proj_bytes"],
                  lines=row["points_on_disk"]),
        track=dict(path=row["track"], sha256=row["track_sha256"], bytes=row["track_bytes"]),
        source=dict(path=src.get("path"), sha256=src.get("sha256"), lines=int(n_ref)),
        rung=dict(label=rung_label(best), mode=best["mode"], time_scale=best.get("time_scale"),
                  vel_scale=best.get("vel_scale"), speed_vs_taught=round(taught_s / best["duration_s"], 4),
                  duration_nominal_s=round(best["duration_s"], 3), band_s=row["band_5p5_5p6_s"],
                  taught_nominal_s=round(taught_s, 3)),
        gates=gates,
        advisory=dict(accel_util_band_limited=adv.get("accel_util_band_limited"),
                      jerk_util_band_limited=adv.get("jerk_util_band_limited"),
                      ticks_over_rating_raw=enf.get("ticks_over_rating_raw"),
                      peak_rate_util_raw=enf.get("peak_rate_util_raw"),
                      slowness=adv.get("slowness")),
        not_covered=("The gates certify these BYTES against the taught recording. What the arm does "
                     "with them — the executed path — is measured only by a run's capture."))


def verify_certificate(row: dict, deep: bool = True) -> dict:
    """Re-check a row's certificate FROM THE ARTIFACTS, trusting nothing it says.

    * the .proj and .txt still hash to the certified values, and the registry
      pins agree;
    * the .proj is byte for byte the project form of the .txt (header, CRLF,
      line count);
    * G1-G3 — and G5 for a certificate that carries its model — recomputed on the
      stored lines, at the certified clock, pass and reproduce the certified numbers;
      G2 is also re-judged at today's 5.0 ms player ceiling when the certificate was
      issued at a slower clock, and the slot is re-checked against the taught /
      CANFD-only ranges;
    * `deep`: the source recording still hashes to its pin and G4 recomputes
      inside the cap (a few seconds of exact geometry).
    Returns {ok, reasons, cert, checked}."""
    reasons, checked = [], []
    track = REPO / str(row.get("track") or "")
    sidecar = track.with_suffix(".json")
    cert = None
    if sidecar.is_file():
        try:
            cert = json.loads(sidecar.read_text()).get("certificate")
        except Exception as exc:                                       # noqa: BLE001
            reasons.append(f"the certificate does not read: {exc!r}")
    if cert is None:
        reasons = reasons or ["no certificate — `taught_tracks.py certify <recording.txt>` issues one"]
        return dict(ok=False, reasons=reasons, cert=None, checked=checked)
    if cert.get("version") not in CERT_VERSIONS_VERIFIABLE:
        return dict(ok=False, reasons=[f"certificate version {cert.get('version')!r} is not one of "
                                       f"{CERT_VERSIONS_VERIFIABLE} — re-certify"], cert=cert, checked=checked)
    if cert.get("verdict") != "CERTIFIED":
        failed = [g["id"] for g in cert.get("gates", []) if not g.get("passed")]
        reasons.append(f"the certificate's verdict is {cert.get('verdict')!r} (failed: {failed})")
    if (cert.get("side"), cert.get("slot"), cert.get("name")) != (row.get("side"), row.get("slot"),
                                                                  row.get("name")):
        reasons.append("the certificate names a different row")
    proj = REPO / str((cert.get("proj") or {}).get("path") or "")
    if not proj.is_file() or not track.is_file():
        reasons.append("the certified .proj or .txt is missing")
        return dict(ok=False, reasons=reasons, cert=cert, checked=checked)
    proj_bytes = proj.read_bytes()
    if hashlib.sha256(proj_bytes).hexdigest() != cert["proj"]["sha256"]:
        reasons.append(f"{proj.name} is not the certified bytes (sha "
                       f"{hashlib.sha256(proj_bytes).hexdigest()[:12]}, certified "
                       f"{cert['proj']['sha256'][:12]})")
    if sha256_of(track) != cert["track"]["sha256"]:
        reasons.append(f"{track.name} is not the certified bytes")
    if (row.get("proj_sha256"), row.get("track_sha256")) != (cert["proj"]["sha256"],
                                                             cert["track"]["sha256"]):
        reasons.append("the registry pins disagree with the certificate")
    checked.append("sha256 of the .proj and .txt, and the registry pins")
    import tempfile
    with tempfile.TemporaryDirectory() as tdir:
        rebuilt = tr.build_project_file(track, pathlib.Path(tdir) / "x.proj")
        same = pathlib.Path(rebuilt["path"]).read_bytes() == proj_bytes
    if not same:
        reasons.append(f"{proj.name} is not the project form of {track.name}")
    lines = proj_bytes.split(b"\r\n")
    try:
        head = json.loads(lines[1])
        header_ok = (lines[0] == b'{"file":7}' and head.get("name") == "Folder"
                     and int(head.get("type")) == int(cert["proj"]["lines"])
                     and lines[-1] == b"" and b"\n" not in b"".join(lines))
    except Exception:                                                  # noqa: BLE001
        header_ok = False
    if not header_ok:
        reasons.append(f"{proj.name}: project header, CRLF terminators or line count are wrong")
    checked.append("project header, CRLF terminators and line count")
    q = read_q_deg(track)
    # a version-2 certificate is re-judged by the MODEL IT WAS ISSUED WITH (carried in it), a
    # version-1 one by the pre-model rules — a later refit never silently invalidates a row
    model = cert.get("model") if int(cert.get("version") or 1) >= 2 else None
    cal_cert = cal_unpack(cert.get("measured_calibration"))
    if cal_cert is not None and cal_cert.size != q.shape[0]:
        reasons.append(f"the certificate's measured calibration is {cal_cert.size} long "
                       f"but the file has {q.shape[0]} lines")
        cal_cert = None
    # re-judged at the clocks it was ISSUED with (rates at clock_gate_s, predictions at
    # clock_predict_s; a certificate from before 2026-09-17 carries only clock_gate_s, which then
    # served both) — so a later correction never silently changes the numbers it certified ...
    g = gate_report(q, row["side"], cert["clock_gate_s"], cert["position_allowance_deg"],
                    cal_lines=cal_cert,
                    pred_clock_s=float(cert.get("clock_predict_s") or cert["clock_gate_s"]),
                    v_near=cert.get("near_limit_rate_cap_deg_s", NEAR_LIMIT_RATE_DEG_S),
                    hard_floor=cert.get("hard_margin_floor_deg", HARD_MARGIN_FLOOR_DEG),
                    model=model, frame=cert.get("frame"),
                    budget_mm=float(((cert.get("timing") or {}).get("budget_mm"))
                                    or cert["max_deviation_mm"]))
    for why in g["reasons"]:
        reasons.append(f"gate: {why}")
    enf = g["enforced"]
    now = {"G1": round(enf["position_margin_min_deg"], 3), "G2": int(enf["ticks_over_rating_band_limited"]),
           "G3": round(enf["max_step_deg"], 3)}
    if model is not None:
        now["G5"] = enf.get("pred_dev_max_mm")
    for entry in cert.get("gates", []):
        if entry["id"] in now and (now[entry["id"]] is None
                                   or abs(float(entry["value"]) - float(now[entry["id"]])) > 1e-2):
            reasons.append(f"{entry['id']} recomputes to {now[entry['id']]}, certificate says "
                           f"{entry['value']}")
        if entry["id"] == "G1" and entry.get("near_limit_rate_deg_s") is not None:
            got = enf.get("near_limit_rate_deg_s")
            if got is None or abs(float(got) - float(entry["near_limit_rate_deg_s"])) > 0.05:
                reasons.append(f"G1 near-limit rate recomputes to {got}, certificate says "
                               f"{entry['near_limit_rate_deg_s']}")
        if entry["id"] == "G1" and entry.get("pred_exec_margin_min_deg") is not None:
            got = enf.get("pred_exec_margin_min_deg")
            if got is None or abs(float(got) - float(entry["pred_exec_margin_min_deg"])) > 0.05:
                reasons.append(f"G1 predicted executed margin recomputes to {got}, certificate says "
                               f"{entry['pred_exec_margin_min_deg']}")
    # ... and, separately, held to TODAY's rate rule: the player plays up to one point per 5 ms,
    # so a file certified at an older, slower gate clock can command a joint over its rating
    # there. That is a reason, named as such, not a changed number.
    if float(cert["clock_gate_s"]) > CLOCK_GATE_S + 1e-9:
        now_rate = gate_report(q, row["side"], CLOCK_GATE_S, cert["position_allowance_deg"])["enforced"]
        if int(now_rate["ticks_over_rating_band_limited"]):
            reasons.append(
                f"G2 at the player's {CLOCK_GATE_S * 1e3:.1f} ms ceiling: "
                f"{now_rate['ticks_over_rating_band_limited']} tick(s) over a joint rating (peak "
                f"{now_rate['peak_rate_util_band_limited']:.3f}x on joint {now_rate['peak_joint']}) — "
                f"certified at {float(cert['clock_gate_s']) * 1e3:.2f} ms before the gate clock was "
                f"corrected; re-certify")
        checked.append(f"G2 re-judged at the player's {CLOCK_GATE_S * 1e3:.1f} ms ceiling")
    lo, hi = ct.TAUGHT_SLOT_RANGE
    if not ct.taught_slot_ok(row.get("slot")):
        reasons.append(f"slot {row.get('slot')} is outside {lo}-{hi} and below the CANFD-only "
                       f"{ct.TAUGHT_CANFD_SLOT_MIN}+")
    checked.append(f"G1-G3 recomputed on the stored lines at {cert['clock_gate_s'] * 1e3:.2f} ms")
    if deep:
        src = REPO / str((cert.get("source") or {}).get("path") or "")
        if not src.is_file() or sha256_of(src) != cert["source"]["sha256"]:
            reasons.append("the taught recording is missing or no longer hashes to its pin")
        else:
            d = deviation_mm(read_q_deg(src), q, row["side"], cert["frame"])
            g4 = next((e for e in cert.get("gates", []) if e["id"] == "G4"), {})
            if d["max_mm"] > cert["max_deviation_mm"]:
                reasons.append(f"G4 recomputes to {d['max_mm']:.2f} mm, over the "
                               f"{cert['max_deviation_mm']:g} mm cap")
            if g4.get("value") is None or abs(d["max_mm"] - float(g4["value"])) > 0.01:
                reasons.append(f"G4 recomputes to {d['max_mm']:.3f} mm, certificate says {g4.get('value')}")
            checked.append("G4 recomputed against the taught recording")
    return dict(ok=not reasons, reasons=reasons, cert=cert, checked=checked)


# ─────────────────────────────────────────────────────────────── admit
def cmd_admit(args) -> int:
    """Screen ONE recording and register it as a row (rung 0: verbatim)."""
    src = pathlib.Path(args.file).expanduser().resolve()
    if not src.is_file():
        raise SystemExit(f"no such file: {src}")
    side = args.side
    slot = int(args.slot)
    lo, hi = ct.TAUGHT_SLOT_RANGE
    if slot < 1:
        raise SystemExit("a taught slot is a positive number")
    if ct.SLOT_BASE <= slot <= ct.SLOT_MAX:
        print(f"** slot {slot} is inside the production range "
              f"{ct.SLOT_BASE}-{ct.SLOT_MAX} — a taught motion must never be able to "
              f"overwrite a cleaning task. Refusing.")
        return 4
    if not ct.taught_slot_ok(slot) and not args.allow_slot:
        print(f"** slot {slot} is outside the taught range {lo}-{hi} and below the CANFD-only "
              f"{ct.TAUGHT_CANFD_SLOT_MIN}+ (refusing; --allow-slot to override, recorded in the row)")
        return 4
    if ct.canfd_only_slot(slot):
        print(f"   slot {slot} is CANFD-only: no controller slot, nothing is uploaded — it runs on "
              f"--channel canfd")

    q = read_q_deg(src)                       # refuses a malformed file outright
    prov = source_provenance(src)
    declared_side = prov.get("side")
    if declared_side and declared_side != side:
        print(f"** the provenance record says this recording was taught on the "
              f"{declared_side.upper()} arm but --side {side} was declared — refusing "
              f"(a motion belongs to the arm it was taught on).")
        return 4
    hardware = args.hardware or prov.get("hardware") or "unknown"
    if hardware != "ok" and not args.allow_analysis_only:
        print(f"** provenance reads `hardware: {hardware}` — a recording made on ANOTHER arm "
              f"mount, or with no provenance at all, is analysis-only and must not be "
              f"replayed. Add `hardware: ok` to its provenance if it was taught on this "
              f"mount, or pass --allow-analysis-only to override (recorded).")
        return 4

    # the recording gets a permanent, immutable home; the admitted bytes are
    # rung 0 — the file exactly as taught.
    ct.TAUGHT_SOURCE_DIR.mkdir(parents=True, exist_ok=True)
    dest_src = ct.TAUGHT_SOURCE_DIR / src.name
    if src != dest_src.resolve():
        if dest_src.is_file() and sha256_of(dest_src) != sha256_of(src):
            print(f"** source/{src.name} already holds a DIFFERENT recording (sha "
                  f"{sha256_of(dest_src)[:12]}); a source is immutable and another row may be "
                  f"pinned to it. Rename the file, or use `certify`, which stores it under a "
                  f"unique name.")
            return 4
        shutil.copyfile(src, dest_src)
    name = args.name and str(args.name) or f"{slot}_taught_{slug(src.stem)}_{side}"
    if not re.fullmatch(r"[0-9]+_taught_[A-Za-z0-9_]+", name):
        name = f"{slot}_taught_{slug(name)}"
    if not name.endswith(f"_{side}"):
        name = f"{name}_{side}"
    track = ct.TAUGHT_DIR / f"{name}.txt"
    ct.TAUGHT_DIR.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(dest_src, track)
    proj = ct.build_proj_for(track)

    gates = gate_report(q, side, args.clock_min_s, args.position_allowance_deg)
    frame = args.frame or prov.get("frame") or f"{side_char(side)}_glove_frame_2"
    hand_state = args.hand_state or prov.get("hand_state")
    if getattr(args, "hand_fingers_pct", None) is not None:
        # the sheets' own step: "hand <pct> % (first 4 fingers)", thumb untouched
        pct = float(args.hand_fingers_pct)
        hand_state = args.hand_state or fingers_pct_label(pct)
        args.hand_counts = fingers_pct_counts(pct)
    hand_counts = (args.hand_counts or prov.get("hand_counts")
                   or hand_counts_for(hand_state, side))
    pole_m = args.pole_m if args.pole_m is not None else prov.get("pole_m")
    points = int(q.shape[0])

    row = dict(
        slot=slot, name=name, side=side, status="EMITTED", kind="taught",
        plan=None, track=ct.repo_rel(track, REPO), points=points,
        track_s=round((points - 1) * CLOCK_NOMINAL_S, 4),
        band_5p5_5p6_s=duration_band_s(points),
        acc_scale=None,
        source=dict(path=ct.repo_rel(dest_src, REPO), sha256=sha256_of(dest_src),
                    side=side, frame=frame, pole_m=pole_m, hand_state=hand_state,
                    hand_counts=(list(hand_counts) if hand_counts else None),
                    hardware=hardware, taught_by=prov.get("taught_by"),
                    date=prov.get("date"), units="millideg",
                    provenance_record=prov.get("source_of_provenance"),
                    note=prov.get("note")),
        pre_stroke=dict(pole_m=pole_m, pole_stage=None, hand_state=hand_state,
                        hand_counts=(list(hand_counts) if hand_counts else None),
                        hand_release_counts=None,
                        source=("the recording's own provenance (SOURCE.json / exp01 "
                                "MANIFEST.yaml) — `flow` stages these before the stroke; a "
                                "single `run` compares the pole and never commands it")),
        retreat_stage=None, retreat_deg=None,
        gates=gates,
        shape=dict(mode="verbatim", rung=None, knot_spacing=None, vel_scale=None,
                   acc_scale=None, deviation_mm=None,
                   what="rung 0: the recording exactly as taught (zero deviation); `shape` "
                        "prices the re-parameterised ladder against it"),
        what=("taught motion admitted 2026-09-14: played verbatim at the controller's own "
              "clock, the point spacing IS the velocity profile"),
    )
    d = ct.disk_provenance(row["track"], build=False)
    row.update(track_sha256=d["track_sha256"], track_bytes=d["track_bytes"],
               points_on_disk=d["points_on_disk"], proj=d["proj"],
               proj_sha256=d["proj_sha256"], proj_bytes=d["proj_bytes"])
    row["proj_points"] = proj["points"]
    row["tool_frame"] = inherited_tool_frame(side)

    reg = ct.load_taught_registry()
    # A SLOT HOLDS ONE PROGRAM PER ARM, so re-admitting into a slot replaces
    # the row AND its artifacts — a stale `.txt`/`.proj`/`.json` left beside the
    # new ones is exactly the confusion the pins exist to prevent.
    #
    # BUT NEVER THE FILES JUST WRITTEN. Re-admitting the same name into the same slot puts the
    # old row's `.txt`/`.proj` at the very paths written above, and until 2026-09-21 this loop
    # deleted them — the row survived pointing at nothing (it happened to slots 54 and 55). The
    # old `.json` certificate still goes: an admitted row is uncertified by definition.
    fresh = {track.resolve(), track.with_suffix(".proj").resolve()}
    keep = []
    for t in reg.get("tasks", []):
        if t.get("side") == side and t.get("slot") == slot:
            # THE SAME RECORDING RE-ADMITTED keeps its rest setting (a runtime stage, not bytes)
            if "rest" not in row and t.get("rest"):
                if (t.get("source") or {}).get("sha256") == row["source"]["sha256"]:
                    row["rest"] = t["rest"]
                else:
                    # a DIFFERENT recording takes the slot: its setting is not carried over, and a
                    # row without one takes the DEFAULT (both legs, Newton 2026-09-22) — say so
                    old_r = t["rest"]
                    legs = " and ".join(w for w in ("before", "after") if old_r.get(w))
                    at = old_r.get("at")
                    where = (" at the stroke's own start (the drag start pose)" if at == "start" else
                             f" at {old_r.get('pose_name') or 'a stored pose'}" if at == "pose" else
                             " at the side's move_to_rest")
                    was = (legs + where) if legs else "opted OUT"
                    if not getattr(args, "rest_around", False):
                        print(f"  NOTE: slot {slot} ({side}) had rest {was}; the new recording takes "
                              f"the DEFAULT, rest before AND after at the side's move_to_rest in "
                              f"`flow` — `taught_tracks.py rest {slot} --side {side} --around --at "
                              f"start` (or --none|--before|--after) changes it")
            old = REPO / str(t.get("track") or "")
            for p in (old, old.with_suffix(".proj"), old.with_suffix(".json")):
                try:
                    if p.is_file() and p.resolve() not in fresh:
                        p.unlink()
                except OSError:
                    pass
            continue
        keep.append(t)
    if getattr(args, "rest_around", False):
        # both legs, at the pose the row already holds (a carried `at: start` is kept)
        row["rest"] = rest_block(True, True, at=(row.get("rest") or {}).get("at") or "rest", deg=(row.get("rest") or {}).get("deg"), pose_name=(row.get("rest") or {}).get("pose_name", ""))
    reg["tasks"] = keep
    reg["tasks"].append(row)
    ct.save_taught_registry(reg)

    enf = gates["enforced"]
    print(f"admitted {row['name']}  (slot {slot}, {side} arm)")
    print(f"  source   {row['source']['path']}  sha {row['source']['sha256'][:12]}  "
          f"hardware {hardware}")
    print(f"  track    {row['track']}  {points} pts  "
          f"band [{row['band_5p5_5p6_s'][0]:.2f}, {row['band_5p5_5p6_s'][1]:.2f}] s  "
          f"(.proj {row['proj_bytes'] / 1024:.1f} KB)")
    print(f"  pole {pole_m} m   hand {hand_state} {hand_counts}   frame {frame}")
    print(f"  gates @ {enf['clock_gate_s'] * 1e3:.2f} ms (fast clock):")
    print(f"    position margin {enf['position_margin_min_deg']:.3f} deg (J"
          f"{enf['position_binding_joint']}), allowance {enf['position_allowance_deg']:.1f}"
          f"  -> after allowance {enf['position_margin_after_allowance_deg']:+.3f}")
    print(f"    ticks over a rating: {enf['ticks_over_rating_band_limited']} band-limited "
          f"(peak util {enf['peak_rate_util_band_limited']:.3f} on J{enf['peak_joint']}), "
          f"{enf['ticks_over_rating_raw']} raw")
    print(f"    largest step {enf['max_step_deg']:.3f} deg (fault at {enf['step_fault_deg']:.0f})")
    a = gates["advisory"]
    print(f"  advisory: accel util {a['accel_util_band_limited']}, jerk util "
          f"{a['jerk_util_band_limited']} (recorded, never refused)")
    sl = a.get("slowness") or {}
    if "pad_speed_p50_mm_s" in sl:
        print(f"  slowness: pad p10/p50/p90 {sl['pad_speed_p10_mm_s']}/{sl['pad_speed_p50_mm_s']}/"
              f"{sl['pad_speed_p90_mm_s']} mm/s; under 20 mm/s {sl['s_below_20_mm_s']} s, under 50 "
              f"{sl['s_below_50_mm_s']} s (longest {sl['longest_below_50_mm_s_s']} s); R4 would still "
              f"remove {sl['r4_still']['lines']} lines in {sl['r4_still']['runs']} runs "
              f"(recorded, never refused)")
    if gates["admissible"]:
        print("  verbatim replay is ADMISSIBLE — `shape` will price the faster rungs "
              "against it")
    else:
        for why in gates["reasons"]:
            print(f"  ** {why}")
        print(f"  -> run `taught_tracks.py shape {slot}` to find the fastest admissible rung"
              f"{' (a velocity failure is shapeable; a position failure is not)' if True else ''}")
    print(f"  registry -> {ct.repo_rel(ct.TAUGHT_REGISTRY, REPO)}")
    return 0


# ─────────────────────────────────────────────────────────────── shape
def _solver_policy(args) -> str:
    if getattr(args, "no_solver", False):
        return "never"
    if getattr(args, "full_ladder", False):
        return "always"
    return str(getattr(args, "solver", None) or "auto")


def shape_search(row: dict, args, log=print) -> dict:
    """Price the ladder for ONE admitted row and choose the rung — NO registry
    writes (`shape_install` does those), so `batch` can run it in worker
    processes side by side.

    THE SEARCH STOPS AS SOON AS THE ANSWER IS KNOWN (2026-09-14, tightened
    2026-09-15). Pricing every rung cost ~2.5 min on a 9.6k-line recording:
    12 s per solver rung and 5.7 s per exact deviation search, while the gates
    themselves cost milliseconds. The choice needs far less:
      * DIAL 1 (uniform re-time) is gated fastest first and stops at the first
        rung that passes — the slower rungs behind it are never gated;
      * DIAL 2 (the solver) runs only when the uniform ladder cannot reach
        SOLVER_AUTO_BELOW (`--solver auto`; `always`/`never` override), is
        walked from the fastest velocity scale down and stops at the first rung
        that passes the gates — a tighter velocity limit can only give TOPP-RA
        a slower optimum, so no later rung can win;
      * deviation is computed fastest-first and stops at the first rung inside
        the preferred bound (or, failing that, the first inside the cap) —
        exactly the rung the full search would pick.
    `--full-ladder` restores the complete pricing for a report."""
    side = row["side"]
    src_rel = (row.get("source") or {}).get("path")
    if not src_rel:
        raise SystemExit(f"{row['name']} carries no source path — re-admit it")
    src = REPO / src_rel
    q_taught = read_q_deg(src)
    smooth_hz = getattr(args, "smooth_hz", None)
    # the working recording: smoothed when asked (every rung, gate and prediction is built on it);
    # G4 always measures against `q_taught`, the recording as taught
    q_ref = smooth_taught(q_taught, float(smooth_hz), CLOCK_RECORD_S) if smooth_hz else q_taught
    n_ref = int(q_ref.shape[0])
    taught_s = (n_ref - 1) * CLOCK_NOMINAL_S
    frame = row["source"].get("frame") or f"{side_char(side)}_glove_frame_2"
    ik_frame, ik_note = resolve_solver_frame(side, getattr(args, "ik_frame", None) or frame)
    # a smoothed ladder gets its own rung folder: the rung files are named by lambda alone, and
    # the unsmoothed certificates point at theirs
    out_dir = ct.TAUGHT_RUNGS_DIR / (src.stem + (f"_smooth{float(smooth_hz):g}hz" if smooth_hz else ""))
    max_speed = getattr(args, "max_speed", None)
    full = bool(getattr(args, "full_ladder", False))
    policy = _solver_policy(args)
    pick = str(args.pick).strip().lower() if getattr(args, "pick", None) else None
    v_near = float(getattr(args, "near_limit_rate_deg_s", None) or NEAR_LIMIT_RATE_DEG_S)
    hard_floor = float(getattr(args, "hard_margin_deg", None) or HARD_MARGIN_FLOOR_DEG)
    # THE EXECUTION MODEL (default since 2026-09-15; `--timing legacy` = the old ladder)
    timing_mode = str(getattr(args, "timing", None) or "model")
    model = _model_from_args(args) if timing_mode == "model" else None
    keep_pauses = bool(getattr(args, "keep_pauses", False))
    sections = parse_sections(getattr(args, "section", None))
    dwells = parse_dwells(getattr(args, "dwell", None))
    q_work, pause_info = q_ref, None
    if model is not None and not keep_pauses:
        q_work, pause_info = strip_pauses(q_ref)
    exec_budget = float(getattr(args, "exec_budget_mm", None) or args.max_deviation_mm)
    timing_rec = dict(mode=timing_mode, remove_pauses=bool(model is not None and not keep_pauses),
                      pauses_removed=pause_info, sections=[list(s) for s in sections],
                      dwells=[list(d) for d in dwells], budget_mm=exec_budget,
                      bytes_budget_mm=float(args.max_deviation_mm),
                      percentile=(model.get("percentile") if model is not None else None),
                      smoothing=(dict(hz=float(smooth_hz), order=SMOOTH_ORDER, zero_phase=True,
                                      sample_clock_s=CLOCK_RECORD_S,
                                      filter="butterworth, forward-backward (scipy filtfilt)",
                                      applied_to="the taught recording at its own 5 ms sample clock, "
                                                 "before pauses are removed and before any timing",
                                      path_change_gate="G4 against the UNSMOOTHED recording, "
                                                       f"<= {float(args.max_deviation_mm):g} mm")
                                 if smooth_hz else None))
    ptr = _ptr()
    res = dict(best=None, candidates=[], note="", why=None, n_ref=n_ref, taught_s=taught_s,
               frame=frame, ik_frame=ik_frame, out_dir=str(out_dir), solver_ran=False,
               refused=False, verbatim_gates=None,
               model=(_model_params(model) if model is not None else None), timing=timing_rec)

    log(f"\n  shaping {row['name']}: {n_ref:,} taught lines (side {side}, deviation at the "
        f"declared frame {frame}){ik_note}")
    if smooth_hz:
        _sd = np.abs(q_ref - q_taught).max(axis=0)
        log(f"  recording SMOOTHED: {float(smooth_hz):g} Hz zero-phase low-pass at the "
            f"{CLOCK_RECORD_S * 1e3:g} ms record clock "
            f"(joint change max {np.round(_sd, 2).tolist()} deg); G4 measures against the unsmoothed "
            f"recording, cap {float(args.max_deviation_mm):g} mm")
    if model is not None:
        log(f"  execution model {model.get('file', 'defaults')}: tau {model['tau_path_s']:g} s (path, "
            f"{model.get('percentile', 'p90')}) / {model['tau_limit_s']:g} s (limits, max), floor "
            f"{model['floor_deg']:g} deg, eps {model['limit_eps_deg']:g} deg, budget "
            f"{exec_budget:g} mm executed (bytes {float(args.max_deviation_mm):g} mm); the ratings "
            f"everywhere else")
        if pause_info and pause_info["removed_lines"]:
            log(f"  pauses removed: {pause_info['removed_lines']} lines ({pause_info['removed_s']} s) in "
                f"{pause_info['runs']} still stretch(es), longest {pause_info['longest_lines']} lines")
        if sections or dwells:
            log(f"  sections {sections or '—'} · dwells {dwells or '—'}")
    if (row["source"].get("hardware") != "ok") and not getattr(args, "force", False):
        log(f"** provenance reads hardware: {row['source'].get('hardware')} — refusing to "
            f"emit a runnable track from it (--force only if you mean it)")
        res.update(refused=True, why=f"provenance reads hardware: {row['source'].get('hardware')}")
        return res

    def speed_of(c):
        return taught_s / c["duration_s"]

    def within_cap(c):
        return max_speed is None or speed_of(c) <= float(max_speed) + 1e-9

    def gate(c):
        if c["gates"] is None:
            c["gates"] = gate_report(c["q"], side, args.clock_min_s, args.position_allowance_deg,
                                     v_near=v_near, hard_floor=hard_floor, model=model, frame=frame,
                                     budget_mm=exec_budget,
                                     cal_lines=cal_on_lines(cal_src, q_work, c["q"]))
        return c["gates"]["admissible"]

    # rung 0 first: verbatim, zero deviation by construction. Its BYTES come
    # from the SOURCE file, NOT from `row["track"]` — a previous `shape` may
    # already have overwritten the admitted track with a faster rung, and
    # pointing the verbatim rung at that mutable file is what left "chosen:
    # verbatim (8579 pts)" pinned against a 4290-pt track (2026-09-14 upload
    # refusal: the `src != dst` guard skipped the copy and the old rung stayed).
    verbatim_track = src
    if smooth_hz:
        verbatim_track = write_track(out_dir / "track_verbatim.txt", np.round(q_ref * 1000.0).astype(np.int64))
    candidates = [dict(mode="verbatim", vel_scale=None, time_scale=None, track=verbatim_track, q=q_ref,
                       duration_s=taught_s, rung=None, gates=None, dev=None, in_cap=False,
                       preferred=False, smooth_hz=(float(smooth_hz) if smooth_hz else None))]

    def add_uniform(lam):
        md = uniform_millideg(q_ref, lam)
        p = write_track(out_dir / f"track_uniform{lam:.2f}.txt", md)
        q = md / 1000.0
        c = dict(mode="uniform", vel_scale=None, time_scale=float(lam), track=p, q=q,
                 duration_s=(q.shape[0] - 1) * CLOCK_NOMINAL_S, rung=ct.repo_rel(p, REPO),
                 gates=None, dev=None, in_cap=False, preferred=False)
        candidates.append(c)
        return c

    # DIAL 1 — POINT SPACING. New points are interpolations ON the taught
    # polyline, so this ladder cannot move the path (exact: 0.01 mm by the
    # global search) and it is cheap; on the three 2026-09-14 recordings it also
    # WINS, because the solver is acceleration-limited (its rungs share one
    # duration across every velocity scale) while uniform speed is stopped only
    # by the velocity rating.
    def add_guarded(lam):
        md = guarded_millideg(q_ref, lam, args.position_allowance_deg, v_near, clock_s=args.clock_min_s)
        p = write_track(out_dir / f"track_guard{lam:.2f}.txt", md)
        q = md / 1000.0
        c = dict(mode="guarded", vel_scale=None, time_scale=float(lam), track=p, q=q,
                 duration_s=(q.shape[0] - 1) * CLOCK_NOMINAL_S, rung=ct.repo_rel(p, REPO),
                 gates=None, dev=None, in_cap=False, preferred=False)
        candidates.append(c)
        return c

    # THE MEASURED CALIBRATION (`--use-measured`): WHERE the arm really departs, from
    # every complete run of the row's CURRENT installed file, carried onto the source's
    # lines. The model gets the magnitude right across path families and the LOCATION
    # wrong (Spearman -0.22 on taught files) while two runs of the same bytes agree at
    # +0.64, so this takes the location from measurement and keeps the model's
    # scaling-with-speed. It returns None unless there are >= 2 complete runs that agree,
    # which is what stops it being used at speeds where nothing reproduces (slot 44 at
    # 6.29x reads -0.17 and its hot spots move between runs).
    cal_src, cal_notes = None, None
    if model is not None and bool(getattr(args, "use_measured", False)):
        # ANY complete run of ANY file emitted from THIS SOURCE counts, not only this
        # row's own: the calibration is carried onto the source's lines, so a sibling
        # row of the same recording (slot 43 for a new slot 49) is exactly as
        # informative, and a brand-new row has no installed file to measure against.
        inst = _measured_sibling_track(row, REPO)
        if inst is not None:
            ran_clock = _measured_clock_s({"track": ct.repo_rel(inst, REPO)}, REPO) or CLOCK_NOMINAL_S
            cal_src, cal_notes = measured_calibration(inst, q_work, side, frame,
                                                      model, ran_clock, repo=REPO)
            if cal_notes is not None:
                cal_notes["measured_from"] = ct.repo_rel(inst, REPO)
            if cal_src is None:
                log(f"  --use-measured: NOT used — {cal_notes.get('why', 'no profile')}")
            else:
                log(f"  --use-measured: measuring against {ct.repo_rel(inst, REPO)}")
                log(f"  --use-measured: {cal_notes['n_runs']} complete run(s) of the installed "
                    f"file agree at Spearman {cal_notes['run_to_run_rho']}; calibration "
                    f"p50 {cal_notes['calibration']['p50']} p95 {cal_notes['calibration']['p95']} "
                    f"max {cal_notes['calibration']['max']} over "
                    f"{cal_notes['calibration']['mapped_lines']} mapped lines")
                log(f"  --use-measured: the BAR is now the calibrated estimate "
                    f"(prediction x this), which on the installed file read 41.5 mm against "
                    f"a measured 40.2-43.2 where the raw prediction said 19.8")
        else:
            log("  --use-measured: NOT used — no file emitted from this source has a "
                "complete run on record")

    # THE MODEL RUNG (R1-R5): the source path — pauses removed — at the joint ratings wherever
    # the execution model lets it, the model's arrival rate at every reversal and every limit,
    # the section factors and the dwells the task listed. lambda_fast is the ceiling on the
    # free stretches; the fastest lambda whose gates (G1 model, G2, G3, G4, G5) pass is installed.
    def add_model(lam):
        md, notes = model_millideg(q_work, lam, side, frame, model, exec_budget,
                                   args.clock_min_s, sections, dwells, cal_src=cal_src)
        p = write_track(out_dir / f"track_model{lam:.2f}.txt", md)
        q = md / 1000.0
        c = dict(mode="model", vel_scale=None, time_scale=float(lam), track=p, q=q,
                 duration_s=(q.shape[0] - 1) * CLOCK_NOMINAL_S, rung=ct.repo_rel(p, REPO),
                 gates=None, dev=None, in_cap=False, preferred=False,
                 timing=dict(timing_rec, caps=notes,
                             measured=(cal_notes if cal_src is not None else None)))
        candidates.append(c)
        return c

    verbatim = candidates[0]
    gate(verbatim)                     # G1 is geometry: read once, on the recording itself
    venf = verbatim["gates"]["enforced"]
    # NEAR A LIMIT the plain uniform ladder cannot pass G1 — it keeps the recording's
    # rates at the limit, times lambda — so the ladder becomes the GUARDED family: the
    # same point-spacing re-time with every joint's rate capped inside the allowance.
    guard = model is None and venf.get("position_rule") in ("rate", "guarded")
    scales = [float(x) for x in args.time_scales if abs(float(x) - 1.0) > 1e-9]
    if pick and (pick.startswith("uniform:") or pick.startswith("guard:") or pick.startswith("model:")):
        want = float(pick.split(":", 1)[1])
        if all(abs(want - x) > 1e-9 for x in scales) and abs(want - 1.0) > 1e-9:
            scales.append(want)
    if model is not None:
        if venf.get("position_rule") == "geometry":
            log(f"  ** {'; '.join(verbatim['gates']['reasons'])}")
        for lam in scales + [1.0]:
            add_model(lam)
    elif guard:
        log(f"  J{venf['position_binding_joint']} comes {venf['position_margin_min_deg']:.2f} deg from "
            f"its limit, inside the {args.position_allowance_deg:g} deg allowance but outside the "
            f"{hard_floor:g} deg floor: the ladder is the GUARDED re-time — the joint held to "
            f"<= {v_near:g} deg/s wherever it is inside the allowance, the rest of the file at lambda")
        for lam in scales + [1.0]:
            add_guarded(lam)
    else:
        for lam in scales:
            add_uniform(lam)

    def admissible_fastest_first():
        """Every candidate inside --max-speed that passes the gates, fastest first,
        gated ON DEMAND: the caller stops iterating once it is settled, and
        whatever it never reached is never gated."""
        for c in sorted(candidates, key=lambda c: c["duration_s"]):
            if not within_cap(c):
                continue
            if gate(c):
                yield c

    first = next(admissible_fastest_first(), None)

    # DIAL 2 — the solver's re-parameterisation, fastest velocity scale first,
    # and only when point spacing alone did not get far enough.
    run_solver = (policy == "always"
                  or (policy == "auto" and (first is None or speed_of(first) < SOLVER_AUTO_BELOW)))
    if run_solver and (guard or (model is not None and policy != "always")):
        log("  solver skipped: its rungs carry none of the model's caps"
            + ("" if model is not None else " near the limit and would fail G1")
            + " (--solver always prices them anyway)")
        run_solver = False
    if run_solver:
        res["solver_ran"] = True
        forced = float(pick.split(":", 1)[1]) if (pick and pick.startswith("vel:")) else None
        scales2 = sorted({float(v) for v in args.vel_scales} | ({forced} if forced else set()),
                         reverse=True)
        settled = False
        for vs in scales2:
            if settled and not full and vs != forced:
                continue
            r = ptr.toppra_ladder(q_ref, side_char(side), out_dir, [vs], ik_frame, args.max_acc_util,
                                  src, row["source"].get("hardware", "unknown"),
                                  row["source"]["sha256"])[0]
            rep = r.get("track_report")
            if not rep or int(r.get("returncode", 1)) != 0:
                tail = " | ".join((r.get("stdout_tail") or [])[-3:])
                log(f"  vel {vs:.2f}: NO TRACK (rc={r.get('returncode')}) — {tail[-200:]}")
                continue
            p = pathlib.Path(r["track"])
            if not p.is_file():
                continue
            q = read_q_deg(p)
            c = dict(mode="reparameterised", vel_scale=float(vs), time_scale=None, track=p, q=q,
                     duration_s=float(rep["track_duration_s"]), rung=ct.repo_rel(p, REPO),
                     gates=None, dev=None, in_cap=False, preferred=False)
            candidates.append(c)
            if gate(c) and within_cap(c):
                settled = True
    elif policy == "auto" and first is not None:
        log(f"  solver not needed: {rung_title(first)} passes the gates at {speed_of(first):.2f}x "
            f"taught, at or above the {SOLVER_AUTO_BELOW:g}x bar (--solver always prices it anyway)")

    # the last resort: nothing passes yet, and not because of geometry
    if (next(admissible_fastest_first(), None) is None
            and (venf.get("position_rule") != "geometry" if model is not None
                 else venf["position_margin_min_deg"] >= args.position_allowance_deg)):
        log(f"  no rung passes yet — pricing the uniform slow-down {SLOW_SCALES_DEFAULT} "
            f"(points stay on the taught path)")
        for lam in SLOW_SCALES_DEFAULT:
            gate(add_model(lam) if model is not None else (add_guarded(lam) if guard else add_uniform(lam)))

    def price(c):
        if c["dev"] is None:
            c["dev"] = deviation_mm(q_taught, c["q"], side, frame)
            c["in_cap"] = c["dev"]["max_mm"] <= args.max_deviation_mm
            c["preferred"] = c["dev"]["max_mm"] <= args.prefer_deviation_mm
            log(f"  deviation  {rung_title(c):<14} max {c['dev']['max_mm']:7.2f} mm  p95 "
                f"{c['dev']['new_to_taught_mm']['p95_mm']:6.2f} mm  "
                + ("inside the cap" if c["in_cap"] else
                   f"** over the {args.max_deviation_mm:.0f} mm cap"))
        return c

    if full:
        for c in admissible_fastest_first():
            price(c)
    best, note = None, ""
    if pick in (None, "fastest"):
        fallback = None
        for c in admissible_fastest_first():
            price(c)
            if not c["in_cap"]:
                continue
            if pick == "fastest" or c["preferred"]:
                best = c
                break
            fallback = fallback or c
        if best is None and fallback is not None:
            best = fallback
            note = (f" (no rung inside the {args.prefer_deviation_mm:.0f} mm advisory; the fastest "
                    f"inside the {args.max_deviation_mm:.0f} mm cap was taken)")
    elif pick == "slowest":
        for c in reversed(list(admissible_fastest_first())):
            if price(c)["in_cap"]:
                best = c
                break
    else:
        hits = [c for c in admissible_fastest_first() if rung_label(c) == pick]
        if hits and price(hits[0])["in_cap"]:
            best = hits[0]
        else:
            raise SystemExit(
                f"--pick {args.pick!r} is not an admissible rung of {row['name']}. Admissible by the "
                f"gates: {', '.join(rung_label(c) for c in admissible_fastest_first()) or 'none'}")
    if pick:
        note = f" (--pick {args.pick})" + note
    # A VERBATIM PICK IS THE SOURCE BYTES, which R4 never touched: the candidate is q_ref, not
    # q_work. So its timing record must describe those bytes, not the --keep-pauses flag. Before
    # 2026-09-21 a verbatim pick WITHOUT --keep-pauses certified `remove_pauses: true` and a
    # pauses_removed count for a file that still held every pause (audit 2026-09-21).
    if best is not None and best.get("mode") == "verbatim" and res.get("timing"):
        best["timing"] = dict(res["timing"], remove_pauses=False, pauses_removed=None,
                              verbatim="rung 0 is the source verbatim: R4 was not applied to "
                                       "these bytes")

    # the ladder, after the search, so it shows what was gated and what never had to be
    pred_col = f" {'pred mm':>8} {'exec mg':>8}" if model is not None else ""
    log(f"\n  {'rung':<14} {'pts':>6} {'dur s':>8} {'x taught':>9} {'margin':>8} "
        f"{'over':>5} {'step':>7}{pred_col}  gates @ {args.clock_min_s * 1e3:.2f} ms")
    for c in sorted(candidates, key=lambda c: c["duration_s"]):
        if c["gates"] is None:
            log(f"  {rung_title(c):<14} {c['q'].shape[0]:>6} {c['duration_s']:>8.3f} "
                f"{speed_of(c):>8.2f}x {'—':>8} {'—':>5} {'—':>7}"
                + (f" {'—':>8} {'—':>8}" if model is not None else "") + "  "
                + ("over --max-speed" if not within_cap(c) else "not needed"))
            continue
        enf = c["gates"]["enforced"]
        verdict = ("pass" if c["gates"]["admissible"] else
                   "** " + "; ".join(c["gates"]["reasons"])[:110])
        if c["gates"]["admissible"] and not within_cap(c):
            verdict = f"pass, but over --max-speed {float(max_speed):g}"
        pred = ""
        if model is not None:
            em = enf.get("pred_exec_margin_min_deg")
            pred = (f" {enf.get('pred_dev_max_mm', float('nan')):>8.2f} "
                    f"{(f'{em:.2f}d' if em is not None else '—'):>8}")
        log(f"  {rung_title(c):<14} {c['q'].shape[0]:>6} {c['duration_s']:>8.3f} "
            f"{speed_of(c):>8.2f}x {enf['position_margin_min_deg']:>7.2f}d "
            f"{enf['ticks_over_rating_band_limited']:>5} {enf['max_step_deg']:>6.3f}d{pred}  {verdict}")

    if best is None:
        gated = [c for c in candidates if c["gates"] is not None]
        if verbatim["gates"]["reasons"] and not any(c["gates"]["admissible"] for c in gated):
            why = "no rung passes: " + "; ".join(verbatim["gates"]["reasons"])
        elif not any(c["gates"]["admissible"] and within_cap(c) for c in gated):
            why = f"no rung that passes the gates is within --max-speed {max_speed}"
        else:
            why = f"no rung that passes the gates is inside the {args.max_deviation_mm:g} mm deviation cap"
        res["why"] = why
    # the CHOSEN rung's measured correction, carried out so `shape_install` can put it
    # IN the certificate — G5 was judged with it, so `verify` has to apply the same
    # number, and must not need the run captures to do so
    cal_best = (cal_on_lines(cal_src, q_work, best["q"])
                if (best is not None and cal_src is not None) else None)
    for c in candidates:
        c["points"] = int(c["q"].shape[0])
        del c["q"]
    res.update(best=best, candidates=candidates, note=note, verbatim_gates=verbatim["gates"],
               cal_lines=(cal_best.tolist() if cal_best is not None else None),
               measured=(cal_notes if cal_src is not None else None))
    return res


def shape_install(reg: dict, row: dict, res: dict, args) -> dict:
    """Install `res["best"]` as the row's admitted bytes, rebuild the .proj,
    re-pin, certify, save the registry and write the sidecar. Returns the
    certificate. `best["track"]` is the SOURCE for verbatim and a rung file
    otherwise, so the copy always restores the right bytes."""
    best = res["best"]
    n_ref, taught_s, frame, ik_frame = res["n_ref"], res["taught_s"], res["frame"], res["ik_frame"]
    side = row["side"]
    max_speed = getattr(args, "max_speed", None)
    full = bool(getattr(args, "full_ladder", False))

    def speed_of(c):
        return taught_s / c["duration_s"]

    src_track, dst_track = pathlib.Path(best["track"]), REPO / row["track"]
    if src_track.resolve() != dst_track.resolve():
        shutil.copyfile(src_track, dst_track)
    proj = ct.build_proj_for(dst_track)
    points = int(best["points"])
    row.update(points=points, track_s=round((points - 1) * CLOCK_NOMINAL_S, 4),
               band_5p5_5p6_s=duration_band_s(points), gates=best["gates"],
               shape=dict(mode=best["mode"], rung=(best["rung"] or "verbatim"),
                          knot_spacing=None, vel_scale=best.get("vel_scale"),
                          time_scale=best.get("time_scale"),
                          acc_scale=None, deviation_mm=best["dev"],
                          clock_gate_s=float(args.clock_min_s),
                          max_deviation_mm=float(args.max_deviation_mm),
                          prefer_deviation_mm=float(args.prefer_deviation_mm),
                          max_speed=(None if max_speed is None else float(max_speed)),
                          full_ladder=full, solver=_solver_policy(args), solver_ran=res["solver_ran"],
                          timing=(best.get("timing") or res.get("timing")), model=res.get("model"),
                          searched=[dict(
                              mode=c["mode"], vel_scale=c.get("vel_scale"),
                              time_scale=c.get("time_scale"),
                              duration_s=round(c["duration_s"], 3),
                              speed_vs_taught=round(speed_of(c), 4),
                              deviation_max_mm=(c["dev"]["max_mm"] if c.get("dev") else None),
                              arc_ratio=(c["dev"].get("arc_ratio") if c.get("dev") else None),
                              admissible=(None if c["gates"] is None else bool(c["gates"]["admissible"])),
                              in_cap=bool(c["in_cap"]),
                              gated=c["gates"] is not None,
                              priced=c.get("dev") is not None)
                              for c in sorted(res["candidates"], key=lambda c: c["duration_s"])],
                          what=("the fastest rung whose gates pass AND whose path deviation is "
                                "inside the cap. Two ladders are priced: DIAL 1 (uniform "
                                "re-time — points interpolated on the taught polyline, so the "
                                "path cannot move) and DIAL 2 (the solver's re-parameterisation, "
                                "which is faster where it is safe but can shortcut a "
                                "self-revisiting path); rung 0 is the recording verbatim. "
                                "`gated: false` / `priced: false` = not needed to settle the choice")))
    d = ct.disk_provenance(row["track"], build=False)
    row.update(track_sha256=d["track_sha256"], track_bytes=d["track_bytes"],
               points_on_disk=d["points_on_disk"], proj=d["proj"],
               proj_sha256=d["proj_sha256"], proj_bytes=d["proj_bytes"])
    row["proj_points"] = proj["points"]
    cert = certificate_for(row, best, verb=getattr(args, "_verb", "shape"),
                           clock_s=args.clock_min_s, allowance_deg=args.position_allowance_deg,
                           max_dev_mm=args.max_deviation_mm, prefer_dev_mm=args.prefer_deviation_mm,
                           n_ref=n_ref, frame=frame, model=res.get("model"),
                           timing=(best.get("timing") or res.get("timing")),
                           cal_lines=(np.asarray(res["cal_lines"], float)
                                      if res.get("cal_lines") else None))
    ct.save_taught_registry(reg)

    # the sidecar beside the admitted bytes: everything a reader needs to see
    # where these bytes came from, what they were screened at, and the certificate.
    sidecar = dict(
        kind="taught_shape", name=row["name"], slot=row["slot"], side=side,
        source=row["source"], mode=best["mode"], rung=best["rung"],
        vel_scale=best.get("vel_scale"), time_scale=best.get("time_scale"),
        ik_frame=ik_frame, frame=frame,
        clock_gate_s=float(args.clock_min_s), points=points,
        duration_band_s=row["band_5p5_5p6_s"], gates=best["gates"], deviation_mm=best["dev"],
        timing=(best.get("timing") or res.get("timing")), model=res.get("model"),
        emitter=("tools/taught_tracks.py shape — the execution-model re-time (the joint ratings "
                 "wherever the model allows, the model's arrival rate at every reversal and limit, "
                 "pauses removed, sections and dwells as listed; points on the source chords) or, "
                 "with --timing legacy, DIAL 1 uniform point-spacing re-time and DIAL 2 "
                 "plan_min_time LSQ B-spline + TOPP-RA + player-clock resample; the rung chosen is "
                 "the fastest whose enforced gates and path deviation pass"),
        written=datetime.datetime.now().isoformat(timespec="seconds"),
        certificate=cert)
    (REPO / row["track"]).with_suffix(".json").write_text(json.dumps(sidecar, indent=1))
    return cert


def _mark_nothing_admissible(reg: dict, row: dict, res: dict) -> None:
    g = dict(res.get("verbatim_gates") or row.get("gates") or {})
    g["reasons"] = [res.get("why") or "nothing admissible"]
    g["admissible"] = False
    row["gates"] = g
    ct.save_taught_registry(reg)


def _chosen_label(best: dict) -> str:
    if best["mode"] == "verbatim":
        return "verbatim (as taught)"
    if best["mode"] == "uniform":
        return f"uniform re-time, {best['time_scale']:.2f}x the taught speed"
    if best["mode"] == "guarded":
        enf = best["gates"]["enforced"]
        return (f"guarded re-time, {best['time_scale']:.2f}x the taught speed except within "
                f"{enf['position_allowance_deg']:g} deg of a limit, where J{enf['position_binding_joint']} "
                f"is held to {enf['near_limit_rate_deg_s']:.1f} deg/s (cap {enf['near_limit_rate_cap_deg_s']:g})")
    if best["mode"] == "model":
        enf = best["gates"]["enforced"]
        t = best.get("timing") or {}
        caps = (t.get("caps") or {})
        return (f"model re-time, up to {best['time_scale']:.2f}x the taught speed at the joint ratings; "
                f"{caps.get('capped', 0)} of {caps.get('reversals', 0)} arrivals capped by the execution "
                f"model (predicted deviation {enf.get('pred_dev_max_mm')} mm, predicted executed margin "
                f"{enf.get('pred_exec_margin_min_deg')} deg)"
                + (f"; {t['pauses_removed']['removed_lines']} pause lines removed" if t.get("pauses_removed") else ""))
    return f"re-parameterised at vel {best['vel_scale']:.2f}"


def cmd_shape(args) -> int:
    """Price rung 0 against the re-parameterised ladder; install the fastest
    rung whose gates pass and whose deviation is inside the cap, and CERTIFY it
    (`shape_search` + `shape_install`)."""
    reg = ct.load_taught_registry()
    row = find_row(reg, args.track, args.side)
    res = shape_search(row, args)
    if res["refused"]:
        return 4
    if res["best"] is None:
        _mark_nothing_admissible(reg, row, res)
        print(f"\n  ** NOTHING ADMISSIBLE — {res['why']}")
        print("  The recording is untouched; nothing was installed or certified.")
        return 4
    best = res["best"]
    cert = shape_install(reg, row, res, args)
    print(f"\n  chosen: {_chosen_label(best)}  {best['duration_s']:.3f} s  "
          f"deviation max {best['dev']['max_mm']:.2f} mm "
          f"(p95 {best['dev']['new_to_taught_mm']['p95_mm']:.2f} mm at "
          f"{best['dev']['frame']}){res['note']}")
    print(f"  installed as {row['track']}  ({row['points']} pts, {row['proj_bytes'] / 1024:.1f} KB "
          f".proj, sha {row['track_sha256'][:12]})")
    print(f"  certificate: {cert['verdict']}"
          + ("" if cert["verdict"] == "CERTIFIED" else
             " — failed: " + ", ".join(g["id"] for g in cert["gates"] if not g["passed"])))
    print(f"  ladder kept under {ct.repo_rel(pathlib.Path(res['out_dir']), REPO)}")
    return 0


# ─────────────────────────────────────────────────────────────── status
def cmd_status(args) -> int:
    reg = ct.load_taught_registry()
    rows = reg.get("tasks", [])
    print(f"taught lane  ({ct.repo_rel(ct.TAUGHT_REGISTRY, REPO)}; slots "
          f"{ct.TAUGHT_SLOT_RANGE[0]}-{ct.TAUGHT_SLOT_RANGE[1]}, CANFD-only "
          f"{ct.TAUGHT_CANFD_SLOT_MIN}+, {ct.SLOT_BASE}-{ct.SLOT_MAX} reserved for cleaning)")
    if not rows:
        print("  (no rows — `taught_tracks.py certify <recording.txt>`)")
        return 0
    print(f"{'slot':>4} {'name':34s} {'side':6s} {'pts':>6} {'band s':>16} {'margin':>8} "
          f"{'over':>5} {'dev max':>8} {'hardware':>10} {'mode':>15}  certificate")
    for t in sorted(rows, key=lambda r: (ct.SIDES.index(r.get("side", "right")), r["slot"])):
        g = (t.get("gates") or {}).get("enforced") or {}
        dev = ((t.get("shape") or {}).get("deviation_mm") or {}).get("max_mm")
        v = verify_certificate(t, deep=False)
        cert = "VALID" if v["ok"] else ("none" if v["cert"] is None else "INVALID")
        print(f"{t['slot']:>4} {t['name']:34s} {t.get('side', '?'):6s} {t.get('points', 0):>6} "
              f"[{t['band_5p5_5p6_s'][0]:6.2f},{t['band_5p5_5p6_s'][1]:6.2f}] "
              f"{g.get('position_margin_min_deg', float('nan')):>7.2f}d "
              f"{g.get('ticks_over_rating_band_limited', '?'):>5} "
              f"{(f'{dev:.2f}mm' if dev is not None else '-'):>8} "
              f"{str((t.get('source') or {}).get('hardware', '?')):>10} "
              f"{(t.get('shape') or {}).get('mode', '?'):>15}  {cert}")
        if not (t.get("gates") or {}).get("admissible"):
            for why in (t.get("gates") or {}).get("reasons") or ["never screened"]:
                print(f"       ** {why}")
        elif cert == "INVALID":
            for why in v["reasons"]:
                print(f"       ** certificate: {why}")
    if args.robot:
        from alix_dispatch import connect
        for side in ct.SIDES:
            rows_side = ct.rows_for(reg, side)
            if not rows_side:
                continue
            print(f"\n{side.upper()} controller — what its slots actually hold:")
            try:
                h = connect.open_arm(side)
            except Exception as exc:                                   # noqa: BLE001
                print(f"  ** no usable connection to the {side} arm ({exc!r})")
                continue
            if h is None:
                print(f"  ** no usable connection to the {side} arm")
                continue
            try:
                ret, items = tr.program_list(h.robot)
                have = {it.get("id"): it for it in items}
                print(f"  (program list ret={ret})")
                for t in rows_side:
                    if ct.canfd_only_slot(t["slot"]):
                        print(f"    slot {t['slot']:>3}: CANFD-only — no controller slot; stream it "
                              f"with --channel canfd")
                        continue
                    it = have.get(t["slot"])
                    if it is None:
                        print(f"    slot {t['slot']:>3}: EMPTY — `upload` it")
                        continue
                    sent = t.get("proj_bytes")
                    got = it.get("size")
                    delta = (int(got) - int(sent)) if (got is not None and sent) else None
                    flag = "" if delta is None or abs(delta) <= max(64, int(sent) // 1000) \
                        else "  ** SIZE MISMATCH (not these bytes)"
                    print(f"    slot {t['slot']:>3}: {it.get('name')!r} "
                          f"{(int(got) / 1024 if got is not None else 0):.1f}KB"
                          f"{f'  delta {delta:+d} B' if delta is not None else ''}{flag}")
            finally:
                connect.close_arm(h)
    return 0


# ─────────────────────────────────────────────────────────── the seam verbs
def _certified_or_refuse(rows: list, force: bool, verb: str) -> bool:
    """The lane's own gate in front of `upload`, `run` and `flow`: the row must be
    admissible AND its certificate must still hold for the bytes on disk.
    Returns True to proceed. `--force` proceeds on a failure and says so."""
    bad = []
    for r in rows:
        ok, why = _row_admissible(r)
        if ok:
            v = verify_certificate(r, deep=False)
            if not v["ok"]:
                ok, why = False, "certificate: " + "; ".join(v["reasons"])
        if not ok:
            bad.append((r, why))
    for r in rows:
        if all(r is not b[0] for b in bad):
            print(f"  certificate VALID  slot {r['slot']} {r['name']} ({r['side']} arm)")
    if bad and not force:
        for r, why in bad:
            print(f"** {r['name']}: {why}")
        return False
    if bad:
        print(f"!! --force: {verb} {len(bad)} row(s) that are NOT certified "
              f"(recorded in the run ledger's provenance by the {verb} itself)")
    return True


def _rows_from_tokens(reg: dict, tokens: list, side: str) -> list:
    rows = ([find_row(reg, t, side) for t in tokens] if tokens else
            (ct.rows_for(reg, side) if side else None))
    if rows is None:
        raise SystemExit("name what to send — a .proj file, a slot or a name — or give --side "
                         "for every row of that arm")
    if not rows:
        raise SystemExit(f"no taught rows for the {side} arm")
    sides = {r["side"] for r in rows}
    if len(sides) != 1:
        raise SystemExit(f"these rows belong to {sorted(sides)} — one arm per command")
    return rows


def cmd_upload(args) -> int:
    """`cleaning_tracks.py upload --taught`, behind the certificate: only bytes a
    certificate still vouches for reach a slot. Takes the certified `.proj`
    itself, or slots/names with `--side`."""
    reg = ct.load_taught_registry()
    rows = _rows_from_tokens(reg, args.track, args.side)
    if not args.track:                        # the whole side: CANFD-only rows are left out
        rows = ct.drop_canfd_only(rows, "upload")
        if not rows:
            raise SystemExit(f"no {args.side} row has a controller slot — nothing to upload")
    if not _certified_or_refuse(rows, args.force, "uploading"):
        return 4
    args.side = rows[0]["side"]
    args.track = [str(r["slot"]) for r in rows]
    args.taught = True
    return ct.cmd_upload(args)


def cmd_preflight(args) -> int:
    """`cleaning_tracks.py preflight --taught` — same checks, taught rows."""
    args.taught = True
    return ct.cmd_preflight(args)


def cmd_run(args) -> int:
    """`cleaning_tracks.py run --taught`, behind the certificate."""
    reg = ct.load_taught_registry()
    row = find_row(reg, args.track, args.side)
    if not _certified_or_refuse([row], args.force, "running"):
        return 4
    args.side = row["side"]
    args.track = str(row["slot"])
    args.taught = True
    return ct.cmd_run(args)


#: after the last line, the arm is still arriving; both channels are measured over
#: the same extra window so neither is credited with an arrival the other is not.
COMPARE_TAIL_S = 0.3
#: bins for the "where does it leave the path" comparison (lines), as for the profile
COMPARE_BIN = MEASURED_PROFILE_BIN


def channel_run_departure(rec: dict, q_cmd: np.ndarray, PC: np.ndarray, side: str, frame: str,
                          repo: pathlib.Path = None, tail_s: float = COMPARE_TAIL_S):
    """Pad-point departure [mm] from the commanded path for ONE run record, of
    EITHER channel, measured the same way: the window is the record's own
    `movej_s + dwell_s` -> `+ replay_s + tail_s` on the capture's clock (the
    upload record takes it from the controller's `move_through_joint` label,
    the CANFD record from the host's send times), then the same banded monotone
    alignment and windowed distance the measured profile uses. None when the
    record has no capture or no span."""
    repo = repo or REPO
    fp = rec.get("frames_file")
    if not fp or not (repo / fp).is_file() or rec.get("replay_s") is None:
        return None
    T, Q, _S = _capture_joints(repo / fp)
    if T.size < 50:
        return None
    t_s = T[0] + float(rec.get("movej_s") or 0.0) + float(rec.get("dwell_s") or 0.0)
    t_e = t_s + float(rec["replay_s"]) + float(tail_s)
    sel = (T >= t_s) & (T <= t_e)
    if sel.sum() < 50:
        return None
    N = q_cmd.shape[0]
    PE = pad_path_m_uncached(Q[sel], side, frame)
    k = _align_monotone(PE, PC)
    d = _windowed_dist_mm(PE, PC, np.clip(k, 0, N - 2))
    prof = np.zeros(N)
    np.maximum.at(prof, k, d)
    # PER LINE, NOT PER FRAME (2026-09-17). Percentiles over FRAMES are not comparable between the two
    # channels: the upload player HOLDS, so it spends more frames at the places where it held and those
    # places get more weight, while a host-timed stream spends one frame per line everywhere. Since the
    # holding sits on the near-stationary lines, where departure is usually small, frame weighting
    # flatters the channel that holds. The line-weighted figures below ask the same question of both —
    # "over the path, how far did the arm get from it" — and are the ones a channel comparison must use.
    # `*_frames_mm` are kept because every earlier number in the reports is frame-weighted.
    visited = np.zeros(N, dtype=bool)
    visited[np.clip(k, 0, N - 1)] = True
    pv = prof[visited]
    return dict(p50_mm=round(float(np.percentile(pv, 50)), 2),
                p95_mm=round(float(np.percentile(pv, 95)), 2),
                max_mm=round(float(pv.max()), 2),
                p50_frames_mm=round(float(np.percentile(d, 50)), 2),
                p95_frames_mm=round(float(np.percentile(d, 95)), 2),
                max_frames_mm=round(float(d.max()), 2),
                lines_visited=int(visited.sum()),
                reached=round(float(k.max()) / max(N - 1, 1), 4),
                frames=int(sel.sum()), profile=prof)


def _frame_held(rec: dict):
    g = rec.get("tool_frame_guard") or {}
    if g.get("selected"):
        return g["selected"]
    return ((rec.get("tool_frame_check") or {}).get("found") or {}).get("name")


def cmd_compare(args) -> int:
    """Upload vs CANFD on the SAME bytes: every complete-or-not run record of
    the row's current track, both channels, measured identically."""
    reg = (ct.load_experiment_registry() if getattr(args, "experiment", False)
           else ct.load_taught_registry())
    row = find_row(reg, args.track, args.side)
    side = row["side"]
    track = REPO / row["track"]
    sha = sha256_of(track)
    q = read_q_deg(track)
    N = q.shape[0]
    frame = args.pad_frame or f"{side_char(side)}_glove_frame_2"
    PC = pad_path_m(q, side, frame)
    recs = []
    for pat in ("run_slot*.json", "canfd_slot*.json"):
        for rp in sorted(REPO.glob(f"Resource/runs/tracks/*/{pat}")):
            if rp.name.endswith("_frames.ndjson"):
                continue
            try:
                rec = json.loads(rp.read_text())
            except Exception:                                          # noqa: BLE001
                continue
            if not isinstance(rec, dict) or rec.get("track_sha256") != sha or rec.get("side") != side:
                continue
            if str(rec.get("verdict", "")).startswith("refused"):
                continue
            # TWO CHANNELS, and anything that is not the stream is the player: a record whose
            # `channel` is absent, "upload", or some later spelling must not open a third bucket that
            # the per-channel comparison then reports as a channel of its own (2026-09-17 review, C7).
            ch = "canfd" if (rec.get("channel") == "canfd" or rp.name.startswith("canfd_")) else "upload"
            held = _frame_held(rec)
            if args.frame_held and held != args.frame_held:
                continue
            recs.append((rp, rec, ch, held))
    print(f"\n  {row['name']} (slot {row['slot']}, {side}) — {N} lines, sha {sha[:12]}; "
          f"departure at {frame}, window = the record's span + {args.tail_s:g} s")
    if not recs:
        print("  no run records of these bytes")
        return 1
    print(f"  {'record':32s} {'channel':7s} {'frame':10s} {'complete':8s} {'replay s':>8s} "
          f"{'ms/line':>7s} {'p50':>6s} {'p95':>6s} {'max':>6s} {'reach':>6s}  notes")
    by_ch: dict = {}
    table = []
    for rp, rec, ch, held in recs:
        m = channel_run_departure(rec, q, PC, side, frame, tail_s=args.tail_s)
        rs = rec.get("replay_s")
        ms = (rs / max(N - 1, 1) * 1e3) if rs else None
        c = rec.get("canfd") or {}
        notes = []
        if ch == "canfd":
            notes.append(f"late max {c.get('late_ms_max')} ms")
            if c.get("aborted"):
                notes.append(f"STOPPED: {c['aborted']}")
        codes = rec.get("codes_in_push") or {}
        if codes:
            notes.append("codes " + ",".join(sorted(codes)))
        print(f"  {rp.parent.name + '/' + rp.stem:32.32s} {ch:7s} {str(held):10.10s} "
              f"{str(rec.get('complete')):8s} {(f'{rs:8.2f}' if rs else '       -')} "
              f"{(f'{ms:7.3f}' if ms else '      -')} "
              + (f"{m['p50_mm']:6.2f} {m['p95_mm']:6.2f} {m['max_mm']:6.2f} {m['reached']:6.3f}"
                 if m else f"{'-':>6s} {'-':>6s} {'-':>6s} {'-':>6s}")
              + ("  " + "; ".join(notes) if notes else ""))
        entry = dict(record=f"{rp.parent.name}/{rp.name}", channel=ch, frame_held=held,
                     complete=rec.get("complete"), replay_s=rs, ms_per_line=ms,
                     canfd=({k: c.get(k) for k in ("period_s", "follow", "trajectory_mode",
                                                    "late_ms_max", "late_ms_mean",
                                                    "frames_late_over_period", "aborted")}
                            if ch == "canfd" else None),
                     codes_in_push=codes or None)
        if m:
            entry.update({k: v for k, v in m.items() if k != "profile"})
            if rec.get("complete"):
                by_ch.setdefault(ch, []).append((entry["record"], m["profile"]))
        table.append(entry)
    W = int(args.bin)
    nb = int(np.ceil(N / W))

    def binned(p):
        return np.array([p[i * W:min((i + 1) * W, N)].max() for i in range(nb)])

    summary = {}
    print(f"\n  WHERE the arm leaves the path, {W}-line bins, complete runs only "
          f"(Spearman of the binned worst departure)")
    for ch, runs in sorted(by_ch.items()):
        B = [binned(p) for _, p in runs]
        rhos = [_spearman(B[i], B[j]) for i in range(len(B)) for j in range(i + 1, len(B))]
        worst = np.maximum.reduce(B)
        top = [int(i) for i in np.argsort(worst)[::-1][:6]]
        summary[ch] = dict(runs=len(runs),
                           within_rho=(round(float(np.nanmedian(rhos)), 3) if rhos else None),
                           worst_bins=top,
                           worst_bins_mm=[round(float(worst[i]), 2) for i in top])
        print(f"    {ch:7s} {len(runs)} run(s)  run-to-run rho "
              f"{summary[ch]['within_rho']}  worst bins {top} = {summary[ch]['worst_bins_mm']} mm")
    if "upload" in by_ch and "canfd" in by_ch:
        Bu = [binned(p) for _, p in by_ch["upload"]]
        Bc = [binned(p) for _, p in by_ch["canfd"]]
        x = [_spearman(a, b) for a in Bu for b in Bc]
        shared = sorted(set(summary["upload"]["worst_bins"]) & set(summary["canfd"]["worst_bins"]))
        summary["cross_rho"] = round(float(np.nanmedian(x)), 3)
        summary["shared_worst_bins"] = shared
        print(f"    across channels: rho {summary['cross_rho']}  shared worst bins {shared} "
              f"({len(shared)} of 6)")
    if args.json:
        outp = pathlib.Path(args.json)
        outp.parent.mkdir(parents=True, exist_ok=True)
        outp.write_text(json.dumps(dict(row=row["name"], slot=row["slot"], side=side, sha256=sha,
                                        lines=N, pad_frame=frame, tail_s=args.tail_s, bin=W,
                                        runs=table, summary=summary), indent=1))
        print(f"\n  -> {outp}")
    return 0


def cmd_verify(args) -> int:
    """Re-check a certificate from the artifacts alone."""
    reg = ct.load_taught_registry()
    rows = _rows_from_tokens(reg, args.track, args.side)
    rc = 0
    for row in rows:
        v = verify_certificate(row, deep=not args.quick)
        cert = v["cert"] or {}
        print(f"\n  {row['name']}  (slot {row['slot']}, {row['side']} arm)")
        if cert:
            print(f"    issued {cert.get('issued')} by {cert.get('issued_by')} · rung "
                  f"{(cert.get('rung') or {}).get('label')} · "
                  f"{float((cert.get('rung') or {}).get('speed_vs_taught') or 0):.2f}x taught")
        for what in v["checked"]:
            print(f"    checked: {what}")
        if v["ok"]:
            print(f"    VALID — {cert['proj']['path']} (sha256 {cert['proj']['sha256'][:12]}) is the "
                  f"certified file")
        else:
            rc = 4
            for why in v["reasons"]:
                print(f"    ** {why}")
            print("    INVALID")
    return rc


# ─────────────────────────────────────────────────────────────── certify
def _hand_states(side: str) -> list:
    """The SRDF hand group states a recording may name (glove_frames.yaml's world)."""
    grp = f"inspire_hand_{side}"
    try:
        text = ct.SRDF.read_text()
    except Exception:                                                  # noqa: BLE001
        return []
    return sorted(set(re.findall(r'<group_state[^>]*name="([^"]+)"[^>]*group="%s"' % re.escape(grp), text)))


def _ask(question: str, choices=None, default=None, cast=str):
    while True:
        hint = f" [{'/'.join(choices)}]" if choices else ""
        dflt = f" (Enter = {default})" if default not in (None, "") else ""
        try:
            ans = input(f"    {question}{hint}{dflt}: ").strip()
        except EOFError:
            raise SystemExit("\n** no answer — pass the value as a flag instead")
        if not ans and default not in (None, ""):
            ans = str(default)
        if not ans:
            continue
        if choices and ans.lower() not in choices:
            print(f"    answer one of: {', '.join(choices)}")
            continue
        try:
            return cast(ans.lower() if choices else ans)
        except ValueError:
            print("    that is not a number")


def _store_source(src_in: pathlib.Path, sha: str) -> pathlib.Path:
    """Where this recording lives in source/: an identical file already there,
    else its own name, else `<stem>_<sha8>` — a source is never overwritten."""
    ct.TAUGHT_SOURCE_DIR.mkdir(parents=True, exist_ok=True)
    for p in sorted(ct.TAUGHT_SOURCE_DIR.glob("*.txt")):
        if p.resolve() == src_in or (p.stat().st_size == src_in.stat().st_size and sha256_of(p) == sha):
            return p
    dest = ct.TAUGHT_SOURCE_DIR / src_in.name
    if dest.exists():
        dest = ct.TAUGHT_SOURCE_DIR / f"{src_in.stem}_{sha[:8]}{src_in.suffix}"
    return dest


def certify_prepare(src_in: pathlib.Path, args) -> dict:
    """Everything `certify` does BEFORE the ladder: parse, refuse an installed rung,
    store the recording, resolve its provenance (flags, record, or a prompt), pick
    the slot, admit. Returns {"rc": 0, "row", "side", "slot", "stored", "sha"} or
    {"rc": 4}.

    ONE command: a taught .txt in, a gates-certified .proj out.

    admit + shape + certificate, with everything the lane needs resolved in
    one place: the recording's provenance (a `<stem>.SOURCE.json` beside it,
    exp01's MANIFEST, flags, or — once per new recording — a short prompt whose
    answers are saved), a slot (the one this recording already holds, else the
    first free taught slot on its arm), and the fastest rung that passes every
    gate. Nothing moves; nothing is uploaded."""
    try:
        lines = tr.parse_track(src_in)
    except ValueError as exc:
        print(f"** {src_in.name} is not a track the upload channel can carry: {exc}")
        return dict(rc=4)
    sha = sha256_of(src_in)
    reg = ct.load_taught_registry()
    installed = [r for r in reg.get("tasks", []) if r.get("track_sha256") == sha
                 and (r.get("source") or {}).get("sha256") != sha]
    if installed:
        r = installed[0]
        print(f"** {src_in.name} is the INSTALLED rung of slot {r['slot']} ({r['name']}), not a "
              f"taught recording — certifying it would speed up an already sped-up track. "
              f"Certify its source: {r['source']['path']}")
        return dict(rc=4)
    print(f"\n  {src_in.name}: {len(lines):,} lines of 7 integer millidegrees · sha {sha[:12]}")

    stored = _store_source(src_in, sha)
    prov = source_provenance(stored if stored.with_suffix(".SOURCE.json").is_file() else src_in)
    interactive = sys.stdin.isatty() and not args.yes
    side = args.side or prov.get("side")
    if args.side and prov.get("side") and args.side != prov["side"]:
        print(f"** its provenance says the {prov['side']} arm, but --side {args.side} was given — "
              f"refusing (a motion belongs to the arm it was taught on)")
        return dict(rc=4)
    hardware = "ok" if args.taught_here else prov.get("hardware") or "unknown"
    pole_m = (args.pole_mm / 1000.0) if args.pole_mm is not None else prov.get("pole_m")
    hand_state = args.hand_state or prov.get("hand_state")
    # THE SHEETS' OWN STEP, when it is given: "hand <pct> % (first 4 fingers)" —
    # the four finger channels at that percentage, the thumb HELD (None). It wins
    # over any stored record, because the sheet is the authority on the devices.
    if getattr(args, "hand_fingers_pct", None) is not None:
        pct = float(args.hand_fingers_pct)
        hand_state = args.hand_state or fingers_pct_label(pct)
        args.hand_counts = fingers_pct_counts(pct)
    # explicit ANGLE_SET counts (flow sheets say things like "close 50 %, first 4
    # fingers" that no SRDF state spells) win over the state's SRDF counts; the
    # state name is then only the label the row carries
    hand_counts = list(getattr(args, "hand_counts", None) or prov.get("hand_counts") or []) or None
    frame = args.frame or prov.get("frame")
    asked = {}
    missing = [k for k, v in (("side", side), ("hardware", None if hardware == "unknown" else hardware),
                              ("pole", pole_m), ("hand", hand_state or hand_counts)) if v is None]
    if missing and not interactive:
        flags = {"side": "--side left|right", "hardware": "--taught-here", "pole": "--pole-mm N",
                 "hand": "--hand-state NAME"}
        print(f"** {src_in.name} has no provenance for: {', '.join(missing)}. Pass "
              f"{' '.join(flags[m] for m in missing)} (or run in a terminal to be asked once).")
        return dict(rc=4)
    if missing:
        print(f"  no provenance saved for this recording — answer once "
              f"(kept in source/{stored.stem}.SOURCE.json):")
    if side is None:
        side = asked["side"] = _ask("arm it was taught on", choices=["left", "right"])
    if hardware == "unknown":
        here = _ask(f"taught on THIS robot's {side} arm (not a file from another mount)?",
                    choices=["y", "n"])
        hardware = asked["hardware"] = "ok" if here == "y" else "analysis_only"
    same_side = [r for r in reg.get("tasks", []) if r.get("side") == side and r.get("pre_stroke")]
    last = same_side[-1]["pre_stroke"] if same_side else {}
    if pole_m is None:
        dflt = None if last.get("pole_m") is None else f"{last['pole_m'] * 1000:g}"
        pole_m = asked["pole_m"] = _ask("pole height when taught, mm", default=dflt, cast=float) / 1000.0
    if hand_state is None:
        states = _hand_states(side)
        while True:
            hand_state = _ask(f"hand state when taught ({', '.join(states) or 'SRDF name'})",
                              default=last.get("hand_state"))
            if not states or hand_state in states or hand_counts:
                break
            print(f"    '{hand_state}' is not an SRDF hand state for the {side} arm")
        asked["hand_state"] = hand_state
    if hardware != "ok" and not args.allow_analysis_only:
        print(f"** provenance reads hardware: {hardware} — a recording from another arm mount, or of "
              f"unknown origin, cannot be certified for this robot")
        return dict(rc=4)
    states = _hand_states(side)
    if hand_counts and (len(hand_counts) != 6
                        or any(c is not None and not 0 <= int(c) <= 1000 for c in hand_counts)):
        print(f"** --hand-counts needs six ANGLE_SET counts in 0..1000 (or a held channel), "
              f"got {hand_counts}")
        return dict(rc=4)
    if states and hand_state not in states and not hand_counts:
        print(f"** hand state {hand_state!r} is not an SRDF hand state for the {side} arm "
              f"({', '.join(states)}) — or give its six counts with --hand-counts")
        return dict(rc=4)
    hand_state = hand_state or "custom"
    pole_m = float(pole_m)

    # the slot: the one this recording already holds, else --slot, else the first free
    lo, hi = ct.TAUGHT_SLOT_RANGE
    rows_side = [r for r in reg.get("tasks", []) if r.get("side") == side]
    mine = [r for r in rows_side if (r.get("source") or {}).get("sha256") == sha]
    if args.slot is not None:
        slot = int(args.slot)
        occupant = next((r for r in rows_side if r["slot"] == slot), None)
        if occupant and (occupant.get("source") or {}).get("sha256") != sha and not args.replace:
            print(f"** {side} slot {slot} holds {occupant['name']} (a different recording) — pick "
                  f"another slot, omit --slot to take the first free one, or --replace")
            return dict(rc=4)
    elif mine:
        slot = mine[0]["slot"]
    else:
        used = {r["slot"] for r in rows_side}
        free = [s for s in range(lo, hi + 1) if s not in used]
        if free:
            slot = free[0]
        else:
            # 40-59 full: a CANFD-only number (Newton 2026-09-22: CANFD is not limited to the
            # controller's 100 slots — nothing is uploaded)
            slot = next(s for s in itertools.count(ct.TAUGHT_CANFD_SLOT_MIN) if s not in used)
            print(f"   every taught slot {lo}-{hi} on the {side} arm is taken — slot {slot}, "
                  f"CANFD-only (nothing uploaded)")

    # keep the recording (never overwriting a different one) and its provenance —
    # a record is written only when none is reachable for the STORED file (typed
    # answers, flags, or a SOURCE.json that sat beside a file copied in from
    # elsewhere); an exp01 MANIFEST entry is the curated record and is not copied.
    if not stored.exists():
        shutil.copyfile(src_in, stored)
    prov_file = stored.with_suffix(".SOURCE.json")
    if source_provenance(stored).get("source_of_provenance") is None:
        record = dict(side=side, frame=frame, pole_m=pole_m, hand_state=hand_state,
                      hand_counts=(hand_counts or hand_counts_for(hand_state, side)), hardware=hardware,
                      taught_by=prov.get("taught_by"),
                      date=prov.get("date") or datetime.date.today().isoformat(), units="millideg",
                      note=(prov.get("note") or
                            f"declared at `taught_tracks.py certify` on "
                            f"{datetime.datetime.now().isoformat(timespec='seconds')}"
                            + (f" (answers: {sorted(asked)})" if asked else "")))
        prov_file.write_text(json.dumps(record, indent=1))

    # RE-CERTIFYING KEEPS THE ROW'S NAME. The name is part of the controller's
    # stored trajectory_name and of every run record's track path; deriving it
    # afresh from the file's stem renamed `46_taught_rim_top_left` (admitted with
    # --name) to `46_taught_rimtopte02_left` and deleted the old files — caught
    # on the first real re-certification, 2026-09-14.
    keep = next((r for r in mine if r["slot"] == slot), None)
    name = args.name or (keep["name"] if keep else None)
    admit_args = argparse.Namespace(
        file=stored, side=side, slot=slot, name=name, frame=frame, pole_m=pole_m,
        hand_state=hand_state, hand_counts=hand_counts, hardware=hardware,
        allow_analysis_only=args.allow_analysis_only, allow_slot=False,
        clock_min_s=args.clock_min_s, position_allowance_deg=args.position_allowance_deg)
    import contextlib
    import io
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        rc = cmd_admit(admit_args)
    if rc != 0:
        print(buf.getvalue())
        return dict(rc=rc)                    # every caller reads pre["rc"] (a bare int crashed)
    # admit's NOTE lines (e.g. a dropped move-to-rest on a replaced slot) must not be swallowed
    for line in buf.getvalue().splitlines():
        if line.strip().startswith("NOTE:"):
            print(line)
    reg = ct.load_taught_registry()
    row = next(r for r in reg["tasks"] if r.get("side") == side and r["slot"] == slot)
    print(f"  admitted as slot {slot} ({row['name']}) · pole {pole_m * 1000:g} mm · hand "
          f"{hand_state} · deviation measured at {row['source']['frame']}")

    return dict(rc=0, row=row, side=side, slot=slot, stored=stored, sha=sha)


def certify_report(row: dict, v: dict) -> int:
    bar = "  " + "─" * 72
    print(bar)
    if not v["ok"]:
        print(f"  NOT CERTIFIED  {row['name']}")
        for why in (row.get("gates") or {}).get("reasons") or v["reasons"] or ["see above"]:
            print(f"    ** {why}")
        print("  nothing to upload: `upload` and `run` refuse this slot (run --force is the recorded "
              "override)")
        return 4
    cert = v["cert"]
    gates = {g["id"]: g for g in cert["gates"]}
    rung = cert["rung"]
    print(f"  CERTIFIED  {cert['proj']['path']}")
    print(f"    slot {row['slot']} · {row['side']} arm · {rung['label']} · {cert['proj']['lines']:,} "
          f"lines · {rung['duration_nominal_s']:.2f} s (taught {rung['taught_nominal_s']:.2f} s, "
          f"{rung['speed_vs_taught']:.2f}x) · {cert['proj']['bytes'] / 1024:.1f} KB")
    print(f"    G1 margin {gates['G1']['value']:.2f}° ≥ {cert['position_allowance_deg']:g} · "
          f"G2 {gates['G2']['value']} ticks over rating · G3 step {gates['G3']['value']:.2f}° ≤ "
          f"{STEP_FAULT_DEG:g} · G4 deviation {gates['G4']['value']:.2f} mm ≤ {cert['max_deviation_mm']:g}")
    print(f"    certificate  {ct.repo_rel((REPO / row['track']).with_suffix('.json'), REPO)} · "
          f"proj sha256 {cert['proj']['sha256'][:12]}")
    if ct.canfd_only_slot(row["slot"]):
        # nothing to upload: the stream reads the .txt; the duration above is the upload player's
        # nominal clock, the stream runs one line per 5.0 ms
        print(f"    CANFD-only slot: {cert['proj']['lines'] * CLOCK_GATE_S:.2f} s at "
              f"{CLOCK_GATE_S * 1e3:g} ms/line on the stream")
        print(f"  next:  python3 tools/taught_tracks.py flow {row['slot']} --side {row['side']} "
              f"--channel canfd")
    else:
        print(f"  next:  python3 tools/taught_tracks.py upload {cert['proj']['path']}")
    return 0


def _shape_args_from(args, verb: str) -> argparse.Namespace:
    return argparse.Namespace(
        track=None, side=None, vel_scales=args.vel_scales, time_scales=args.time_scales,
        max_acc_util=args.max_acc_util, max_deviation_mm=args.max_deviation_mm,
        prefer_deviation_mm=args.prefer_deviation_mm, clock_min_s=args.clock_min_s,
        position_allowance_deg=args.position_allowance_deg, ik_frame=None, pick=args.pick,
        force=False, max_speed=args.max_speed, full_ladder=args.full_ladder,
        no_solver=getattr(args, "no_solver", False), solver=getattr(args, "solver", "auto"),
        near_limit_rate_deg_s=getattr(args, "near_limit_rate_deg_s", NEAR_LIMIT_RATE_DEG_S),
        hard_margin_deg=getattr(args, "hard_margin_deg", HARD_MARGIN_FLOOR_DEG),
        timing=getattr(args, "timing", "model"), model_file=getattr(args, "model_file", None),
        keep_pauses=getattr(args, "keep_pauses", False), section=getattr(args, "section", None),
        dwell=getattr(args, "dwell", None), limit_eps_deg=getattr(args, "limit_eps_deg", None),
        budget_percentile=getattr(args, "budget_percentile", None),
        exec_budget_mm=getattr(args, "exec_budget_mm", None),
        # this namespace is rebuilt FIELD BY FIELD, so every new shaping flag has to be
        # added here too or `certify` silently drops it — `--use-measured` was added on
        # 2026-09-16 and the first slot-49 certify came out byte-identical to slot 43
        # because of exactly that.
        use_measured=getattr(args, "use_measured", False),
        smooth_hz=getattr(args, "smooth_hz", None),
        _verb=verb)


#: `--use-measured` is implemented and measured but NOT wired, and it refuses BEFORE
#: anything is written. Placing the timing with the measurement works -- 49.99 s at a
#: calibrated worst of 19.56 mm against the model-placed 68.02 s whose real worst measured
#: 40.2-43.2 -- but it only works if the GATE also judges the calibrated number, and that
#: makes a certificate depend on run CAPTURES instead of on the files alone, which is the
#: property `verify` exists to provide. The half-wired version proved the point: it emitted
#: a file whose own G5 recomputed to 36.88 against a certificate claiming 19.64. Deciding
#: what a certificate may assert is Newton's call, not this tool's.
_USE_MEASURED_REFUSAL = (
    "--use-measured: the measurement can place the slowing, and does it well -- slot 43's "
    "two complete runs agree about WHERE at Spearman 0.643, and the calibrated estimate "
    "reads 41.5 mm on the installed file against a measured 40.2-43.2 where the raw "
    "prediction says 19.8. Acting on it also requires the GATE to judge that calibrated "
    "number, which makes the certificate depend on the run captures instead of on the "
    "files alone -- the property `verify` exists for. That is a decision about what a "
    "certificate asserts; make it first. `taught_tracks.py` exposes "
    "measured_departure_profile() and measured_calibration() for the analysis meanwhile.")


def _refuse_use_measured(args) -> None:
    """Refuse BEFORE the row is admitted: the first version raised inside the ladder,
    after `certify_prepare` had already written the row and its verbatim .txt/.proj,
    which left slot 49 half-created every time it was tried."""
    if bool(getattr(args, "use_measured", False)):
        raise SystemExit(_USE_MEASURED_REFUSAL)


def cmd_certify(args) -> int:
    """ONE command: a taught .txt in, a gates-certified .proj out — `certify_prepare`
    (parse, store, provenance, slot, admit), then `shape` (search + install +
    certificate). Nothing moves; nothing is uploaded."""
    src_in = pathlib.Path(args.file).expanduser().resolve()
    if not src_in.is_file():
        raise SystemExit(f"no such file: {src_in}")
    pre = certify_prepare(src_in, args)
    if pre["rc"]:
        return pre["rc"]
    shape_args = _shape_args_from(args, "certify")
    shape_args.track, shape_args.side = str(pre["slot"]), pre["side"]
    rc = cmd_shape(shape_args)
    reg = ct.load_taught_registry()
    row = next(r for r in reg["tasks"] if r.get("side") == pre["side"] and r["slot"] == pre["slot"])
    if getattr(args, "rest_around", False):
        row["rest"] = rest_block(True, True, at=(row.get("rest") or {}).get("at") or "rest", deg=(row.get("rest") or {}).get("deg"), pose_name=(row.get("rest") or {}).get("pose_name", ""))
        ct.save_taught_registry(reg)
    v = verify_certificate(row, deep=False) if rc == 0 else dict(ok=False, cert=None, reasons=[])
    return certify_report(row, v)


# ─────────────────────────────────────────────────────────────── screen / batch / flow
SCREEN_VERDICTS = ("READY", "RETIME", "NEEDS-PROVENANCE", "ALREADY", "REJECT")


# ─────────────────────────────────────────────────────────────── the rest verb
def rest_block(before: bool, after: bool, at: str = "rest", deg=None, pose_name: str = "") -> dict:
    """What a taught row records about moving to rest around its stroke. `flow` reads the pose at
    run time (`cleaning_tracks.taught_rest`) — at="rest": the side's move_to_rest from the cleaning
    plans; at="start": the row's own first line, the drag start pose (the flow sheet's "START POSE
    (Drag_start_pose)", Newton 2026-09-22); at="pose": the 7 joint angles *deg*, stored — for a
    recording that does NOT begin at its fixture's drag start pose (FILE_01.2 begins where FILE_01.1
    ends), so the arm still waits at that pose and `flow` bridges to the stroke's first line."""
    blk = dict(before=bool(before), after=bool(after))
    if at == "pose":
        if deg is None or len(deg) != 7:
            raise SystemExit("rest --at pose needs --deg with 7 joint angles")
        blk.update(at="pose", deg=[round(float(x), 3) for x in deg],
                   pose_name=pose_name or "a named pose",
                   pose_from=f"{pose_name or 'a named pose'}, stored in the row; movej at --rest-speed, "
                             f"then flow's bridge to the stroke's first line")
    elif at == "start":
        blk.update(at="start", pose_from="the stroke's own first line, the drag start pose, read "
                                         "from the certified .txt at run time; movej at --rest-speed")
    else:
        blk.update(pose_from="the side's move_to_rest, read from its cleaning plans at run time "
                             "(cleaning_tracks.side_rest_pose); movej at --rest-speed")
    blk["set"] = datetime.datetime.now().isoformat(timespec="seconds")
    return blk


def cmd_rest(args) -> int:
    """Newton, 2026-09-21: "append and prepend with a movej to rest". Set (or clear) the move to
    rest BEFORE and/or AFTER a taught row's stroke, at the pose `--at` names (the side's
    move_to_rest, the stroke's own first line, or a stored pose). Like `hand`, this is a runtime
    stage, not certified bytes: no track byte changes, the certificate holds, nothing is uploaded."""
    if not args.track and not args.all:
        raise SystemExit("name the rows (slots/names with --side), or --all for every row of --side")
    reg = ct.load_taught_registry()
    rows = _rows_from_tokens(reg, args.track, args.side)
    # EACH CALL SETS THE WHOLE SETTING (--before alone means before only), so what is printed is
    # what the row now holds.
    before = bool(args.around or args.before)
    after = bool(args.around or args.after)
    for row in rows:
        if before or after:
            row["rest"] = rest_block(before, after, at=args.at, deg=args.deg,
                                     pose_name=args.pose_name)
            pose = ct.taught_rest(row, "before" if before else "after")
            print(f"    {row['side']:5} slot {row['slot']:>3} {row['name']:38} rest "
                  f"{'before ' if before else ''}{'after' if after else ''}-> "
                  f"{[round(x, 2) for x in pose['deg']]}  ({pose['stage']})")
        else:
            # AN EXPLICIT OPT-OUT, not a removal: a taught row without a `rest` block takes both
            # legs by default (Newton, 2026-09-22), so "none" has to be written down
            row["rest"] = rest_block(False, False)
            print(f"    {row['side']:5} slot {row['slot']:>3} {row['name']:38} rest none (flow "
                  f"will not move to rest)")
    ct.save_taught_registry(reg)
    print(f"  {len(rows)} row(s) updated; no track byte changed, nothing uploaded")
    return 0


# ─────────────────────────────────────────────────────────────── the hand verb
def cmd_hand(args) -> int:
    """Correct what the lane will command the HAND to for rows already on the
    shelf — the flow sheet is the authority on the devices, and the hand is
    PROVENANCE, not certified bytes.

    The certificate covers the .proj, the .txt and the recording (G1-G5); it
    says nothing about the hand, so this rewrites the row's `source` and
    `pre_stroke` hand fields, the recording's own `SOURCE.json` record and the
    sidecar's copy, leaves every byte of the track alone, and re-verifies each
    certificate afterwards to prove it still holds. Nothing is uploaded and
    nothing moves."""
    reg = ct.load_taught_registry()
    rows = _rows_from_tokens(reg, args.track, args.side)
    if args.fingers_pct is None and not args.counts and not args.state:
        raise SystemExit("say what the hand should be: --fingers-pct (the sheets' own step), "
                         "--state <SRDF name>, or --counts <six>")
    state, counts = args.state, (list(args.counts) if args.counts else None)
    if args.fingers_pct is not None:
        state = state or fingers_pct_label(args.fingers_pct)
        counts = counts or fingers_pct_counts(args.fingers_pct)
    counts = counts or hand_counts_for(state, rows[0]["side"])
    if counts is None:
        raise SystemExit(f"cannot resolve counts for hand state {state!r} — give --counts")
    held = [i + 1 for i, c in enumerate(counts) if c is None]
    print(f"\n  hand -> {state} {counts}"
          + (f"   (channel(s) {held} are HELD: never commanded, left where they are)" if held else ""))
    rc = 0
    for row in rows:
        was = dict(state=(row.get("source") or {}).get("hand_state"),
                   counts=(row.get("source") or {}).get("hand_counts"))
        for block in ("source", "pre_stroke"):
            if isinstance(row.get(block), dict):
                row[block]["hand_state"] = state
                row[block]["hand_counts"] = list(counts)
        # The flow opens the hand before it moves the pole, and with no release
        # stage of its own a taught row fell back to the SRDF `release` — which
        # commands the thumb. The sheets' opening step is "hand open 100 % (first
        # 4 fingers)", so that is what the row now carries.
        if isinstance(row.get("pre_stroke"), dict):
            row["pre_stroke"]["hand_release_counts"] = fingers_pct_counts(100)
        src = REPO / str((row.get("source") or {}).get("path") or "")
        rec = src.with_name(src.stem + ".SOURCE.json") if src.name else None
        rec_note = "no SOURCE.json (provenance came from the exp01 MANIFEST)"
        if rec is not None and rec.is_file():
            d = json.loads(rec.read_text())
            d["hand_state"], d["hand_counts"] = state, list(counts)
            d.setdefault("amended", []).append(
                dict(when=datetime.datetime.now().isoformat(timespec="seconds"),
                     what="hand", was=was, now=dict(state=state, counts=list(counts)),
                     why="the flow sheet is the authority on the devices"))
            rec.write_text(json.dumps(d, indent=1) + "\n")
            rec_note = ct.repo_rel(rec, REPO)
        sidecar = (REPO / row["track"]).with_suffix(".json")
        if sidecar.is_file():
            d = json.loads(sidecar.read_text())
            if isinstance(d.get("source"), dict):
                d["source"]["hand_state"] = state
                d["source"]["hand_counts"] = list(counts)
                sidecar.write_text(json.dumps(d, indent=1))
        v = verify_certificate(row, deep=False)
        if not v["ok"]:
            rc = 4
        print(f"    {row['side']:5} slot {row['slot']:>3} {row['name']:38} "
              f"was {was['state']} {was['counts']}  ->  certificate "
              f"{'VALID (bytes untouched)' if v['ok'] else 'INVALID: ' + '; '.join(v['reasons'])}")
        print(f"          record: {rec_note}")
    ct.save_taught_registry(reg)
    print(f"  {len(rows)} row(s) updated; no track byte changed, nothing uploaded. "
          f"`flow` will command this hand for each of them, and open to "
          f"{fingers_pct_counts(100)} before any pole move.")
    return rc


# ─────────────────────────────────────────────────────────────── the model verb
def _model_from_args(args) -> dict:
    """The live model with the command line's overrides: the limit eps and the
    PERCENTILE the path budget is held at — p90 (the requirement's default: one
    reversal in ten may exceed the budget) or p50 (the typical reversal is inside
    it, one in ten may be up to 5x over). The certificate records which."""
    m = execution_model(getattr(args, "model_file", None))
    eps = getattr(args, "limit_eps_deg", None)
    if eps is not None:
        m["limit_eps_deg"] = float(eps)
    pct = str(getattr(args, "budget_percentile", None) or m.get("percentile") or "p90")
    if pct == "p50":
        m["tau_path_s"] = float(m.get("tau_path_p50_s") or DEFAULT_EXECUTION_MODEL["tau_path_p50_s"])
    m["percentile"] = pct
    return m


def refit_execution_model(log=print) -> dict:
    """R7: the model re-fitted from the run analyses on disk — tau_path = the 90th
    percentile of overshoot / executed arrival rate over the taught runs' turns
    at >= 25 deg/s (or the planned population's bucket p90, whichever is larger),
    tau_limit = the largest maximum / bucket-midpoint of the planned population
    at >= 75 deg/s, floor = the 90th percentile overshoot of the taught turns
    below 25 deg/s. Reads results.json / summary.json; nothing else."""
    m = dict(DEFAULT_EXECUTION_MODEL)
    fast, slow, srcs = [], [], []
    for rel in ("Resource/reports/2026-09-14_taught_track_lane/runs_2026-09-16/results.json",
                "Resource/reports/2026-09-14_taught_track_lane/runs_2026-09-15/results.json",
                "Resource/reports/2026-09-14_taught_track_lane/runs_2026-09-14/results.json"):
        p = REPO / rel
        if not p.is_file():
            continue
        n = 0
        for e in (json.loads(p.read_text()).get("events") or []):
            v, o = e.get("v_app_exec"), e.get("over")
            if v is None or o is None:
                continue
            if float(v) >= 25.0:
                fast.append(float(o) / float(v))
            else:
                slow.append(float(o))
            n += 1
        srcs.append(f"{rel} ({n} turns)")
    planned = REPO / "Resource/reports/2026-09-15_planned_task_runs/summary.json"
    tau_limit, tau_p90_planned = None, None
    if planned.is_file():
        t = json.loads(planned.read_text())["turns"]["planned"]
        mids = [(b, 0.5 * (float(b["lo"]) + float(b["hi"]))) for b in t["by_approach"] if b.get("hi")]
        lim = [float(b["over_max"]) / mid for b, mid in mids if float(b["lo"]) >= 75]
        p90 = [float(b["over_p90"]) / mid for b, mid in mids if float(b["lo"]) >= 25]
        tau_limit = max(lim) if lim else None
        tau_p90_planned = max(p90) if p90 else None
        srcs.append(f"{ct.repo_rel(planned, REPO)} ({t.get('n')} turns)")
    cands = ([float(np.percentile(fast, 90))] if fast else []) + ([tau_p90_planned] if tau_p90_planned else [])
    if cands:
        m["tau_path_s"] = round(max(cands), 3)
    if fast:
        m["tau_path_p50_s"] = round(float(np.percentile(fast, 50)), 3)
    if tau_limit:
        m["tau_limit_s"] = round(float(tau_limit), 3)
    if slow:
        m["floor_deg"] = round(float(np.percentile(slow, 90)), 2)
    m["fitted"] = datetime.date.today().isoformat()
    m["fitted_from"] = srcs
    log(f"  refit from {len(srcs)} analysis file(s): {len(fast)} taught turns >= 25 deg/s, "
        f"{len(slow)} below")
    return m


def cmd_model(args) -> int:
    m = execution_model(getattr(args, "model_file", None))
    print(f"execution model ({m.get('file', 'built-in defaults')}), fitted {m['fitted']}:")
    for k in ("tau_path_s", "tau_path_p50_s", "percentile", "tau_limit_s", "floor_deg", "limit_eps_deg",
              "pad_factor", "prominence_deg", "approach_window_s", "window_after_s"):
        print(f"  {k:<20} {m.get(k)}")
    for s in m.get("fitted_from", []):
        print(f"  from  {s}")
    if not args.refit:
        return 0
    r = refit_execution_model()
    print("refit:")
    for k in ("tau_path_s", "tau_path_p50_s", "tau_limit_s", "floor_deg"):
        flag = "" if abs(float(r[k]) - float(m[k])) < 1e-9 else f"   (was {m[k]})"
        print(f"  {k:<20} {r[k]}{flag}")
    if args.write:
        keys = [k for k in DEFAULT_EXECUTION_MODEL if k not in ("version", "fitted", "fitted_from")]
        changed = any(r[k] != m[k] for k in keys)
        out = {k: r[k] for k in DEFAULT_EXECUTION_MODEL}
        out["version"] = int(m.get("version", 1)) + (1 if changed else 0)
        EXECUTION_MODEL_FILE.write_text(json.dumps(out, indent=1) + "\n")
        print(f"  written {ct.repo_rel(EXECUTION_MODEL_FILE, REPO)} (version {out['version']}); certificates "
              f"issued under an earlier version stay judged by it — re-certify to adopt this one")
    return 0


def _expand_paths(tokens) -> list:
    """Recordings and/or directories of them -> unique files, in the order given
    (a directory's .txt files sorted by name; its .json / .md sidecars skipped)."""
    out, seen = [], set()
    for tok in tokens:
        p = pathlib.Path(tok).expanduser()
        files = sorted(p.glob("*.txt")) if p.is_dir() else [p]
        for f in files:
            f = f.resolve()
            if not f.is_file():
                raise SystemExit(f"no such file: {f}")
            if f not in seen:
                seen.add(f)
                out.append(f)
    return out


def screen_file(path: pathlib.Path, args, reg: dict) -> dict:
    """Pre-flight ONE recording in well under a second, writing nothing: format,
    the verbatim gates (G1 geometry decides refusal; G2/G3 only decide that the
    ladder must slow it down), provenance completeness, and whether the lane
    already holds it. The verdicts are the columns of `screen` and the triage
    of `batch`."""
    out = dict(file=str(path), name=path.name, verdict=None, notes=[], lines=None, taught_s=None,
               sha12=None, margin_deg=None, binding_joint=None, ticks_over=None, max_step_deg=None,
               side=None, slot=None)
    try:
        rows = tr.parse_track(path)
    except ValueError as exc:
        out.update(verdict="REJECT", notes=[f"not a track the upload channel can carry: {exc}"])
        return out
    sha = sha256_of(path)
    q = np.array(rows, dtype=float) / 1000.0
    out.update(lines=len(rows), taught_s=round((len(rows) - 1) * CLOCK_NOMINAL_S, 2), sha12=sha[:12])
    installed = [r for r in reg.get("tasks", []) if r.get("track_sha256") == sha
                 and (r.get("source") or {}).get("sha256") != sha]
    if installed:
        r = installed[0]
        out.update(verdict="REJECT", slot=r["slot"], side=r["side"],
                   notes=[f"this is the INSTALLED rung of slot {r['slot']} ({r['name']}), not a "
                          f"recording — certify its source {r['source']['path']}"])
        return out
    prov = source_provenance(path)
    side = args.side or prov.get("side")
    if args.side and prov.get("side") and args.side != prov["side"]:
        out.update(verdict="REJECT", notes=[f"its provenance says the {prov['side']} arm, --side "
                                            f"{args.side} was given"])
        return out
    hardware = "ok" if getattr(args, "taught_here", False) else prov.get("hardware") or "unknown"
    pole_m = (args.pole_mm / 1000.0) if getattr(args, "pole_mm", None) is not None else prov.get("pole_m")
    hand_state = (getattr(args, "hand_state", None)
                  or (fingers_pct_label(args.hand_fingers_pct)
                      if getattr(args, "hand_fingers_pct", None) is not None else None)
                  or prov.get("hand_state")
                  or (getattr(args, "hand_counts", None) and "custom") or None)
    out["side"] = side
    if hardware == "analysis_only" and not getattr(args, "allow_analysis_only", False):
        out.update(verdict="REJECT", notes=["provenance reads hardware: analysis_only (another arm "
                                            "mount) — cannot be certified for this robot"])
        return out
    v_near = float(getattr(args, "near_limit_rate_deg_s", None) or NEAR_LIMIT_RATE_DEG_S)
    hard_floor = float(getattr(args, "hard_margin_deg", None) or HARD_MARGIN_FLOOR_DEG)
    timing_mode = str(getattr(args, "timing", None) or "model")
    model = _model_from_args(args) if timing_mode == "model" else None
    frame = (getattr(args, "frame", None) or prov.get("frame")
             or f"{side_char(side or 'left')}_glove_frame_2")
    gates = gate_report(q, side or "left", args.clock_min_s, args.position_allowance_deg,
                        v_near=v_near, hard_floor=hard_floor, model=model, frame=frame,
                        budget_mm=float(getattr(args, "exec_budget_mm", None)
                                        or getattr(args, "max_deviation_mm", None) or MAX_DEVIATION_MM))
    enf = gates["enforced"]
    out.update(margin_deg=enf["position_margin_min_deg"], binding_joint=enf["position_binding_joint"],
               ticks_over=enf["ticks_over_rating_band_limited"], max_step_deg=enf["max_step_deg"],
               pred_dev_mm=enf.get("pred_dev_max_mm"), reversals=enf.get("reversals"))
    if enf["position_rule"] == "geometry":
        out.update(verdict="REJECT", notes=([r_ for r_ in gates["reasons"] if "GEOMETRY" in r_]
                                            or gates["reasons"]))
        return out
    if enf["position_rule"] == "floor":
        out.update(verdict="REJECT", notes=[f"joint-position margin {enf['position_margin_min_deg']:.2f}° "
                                            f"on J{enf['position_binding_joint']} is inside the "
                                            f"{hard_floor:g}° hard floor — geometry, no speed fixes it; "
                                            f"re-teach"])
        return out
    guard_note = []
    if model is not None:
        if enf["position_rule"] == "rate":
            w = enf.get("pred_exec_margin_worst") or {}
            guard_note.append(f"J{w.get('joint')} arrives at its limit at {w.get('arrival_deg_s')}°/s at the "
                              f"taught speed ({w.get('commanded_margin_deg')}° of margin): the model caps "
                              f"the arrival there and runs the rest at the ratings")
        if not enf.get("g5_ok", True):
            w = enf.get("pred_dev_worst") or {}
            guard_note.append(f"predicted deviation {enf['pred_dev_max_mm']:.1f} mm at the taught speed "
                              f"(line {w.get('line')}, J{'/J'.join(str(j) for j in w.get('joints', []))}): "
                              f"the model caps those arrivals")
    elif enf["position_rule"] in ("rate", "guarded"):
        guard_note = [f"J{enf['position_binding_joint']} comes {enf['position_margin_min_deg']:.2f}° from "
                      f"its limit (allowance {float(args.position_allowance_deg):g}°): the ladder will hold it to "
                      f"{v_near:g}°/s there (guarded rung) and run the rest faster"]
    mine = [r for r in reg.get("tasks", []) if (r.get("source") or {}).get("sha256") == sha
            and (side is None or r.get("side") == side)]
    if mine and not getattr(args, "recertify", False):
        r = mine[0]
        v = verify_certificate(r, deep=False)
        out.update(slot=r["slot"], side=r["side"])
        if v["ok"]:
            out.update(verdict="ALREADY", notes=[f"certified as slot {r['slot']} ({r['name']}), "
                                                 f"{(v['cert'].get('rung') or {}).get('label')} — "
                                                 f"--recertify to redo it"])
            return out
        out["notes"].append(f"slot {r['slot']} holds this recording but its certificate is not valid")
    missing = [k for k, v_ in (("side", side), ("hardware", None if hardware == "unknown" else hardware),
                               ("pole", pole_m), ("hand", hand_state)) if v_ is None]
    if missing:
        out.update(verdict="NEEDS-PROVENANCE",
                   notes=[f"no provenance for: {', '.join(missing)} (asked once in a terminal, or "
                          f"--side/--taught-here/--pole-mm/--hand-state)"] + out["notes"])
        return out
    if gates["admissible"]:
        out.update(verdict="READY", notes=["verbatim passes every gate; the ladder will only speed it up"]
                   + guard_note + out["notes"])
    else:
        out.update(verdict="RETIME", notes=(guard_note or [r_ for r_ in gates["reasons"]]) + out["notes"])
    return out


def _screen_line(sc: dict) -> str:
    v = sc["verdict"]
    left = f"  {sc['name'][:34]:<34} {v:<17}"
    if sc["lines"] is None:
        return left + " " + "; ".join(sc["notes"])
    mid = (f"{sc['lines']:>6} lines {sc['taught_s']:>7.2f} s  ")
    if sc["margin_deg"] is not None:
        mid += (f"margin {sc['margin_deg']:5.1f}° J{sc['binding_joint']}  over-rating {sc['ticks_over']:>4}  "
                f"step {sc['max_step_deg']:.2f}°  ")
    if sc.get("pred_dev_mm") is not None:
        mid += f"pred {sc['pred_dev_mm']:5.1f} mm at {sc.get('reversals', 0)} reversals  "
    return left + mid + "; ".join(sc["notes"])[:150]


def cmd_screen(args) -> int:
    """Triage a batch of recordings in seconds, writing nothing."""
    files = _expand_paths(args.paths)
    reg = ct.load_taught_registry()
    print(f"\n  screening {len(files)} file(s) at the {args.clock_min_s * 1e3:.2f} ms clock, "
          f"{args.position_allowance_deg:g}° allowance")
    counts = {v: 0 for v in SCREEN_VERDICTS}
    for f in files:
        sc = screen_file(f, args, reg)
        counts[sc["verdict"]] += 1
        print(_screen_line(sc))
    print("  " + " · ".join(f"{k} {n}" for k, n in counts.items() if n))
    print("  READY/RETIME/NEEDS-PROVENANCE certify with `batch <files> [--side S --taught-here "
          "--pole-mm N --hand-state NAME]`; REJECT cannot be certified; ALREADY is on the shelf")
    return 4 if counts["REJECT"] else 0


def _shape_worker(payload) -> dict:
    """Runs in a worker process (fork): the search only, with its log captured."""
    row, shape_args = payload
    logs = []
    try:
        res = shape_search(row, shape_args, log=logs.append)
    except (SystemExit, Exception) as exc:                             # noqa: BLE001
        res = dict(best=None, candidates=[], note="", why=f"{exc}", error=repr(exc), refused=False,
                   verbatim_gates=None, solver_ran=False)
    res["log"] = logs
    return res


def cmd_batch(args) -> int:
    """MANY recordings in one go: screen (seconds), certify the survivors with
    the ladder searches running in parallel, print one table, then — with
    --upload / --run — send the certified files to their arm and run them in
    ONE session (`flow --taught`). Flags for provenance apply to every file;
    a file's own SOURCE.json wins over them where they disagree on the arm."""
    import time
    files = _expand_paths(args.paths)
    if not files:
        raise SystemExit("no recordings given")
    t_all = time.time()
    reg = ct.load_taught_registry()
    print(f"\n  batch: {len(files)} file(s)")
    screened = []
    for f in files:
        sc = screen_file(f, args, reg)
        screened.append((f, sc))
        print(_screen_line(sc))
    todo = [f for f, sc in screened if sc["verdict"] in ("READY", "RETIME", "NEEDS-PROVENANCE")]
    if getattr(args, "recertify", False):
        todo += [f for f, sc in screened if sc["verdict"] == "ALREADY"]
    results_table = []
    for f, sc in screened:
        if f not in todo:
            results_table.append(dict(file=f, verdict=sc["verdict"], slot=sc["slot"], why="; ".join(sc["notes"])))
    if not todo:
        print("  nothing to certify")
        return 0 if all(sc["verdict"] != "REJECT" for _, sc in screened) else 4

    # prepare (store, provenance, slot, admit) — sequential: it writes the registry
    prepared = []
    for f in todo:
        pre = certify_prepare(f, args)
        if pre["rc"]:
            results_table.append(dict(file=f, verdict="NOT CERTIFIED", slot=None, why="refused before the ladder (see above)"))
            continue
        pre["file"] = f
        prepared.append(pre)
    if not prepared:
        _print_batch_table(results_table)
        return 4

    # search — the heavy part — in parallel, no registry access in the workers
    shape_args = _shape_args_from(args, "batch")
    jobs = max(1, min(int(args.jobs), len(prepared)))
    payloads = [(p["row"], shape_args) for p in prepared]
    t0 = time.time()
    print(f"\n  searching the ladder for {len(prepared)} recording(s) on {jobs} worker(s) "
          f"(solver {shape_args.solver})…")
    if jobs > 1:
        import multiprocessing
        with multiprocessing.get_context("fork").Pool(jobs) as pool:
            results = pool.map(_shape_worker, payloads)
    else:
        results = [_shape_worker(pl) for pl in payloads]
    t_search = time.time() - t0

    # install + certify — sequential: the registry and the sidecars
    reg = ct.load_taught_registry()
    certified = []
    for pre, res in zip(prepared, results):
        row = next(r for r in reg["tasks"] if r.get("side") == pre["side"] and r["slot"] == pre["slot"])
        if getattr(args, "verbose", False):
            print("\n".join(res.get("log") or []))
        if res.get("error") or res.get("refused") or res.get("best") is None:
            _mark_nothing_admissible(reg, row, res)
            if not getattr(args, "verbose", False):
                print("\n".join((res.get("log") or [])[-6:]))
            results_table.append(dict(file=pre["file"], verdict="NOT CERTIFIED", slot=row["slot"],
                                      why=res.get("why") or res.get("error") or "nothing admissible"))
            continue
        cert = shape_install(reg, row, res, shape_args)
        v = verify_certificate(row, deep=False)
        ok = v["ok"] and cert["verdict"] == "CERTIFIED"
        results_table.append(dict(file=pre["file"], verdict="CERTIFIED" if ok else "NOT CERTIFIED",
                                  slot=row["slot"], side=row["side"], name=row["name"],
                                  rung=cert["rung"]["label"], speed=cert["rung"]["speed_vs_taught"],
                                  duration=cert["rung"]["duration_nominal_s"], lines=cert["proj"]["lines"],
                                  gates={g["id"]: g["value"] for g in cert["gates"] if g["id"].startswith("G")},
                                  proj=cert["proj"]["path"], why="; ".join(v["reasons"])))
        if ok:
            certified.append(row)
    _print_batch_table(results_table)
    print(f"  search {t_search:.1f} s · total {time.time() - t_all:.1f} s")

    rc = 0 if len(certified) == len(prepared) and not any(sc["verdict"] == "REJECT" for _, sc in screened) else 4
    if not certified:
        return rc
    sides = sorted({r["side"] for r in certified})
    if getattr(args, "upload", False):
        for side in sides:
            rows = ct.drop_canfd_only([r for r in certified if r["side"] == side], "batch --upload")
            if not rows:
                continue
            print(f"\n  uploading {len(rows)} certified file(s) to the {side} arm")
            up = argparse.Namespace(track=[r["proj"] for r in rows], side=None, ip=args.ip, yes=args.yes,
                                    force=False)
            if cmd_upload(up) != 0:
                return 4
    if getattr(args, "run", False):
        channel = getattr(args, "channel", None) or "upload"
        for side in sides:
            rows = [r for r in certified if r["side"] == side]
            if channel != "canfd":
                rows = ct.drop_canfd_only(rows, "batch --run on the upload channel")
            if not rows:
                continue
            argv = ["flow", *[str(r["slot"]) for r in rows], "--side", side, "--taught",
                    "--channel", channel]
            if args.yes:
                argv.append("--yes")
            if args.ip:
                argv += ["--ip", args.ip]
            if getattr(args, "no_pause", False):
                argv.append("--no-pause")
            if getattr(args, "log_events", False):
                argv.append("--log-events")
            if getattr(args, "blocking_probe", False):
                argv.append("--blocking-probe")
            if getattr(args, "speed", None) is not None:
                argv += ["--speed", str(args.speed)]
            print(f"\n  running {len(rows)} certified file(s) on the {side} arm in one session: "
                  f"cleaning_tracks.py {' '.join(argv)}")
            if ct.main(argv) != 0:
                return 4
    return rc


def _print_batch_table(rows: list) -> None:
    print(f"\n  {'file':<34} {'verdict':<14} {'slot':>4} {'rung':<12} {'x taught':>8} {'dur s':>7} "
          f"{'lines':>6}  {'G1°':>6} {'G2':>3} {'G3°':>5} {'G4mm':>6}  note")
    for r in rows:
        name = pathlib.Path(str(r["file"])).name[:34]
        if r["verdict"] == "CERTIFIED":
            g = r["gates"]
            print(f"  {name:<34} {r['verdict']:<14} {r['slot']:>4} {r['rung']:<12} {r['speed']:>7.2f}x "
                  f"{r['duration']:>7.2f} {r['lines']:>6}  {g['G1']:>6.2f} {g['G2']:>3} {g['G3']:>5.2f} "
                  f"{g['G4']:>6.2f}  {r['proj']}")
        else:
            slot = "" if r.get("slot") is None else r["slot"]
            print(f"  {name:<34} {r['verdict']:<14} {slot:>4} {'':<12} {'':>8} {'':>7} {'':>6}  "
                  f"{'':>6} {'':>3} {'':>5} {'':>6}  {str(r.get('why', ''))[:110]}")


def cmd_flow_taught(args) -> int:
    """`cleaning_tracks.py flow --taught`, behind the certificate: several
    certified rows of ONE arm, one session, devices from each recording's own
    provenance, a keypress between tasks unless --no-pause."""
    reg = ct.load_taught_registry()
    rows = _rows_from_tokens(reg, args.track, args.side)
    if not args.track and (getattr(args, "channel", None) or "upload") != "canfd":
        rows = ct.drop_canfd_only(rows, "flow on the upload channel")
        if not rows:
            raise SystemExit(f"no {args.side} row has a controller slot — stream with --channel canfd")
    if not _certified_or_refuse(rows, args.force, "running"):
        return 4
    args.side = rows[0]["side"]
    args.track = [str(r["slot"]) for r in rows]
    args.taught = True
    return ct.cmd_flow(args)


# ─────────────────────────────────────────────────────────────── CLI
def _gate_args(p) -> None:
    p.add_argument("--smooth-hz", type=float, default=None,
                   help="low-pass the taught recording at its own clock (zero-phase Butterworth, "
                        "order 2) before pauses and timing — removes the line-to-line speed ripple "
                        "a drag-teach recording carries. G4 then holds the result within "
                        "--max-deviation-mm of the UNSMOOTHED recording (Newton 2026-09-17: 5 Hz, "
                        "10 mm)")
    p.add_argument("--vel-scales", type=float, nargs="*", default=VEL_SCALES_DEFAULT,
                   help=f"DIAL 2 rungs (default {VEL_SCALES_DEFAULT}); 1.0 is the time-optimal "
                        f"re-parameterisation. Walked fastest first; stops at the first that passes")
    p.add_argument("--time-scales", type=float, nargs="*", default=TIME_SCALES_DEFAULT,
                   help=f"DIAL 1 rungs: uniform re-time factors, played by point spacing alone "
                        f"(default {TIME_SCALES_DEFAULT}). New points are interpolated ON the taught "
                        f"polyline, so the path cannot move. {SLOW_SCALES_DEFAULT} are added only when "
                        f"no other rung passes")
    p.add_argument("--max-acc-util", type=float, default=MAX_ACC_UTIL_DEFAULT,
                   help=f"acceleration readback ceiling for the SOLVER, as a multiple of the "
                        f"10.472 rad/s^2 planning figure (default {MAX_ACC_UTIL_DEFAULT} = the "
                        f"measured 15.7 bound, which is advisory on this channel)")
    p.add_argument("--max-deviation-mm", type=float, default=MAX_DEVIATION_MM)
    p.add_argument("--prefer-deviation-mm", type=float, default=PREFER_DEVIATION_MM)
    p.add_argument("--clock-min-s", type=float, default=CLOCK_GATE_S)
    p.add_argument("--position-allowance-deg", type=float, default=POSITION_ALLOWANCE_DEG)
    p.add_argument("--near-limit-rate-deg-s", type=float, default=NEAR_LIMIT_RATE_DEG_S,
                   help=f"inside the allowance a joint may move at most this fast (default "
                        f"{NEAR_LIMIT_RATE_DEG_S:g}); the guarded rung enforces it")
    p.add_argument("--hard-margin-deg", type=float, default=HARD_MARGIN_FLOOR_DEG,
                   help=f"below this margin a file is refused whatever its speed (default "
                        f"{HARD_MARGIN_FLOOR_DEG:g})")
    p.add_argument("--timing", choices=("model", "legacy"), default="model",
                   help="model (default): the execution-model re-time — the joint ratings wherever the "
                        "model allows, its arrival rate at every reversal and limit, pauses removed; "
                        "legacy: the pre-2026-09-15 uniform/guarded ladder")
    p.add_argument("--model-file", default=None,
                   help=f"execution model to judge and shape with (default "
                        f"{ct.repo_rel(EXECUTION_MODEL_FILE, REPO)} when it exists, else the built-in "
                        f"defaults; `taught_tracks.py model` prints it)")
    p.add_argument("--limit-eps-deg", type=float, default=None,
                   help="override the model's limit_eps_deg: the executed extreme must keep this far "
                        "from a joint limit")
    p.add_argument("--exec-budget-mm", type=float, default=EXEC_BUDGET_MM,
                   help=f"the budget for the PREDICTED executed deviation at the reversals (G5, the "
                        f"overrun past a stroke end); default {EXEC_BUDGET_MM:g} mm — set it per file "
                        f"from the room beyond its stroke ends. G4, the bytes against the recording, "
                        f"keeps --max-deviation-mm either way")
    p.add_argument("--budget-percentile", choices=("p90", "p50"), default=None,
                   help="hold the deviation budget at the 90th percentile of the overshoot law "
                        "(default; one reversal in ten may exceed it) or at the median (p50: the "
                        "typical reversal is inside it, one in ten may be up to 5x over)")
    p.add_argument("--keep-pauses", action="store_true",
                   help="keep the recording's still stretches (R4 removes them by default)")
    p.add_argument("--section", action="append", default=None, metavar="A:B:F",
                   help="run the source stretch A..B (fractions 0-1 of the file, or line numbers) at "
                        "factor F (0 < F <= 1) of the fastest speed; repeatable")
    p.add_argument("--dwell", action="append", default=None, metavar="S:SEC",
                   help="hold still SEC seconds at station S (fraction 0-1 of the file, or a line "
                        "number) — the only stops the file will contain; repeatable")
    p.add_argument("--pick", default=None,
                   help="install THIS rung instead of the fastest admissible one: `verbatim`, "
                        "`vel:0.9`, `uniform:1.5`, `guard:1.5`, `model:2.4`, `slowest` or `fastest`")
    p.add_argument("--max-speed", type=float, default=None,
                   help="never pick a rung faster than this multiple of the taught speed "
                        "(e.g. 1.5)")
    p.add_argument("--solver", choices=("auto", "always", "never"), default="auto",
                   help=f"DIAL 2 policy: auto = only when the uniform ladder cannot reach "
                        f"{SOLVER_AUTO_BELOW:g}x (default); always = price it (~11 s per velocity "
                        f"scale); never = skip it")
    p.add_argument("--no-solver", action="store_true", help="= --solver never")
    p.add_argument("--full-ladder", action="store_true",
                   help="price every rung, not only the ones that settle the choice (for a report)")
    p.add_argument("--use-measured", action="store_true",
                   help="place the timing with the MEASURED departure of this row's installed "
                        "file instead of the model's predicted one, and hold the budget against "
                        "the calibrated estimate (prediction x measured/predicted). Needs >= 2 "
                        "COMPLETE runs of the installed bytes that AGREE about where the arm "
                        "departs (Spearman >= 0.35) and refuses otherwise, which is what keeps "
                        "it off the speeds where nothing reproduces. The model still supplies "
                        "the scaling with speed, so the result is a candidate to VERIFY BY "
                        "RUNNING, not a fact")


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)

    c = sub.add_parser("certify", help="ONE command: a taught .txt in, a gates-certified .proj out "
                                       "(no motion, nothing uploaded)")
    c.add_argument("file", type=pathlib.Path)
    c.add_argument("--side", choices=ct.SIDES, default=None,
                   help="the arm it was taught on (asked once when the recording has no provenance)")
    c.add_argument("--taught-here", action="store_true",
                   help="declare it was taught on THIS robot (hardware: ok) — asked when unknown")
    c.add_argument("--rest-around", action="store_true",
                   help="records rest BEFORE and AFTER the stroke in `flow` explicitly — it is "
                        "the DEFAULT for every taught row since 2026-09-22 (Newton), so it changes "
                        "nothing for a row that has not opted out; on a row that ran `rest --none` "
                        "it turns both legs back on (at the pose the row already holds)")
    c.add_argument("--pole-mm", type=float, default=None, help="pole height it was taught at, mm")
    c.add_argument("--hand-state", default=None, help="hand state it was taught with (SRDF name)")
    c.add_argument("--hand-counts", type=int, nargs=6, default=None,
                   help="the hand's six ANGLE_SET counts when taught, when no SRDF state spells it "
                        "(e.g. 516 516 516 516 981 992 = first four fingers half closed, thumb open)")
    c.add_argument("--hand-fingers-pct", type=float, default=None, metavar="PCT",
                   help="THE FLOW SHEETS' OWN STEP: the first four fingers at PCT %% of open "
                        "(100 = open, 50 = half closed) and the thumb left untouched — the counts "
                        "become PCT x 10 on channels 1-4 and HOLD on 5-6")
    c.add_argument("--frame", default=None, help="contact frame for the deviation figure "
                                                 "(default: the provenance's, else <S>_glove_frame_2)")
    c.add_argument("--slot", type=int, default=None,
                   help=f"taught slot (default: the one this recording holds, else the first free "
                        f"in {ct.TAUGHT_SLOT_RANGE[0]}-{ct.TAUGHT_SLOT_RANGE[1]}, else the first free "
                        f"CANFD-only one from {ct.TAUGHT_CANFD_SLOT_MIN}: no controller slot, nothing "
                        f"uploaded, --channel canfd only)")
    c.add_argument("--replace", action="store_true",
                   help="with --slot: replace a different recording that holds that slot")
    c.add_argument("--name", default=None, help="slug for the row/file (default: the file's)")
    c.add_argument("--allow-analysis-only", action="store_true",
                   help="admit a recording that is not `hardware: ok` (it can never be CERTIFIED)")
    c.add_argument("--yes", action="store_true",
                   help="never prompt: refuse when something is missing (automation)")
    _gate_args(c)
    c.set_defaults(fn=cmd_certify)

    sc = sub.add_parser("screen", help="pre-flight MANY recordings in seconds — format, geometry, "
                                       "rating, provenance, already-on-the-shelf — nothing written")
    sc.add_argument("paths", nargs="+", help="recordings and/or directories of them")
    sc.add_argument("--side", choices=ct.SIDES, default=None)
    sc.add_argument("--taught-here", action="store_true")
    sc.add_argument("--pole-mm", type=float, default=None)
    sc.add_argument("--hand-state", default=None)
    sc.add_argument("--hand-counts", type=int, nargs=6, default=None)
    sc.add_argument("--hand-fingers-pct", type=float, default=None, metavar="PCT")
    sc.add_argument("--allow-analysis-only", action="store_true")
    sc.add_argument("--recertify", action="store_true", help="do not report already-certified files as ALREADY")
    sc.add_argument("--clock-min-s", type=float, default=CLOCK_GATE_S)
    sc.add_argument("--position-allowance-deg", type=float, default=POSITION_ALLOWANCE_DEG)
    sc.add_argument("--near-limit-rate-deg-s", type=float, default=NEAR_LIMIT_RATE_DEG_S)
    sc.add_argument("--hard-margin-deg", type=float, default=HARD_MARGIN_FLOOR_DEG)
    sc.add_argument("--timing", choices=("model", "legacy"), default="model")
    sc.add_argument("--model-file", default=None)
    sc.add_argument("--limit-eps-deg", type=float, default=None)
    sc.add_argument("--budget-percentile", choices=("p90", "p50"), default=None)
    sc.add_argument("--max-deviation-mm", type=float, default=MAX_DEVIATION_MM)
    sc.add_argument("--exec-budget-mm", type=float, default=EXEC_BUDGET_MM)
    sc.add_argument("--frame", default=None)
    sc.set_defaults(fn=cmd_screen)

    rs = sub.add_parser("rest", help="which rest legs a taught row's stroke takes in `flow` — "
                                     "BOTH by default (Newton 2026-09-22); a runtime stage, not "
                                     "bytes")
    rs.add_argument("track", nargs="*", help="slots/names with --side, or .proj files")
    rs.add_argument("--side", choices=ct.SIDES, default=None)
    rs.add_argument("--all", action="store_true", help="every row of --side (never implied)")
    rg = rs.add_mutually_exclusive_group(required=True)
    rg.add_argument("--around", action="store_true", help="before AND after the stroke")
    rg.add_argument("--before", action="store_true", help="before the stroke only")
    rg.add_argument("--after", action="store_true", help="after the stroke only")
    rg.add_argument("--none", action="store_true",
                    help="opt the row out: flow moves to rest neither before nor after")
    rs.add_argument("--at", choices=("rest", "start", "pose"), default="rest",
                    help="WHICH pose: rest = the side's move_to_rest from the cleaning plans "
                         "(default); start = the stroke's own first line, the drag start pose the "
                         "recording began at (the flow sheet's START POSE, Newton 2026-09-22); "
                         "pose = the 7 angles of --deg (a recording that does not begin at the "
                         "fixture's drag start pose waits there all the same)")
    rs.add_argument("--deg", type=float, nargs=7, default=None, help="with --at pose: J1..J7, deg")
    rs.add_argument("--pose-name", default="", help="with --at pose: what the pose is, in words")
    rs.set_defaults(fn=cmd_rest)

    hd = sub.add_parser("hand", help="correct what the lane commands the HAND to for rows already "
                                     "on the shelf (provenance, not bytes — the certificate holds)")
    hd.add_argument("track", nargs="*", help="certified .proj files, or slots/names with --side; "
                                             "default: every row of --side")
    hd.add_argument("--side", choices=ct.SIDES, default=None)
    hd.add_argument("--fingers-pct", type=float, default=None, metavar="PCT",
                    help="THE FLOW SHEETS' OWN STEP: the first four fingers at PCT %% of open "
                         "(100 = open, 50 = half closed), the thumb left untouched")
    hd.add_argument("--state", default=None, help="the label the row carries (default: fingers_<PCT>pct)")
    hd.add_argument("--counts", type=int, nargs=6, default=None,
                    help="six ANGLE_SET counts outright, when no percentage or SRDF state spells it")
    hd.set_defaults(fn=cmd_hand)

    mo = sub.add_parser("model", help="the execution model the lane shapes and judges with: print it, "
                                      "refit it from the run analyses, write it")
    mo.add_argument("--refit", action="store_true",
                    help="recompute tau (path p90, limits max) and the floor from the run analyses on disk")
    mo.add_argument("--write", action="store_true", help=f"with --refit: write {ct.repo_rel(EXECUTION_MODEL_FILE, REPO)}")
    mo.add_argument("--model-file", default=None)
    mo.set_defaults(fn=cmd_model)

    b = sub.add_parser("batch", help="screen + certify MANY recordings (ladder searches in parallel), "
                                     "then --upload / --run them in ONE session")
    b.add_argument("paths", nargs="+", help="recordings and/or directories of them")
    b.add_argument("--side", choices=ct.SIDES, default=None, help="the arm every file was taught on")
    b.add_argument("--taught-here", action="store_true", help="every file was taught on THIS robot")
    b.add_argument("--pole-mm", type=float, default=None, help="pole height they were taught at, mm")
    b.add_argument("--hand-state", default=None, help="hand state they were taught with (SRDF name)")
    b.add_argument("--hand-counts", type=int, nargs=6, default=None,
                   help="six ANGLE_SET counts when no SRDF state spells the hand (label: --hand-state)")
    b.add_argument("--hand-fingers-pct", type=float, default=None, metavar="PCT",
                   help="the flow sheets' step: first four fingers at PCT %% of open, thumb untouched")
    b.add_argument("--frame", default=None)
    b.add_argument("--allow-analysis-only", action="store_true")
    b.add_argument("--recertify", action="store_true", help="redo files that are already certified")
    b.add_argument("--yes", action="store_true", help="never prompt (refuse instead); also skips the "
                                                    "upload/run confirmations")
    b.add_argument("--jobs", type=int, default=4, help="parallel ladder searches (default 4)")
    b.add_argument("--verbose", action="store_true", help="print every file's ladder")
    b.add_argument("--upload", action="store_true", help="then upload every certified file (one session per arm)")
    b.add_argument("--run", action="store_true", help="then run them in one session (flow --taught)")
    b.add_argument("--channel", choices=("upload", "canfd"), default="upload",
                   help="with --run: the channel (a CANFD-only row, slot 101+, runs on canfd only)")
    b.add_argument("--ip", default=None)
    b.add_argument("--no-pause", action="store_true", help="with --run: no keypress between tasks")
    b.add_argument("--log-events", action="store_true", help="with --run: record the SDK event stream")
    b.add_argument("--blocking-probe", action="store_true", help="with --run: blocking program start")
    b.add_argument("--speed", type=int, default=None, help="with --run: program speed (default 50)")
    b.add_argument("--slot", type=int, default=None, help=argparse.SUPPRESS)
    b.add_argument("--replace", action="store_true", help=argparse.SUPPRESS)
    b.add_argument("--name", default=None, help=argparse.SUPPRESS)
    _gate_args(b)
    b.set_defaults(fn=cmd_batch)

    fl = sub.add_parser("flow", help="run SEVERAL certified taught rows in one session "
                                     "(= cleaning_tracks flow --taught, behind the certificate)")
    fl.add_argument("track", nargs="*", help="certified .proj files, or slots/names with --side; "
                                             "default: every certified row of --side")
    fl.add_argument("--side", choices=ct.SIDES, default=None)
    fl.add_argument("--force", action="store_true", help="run rows that are not certified (recorded)")
    fl.add_argument("--start", default=None, help="begin at this slot")
    fl.add_argument("--repeat", type=int, default=1, help="passes over the list")
    fl.add_argument("--no-pause", action="store_true", help="no keypress between tasks")
    fl.add_argument("--no-retreat", dest="retreat", action="store_false")
    fl.add_argument("--no-devices", action="store_true",
                    help="never command the pole or the hand (bench / mock only)")
    ct._run_args(fl)          # noqa: SLF001
    fl.set_defaults(fn=cmd_flow_taught, retreat=True)

    cp = sub.add_parser("compare", help="upload vs CANFD runs of the SAME bytes, measured "
                                        "identically (departure, where, duration, clock)")
    cp.add_argument("track", help="the .proj/.txt, or a slot/name with --side")
    cp.add_argument("--side", choices=ct.SIDES, default=None)
    cp.add_argument("--experiment", action="store_true",
                    help="the row is in EXPERIMENT_TRACKS.json (slots 61-69), not the taught registry")
    cp.add_argument("--frame-held", default=None,
                    help="only runs supervised at this controller tool frame (e.g. Arm_Tip)")
    cp.add_argument("--pad-frame", default=None,
                    help="the point measured (default <L|R>_glove_frame_2, the pad)")
    cp.add_argument("--tail-s", type=float, default=COMPARE_TAIL_S)
    cp.add_argument("--bin", type=int, default=COMPARE_BIN)
    cp.add_argument("--json", default=None, help="also write the comparison here")
    cp.set_defaults(fn=cmd_compare)

    vf = sub.add_parser("verify", help="re-check a certificate from the files alone")
    vf.add_argument("track", nargs="*", help="the .proj file, or slots/names with --side")
    vf.add_argument("--side", choices=ct.SIDES, default=None)
    vf.add_argument("--quick", action="store_true",
                    help="skip re-measuring the deviation (G4) against the source")
    vf.set_defaults(fn=cmd_verify)

    a = sub.add_parser("admit", help="screen ONE taught recording and register it "
                                     "(rung 0: verbatim) — `certify` does this for you")
    a.add_argument("file", type=pathlib.Path)
    a.add_argument("--side", choices=ct.SIDES, required=True,
                   help="the arm it was taught on — DECLARED here, never guessed")
    a.add_argument("--slot", type=int, required=True,
                   help=f"taught slot: {ct.TAUGHT_SLOT_RANGE[0]}-{ct.TAUGHT_SLOT_RANGE[1]} (a "
                        f"controller slot too), or {ct.TAUGHT_CANFD_SLOT_MIN}+ (CANFD-only, nothing "
                        f"uploaded)")
    a.add_argument("--name", default=None, help="slug for the row/file (default: the file's)")
    a.add_argument("--frame", default=None, help="the contact frame every Cartesian figure "
                                                 "is stated at (default: the provenance's)")
    a.add_argument("--pole-m", type=float, default=None, help="pole height it was taught at")
    a.add_argument("--hand-state", default=None, help="hand state at the start (SRDF name)")
    a.add_argument("--hand-counts", type=int, nargs=6, default=None,
                   help="MODBUS ANGLE_SET vector, when it is known")
    a.add_argument("--hardware", default=None, choices=("ok", "analysis_only", "unknown"),
                   help="override the provenance's `hardware:` gate (recorded)")
    a.add_argument("--allow-analysis-only", action="store_true",
                   help="accept a recording whose provenance is not `ok` (recorded; refuses by "
                        "default because such a file was taught on ANOTHER arm mount)")
    a.add_argument("--rest-around", action="store_true",
                   help="records rest BEFORE and AFTER the stroke in `flow` explicitly — it is "
                        "the DEFAULT for every taught row since 2026-09-22 (Newton), so it changes "
                        "nothing for a row that has not opted out; on a row that ran `rest --none` "
                        "it turns both legs back on (at the pose the row already holds)")
    a.add_argument("--allow-slot", action="store_true",
                   help=f"accept a slot outside {ct.TAUGHT_SLOT_RANGE[0]}-"
                        f"{ct.TAUGHT_SLOT_RANGE[1]} and {ct.TAUGHT_CANFD_SLOT_MIN}+ (recorded; "
                        f"70-100 is always refused)")
    a.add_argument("--clock-min-s", type=float, default=CLOCK_GATE_S,
                   help=f"the FAST clock every rate/step gate is judged at (default "
                        f"{CLOCK_GATE_S * 1e3:.2f} ms: the upload player advances at most one point per "
                        f"5 ms cycle). Predictions stay at {CLOCK_PREDICT_S * 1e3:.2f} ms")
    a.add_argument("--position-allowance-deg", type=float, default=POSITION_ALLOWANCE_DEG,
                   help=f"position margin the recording must keep (default "
                        f"{POSITION_ALLOWANCE_DEG}, matching plan_min_time's own refusal)")
    a.set_defaults(fn=cmd_admit)

    s = sub.add_parser("shape", help="price the ladder for an admitted row and install + certify "
                                     "the fastest admissible rung — `certify` does this for you")
    s.add_argument("track", help="the row's .proj/.txt, or a slot/name with --side")
    s.add_argument("--side", choices=ct.SIDES, default=None)
    _gate_args(s)
    s.add_argument("--ik-frame", default=None, help="frame for the solver's Cartesian caps "
                                                    "(default: the declared contact frame)")
    s.add_argument("--force", action="store_true",
                   help="shape even a row whose provenance is not `hardware: ok` (NOT certified)")
    s.set_defaults(fn=cmd_shape)

    st = sub.add_parser("status", help="the lane's rows, gates, shape and certificates")
    st.add_argument("--robot", action="store_true", help="also list the controllers' slots")
    st.set_defaults(fn=cmd_status)

    u = sub.add_parser("upload", help="store certified rows in their slots (only_save — NO motion)")
    u.add_argument("track", nargs="*", help="the certified .proj file(s), or slots/names with --side; "
                                            "default: every row of --side")
    u.add_argument("--side", choices=ct.SIDES, default=None)
    u.add_argument("--ip", default=None)
    u.add_argument("--yes", action="store_true")
    u.add_argument("--force", action="store_true",
                   help="upload rows that are not certified (recorded)")
    u.set_defaults(fn=cmd_upload)

    p = sub.add_parser("preflight", help="controller's slots vs the taught rows (read-only)")
    p.add_argument("--side", choices=ct.SIDES, required=True)
    p.add_argument("--ip", default=None)
    p.set_defaults(fn=cmd_preflight)

    r = sub.add_parser("run", help="run ONE certified taught slot, watch to the end, verdict")
    r.add_argument("track", help="the certified .proj file, or a slot/name with --side")
    r.add_argument("--side", choices=ct.SIDES, default=None)
    r.add_argument("--force", action="store_true",
                   help="run though the row is not certified (recorded)")
    ct._run_args(r)          # adds --ip/--yes/--speed/... exactly as `run` expects  # noqa: SLF001
    r.set_defaults(fn=cmd_run)

    return ap


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    return args.fn(args)


if __name__ == "__main__":
    sys.exit(main())
