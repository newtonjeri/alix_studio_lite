#!/usr/bin/env python3
"""cleaning_tracks.py — the upload channel, end to end, one tool.

The pipeline (Newton, 2026-08-31, revised 2026-09-02): the upload channel
carries CLEANING STAGES ONLY (execute_path), emitted at the shortest time the
geometry allows under the channel's acceleration policy (below), stored in
controller slots 70-100 PER ARM — each arm's controller holds ITS OWN tasks
from 70 upward, +1 per task in schedule order (ruling 2026-09-02: "the left
arm plans are stored on the left arm controller from 70") — named from the
fixture's region keys. The task set is Newton's list (CLEANING_TASKS); hinge
and lid-seat rows are not in it and are marked EXCLUDED, never given a slot.
``Resource/tracks/<fixture>/MANIFEST.json`` is the single source of truth.

    # offline (this machine)
    tools/cleaning_tracks.py emit --side right            # 9 tracks, slots 70-78 + manifest
    tools/cleaning_tracks.py emit --side right --acc-scale 3.0
    tools/cleaning_tracks.py emit --side right --region bottomlid   # re-emit into its slot
    tools/cleaning_tracks.py status                       # manifest table + bands

    tools/cleaning_tracks.py provenance                   # pins vs the bytes on disk
    tools/cleaning_tracks.py provenance --write           # write the pins into the manifest
    tools/cleaning_tracks.py dry --side right --slot 60   # a zero-exposure in-place track

    # hardware (the robot host; every motion step confirmed + speed-restored)
    tools/cleaning_tracks.py upload --side right          # store the side's slots, no motion
    tools/cleaning_tracks.py preflight --side right       # controller slot list vs manifest
    tools/cleaning_tracks.py run bottomlid                # ONE slot: movej bridge + replay + verdict
    tools/cleaning_tracks.py flow                         # the whole right flow, task after task

PROVENANCE (B15, 2026-09-08). The controller can hold bytes that are not on
disk: `toplid_left` is 1891 points sha 8187aeca here, and all four recorded
runs of that slot played 2218 points sha 4e2ec971 — a file not in this tree —
while `track.preflight_slots` answered `ok=None` for it because it was also
the one row with no local `.proj`. So every emitted row now PINS its bytes:
`track_sha256`, `proj_sha256`, `proj_bytes` and the point count read back
through the uploader's own parser, plus `observed_runs` — which bytes the run
ledger says the slot has actually played. `emit` writes them (and builds the
`.proj`), `provenance --write` backfills them, `upload` refuses a row that no
longer hashes to its pin, `preflight` compares the controller's stored size
AND `trajectory_name` against the row, and `run` refuses a drifted row — AND
reads the slot back off the controller before it starts, because the disk pin
cannot see the arm: B15 was a file replaced on disk while the slot kept the
old bytes, and every disk-side check stays green through that.

IDENTITY (H25). `track.assert_side(side, h.side)` compares the string that
chose the IP with itself and can never fail. Slots 70..78 name DIFFERENT tasks
on the two arms, so the identity that can be wrong lives at the far end of the
cable: the controller's stored `trajectory_name` — and, on the two shared
slots whose names truncate identically at the vendor's 31 characters, the
stored size. That pair is what `preflight` asserts.

THE TOOL FRAME (H35, reframed 2026-09-09). TWO NAMESPACES MEET HERE AND THEY
ARE NOT REQUIRED TO AGREE:

  * a task's `ik_frame` (`R_glove_frame_1`, `L_glove_frame_2`, ...) selects
    the HAND STATE for the stroke out of `glove_frames.yaml` (`hand_state`:
    `open_tenth`, `half_grasp`, ...), along with the contact axis and pad
    footprint the stroke was authored against. Newton, 2026-09-09: "every task
    has an associated ik frame, we have already addressed that." SETTLED.
  * the controller's TOOL FRAME (`R_glove_2`, `Arm_Tip`) is a CONTROLLER-SIDE
    supervision setting. Nothing in this channel has ever called
    `rm_change_tool_frame`, so it is whatever the last session left.

WHAT THE TOOL FRAME CANNOT AFFECT. This channel replays a JOINT-SPACE track.
`build_project_file` writes one `{"point":[7 millidegrees]}` per line and the
controller's player streams those joint angles verbatim — the emulated player
does literally that (`emulated_program.py` -> `movej_canfd(deg)`). No pose, no
frame, no IK anywhere in the path. So the tool frame cannot change the
executed joint path. It is likewise not an input to ANY inter-arm clearance
figure: every number in `PAIR_VERDICTS.json` is produced by
`dual_arm_verdicts.py`, which feeds those same joint angles (plus the pole and
hand columns) through URDF FK (`link_world_transforms`) into FCL — that file
contains the string "tool_frame" zero times. A dual PAIRING decision therefore
does not depend on the tool frame at all, and the two arms carrying different
frames is not a reason to refuse a pair.

WHY THE TWO SIDES READ DIFFERENTLY, and it is not a finding of any kind
(Newton, 2026-09-10: *"It is not a hazard at all it is because the set
toolframe is the Armtip in the GUI so of course you expect such, if we set a
different frame of course we will get values associated with those frames"*).
The right pendant has `R_glove_2` selected and the left has `Arm_Tip`;
`Arm_Tip` IS the zero-offset frame, so 0.000 kg and a zero centroid are what
that selection means. The readback reports the operator's choice.

This was called a live hazard, then downgraded to a "WATCH-ITEM, not a live
hazard" on 2026-09-09, and that was still wrong — a note defending a setting
against the suspicion of being a defect keeps the suspicion alive. There is
nothing to defend and the 157-run argument that used to sit here has been
removed with it. Reporting a selected frame as a latent risk invents a defect
out of a setting, which is the same error as reading a task's `ik_frame` as a
motion constraint.

RECORDED, NEVER REFUSED (Newton, 2026-09-15: *"the frame should never be an
issue ... just collect data on the frame set, instead of blocking the run"*).
Each row still pins the frame ITS OWN runs were recorded under, and `run`,
`preflight` and the dual session gate still read the controller back and
compare — but a CHANGE since those runs is a recorded fact (the run record's
`tool_frame_check`, the reasons spelled out on the console), not a refusal.
Until 2026-09-15 it refused (`refused_tool_frame`): that stopped the day's
first right-arm taught flow on a pendant holding `Arm_Tip` instead of
`R_glove_2`, and the same flow then ran clean at `R_glove_2`. Nothing here
switches the frame — that stays the pendant's — and nothing here claims which
frame is "correct".

THE DRY VERB (H37). `dry --side S --slot N --points P --roll-deg 2`
synthesises a track that holds that side's own `move_to_rest` pose and dithers
one joint (default J7, the wrist roll) by at most the roll angle, at the
player clock, and uploads it through the same `build_project_file` /
`send_project` path. It is registered in `DRY_TRACKS.json`, NOT the manifest,
and defaults to slot 60 — below the reserved 70-100 range. `run --dry` and
`preflight --dry` resolve against it. This is the ladder's zero-exposure rung:
neither arm can leave rest by more than the roll angle, arithmetically.

ACCELERATION POLICY (F110, 2026-09-02). Every emitted track and every
ruckig_pro plan was bound by the 10.47 rad/s^2 (600 deg/s^2) joint
acceleration figure in joint_limits.yaml — the vendor's libalgo joint acc
limit for PLANNED moves — over ~half of every stroke, while the rated joint
VELOCITY (180/180/225x5 deg/s) bound at most 3 % of any stroke. The player
replays points verbatim with no rate limit, and hardware has traced spikes of
3.2x (drag-taught right_test) and 5.7x (H0) that figure without fault. The
trial matrix (Resource/reports/2026-09-02_upload_acc_policy) shows the nine
right tasks at 187 s -> 135 s (acc 2.0) -> 114 s (acc 3.0, where velocity
starts to bind everywhere). ACC_SCALE_DEFAULT is the channel policy; every
manifest row records the scale it was emitted at. Sustained operation above
the vendor's planned-move figure is UNPROVEN on hardware: go up the ladder one
task at a time with --record-joints and read the tracking evidence.

Durations: the player's clock is nominally ~5.5545 ms/pt but the SAME track
replays +-6% apart run to run (F108/F108b, hardware 2026-09-01; the UDP push
cadence meanwhile holds 5.008-5.013 ms, so this is the player, not our timing).
ACCEPTED as-is (Newton, 2026-09-01). A run therefore PASSES on what it must
actually prove — COMPLETENESS (the arm traversed the whole track) and the
clock landing inside the hardware envelope [5.4, 6.2] ms/pt — both measured
from the --record-joints capture; path-fidelity stats are reported alongside.
The old +-1% band survives in the manifest as an ADVISORY flag only.

THE FLOW (Newton, 2026-09-02). ``flow`` runs a side's tracks one after another
— in schedule order, or in the order the slots are given, repeats included —
conserving the working principle of every single run: an explicit movej bridge
to the track's first point at --movej-speed (default 80; 0 hands it back to the
controller's own implicit transit at --speed, default 50), then the stroke. Between tasks the devices are set from what
each PLAN had them at when its stroke began (manifest ``pre_stroke``, derived
at emit): the pole height (rm_set_lift_height at 100 %, feedback-verified) and
the hand state (ALL-MODBUS register writes, SPEED_SET 1000). Both come
OUTSIDE the program, before its movej. When the pole height changes the hand
is opened first (the plans' own choreography: hand_release -> pole ->
hand_grasp). The arm retreats along the plan's own first post-stroke move
(manifest ``retreat_deg``) so no device moves with the pad on the surface —
but only when the previous task did not park at rest: with the rest legs on
(the default) the park has already cleared the fixture, so the retreat is
skipped; --no-retreat disables it outright. A TAUGHT row also moves to rest
around its stroke by default (Newton, 2026-09-22): after this task's pole and
hand are set and before the bridge, and again after the replay, at
min(--rest-speed, --movej-speed) — joint-space movejs, NOT collision-checked;
--no-rest skips both. A row with a TCP budget (`tcp_gate`) has its executed
pad path measured against the raw recording after the run; a FAIL is rc 1.
A keypress gates each task (--no-pause for a continuous run), a failed verdict
stops the flow, and the flow is OFF-DRIVER only — the driver owns pole and hand
while it is up.
"""
from __future__ import annotations

import argparse
import csv
import datetime
import hashlib
import json
import pathlib
import re
import socket as _socket
import subprocess
import sys
import time

REPO = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "core/python/alix_dispatch/src"))
sys.path.insert(0, str(REPO / "external/butterfli-realman-alix/Python"))

FIXTURE = "commode_c"
TRACKS = REPO / "Resource/tracks" / FIXTURE
PLANS = REPO / "Resource/plans" / FIXTURE / "hardware"
SCHEDULE = REPO / "ros/orchestration/alix_orchestrator/config/schedule_commode.csv"
REGIONS = REPO / "ros/orchestration/alix_orchestrator/config/regions_commode.csv"
CFG_DIR = REPO / "ros/orchestration/alix_tasks/config/commode_cleaning" / FIXTURE
#: EVERY live home of a region's cleaning-points config, in deployment order. The orchestration config
#: is the deployed one and wins; the generator's output is the other live home and holds the regions the
#: orchestration tree has not taken a copy of. Read-only here — this tool never writes a task config
#: (they are outputs, and `alix_tasks/config/` is another session's to write).
CFG_SEARCH_DIRS = (CFG_DIR, REPO / "core/python/alix_taskgen/auto-generated_config" / FIXTURE)
SLOT_BASE, SLOT_MAX = 70, 100
SIDES = ("right", "left")

#: Newton's task list (2026-09-02), the cleaning tasks the upload channel
#: carries, per arm, in schedule order: toplid, side, bottomlid, seat_ring,
#: seat_ring_bottom, top ("commode top"), bowl_inside ring / rim / front (the
#: left arm's mirror of front is bowl_inside_back). Every other schedule row
#: (hinge areas, lid-seat rows, mechanism cells) is EXCLUDED — no slot.
#: Newton's task list. The three hinge-area regions were added 2026-09-04 and
#: sit MID-SCHEDULE (positions 9, 15 and 21 of the right column), so including
#: them renumbers every later slot — that is the convention working as ruled
#: ("+1 per task in schedule order"), not a bug, and it costs a full re-upload.
CLEANING_TASKS = ("toplid", "side", "hinge_area", "bottomlid", "seat_ring",
                  "lid_seat_hinge_area", "seat_ring_bottom", "top",
                  "seat_body_hinge_area", "bowl_inside_ring", "bowl_inside_rim",
                  "bowl_inside_front", "bowl_inside_back")

#: Channel acceleration policy (module docstring, F110). 1.0 reproduces the
#: 2026-09-01 tracks; 2.0 is the first rung of the hardware ladder; 3.0 is
#: where the rated joint velocity binds on every task.
ACC_SCALE_DEFAULT = 2.0
#: Velocity margin below the RATED joint speeds (180/180/225x5 deg/s). The
#: solver otherwise rides the limit exactly and integer rounding puts single
#: ticks a hair over it; the controller flags every such tick as error 4105
#: "speed out of limit" (latched for the rest of the run; hardware 2026-09-02:
#: every acc-2.0 track and the 2026-09-01 side track raised it, the 2026-09-01
#: toplid track at 92 % of the limit never did) and ended bottomlid's program
#: at 58 % of its track with the flag up. 0.97 keeps every commanded tick
#: under the limit (verified by the emit's own over-limit tick count).
VEL_SCALE_DEFAULT = 0.97
KNOT_SPACING_RAD = 0.03
#: Readback tolerances on the WRITTEN integers (band-limited at 20 Hz), as a
#: multiple of acc_scale: a bang-bang optimum's readback overshoots the solver's
#: acceleration by 12-28 % (F103) and its sampled jerk measures ~0.55 x acc_scale
#: of the config ceiling (matrix 2026-09-02) — neither is a robot property.
ACC_READBACK_TOL = 1.30
JERK_READBACK_TOL = 0.70
#: one-tick commanded speed-step self-check on the WRITTEN integers, deg/s,
#: PER UNIT of acc_scale (a bang-bang step is acc x clock: 3.3 deg/s at 1.0).
#: Good emitter output measures 3.06-3.6 per unit; the two genuinely defective
#: track classes measured 19.98 (v1 sampler bug) and 78.85 (F101 dwell) at 1.0.
TICK_STEP_PER_ACC_DEG_S = 10.0
#: THE EMIT CLOCK — every track `emit` writes is sampled at 5.0 ms, the CANFD stream period (Newton
#: 2026-10-02: "the rate should be 5ms, so the planner should emit at 5ms"). The production CANFD driver
#: refuses a track whose declared clock_s is not its 5.0 ms period and never re-times one
#: (docs/motion/CANFD_PRODUCTION_DRIVER.md §0), so every track sampled at the old 0.0055545 was refused on
#: the production channel. 5.5545 ms is the upload player's whole-run AVERAGE (F98; the 5 ms cycle plus its
#: holds), not a clock — that figure lives on as PLAYER_CLOCK_S below, for the upload lane's playback.
#: THE PARITY TWIN: `emit_fastpath` here is the byte-identity reference of the planner's C++
#: `track_emit::kClockS` (track_emit.hpp; tools/track_emit_parity.py) and of tools/plan_min_time.py CLOCK —
#: all three spell the same number. Rows emitted before 2026-10-02 are at 5.5545 ms and say so in their
#: sidecar's `clock_s`; `track_clock_s` reads it, so nothing here assumes a track's clock.
CLOCK_S = 0.005
#: The UPLOAD player's measured average pace, ms/pt (F98: 29.167 s / 5251 intervals; =
#: alix_dispatch.track.REPLAY_CLOCK_S). A file plays on that player at this pace WHATEVER its sample
#: clock. Used where this tool models upload playback (the `dry` verb's zero-exposure tracks), never to
#: sample a plan.
PLAYER_CLOCK_S = 0.0055545
#: JOINT-POSITION MARGIN AT EMIT = AN ADVISORY, NOT A GATE (Newton 2026-10-02, front_left: "Relax the
#: emit gate" — accept plans within 5 deg of a joint limit at emit time). Until then a track whose
#: closest approach to a joint-position limit was under 5 deg was REFUSED (emit_fastpath; the re-solve
#: route through plan_min_time's --min-position-margin-deg 5.0): front_left's plan takes J1 to
#: -177.284 deg, 0.716 deg inside the -178 deg limit (Resource/reports/2026-10-02_tracks5ms §3). Now a
#: track INSIDE the limits is emitted and the margin is RECORDED (joint, deg) in its sidecar and its
#: manifest row; a sample BEYOND a limit (margin < 0) is still refused — never accepted. The planner's
#: C++ track_emit already recorded instead of refusing (position_margin_ok, kPositionMarginGateDeg).
POSITION_MARGIN_ADVISORY_DEG = 5.0
#: ...and the floor that still refuses: a margin below this is a sample beyond a joint-position limit.
POSITION_MARGIN_REFUSE_BELOW_DEG = 0.0


def position_margin_record(pos_ok) -> dict:
    """The sidecar's record of a track's closest approach to a joint-position limit: per-joint margins
    (deg, plan_min_time.check_positions), the binding joint (1-based) and, under
    POSITION_MARGIN_ADVISORY_DEG, the advisory (Newton 2026-10-02 relaxed the emit gate to this)."""
    m = [float(x) for x in pos_ok]
    j = min(range(len(m)), key=lambda i: m[i])
    adv = None
    if m[j] < POSITION_MARGIN_ADVISORY_DEG:
        adv = dict(joint=f"J{j + 1}", margin_deg=round(m[j], 3), advisory_below_deg=POSITION_MARGIN_ADVISORY_DEG,
                   refused=False,
                   why=("inside the joint-position limits; the emit no longer refuses a margin under "
                        f"{POSITION_MARGIN_ADVISORY_DEG:g} deg (Newton 2026-10-02 'Relax the emit gate'); "
                        "recorded, not refused"))
    return dict(position_margin_min_deg=m[j], position_margin_min_joint=j + 1,
                position_margin_deg=[round(x, 4) for x in m], position_margin_advisory=adv)


#: THE ACCELERATION FLAG — AN ADVISORY, NEVER A GATE (Newton 2026-10-02, asked with the figures: "Keep advisory,
#: flag it": no re-timing, no emit refusal; record the acceleration figures per row and judge on the first
#: hardware runs). The measure is the CANFD-window agent's (Resource/reports/2026-10-02_canfd_window/S2_model/
#: sustained_acceleration.txt; `canfd_drift.f20_acceleration_advisory`, the same 20 Hz filter as its
#: `f20_dynamics`): the COMMANDED peak joint acceleration of the bytes, band-limited at 20 Hz, and the % of
#: MOVING lines (any joint > 1 deg/s) whose |acc| (max over joints) exceeds 600 / 900 deg/s^2. 900 deg/s^2 =
#: 15.7 rad/s^2, the CANFD bound the 2026-09-07 revised ruling made advisory; the corpus reads 0 % above it on
#: Newton's clean named runs, 6.3 % on slot 55 (degraded) and 19.6 % on slot 54 (failed), and the 26 production
#: 5.0 ms tracks of 2026-10-02 0.0-7.7 % (peaks 857-1400). `flagged` = any moving time above 900. Recorded in the
#: track's sidecar and its manifest row under this key by `emit`, and stamped onto existing rows by `accflag`.
ACC_ADVISORY_KEY = "acceleration_advisory"
ACC_ADVISORY_FLAG_DEG_S2 = 900.0
ACC_ADVISORY_RULE = ("Newton 2026-10-02 'Keep advisory, flag it': the commanded 20 Hz acceleration is RECORDED and "
                     "flagged, never refused and never re-timed; judged on the first hardware runs")
#: Where a track's % of moving time above ACC_ADVISORY_FLAG_DEG_S2 sits against the CANFD hardware corpus (MEASURED,
#: canfd_window S2_model/sustained_acceleration.txt): Newton's clean named runs 0.0 % (slots 52/53/56/58/59 and
#: 101-111), slot 55 DEGRADED at 6.3 %, slot 54 FAILED at 19.6 %. A reading aid for the first hardware runs.
ACC_ADVISORY_CORPUS_PCT_ABOVE = (("slot 55 (degraded)", 6.3), ("slot 54 (failed)", 19.6))


def acceleration_corpus_band(pct_above):
    """The corpus band of a % of moving time above 900 deg/s^2 (ACC_ADVISORY_CORPUS_PCT_ABOVE), or None."""
    if pct_above is None:
        return None
    (n55, p55), (n54, p54) = ACC_ADVISORY_CORPUS_PCT_ABOVE
    if pct_above <= 0.0:
        return "0 %: as the clean named CANFD runs"
    if pct_above < p55:
        return f"above the clean runs (0 %), below {n55} ({p55:g} %)"
    if pct_above < p54:
        return f"at or above {n55} ({p55:g} %), below {n54} ({p54:g} %)"
    return f"at or above {n54} ({p54:g} %)"


def acceleration_advisory(track, clock_s: float = None) -> dict:
    """The ACCELERATION FLAG of the bytes in `track` (see ACC_ADVISORY_KEY): measured on the integer millidegrees
    written to the file, at the track's own clock (`track_clock_s`: 5.0 ms since 2026-10-02, 5.5545 ms before).
    Pure: reads the file and its sidecar's clock, writes nothing, never raises a refusal."""
    import numpy as np
    from alix_dispatch import track as tr
    mdir = str(REPO / "tools" / "motion_analysis")
    if mdir not in sys.path:
        sys.path.insert(0, mdir)
    import canfd_drift as cdm
    clk = float(clock_s if clock_s is not None else track_clock_s(track))
    q = np.asarray(tr.parse_track(track), float) / 1000.0
    adv = cdm.f20_acceleration_advisory(q, clk)
    above = (adv.get("pct_moving_above") or {}).get(f"{ACC_ADVISORY_FLAG_DEG_S2:g}")
    flagged = None if above is None else bool(above > 0.0)
    return dict(advisory=True, refuses=False, flagged=flagged, flag_above_deg_s2=ACC_ADVISORY_FLAG_DEG_S2,
                corpus_band=acceleration_corpus_band(above), clock_s=clk, fc_hz=20.0, **adv,
                measure=("20 Hz band-limited commanded acceleration (canfd_drift.f20_acceleration_advisory): peak "
                         "over joints, and the % of moving lines (any joint > "
                         f"{cdm.ACC_ADVISORY_MOVING_DEG_S:g} deg/s) whose |acc| (max over joints) exceeds each "
                         "level; measured on the integer millidegrees written to the file"),
                reference=("corpus, % of moving time above 900 deg/s^2: clean named CANFD runs 0 %, slot 55 "
                           "(degraded) 6.3 %, slot 54 (failed) 19.6 % (2026-10-02_canfd_window "
                           "S2_model/sustained_acceleration.txt)"),
                rule=ACC_ADVISORY_RULE)


def acceleration_advisory_row(adv: dict) -> dict:
    """The manifest row's copy of `acceleration_advisory`: the figures, without the long texts (those live in
    the track's sidecar)."""
    keep = ("advisory", "refuses", "flagged", "flag_above_deg_s2", "corpus_band", "clock_s", "peak_deg_s2",
            "peak_joint", "p90_moving_deg_s2", "pct_moving_above", "moving_lines", "lines")
    return {k: adv.get(k) for k in keep} | {"rule": ACC_ADVISORY_RULE}


def stamp_acceleration_advisory(rows: list, write_sidecars: bool = False) -> list:
    """Record the ACCELERATION FLAG on EMITTED rows (and, with `write_sidecars`, in each track's sidecar) from the
    bytes already on disk — nothing is re-timed and no track byte changes. Returns one summary per row."""
    out = []
    for t in rows:
        if t.get("status") != "EMITTED" or not t.get("track"):
            continue
        track = REPO / t["track"]
        if not track.is_file():
            out.append(dict(name=t.get("name"), missing=str(t["track"])))
            continue
        adv = acceleration_advisory(track)
        t[ACC_ADVISORY_KEY] = acceleration_advisory_row(adv)
        if write_sidecars:
            sc_path = track.with_suffix(".json")
            sc = json.loads(sc_path.read_text()) if sc_path.is_file() else {}
            if sc.get(ACC_ADVISORY_KEY) != adv:
                sc[ACC_ADVISORY_KEY] = adv
                sc_path.write_text(json.dumps(sc, indent=1))
        out.append(dict(name=t.get("name"), slot=t.get("slot"), side=t.get("side"), flagged=adv["flagged"],
                        corpus_band=adv["corpus_band"], peak_deg_s2=adv["peak_deg_s2"], peak_joint=adv["peak_joint"],
                        pct_moving_above=adv["pct_moving_above"], p90=adv["p90_moving_deg_s2"]))
    return out


def track_clock_s(track, default: float = None) -> float:
    """The clock a track file was SAMPLED at: its sidecar's `clock_s` (every emitter writes one), else
    `default` (the emit clock). Since 2026-10-02 tracks on disk come at two clocks — 5.5545 ms before
    Newton's 5.0 ms ruling, 5.0 ms after — so a duration or a rate measured ON a track uses the track's
    own clock, never a module constant."""
    sc = pathlib.Path(track).with_suffix(".json")
    try:
        c = json.loads(sc.read_text()).get("clock_s")
        if c:
            return float(c)
    except (OSError, ValueError, AttributeError):
        pass
    return float(CLOCK_S if default is None else default)


def emit_flags(acc_scale: float, vel_scale: float = VEL_SCALE_DEFAULT) -> list:
    return ["--stages", "execute_path", "--acc-scale", f"{acc_scale:g}",
            "--vel-scale", f"{vel_scale:g}",
            "--knot-spacing", f"{KNOT_SPACING_RAD:g}",
            "--max-acc-util", f"{ACC_READBACK_TOL * acc_scale:.2f}",
            "--max-jerk-util", f"{JERK_READBACK_TOL * acc_scale:.2f}"]


# ----------------------------------------------------------------- inventory
def region_keys() -> dict:
    out = {}
    for row in csv.DictReader(l for l in REGIONS.open() if not l.startswith("#")):
        if row.get("region_id", "").strip().isdigit():
            out[int(row["region_id"])] = row["region_key"].strip()
    return out


def schedule_order() -> list:
    """Ordered (region_key, arm) tasks: schedule rows top-down, col1 before
    col2, chain (>) order within a cell; compound (C) mechanism cells are
    excluded by policy."""
    keys, order = region_keys(), []
    for line in SCHEDULE.open():
        line = line.strip()
        if not line or line.startswith("#") or line.startswith("col1"):
            continue
        for cell in line.split(","):
            for item in cell.split(">"):
                m = re.match(r"^(\d+)([RLC])$", item.strip().lstrip(">"))
                if not m or m.group(2) == "C":
                    continue
                order.append((keys.get(int(m.group(1)), f"region{m.group(1)}"),
                              "right" if m.group(2) == "R" else "left"))
    return order


def side_tasks(side: str) -> list:
    """(region_key, included) for ONE arm in schedule order; included = in
    Newton's task list. Slots are allocated over the included ones only."""
    return [(key, key in CLEANING_TASKS) for key, arm in schedule_order() if arm == side]


#: Which pad each region wipes with, when its own config does not say.
#:
#: The ik_frame is not decoration: it selects the PAD (frames 1/2 are the palm
#: pads, 35x80 mm; 3/4 the finger pads, 20x80 mm), it fixes the hand state
#: through glove_frames.yaml, and it is the frame every Cartesian number is
#: measured at. Only 2 of the 12 LEFT regions carry a config
#: (side_left, toplid_left), so the other ten silently took the default
#: `L_glove_frame_2` on the 2026-09-05 emit — giving hinge_area the palm pad
#: where its right counterpart uses the finger pad (frame_4), and both bowl
#: tasks the wrong pad too. A region's pad is a property of the SURFACE, not of
#: the arm, so the mirror counterpart is the right source: it is explicit,
#: reviewable, and it matches Newton's ruling that the left does exactly what
#: the right does.
MIRROR_FRAME_SOURCE = {
    # left region -> the RIGHT region whose pad it inherits
    "hinge_area": "hinge_area", "seat_ring": "seat_ring", "bottomlid": "bottomlid",
    "lid_seat_hinge_area": "lid_seat_hinge_area", "seat_ring_bottom": "seat_ring_bottom",
    "top": "top", "seat_body_hinge_area": "seat_body_hinge_area",
    "bowl_inside_ring": "bowl_inside_ring", "bowl_inside_rim": "bowl_inside_rim",
    # region 8 (left-only) is the bowl's BACK section; region 7 (right-only) is
    # its FRONT. The schedule pairs them, and they are the same surface class.
    "bowl_inside_back": "bowl_inside_front",
}


def ik_frame_for(key: str, arm: str, provenance: dict = None) -> str:
    """The region's declared ik_frame, side-corrected.

    A LEFT plan emitted against an R frame silently reports zero tool speed
    (audit 2026-08-31), so the side prefix is always forced from the arm.

    Resolution order, and it RECORDS which one was used: the region's own
    config; else the mirror counterpart's config; else refuse. The old code
    returned `<side>_glove_frame_2` with no signal at all, which is how ten
    left regions were emitted against the wrong pad.
    """
    pref = "R" if arm == "right" else "L"

    def _declared(k, a):
        # BOTH LIVE CONFIG ROOTS, in deployment order (2026-09-17). `CFG_DIR` holds 5 of the 24 emitted
        # regions' configs and the generator's `auto-generated_config` holds 11; `hinge_area_right`
        # exists ONLY in the second, so searching one root refused a region whose config is right there
        # and aborted a whole-side emit. Which root answered is recorded in the provenance.
        for root in CFG_SEARCH_DIRS:
            cfg = root / f"{k}_{a}_cleaning_points.yaml"
            if not cfg.exists():
                continue
            # strip comments first: a superseded `# ik_frame:` above the live one
            # would otherwise win the regex (rm_emulator.py:175-187 documents the
            # same hazard).
            text = "\n".join(ln.split("#", 1)[0] for ln in cfg.read_text().splitlines())
            m = re.search(r"ik_frame:\s*([RL]_glove_frame_\d)", text)
            if m:
                return m.group(1), root
        return None

    own = _declared(key, arm)
    if own:
        frame, root = own
        if provenance is not None:
            provenance.update(ik_frame_source="own_config", ik_frame_declared=frame,
                              ik_frame_config=str(root.relative_to(REPO)))
        return pref + frame[1:]
    other = "right" if arm == "left" else "left"
    mirror_key = MIRROR_FRAME_SOURCE.get(key, key) if arm == "left" else key
    mirrored = _declared(mirror_key, other)
    if mirrored:
        frame, root = mirrored
        if provenance is not None:
            provenance.update(ik_frame_source=f"mirror:{mirror_key}_{other}",
                              ik_frame_declared=frame,
                              ik_frame_config=str(root.relative_to(REPO)))
        return pref + frame[1:]
    raise SystemExit(
        f"{key}_{arm}: no ik_frame — neither {key}_{arm}_cleaning_points.yaml nor the "
        f"mirror {mirror_key}_{other} declares one in any of "
        f"{[str(r.relative_to(REPO)) for r in CFG_SEARCH_DIRS]}. The pad decides the hand state and "
        f"every Cartesian number; it is not something to default.")


#: The pad's own hand state, and the SRDF that gives it joint values.
GLOVE_FRAMES = (REPO / "core/python/alix_taskgen/src/alix_taskgen/config/glove_frames.yaml")
SRDF = (REPO / "ros/manipulation/alix_moveit_config/config/alix.srdf")
_HAND_CACHE: dict = {}


def hand_from_frame_release(arm: str):
    """The SRDF's own `release` state for this hand — the honest fallback when
    a plan authors no pre-stroke release."""
    return _srdf_hand_state("release", arm)


def hand_from_frame(frame: str, arm: str):
    """(state_name, six ANGLE_SET counts) for the pad this task wipes with.

    Newton's ruling, 2026-09-05: "use the glove_frames.yml states, it carries
    the latest correct changes". The hand pose is a property of the PAD, not of
    whatever hand stage a plan happens to carry before its stroke — and the two
    disagreed on 4 of 23 emitted tasks (bottomlid R/L stroked at `grasp` and
    hinge_area_left at `quarter_grasp`, bowl_inside_rim_left at `half_grasp`,
    where their frames declare `open_tenth`). glove_frames.yaml stores only the
    NAME; the finger values are read live from the SRDF, so editing the state
    there reflects here.
    """
    key = (frame, arm)
    if key in _HAND_CACHE:
        return _HAND_CACHE[key]
    import yaml
    from alix_dispatch import peripherals as per
    frames = yaml.safe_load(GLOVE_FRAMES.read_text())["frames"]
    row = frames.get(frame)
    if not row or not row.get("hand_state"):
        raise SystemExit(f"glove_frames.yaml declares no hand_state for {frame!r}")
    out = _srdf_hand_state(row["hand_state"], arm)
    _HAND_CACHE[key] = out
    return out


def _srdf_hand_state(state: str, arm: str):
    """(state, six ANGLE_SET counts) for an SRDF hand group state."""
    key = ("srdf", state, arm)
    if key in _HAND_CACHE:
        return _HAND_CACHE[key]
    from alix_dispatch import peripherals as per
    grp = f"inspire_hand_{arm}"
    m = re.search(r'<group_state[^>]*name="%s"[^>]*group="%s"[^>]*>(.*?)</group_state>'
                  % (re.escape(state), re.escape(grp)), SRDF.read_text(), re.S)
    if m is None:
        raise SystemExit(f"alix.srdf has no group_state {state!r} for {grp!r}")
    rad = {jm.group(1): float(jm.group(2))
           for jm in re.finditer(r'joint\s+name="([^"]+)"\s+value="([^"]+)"', m.group(1))}
    out = (state, per.counts_from_rad(rad, arm))
    _HAND_CACHE[key] = out
    return out


def side_of(name: str) -> str:
    return "right" if name.endswith("_right") else "left"


def load_manifest() -> dict:
    p = TRACKS / "MANIFEST.json"
    if not p.exists():
        raise SystemExit(f"no manifest at {p} — run `cleaning_tracks.py emit --side <arm>` first")
    return json.load(open(p))


def save_manifest(man: dict) -> None:
    man["fixture"] = FIXTURE
    man["convention"] = (
        "save_id = slot, PER ARM: each arm's controller holds its own tasks in 70..100, "
        "+1 per task in schedule_commode.csv order (col1 before col2, chain order within a "
        "cell), over Newton's task list only (CLEANING_TASKS; other rows EXCLUDED); "
        "name = <fixture>_<region_key>_<arm>; execute_path ONLY — transits stay on the "
        "ROS controllers; pole/hand are set from pre_stroke by `flow` before each run")
    man["emitter"] = (
        "tools/plan_min_time.py " + " ".join(emit_flags(ACC_SCALE_DEFAULT)) +
        " (vel 1.0, no CANFD rate cap — upload channel only; --acc-scale per side, "
        "recorded per row); raw tick-step self-check <= "
        f"{TICK_STEP_PER_ACC_DEG_S:g} x acc_scale deg/s; per-region plan/acc_scale/ik_frame "
        "overrides from EMIT_OVERRIDES.json")
    man["clock_note"] = (f"tracks are SAMPLED at the emit clock, {CLOCK_S * 1e3:g} ms = the CANFD stream "
                         "period (Newton 2026-10-02); each row's sidecar carries its own clock_s (rows "
                         "emitted before 2026-10-02 are at 5.5545 ms). The UPLOAD player plays any file at "
                         "its own pace: executed window = (points-1) x [5.5, 5.6] ms, and that pace drifts "
                         "— judge out-of-band upload runs as ratios vs a same-session reference")
    man["tasks"].sort(key=lambda t: (SIDES.index(t.get("side", side_of(t["name"]))),
                                     t.get("slot") if t.get("slot") is not None else 999,
                                     t["name"]))
    json.dump(man, open(TRACKS / "MANIFEST.json", "w"), indent=1)


def rows_for(man: dict, side: str, emitted_only: bool = True) -> list:
    rows = [t for t in man["tasks"] if t.get("side", side_of(t["name"])) == side]
    if emitted_only:
        rows = [t for t in rows if t["status"] == "EMITTED"]
    return sorted(rows, key=lambda t: t["slot"] if t.get("slot") is not None else 999)


def by_name(man: dict, token: str, side: str) -> dict:
    """Resolve ONE emitted manifest row of *side* by slot number, region key,
    or full name. Slots repeat across arms, so the side is always required."""
    rows = rows_for(man, side)
    if token.isdigit():
        hits = [t for t in rows if t["slot"] == int(token)]
    else:
        hits = [t for t in rows if t["name"] == token or t["name"] == f"{FIXTURE}_{token}"
                or t["name"] == f"{FIXTURE}_{token}_{side}"]
        if not hits:
            hits = [t for t in rows if token in t["name"]]
    if len(hits) != 1:
        raise SystemExit(f"'{token}' matches {len(hits)} emitted {side} tracks — use the slot "
                         f"number or the region key (see: cleaning_tracks.py status)")
    return hits[0]


# --------------------------------------------------------------- provenance
# B15 (2026-09-08 audit). The controller can hold bytes that are not on disk
# and nothing noticed: `toplid_left` sits on disk as 1891 points sha 8187aeca
# with NO .proj, while all four recorded runs of that slot played 2218 points
# sha 4e2ec971 — a file that is not in this tree. `track.preflight_slots`
# returns ok=None for a row with no local .proj and only counts `ok is False`,
# so the stored-size gate was blind exactly where it mattered. The fix is to
# PIN the bytes: emit records the sha of the track AND of the .proj it builds,
# `upload` refuses a slot whose on-disk track no longer hashes to the pinned
# value, and `preflight` compares the controller's stored size and
# trajectory_name against the pinned row.
#
#: The per-row provenance fields. `points_on_disk` is counted independently of
#: `points` so a manifest row that drifted from its file is caught by the row
#: itself, not by a downstream band.
PROVENANCE_FIELDS = ("track_sha256", "track_bytes", "points_on_disk",
                     "proj", "proj_sha256", "proj_bytes")


def repo_rel(path, repo: pathlib.Path = REPO) -> str:
    """Repo-relative when it can be, absolute otherwise. Manifest rows are
    repo-relative; a synthesised track under a scratch directory is not, and
    must not raise on the way past."""
    path = pathlib.Path(path)
    try:
        return str(path.relative_to(repo))
    except ValueError:
        return str(path)


def sha256_of(path) -> str:
    """Hex sha256 of a file, or None when it is not there."""
    path = pathlib.Path(path)
    return hashlib.sha256(path.read_bytes()).hexdigest() if path.is_file() else None


def build_proj_for(track: pathlib.Path) -> dict:
    """Build (or rebuild) the .proj beside *track*. `build_project_file` is a
    pure function of the track's bytes — header + CRLF terminators — so this is
    deterministic and re-running it is a no-op when nothing changed. Emit calls
    it so a row can never again reach the controller without a local .proj to
    compare against (B15: toplid_left had none, and that is what blinded the
    size gate)."""
    from alix_dispatch import track as tr
    track = pathlib.Path(track)
    return tr.build_project_file(track, track.with_suffix(".proj"))


def disk_provenance(track_rel, repo: pathlib.Path = REPO, build: bool = False) -> dict:
    """What is ACTUALLY on disk for one row: hashes, sizes, and the point count
    read back through the same parser `upload` uses. Never consults the
    manifest — this is the measurement the manifest is compared against."""
    from alix_dispatch import track as tr
    t = pathlib.Path(track_rel)
    if not t.is_absolute():
        t = repo / t
    out = dict(track=str(track_rel), track_exists=t.is_file(), track_sha256=None,
               track_bytes=None, points_on_disk=None, parse_error=None,
               proj=None, proj_exists=False, proj_sha256=None, proj_bytes=None)
    if not out["track_exists"]:
        return out
    out["track_sha256"] = sha256_of(t)
    out["track_bytes"] = t.stat().st_size
    try:
        out["points_on_disk"] = len(tr.parse_track(t))
    except Exception as exc:                                           # noqa: BLE001
        out["parse_error"] = str(exc)[:200]
    proj = t.with_suffix(".proj")
    if build and out["parse_error"] is None:
        build_proj_for(t)
    out["proj"] = repo_rel(proj, repo)
    out["proj_exists"] = proj.is_file()
    if out["proj_exists"]:
        out["proj_sha256"] = sha256_of(proj)
        out["proj_bytes"] = proj.stat().st_size
    return out


def check_row_provenance(row: dict, repo: pathlib.Path = REPO, need_proj: bool = False) -> dict:
    """Is the file on disk still the file this row was pinned to?

    Returns {'ok', 'reasons', 'warnings', 'disk'}. A REASON is a refusal; a
    WARNING is a statement of fact the operator must read. An UNPINNED row —
    one emitted before the pins existed — is a refusal too: 'we do not know'
    is not 'it is fine' (B15 is exactly the case where nobody knew).
    """
    disk = disk_provenance(row.get("track"), repo)
    res, warn = [], []
    if not row.get("track"):
        return dict(ok=False, reasons=["the row carries no track path"], warnings=[], disk=disk)
    if not disk["track_exists"]:
        res.append(f"the track file is missing: {row['track']}")
        return dict(ok=False, reasons=res, warnings=warn, disk=disk)
    if disk["parse_error"]:
        res.append(f"the track file does not parse: {disk['parse_error']}")
    pinned = row.get("track_sha256")
    if not pinned:
        res.append("no track_sha256 in the manifest — this row is UNPINNED "
                   "(run `cleaning_tracks.py provenance --write`)")
    elif pinned != disk["track_sha256"]:
        res.append(f"track sha MISMATCH: manifest {pinned[:8]} vs disk "
                   f"{(disk['track_sha256'] or '')[:8]} — the file changed under the row")
    if (row.get("points") is not None and disk["points_on_disk"] is not None
            and int(row["points"]) != disk["points_on_disk"]):
        res.append(f"point count MISMATCH: manifest {row['points']} vs disk "
                   f"{disk['points_on_disk']}")
    pinned_proj = row.get("proj_sha256")
    if disk["proj_exists"] and pinned_proj and pinned_proj != disk["proj_sha256"]:
        res.append(f".proj sha MISMATCH: manifest {pinned_proj[:8]} vs disk "
                   f"{(disk['proj_sha256'] or '')[:8]}")
    if pinned_proj and not disk["proj_exists"]:
        warn.append("the .proj this row was pinned to is GONE from disk — `upload` rebuilds "
                    "it from the track, and preflight falls back to the pinned proj_bytes, "
                    "but nothing local can be compared against the controller until it does")
    if need_proj and not disk["proj_exists"]:
        res.append("no local .proj — nothing to compare the controller's stored "
                   "size against (this is the B15 blind spot); `upload` writes it")
    obs = row.get("observed_runs") or {}
    if obs.get("other_bytes"):
        warn.append(
            f"{obs.get('n_runs_other', 0)} recorded run(s) of this slot played bytes that are "
            f"NOT on disk ({', '.join(str(o['points']) + ' pts ' + o['track_sha256'][:8] for o in obs['other_bytes'])})"
            f" — the disk bytes have {obs.get('n_runs_on_disk', 0)} recorded run(s)")
    if obs.get("never_run_on_disk_bytes"):
        warn.append("NO recorded run has ever played the bytes now on disk for this row "
                    "(B15) — treat the first run of it as a first run, not a repeat")
    elif obs and obs.get("clean_single_arm_run_on_disk_bytes") is False:
        warn.append(
            f"the bytes on disk have run {obs.get('n_runs_on_disk')} time(s) and NOT ONE "
            f"completed ({obs.get('verdicts_on_disk')}) — this row has no clean single-arm "
            f"run, so it is not ready to be paired with a live partner (B15)")
    return dict(ok=not res, reasons=res, warnings=warn, disk=disk)


def pairing_gate(row: dict) -> dict:
    """MAY this row be run beside a live partner yet? (B15, for lane E.)

    The design's own precondition: "leg B stays refused until one clean
    single-arm run on the disk bytes exists". A run COUNT cannot answer it —
    two of `toplid_left`'s four runs cut — so the question is asked of
    `complete`, the finish-event label (F123), against the sha now on disk.
    A row that has never been pinned, or whose file no longer matches its pin,
    is refused here too: a pairing verdict computed from bytes nobody can
    identify certifies nothing.

    Deliberately NOT wired into `run` or `flow`: a single-arm run is exactly
    how a row EARNS its clean run, and a gate that refused it could never be
    satisfied. It is the dual executive's to call.
    """
    obs = row.get("observed_runs") or {}
    chk = check_row_provenance(row, REPO)
    reasons = list(chk["reasons"])
    if not obs:
        reasons.append("the row carries no observed_runs block — run "
                       "`cleaning_tracks.py provenance --write` before pairing it")
    elif not obs.get("clean_single_arm_run_on_disk_bytes"):
        # THE REMEDY DEPENDS ON WHICH THING IS MISSING, and this used to name
        # only one of them. `observed_runs` is the manifest's PINNED SNAPSHOT,
        # refreshed by `provenance --write`, not a live read of the ledger — so
        # a row that has just earned its clean run still reads as unpaired here
        # until the record is refreshed. Newton hit exactly that on 2026-09-10:
        # he ran `toplid_left` to completion on the disk bytes (the ledger shows
        # 1 run, in_band) and this gate answered "run it alone, to completion",
        # which is the one thing he had already done. Both remedies are named
        # now, in the order they apply.
        reasons.append(
            f"no clean single-arm run exists on the bytes now on disk "
            f"({obs.get('n_runs_on_disk', 0)} run(s), {obs.get('n_clean_runs_on_disk', 0)} "
            f"complete; {obs.get('n_runs_other', 0)} run(s) were on OTHER bytes) — run it "
            f"alone, to completion, before it is paired with a live partner (B15). "
            f"These counts are the manifest's PINNED SNAPSHOT, not a live read: if the row "
            f"has since had that clean run, the snapshot is simply stale — refresh it with "
            f"`cleaning_tracks.py provenance --write` and ask again.")
    return dict(ok=not reasons, name=row.get("name"), slot=row.get("slot"),
                reasons=reasons, warnings=chk["warnings"],
                n_clean_runs_on_disk=obs.get("n_clean_runs_on_disk"),
                track_sha256=chk["disk"]["track_sha256"])


def provenance_gate(rows: list, repo: pathlib.Path = REPO, need_proj: bool = False,
                    log=print) -> bool:
    """Run `check_row_provenance` over rows; print; True when all pass."""
    ok = True
    for r in rows:
        c = check_row_provenance(r, repo, need_proj=need_proj)
        for w in c["warnings"]:
            log(f"   !! {r.get('name')}: {w}")
        for why in c["reasons"]:
            log(f"   ** {r.get('name')} (slot {r.get('slot')}): {why}")
        ok = ok and c["ok"]
    return ok


# ------------------------------------------------- the controller's own name
#: How many characters of `rm_trajectory_data_t.trajectory_name` survive.
#: MEASURED, 2026-09-10, on the right controller (V1.7.4) — the first program
#: list ever fetched from an arm. `70_70_commode_c_toplid_right_exe` is exactly
#: 32 characters with no NUL, so the field is 32 usable, not 31. Everything
#: that truncates an expected name must use THIS, so the hardware fact has one
#: home.
NAME_FIELD_CHARS = 32


def expected_trajectory_name(row: dict) -> str:
    """What the controller stores as `trajectory_name` for this row.

    MEASURED 2026-09-10, and it is NOT what this function used to return.
    The controller prepends the program id to the basename of the file it was
    sent:

        trajectory_name = (f"{id}_" + <sent basename, extension replaced by .txt>)[:32]

    Proof, from the right arm's own program list — the first one ever fetched
    from hardware:
      * `id 90: 90_project.txt`. Lane G sent a project file whose basename does
        NOT begin with "90", and the controller stored `90_` in front of it. So
        the prefix is the ID, not something in our filename.
      * THE EXTENSION IS THE CONTROLLER'S, NOT OURS, and the SIZE is what
        proves it. Slot 60 reports `60_60_dry_right_rest_j7_020.txt` at
        50.9 KB. We sent `60_dry_right_rest_j7_020.proj` (52 084 B = 50.86 KB);
        the sibling `.txt` on disk is 51 000 B = 49.8 KB. So the bytes are the
        .proj's and the NAME says .txt — the controller strips what it was sent
        and stores every program as a .txt. `rm_send_project` in libapi_c.so
        does the stripping (strrchr('.') then snprintf "%.*s", passing
        `project_name` and `save_id` to `rm_run_project` as separate JSON
        fields); the controller re-adds .txt on the far side. Newton's own
        pendant programs are consistent (`1_Hand_test_new.txt`).
      * This is invisible on the twelve commode slots — every one saturates the
        32-char field long before an extension could appear — which is exactly
        why it had to be checked against a SHORT name. On the dry slot the two
        models differ, and the dry slot is the ladder's first hardware rung.
      * Newton's own pendant programs obey it too (`1_Hand_test_new.txt`,
        `99_Right_Twl_Prg02.txt`), so the rule is the CONTROLLER's, not ours.
      * All 12 of our slots are reproduced exactly by the formula above, e.g.
        `70_` + `70_commode_c_toplid_right_execute_path.proj` truncated to 32
        = `70_70_commode_c_toplid_right_exe`.

    The DOUBLE `70_70_` is therefore not a defect on either side: we name our
    .proj files with the slot number already, and the controller adds the id
    again. It costs 6 of the 32 characters.

    This function used to return the bare basename — which the docstring of
    `name_convention_confirmed` correctly flagged as never having been observed
    on an arm. It hadn't; and it was wrong. The consequence was that the H25
    name check fired on 12 of 12 slots, which is the same as not checking at
    all: a slot genuinely holding another task's bytes looked exactly like the
    other eleven.

    Returned UNTRUNCATED — callers compare on the prefix, because the stored
    value is cut at `NAME_FIELD_CHARS`.
    """
    t = row.get("track")
    if not t:
        return None
    # `.txt`, NOT `.proj`: we upload the .proj, the controller stores the stem
    # under its own .txt extension. Our track file is already the .txt, so this
    # is the same stem either way — spelled from the track to make that plain.
    base = pathlib.Path(t).with_suffix(".txt").name
    slot = row.get("slot")
    # A row with bytes but no slot cannot predict the id prefix. Fall back to
    # the basename rather than inventing one: `trajectory_name_check` will find
    # no prefix match and report it, which is the honest outcome.
    return f"{slot}_{base}" if slot is not None else base


#: Our names share a long prefix, so a prefix shorter than this cannot separate
#: two slots (or two SIDES) and must not be trusted. Note the shared prefix is
#: now LONGER than it was: `<id>_<id>_commode_c_` is 16 characters for a 2-digit
#: slot, so this floor alone does not prove discrimination — the rival/size
#: branch of `trajectory_name_check` is what actually does, and it is reached
#: for any stored name short enough to prefix-match a sibling.
NAME_MIN_DISCRIMINATING = 12


def name_convention_confirmed(stored_names, rows) -> bool:
    """Does THIS controller name its stored programs the way we assume?

    The expectation is the basename of the `.proj` we send, which is what the
    emulated double stores and what `rm_trajectory_data_t` implies — but NO
    hardware record anywhere carries a program list, so the model has never
    been observed on an arm. Where it is unconfirmed a name finding must be
    reported, not refused: a bringup that cannot upload because a getter
    spells its slots `Project_70` is a worse failure than the one the check
    prevents (hardening review 2026-09-08 — `upload` and `dry` were refusing
    on this model outright while `preflight` already knew better).

    A swapped controller cannot hide in the unconfirmed branch: it would be
    holding OUR names, which confirms the model and makes its mismatch hard.
    """
    expected = [expected_trajectory_name(r) or "" for r in (rows or [])]
    seen = [str(n).strip().rstrip("\x00") for n in (stored_names or []) if n]
    return bool(seen) and any(w and n and (w.startswith(n) or n.startswith(w))
                              for n in seen for w in expected)


def trajectory_name_check(stored, row: dict, other_side_rows: list = None,
                          stored_size: int = None) -> dict:
    """H25, the REAL side-identity assert.

    `track.assert_side(side, h.side)` compares the string that chose the IP
    with itself — a tautology that can never fail. The identity that can
    actually be wrong is the one on the controller: slots 70..78 mean
    DIFFERENT tasks on the two arms, so a swapped cable puts the left arm's
    toplid on the right arm's toplid slot with every other check green. The
    controller's stored `trajectory_name` is evidence from the far end of the
    cable, so that is what gets asserted.
    """
    want = expected_trajectory_name(row)
    out = dict(ok=False, stored=stored, expected=want, reasons=[])
    if want is None:
        out["reasons"].append("the row carries no track path — no name to expect")
        return out
    if stored is None:
        out["reasons"].append("the slot is not on this controller")
        return out
    s = str(stored).strip().rstrip("\x00")
    if len(s) < NAME_MIN_DISCRIMINATING:
        out["reasons"].append(f"the stored name {s!r} is shorter than "
                              f"{NAME_MIN_DISCRIMINATING} chars — it cannot identify a side")
        return out
    if not want.startswith(s) and not s.startswith(want):
        out["reasons"].append(f"stored {s!r} is not the truncation of {want!r} — this slot "
                              f"holds ANOTHER task's bytes")
        return out
    # THE NAME IS NOT ALWAYS ENOUGH, and it is enough for FEWER slots than this
    # comment used to claim. The id prefix measured on 2026-09-10 costs 6 of the
    # 32 characters, which pushes FIVE of the twelve shared slots — not two —
    # into storing the same name on both arms:
    #
    #   slot  right bytes   left bytes    delta   tol   stored name
    #     75       33,406       33,741      335    64   75_75_commode_c_lid_seat_hinge_a
    #     76      249,158      242,783    6,375   242   76_76_commode_c_seat_ring_bottom
    #     78       44,150       44,310      160    64   78_78_commode_c_seat_body_hinge_
    #     79      104,432      108,827    4,395   108   79_79_commode_c_bowl_inside_ring
    #     80       66,594       68,925    2,331    68   80_80_commode_c_bowl_inside_rim_
    #
    # The controller's stored SIZE separates all five, the tightest being slot
    # 78 at 160 B against a 64 B tolerance (2.5x). So identity is name AND
    # size, and a slot that neither can attribute to a side is a refusal, not a
    # shrug. If a re-emit ever brings one of those pairs inside the tolerance,
    # that slot loses its side identity entirely — which is why the refusal
    # below exists rather than a warning.
    rivals = [o for o in (other_side_rows or [])
              if (expected_trajectory_name(o) or "") not in ("", want)
              and ((expected_trajectory_name(o) or "").startswith(s) or
                   s.startswith(expected_trajectory_name(o) or ""))]
    if rivals:
        out["ambiguous_with"] = [o.get("name") for o in rivals]
        mine, theirs = row.get("proj_bytes"), [o.get("proj_bytes") for o in rivals]
        def near(a, b):
            return a is not None and b is not None and abs(int(a) - int(b)) <= max(64, int(b) // 1000)
        if stored_size is None or mine is None or any(x is None for x in theirs):
            out["reasons"].append(
                f"stored {s!r} truncates the same as {out['ambiguous_with']} and there is no "
                f"stored size to separate them — this slot cannot be attributed to a side")
            return out
        if not near(stored_size, mine):
            out["reasons"].append(f"stored {s!r} is ambiguous with {out['ambiguous_with']} and "
                                  f"the stored size {stored_size} B does not match this row's "
                                  f"{mine} B")
            return out
        if any(near(stored_size, x) for x in theirs):
            out["reasons"].append(f"stored {s!r} AND its size {stored_size} B fit both this row "
                                  f"and {out['ambiguous_with']} — the slot cannot be attributed")
            return out
        out["resolved_by"] = "stored size (the truncated name is ambiguous)"
    out["ok"] = True
    return out


def controller_slot_check(robot, row: dict, other_side_rows: list = None) -> dict:
    """What the CONTROLLER is actually holding in this row's slot, versus the
    row's pin — read on the RUN path, not only at bringup (hardening review
    2026-09-08).

    B15's fix pinned the bytes on DISK and compared the controller's copy in
    `upload` and `preflight`. Neither is forced before a run, and the disk pin
    cannot see the slot: the failure was a re-emit that never reached the arm,
    which leaves every disk-side check green while the controller plays the
    old file. `rm_get_program_trajectory_list` is the only vendor readback of
    what is stored, and the SIZE half of it is confirmed hardware behaviour —
    `upload` has compared it on every hardware upload.

    Conservative on purpose: an unreadable list is a WARNING (the getter, not
    the arm, is the thing that failed), a missing slot and a size that does
    not match the pin are refusals, and the stored NAME is advisory unless
    this controller demonstrably uses our naming model.
    """
    from alix_dispatch import track as tr
    out = dict(ok=True, reasons=[], warnings=[], listed=None, stored_size=None,
               stored_name=None, pinned_bytes=row.get("proj_bytes"), readable=False)
    slot = row.get("slot")
    try:
        rc, items = tr.program_list(robot)
    except Exception as exc:                                           # noqa: BLE001
        out["warnings"].append(f"the controller's program list could not be read ({exc!r}) "
                               f"— the stored slot was NOT compared with the pin")
        return out
    out["list_ret"] = rc
    # ret != 0 is "the getter did not answer" — a warning. ret == 0 with an
    # EMPTY list is an answer: this controller holds no programs at all, and
    # the slot we are about to run is definitively not one of them.
    if rc != 0:
        out["warnings"].append(f"the controller's program list came back ret={rc} — the "
                               f"stored slot was NOT compared with the pin")
        return out
    out["readable"] = True
    it = next((i for i in items if i.get("id") == slot), None)
    out["listed"] = it is not None
    if it is None:
        out["ok"] = False
        out["reasons"].append(f"slot {slot} is NOT on this controller — nothing to run "
                              f"(upload it)")
        return out
    try:
        out["stored_size"] = int(it.get("size") or 0)
    except (TypeError, ValueError):
        out["warnings"].append(f"slot {slot} reports a size this SDK cannot read "
                               f"({it.get('size')!r}) — not compared with the pin")
        out["stored_name"] = it.get("name")
        return out
    out["stored_name"] = it.get("name")
    want = row.get("proj_bytes")
    if not want:
        out["warnings"].append("the row pins no proj_bytes, so the controller's stored size "
                               "cannot be compared with anything (the B15 blind spot)")
    else:
        delta = out["stored_size"] - int(want)
        out["delta"] = delta
        if abs(delta) > max(64, int(want) // 1000):
            out["ok"] = False
            out["reasons"].append(
                f"slot {slot} holds {out['stored_size']} B but this row's .proj is "
                f"{want} B ({delta:+d}) — the controller is NOT holding these bytes "
                f"(B15). Re-upload the slot.")
    nc = trajectory_name_check(out["stored_name"], row, other_side_rows=other_side_rows,
                               stored_size=out["stored_size"])
    out["name_check"] = nc
    if not nc["ok"]:
        if name_convention_confirmed([i.get("name") for i in items],
                                     [row] + list(other_side_rows or [])):
            out["ok"] = False
            out["reasons"] += nc["reasons"]
        else:
            out["warnings"] += [f"name (advisory, this controller does not use our naming "
                                f"model): {r}" for r in nc["reasons"]]
    return out


# ------------------------------------------------------------- the tool frame
# H35 (2026-09-08 audit, REFRAMED 2026-09-09). The two controllers are
# supervised at DIFFERENT TCPs: `controller_before.tool_frame` reads
# `R_glove_2`, payload 0.711 kg, pose (0.0135, 0, 0.1803) on 101 of 101 right
# runs and `Arm_Tip`, payload 0.000 kg, pose all zeros on 29 of 29 left runs —
# never once equal. That is a FACT about two controllers, not a defect: the
# arms run different tasks, and nothing in the track channel has ever called
# `rm_change_tool_frame`, so each holds whatever its last session left.
#
# It cannot move either arm off its commanded joint path — the replay is a
# joint-space track, see THE TOOL FRAME in the module docstring — and it is not
# an input to any inter-arm clearance figure. `TOOL_FRAME_NOTE` is the whole of
# what it does reach and what the record says about it. So: pin per task the
# frame that task's OWN runs were recorded under, READ IT BACK, and RECORD a
# change (a refusal until 2026-09-15 — Newton: the frame must never block a
# run). Two arms holding different frames is likewise recorded, never refused.

#: THE ONE SENTENCE, and every caller says exactly this (Newton, 2026-09-09:
#: "why is it now an issue yet we have run and executed motions on both arms").
#: The empirical record leads; the mechanism follows as a watch-item.
#: WHY THE TWO SIDES READ DIFFERENTLY, and it is NOT a finding (Newton,
#: 2026-09-10: "It is not a hazard at all it is because the set toolframe is
#: the Armtip in the GUI so of course you expect such, if we set a different
#: frame of course we will get values associated with those frames").
#:
#: `Arm_Tip` IS the zero-offset frame. A controller holding it reports
#: 0.000 kg and a zero centroid because that is what selecting Arm_Tip MEANS —
#: not because a payload was left unconfigured. The numbers are a readback of
#: the operator's frame choice on that pendant, and they change when the choice
#: changes. Reporting them as a latent hazard invents a defect out of a
#: setting, which is the same mistake as reading a task's `ik_frame` as a
#: motion constraint.
#:
#: WHAT IS STILL WORTH SAYING: the two arms are evaluated against Cartesian
#: caps at their OWN TCP, so an identical joint path counts as a different
#: Cartesian speed on the two sides. The joint path and every pairing
#: clearance in this table are frame-free and unaffected.
#:
#: WHAT THE REAL GATE IS: a controller that comes up holding something OTHER
#: than what its own recorded runs were made under — the CHANGED-frame
#: regression, which `preflight` and `run` refuse on. A difference BETWEEN the
#: sides is not that gate and never was.
#: Standalone sentence: every caller appends it after a full stop, so it may
#: not open with a relative pronoun.
TOOL_FRAME_NOTE = (
    "That is the frame each pendant has selected, nothing more: Arm_Tip is "
    "the zero-offset frame, so 0.000 kg and a zero centroid are what choosing "
    "it MEANS. Set a different frame and you get that frame's values. The "
    "Cartesian caps are evaluated at each arm's own TCP, so the same joint "
    "path counts as a different Cartesian speed on the two arms; the joint "
    "path and every pairing clearance here are frame-free and unaffected. A "
    "task's ik_frame is a different thing again — it selects the HAND STATE "
    "from glove_frames.yaml. A CHANGE from what a row's own runs were recorded "
    "under is written into the run record (tool_frame_check) and said on the "
    "console; since 2026-09-15 it is not a refusal either.")

#: Comparison tolerances. The pose is the controller's own stored value read
#: straight back, so it is exact to its own rounding; payload and centroid are
#: reported to 3 decimals (kg) and 0.1 mm.
TOOL_FRAME_TOL = dict(payload_kg=0.005, pose_m=0.0005, centroid_mm=0.5)


def normalise_tool_frame(raw) -> dict:
    """`rm_get_current_tool_frame`'s answer -> a comparable dict, or None.

    Handles the three spellings in play: the `(ret, dict)` tuple the SDK and
    the emulator return, the bare dict a snapshot stores, and the two key
    names (`name` on hardware and in the emulator, `frame_name` in
    `track.snapshot_line`, which is why that line prints None on every real
    record).

    CENTROID UNITS. The hardware records hold (-23.6, 25.4, 164.8) for
    R_glove_2 — MILLIMETRES — while the emulator's getter and Newton's
    payload_audit consensus hold the same centroid as (-0.0236, 0.0254,
    0.1648) — METRES. Both are live, so the unit is decided by magnitude: a
    tool centroid a whole metre from the flange is not a tool centroid. The
    guess is REPORTED (`centroid_units`) rather than hidden.
    """
    try:
        return _normalise_tool_frame(raw)
    except Exception:                                                  # noqa: BLE001
        # A malformed answer is UNREADABLE, not a match: `tool_frame_check`
        # turns None into a refusal. Raising here would take the whole run out
        # past its own gates instead (hardening review 2026-09-08).
        return None


def _normalise_tool_frame(raw) -> dict:
    tf = raw
    if isinstance(tf, (tuple, list)) and len(tf) == 2 and isinstance(tf[1], dict):
        tf = tf[1]
    if not isinstance(tf, dict):
        return None
    name = tf.get("name") or tf.get("frame_name")
    if isinstance(name, bytes):
        name = name.decode("utf-8", "replace")
    name = None if name is None else str(name).strip().rstrip("\x00")
    pose = tf.get("pose")
    pose = [round(float(v), 6) for v in pose] if isinstance(pose, (list, tuple)) else None
    payload = tf.get("payload")
    payload = None if payload is None else round(float(payload), 6)
    com, units = None, None
    if all(tf.get(k) is not None for k in ("x", "y", "z")):
        com = [float(tf["x"]), float(tf["y"]), float(tf["z"])]
        units = "m" if max(abs(v) for v in com) <= 1.0 else "mm"
        com = [round(v * 1000.0, 4) if units == "m" else round(v, 4) for v in com]
    return dict(name=name, payload_kg=payload, pose_m=pose, centroid_mm=com,
                centroid_units=units)


def tool_frame_wanted(row: dict):
    """The tool frame THIS row's own recorded runs were supervised at, or None.

    `tool_frame_check` compares the controller against this and RECORDS a change
    (never a refusal, Newton 2026-09-15). `tool_frame_guard` now also SELECTS it
    for the run and puts the pendant's choice back afterwards — the other branch
    of the same ruling, and the one that stops a certified file being replayed
    under supervision it was never measured at. None means the manifest does not
    record one, and then nothing is selected: this guard invents no expectation.
    """
    exp = row.get("tool_frame")
    if isinstance(exp, dict) and exp.get("name"):
        return str(exp["name"])
    return None


def tool_frame_check(found_raw, row: dict) -> dict:
    """HAS THIS ARM'S FRAME CHANGED since THIS task's own recorded runs? A
    change is RECORDED — it was a refusal (H35) until 2026-09-15, when Newton
    ruled that the frame must never block a run: collect the data instead.

    It is a REGRESSION CHECK on one controller, not a claim that any frame is
    "correct" and not a comparison against the task's `ik_frame` (a different
    namespace; see THE TOOL FRAME in the module docstring). A joint-space
    replay cannot be steered by the TCP, but the Cartesian caps `limits.guard`
    raises ARE evaluated at it and the payload feeds gravity compensation — so
    a controller that comes up holding something other than what these runs
    were made under is not the machine those runs describe, and the operator
    must be told before the stroke.

    An ABSENT expectation is a refusal too: the manifest not recording which
    TCP supervised the stroke is what left 130 runs unaccounted for.
    """
    exp = row.get("tool_frame")
    found = normalise_tool_frame(found_raw)
    out = dict(ok=False, expected=exp, found=found, reasons=[])
    if not isinstance(exp, dict) or not exp.get("name"):
        out["reasons"].append("no tool_frame expectation in the manifest for this task "
                              "(run `cleaning_tracks.py provenance --write`)")
        return out
    if found is None:
        out["reasons"].append(f"the controller's tool frame is unreadable ({found_raw!r})")
        return out
    if found["name"] != exp["name"]:
        out["reasons"].append(f"tool frame is {found['name']!r} but this row's own "
                              f"recorded runs were made at {exp['name']!r} — the frame "
                              f"CHANGED, so the Cartesian caps, gravity compensation and "
                              f"the controller's own COLLISION STAGE (which models this "
                              f"payload's mass and centroid — F31) are not the ones "
                              f"those runs were made under (another TCP)")
    # AN UNANSWERED FIELD IS NOT AN AGREEMENT (hardening review 2026-09-08).
    # Each comparison below used to be skipped when the controller's value was
    # None, so a getter that returned a name and nothing else passed a row that
    # pins a 0.711 kg payload 180.8 mm distal of the flange. Every one of the
    # 130 records carries all three fields, and so does the emulator, so a
    # missing one is a broken read, not a match.
    for key, label in (("payload_kg", "payload"), ("pose_m", "TCP offset"),
                       ("centroid_mm", "payload centroid")):
        if exp.get(key) is not None and found.get(key) is None:
            out["reasons"].append(f"the controller did not report a {label} — the row pins "
                                  f"{exp[key]} and it cannot be compared with nothing")
    if exp.get("payload_kg") is not None and found["payload_kg"] is not None:
        d = abs(found["payload_kg"] - float(exp["payload_kg"]))
        if d > TOOL_FRAME_TOL["payload_kg"]:
            out["reasons"].append(f"payload {found['payload_kg']} kg vs the row's "
                                  f"{exp['payload_kg']} kg ({d:.3f} kg off) — gravity "
                                  f"compensation and the collision stage both read it")
    if exp.get("pose_m") and found["pose_m"] and len(exp["pose_m"]) == len(found["pose_m"]):
        d = max(abs(a - float(b)) for a, b in zip(found["pose_m"], exp["pose_m"]))
        if d > TOOL_FRAME_TOL["pose_m"]:
            out["reasons"].append(f"TCP offset differs by {d * 1000:.1f} mm from the row's "
                                  f"{exp['pose_m']}")
    if exp.get("centroid_mm") and found["centroid_mm"]:
        d = max(abs(a - float(b)) for a, b in zip(found["centroid_mm"], exp["centroid_mm"]))
        if d > TOOL_FRAME_TOL["centroid_mm"]:
            out["reasons"].append(f"payload centroid differs by {d:.1f} mm from the row's "
                                  f"{exp['centroid_mm']} mm")
    out["ok"] = not out["reasons"]
    return out


def dual_tool_frame_check(right_raw, left_raw) -> dict:
    """READ both controllers' tool frames for a dual row and RECORD them.
    THE TWO ARMS DIFFERING IS NOT A REFUSAL (2026-09-09).

    WHAT THE TOOL FRAME DOES NOT AFFECT. The upload channel replays a
    JOINT-SPACE track: the project file is one `{"point":[7 millidegrees]}`
    per line and the player streams those joint angles verbatim (the emulated
    player is literally `movej_canfd(deg)`), so the TCP is not in the path
    that moves the arm and cannot alter the executed joint path. It is not an
    input to a single inter-arm clearance figure either: `dual_arm_verdicts.py`
    builds every number in `PAIR_VERDICTS.json` from those joint angles through
    URDF FK (`link_world_transforms`) into FCL, and never reads a tool frame.
    So the PAIRING decision does not depend on this at all.

    WHAT IT DOES AFFECT is `TOOL_FRAME_NOTE`, and that note is a NOTE: the
    record comes first (157 runs, no collision-stage abort ever) and the
    mechanism second, as a watch-item.

    The two arms therefore legitimately carry different frames: they run
    DIFFERENT TASKS, and `R_glove_2` at 0.711 kg on the right against `Arm_Tip`
    at 0.000 kg on the left is what 130 records show. That lands in
    `differences` — a recorded fact — not in `reasons`. `ok` is False only for
    what really is not a clean read: a controller whose frame is UNREADABLE, or
    one that answers with no payload at all.

    The per-row question "has this arm's frame CHANGED since THIS task's own
    runs?" is a different one, it is real, and `tool_frame_check` still
    REFUSES on it.
    """
    r, l = normalise_tool_frame(right_raw), normalise_tool_frame(left_raw)
    out = dict(ok=False, right=r, left=l, reasons=[], differences=[], matched=None)
    for side, tf in (("right", r), ("left", l)):
        if tf is None:
            out["reasons"].append(f"the {side} controller's tool frame is unreadable")
    if out["reasons"]:
        return out
    if r.get("payload_kg") is None or l.get("payload_kg") is None:
        out["reasons"].append("a controller did not report a payload — an unanswered "
                              "field is a broken read, not a match")
    elif abs(r["payload_kg"] - l["payload_kg"]) > TOOL_FRAME_TOL["payload_kg"]:
        out["differences"].append(
            f"the two controllers carry different payloads ({r['payload_kg']} kg right "
            f"vs {l['payload_kg']} kg left). " + TOOL_FRAME_NOTE)
    # THE CENTROID AND THE TCP OFFSET ARE COMPARED TOO (adversarial review
    # 2026-09-09). `matched` is printed as "the same tool frame and payload",
    # and comparing only the NAME and the MASS could say that of two frames
    # whose centroids differ — which is F31's defect exactly: the abort that
    # cost 2026-08-08 and 2026-08-10 was a payload centroid 1000x too large
    # under an unchanged name and mass. A difference here is still only a
    # NOTE; what it may not be is invisible.
    if r.get("pose_m") and l.get("pose_m") and len(r["pose_m"]) == len(l["pose_m"]):
        d = max(abs(a - b) for a, b in zip(r["pose_m"], l["pose_m"]))
        if d > TOOL_FRAME_TOL["pose_m"]:
            out["differences"].append(
                f"the two TCP offsets differ by {d * 1000:.1f} mm ({r['pose_m']} right "
                f"vs {l['pose_m']} left). The Cartesian caps `limits.guard` raises are "
                f"evaluated at each arm's own TCP, so the same joint path counts as a "
                f"different Cartesian speed on the two arms; the joint path itself and "
                f"the pairing clearances are unaffected.")
    if r.get("centroid_mm") and l.get("centroid_mm"):
        d = max(abs(a - b) for a, b in zip(r["centroid_mm"], l["centroid_mm"]))
        if d > TOOL_FRAME_TOL["centroid_mm"]:
            out["differences"].append(
                f"the two payload centroids differ by {d:.1f} mm ({r['centroid_mm']} "
                f"right vs {l['centroid_mm']} left). " + TOOL_FRAME_NOTE)
    if r.get("name") != l.get("name"):
        out["differences"].append(
            f"the two controllers hold different tool frames ({r.get('name')!r} right "
            f"vs {l.get('name')!r} left), which is expected: the arms run different "
            f"tasks and nothing in this channel sets the frame. The Cartesian caps "
            f"`limits.guard` raises are evaluated at each arm's own TCP, so the same "
            f"joint path counts as a different Cartesian speed on the two arms; the "
            f"joint path itself and the pairing clearances are unaffected.")
    out["matched"] = not out["differences"]
    out["ok"] = not out["reasons"]
    return out


def tool_frame_line(tf: dict) -> str:
    if not isinstance(tf, dict):
        return str(tf)[:60]
    return (f"{tf.get('name')} payload {tf.get('payload_kg')} kg pose {tf.get('pose_m')} "
            f"centroid {tf.get('centroid_mm')} mm")


# ----------------------------------------------------- evidence from the runs
RUNS_ROOT = REPO / "Resource" / "runs" / "tracks"


def _frame_a_run_ran_under(rec: dict):
    """The tool frame the RUN PLAYED UNDER: `tool_frame_guard`'s read-back if it selected, else
    the frame read off the controller before the run.

    The pre-run snapshot alone is the frame the PENDANT held, which for every run the guard
    switched is the frame the run did NOT use: the three slot-61 runs of 2026-09-17 played at
    `R_glove_2` (guard `took`, `restored`) and were counted as `Arm_Tip` (2026-09-17 review, B1).
    `taught_tracks._frame_held` already prefers the guard's proved selection.

    The guard's read-back is taken whether or not the wanted frame TOOK — a selection that did
    not take still reports the frame the controller actually holds, and that is what ran. Records
    written before the guard kept the whole frame carry the proved NAME only; that is reported as
    a name-only frame (`tool_frame_check` then compares the name and skips what it was not told)
    rather than as the frame it displaced.
    """
    g = rec.get("tool_frame_guard") or {}
    tf = normalise_tool_frame(g.get("selected_frame"))
    if tf and tf.get("name"):
        return tf
    if g.get("selected"):
        return dict(name=str(g["selected"]), payload_kg=None, pose_m=None,
                    centroid_mm=None, centroid_units=None)
    return normalise_tool_frame((rec.get("controller_before") or {}).get("tool_frame"))


def restamp_rate_fields(reg: dict, loader_name: str = "experiment") -> dict:
    """Recompute the RATE evidence on every EMITTED row of a registry from the bytes already on disk.

    The bytes are NOT touched and nothing is re-timed — this only replaces figures that were written
    before the rate instruments were corrected. Rows installed before 2026-09-17 carry `over_rating`
    with the raw per-joint peaks but no `peak_util`, no raw tick count and no `clock_s`, and no
    `rating_at_ceiling` block at all, so a reader cannot tell how far over a rating a row sits or at
    which clock it was judged. Returns a summary; the caller saves the registry.
    """
    import numpy as np
    from alix_dispatch import track as tr
    changed, missing = [], []
    for t in reg.get("tasks", []):
        if t.get("status") != "EMITTED" or not t.get("track"):
            continue
        f = REPO / t["track"]
        if not f.is_file():
            missing.append(t["name"]); continue
        q = np.array(tr.parse_track(f), float) / 1000.0
        rr = rating_report_at_ceiling(q)
        before = json.dumps(t.get("rating_at_ceiling"), sort_keys=True)
        t["rating_at_ceiling"] = rr
        if rr["ticks_over_rating"] or (t.get("over_rating") or {}).get("allowed"):
            t["over_rating"] = dict(
                allowed=bool((t.get("over_rating") or {}).get("allowed", False)),
                ticks=int(rr["ticks_over_rating"]), peaks=rr["per_joint_peak_deg_s_raw"],
                peak_util=rr["peak_util"], peak_joint=rr["peak_joint"],
                ticks_raw=rr["ticks_over_rating_raw"], peak_util_raw=rr["peak_util_raw"],
                clock_s=RATE_CEILING_CLOCK_S)
        else:
            t["over_rating"] = None
        if json.dumps(t["rating_at_ceiling"], sort_keys=True) != before:
            changed.append((t["name"], rr["ticks_over_rating"], rr["peak_util"]))
    return dict(registry=loader_name, rows=len(reg.get("tasks", [])), restamped=changed, missing=missing)


def run_record_evidence(runs_root: pathlib.Path = None) -> dict:
    """Per task name, what the RUN LEDGER says was actually played and under
    which tool frame. Read-only; the ledger is the only record of what the
    controller held, and it is where B15 and H35 were both found.

    BOTH CHANNELS COUNT FOR THE BYTES (B15), only the upload channel for the frame.
    Until 2026-10-02 this read UPLOAD runs only (`run_slot*.json`): `observed_runs_for`
    feeds B15's gate, and a host-streamed run was held not to answer "a clean run on
    the disk bytes". Newton 2026-10-02: ONE set of bytes serves both channels ("any
    plans can be channeled to CANFD or track upload channel irregardless of the channel
    set during generation") — the CANFD lane streams the very lines the player would
    play, identified by the same `track_sha256` — so a COMPLETED hardware CANFD run
    (`canfd_slot*.json`, `complete` True) of the bytes on disk answers B15's byte
    question as an upload run does. Each byte-set keeps the split (`by_channel`) so a
    reader can still tell a player-timed run from a host-streamed one; the timing
    readers that need the PLAYER (clock envelopes, the per-pair window term,
    `dual_arm_verdicts.replay_stats`) read `run_slot*` themselves and are unchanged.
    `n` and the byte-sets count both channels; `sides` and `tool_frames` stay UPLOAD
    ONLY (the controller-side frame evidence the upload lane's expectation is built on;
    unchanged by the ruling).

    The tool frame is the one each run PLAYED UNDER, not the pendant's pre-run
    frame — see `_frame_a_run_ran_under`."""
    runs_root = pathlib.Path(runs_root or RUNS_ROOT)
    out = {}
    recs = [(p, "upload") for p in sorted(runs_root.glob("*/run_slot*.json"))] + \
           [(p, "canfd") for p in sorted(runs_root.glob("*/canfd_slot*.json"))]
    for p, channel in recs:
        if "emulated" in p.parts:
            continue
        try:
            d = json.loads(p.read_text())
        except Exception:                                              # noqa: BLE001
            continue
        name = d.get("name")
        if not name:
            continue
        # a CANFD record counts only as a HARDWARE run (the emulator's streams prove nothing about the bytes)
        if channel == "canfd" and str(d.get("sdk") or "") != "hardware":
            continue
        e = out.setdefault(name, dict(n=0, bytes={}, tool_frames={}, sides={}))
        e["n"] += 1
        e.setdefault("n_by_channel", {})
        e["n_by_channel"][channel] = e["n_by_channel"].get(channel, 0) + 1
        sha = d.get("track_sha256")
        if sha:
            b = e["bytes"].setdefault(sha, dict(track_sha256=sha, points=d.get("points"),
                                                runs=0, clean=0, verdicts={},
                                                first=None, last=None, by_channel={}))
            b["runs"] += 1
            bc = b.setdefault("by_channel", {}).setdefault(channel, dict(runs=0, clean=0))
            bc["runs"] += 1
            if d.get("complete") is True:
                bc["clean"] += 1
            # WHICH RUNS WERE CLEAN. B15's remedy is "one clean single-arm run
            # on the disk bytes before the row is paired with anything", and a
            # count of runs cannot answer that: two of toplid_left's four runs
            # CUT. `complete` is the finish-event label (F123), so it is the
            # one that decides (hardening review 2026-09-08).
            b.setdefault("verdicts", {})
            v = str(d.get("verdict"))
            b["verdicts"][v] = b["verdicts"].get(v, 0) + 1
            if d.get("complete") is True:
                b["clean"] = b.get("clean", 0) + 1
            # A record with no `when` must not silently become the earliest run
            # ("" sorts below every timestamp) — skip it instead.
            when = d.get("when") or ""
            if when:
                b["first"] = when if b["first"] is None else min(b["first"], when)
                b["last"] = when if b["last"] is None else max(b["last"], when)
        if channel != "upload":
            continue                       # frame + side evidence: the upload channel's records only
        tf = _frame_a_run_ran_under(d)
        if tf and tf.get("name"):
            key = json.dumps(tf, sort_keys=True)
            t = e["tool_frames"].setdefault(key, dict(frame=tf, runs=0))
            t["runs"] += 1
        if d.get("side"):
            e["sides"][d["side"]] = e["sides"].get(d["side"], 0) + 1
    return out


def observed_runs_for(row: dict, evidence: dict, disk_sha: str) -> dict:
    """The `observed_runs` block: which bytes this slot has actually played,
    and whether the bytes now on disk are among them. This is how B15 STATES
    itself instead of being silently absorbed."""
    e = evidence.get(row.get("name"))
    if not e:
        return dict(n_runs=0, n_runs_on_disk=0, n_runs_other=0, other_bytes=[],
                    never_run_on_disk_bytes=None,
                    note="no run record carries this task name")
    mine = e["bytes"].get(disk_sha, {}) if disk_sha else {}
    on_disk = mine.get("runs", 0)
    clean_on_disk = mine.get("clean", 0)
    other = [dict(track_sha256=b["track_sha256"], points=b["points"], runs=b["runs"],
                  clean=b.get("clean", 0), verdicts=b.get("verdicts", {}),
                  first=b["first"], last=b["last"], by_channel=b.get("by_channel", {}))
             for sha, b in sorted(e["bytes"].items()) if sha != disk_sha]
    n_other = sum(o["runs"] for o in other)
    # BOTH CHANNELS (Newton 2026-10-02, one set of bytes; `run_record_evidence`): the counts below are upload +
    # hardware CANFD runs of the disk bytes; `runs_on_disk_by_channel` keeps the split
    out = dict(n_runs=e["n"], n_runs_on_disk=on_disk,
               n_clean_runs_on_disk=clean_on_disk,
               runs_on_disk_by_channel=mine.get("by_channel", {}),
               verdicts_on_disk=mine.get("verdicts", {}),
               n_runs_other=n_other,
               other_bytes=other, never_run_on_disk_bytes=bool(on_disk == 0),
               note=("every recorded run of this task played the bytes now on disk"
                     if on_disk and not n_other else
                     "the bytes on disk have NEVER been run; the controller was fed "
                     "something else" if not on_disk else
                     "this slot has played more than one set of bytes"))
    # THE GATE THE DESIGN ASKS FOR, in the row itself: "leg B stays refused
    # until one clean single-arm run on the disk bytes exists" (B15). A run
    # COUNT cannot answer it — two of toplid_left's four runs cut — so the
    # question is asked of `complete`, the finish-event label.
    out["clean_single_arm_run_on_disk_bytes"] = bool(clean_on_disk)
    if on_disk and not clean_on_disk:
        out["remedy"] = (
            f"the bytes on disk have run {on_disk} time(s) and NOT ONE of them completed "
            f"({mine.get('verdicts', {})}): this row has no clean single-arm run to pair "
            f"anything with. Run it alone, to completion, first.")
    if not on_disk:
        out["remedy"] = (
            "the controller still holds the bytes it last accepted, and no upload has "
            "replaced them: RE-UPLOAD this slot, re-run `preflight` (which now compares "
            "the stored size and name), and give it one clean single-arm run before it "
            "is paired with anything — on either channel: a completed CANFD run of these "
            "bytes counts as an upload run does (Newton 2026-10-02, one set of bytes). "
            "File mtimes cannot order these events — the sha and the run ledger can.")
    return out


def attach_evidence(rows: list, evidence: dict = None) -> dict:
    """Recompute the ledger-derived fields on freshly written rows.

    Called by `emit` as well as by `provenance --write` (hardening review
    2026-09-08). Without it a re-emitted row carried NO `observed_runs`, so the
    one warning that states B15's shape in words — "no recorded run has ever
    played the bytes now on disk" — went silent exactly when it had just become
    true, and the only thing standing between the new bytes and the old ones
    still on the controller was a preflight nobody is forced to run.

    The tool frame is EVIDENCE, not an emit product: it is filled in only where
    the row does not already carry one, so a re-emit never rewrites Newton's
    pinned expectation.
    """
    ev = evidence if evidence is not None else (
        run_record_evidence() if RUNS_ROOT.is_dir() else {})
    for r in rows:
        if r.get("status") != "EMITTED" or not r.get("track"):
            continue
        r["observed_runs"] = observed_runs_for(r, ev, r.get("track_sha256"))
        if not r.get("tool_frame"):
            tf = tool_frame_from_records(r, ev)
            if tf is not None:
                r["tool_frame"] = tf
    return ev


def tool_frame_from_records(row: dict, evidence: dict) -> dict:
    """The per-task tool-frame EXPECTATION, derived from that task's own run
    records. Never invented: a task with no snapshot gets None, and a task
    whose records disagree gets None with the disagreement recorded."""
    e = evidence.get(row.get("name")) or {}
    tfs = e.get("tool_frames") or {}
    if not tfs:
        return None
    ranked = sorted(tfs.values(), key=lambda t: -t["runs"])
    if len(ranked) > 1:
        return dict(name=None, payload_kg=None, pose_m=None, centroid_mm=None,
                    provenance=dict(source="run records controller_before.tool_frame",
                                    unanimous=False,
                                    seen=[dict(frame=t["frame"], runs=t["runs"]) for t in ranked],
                                    note="the records DISAGREE about which frame this "
                                         "task ran under, so no expectation can be "
                                         "derived from them: re-observe the frame on a "
                                         "clean single-arm run and pin it with "
                                         "`provenance --write`. Until then this row has "
                                         "no unchanged-since-the-runs gate."))
    tf = dict(ranked[0]["frame"])
    ik = row.get("ik_frame") or ""
    out = dict(name=tf["name"], payload_kg=tf["payload_kg"], pose_m=tf["pose_m"],
               centroid_mm=tf["centroid_mm"])
    # TWO FACTS, NOT A DISAGREEMENT (Newton, 2026-09-09: "every task has an
    # associated ik frame, we have already addressed that"). `ik_frame` is the
    # task's AUTHORED CONTACT FRAME; `name` is the CONTROLLER-SIDE supervision
    # frame the arm was observed holding. Two namespaces, not required to be
    # equal — the replay streams joint millidegrees, so the controller's TCP
    # cannot move the arm off the authored path. Both are recorded because
    # both are useful provenance. Neither is a question for anybody, and
    # nothing here derives a controller name from the ik_frame to compare them.
    out["provenance"] = dict(
        source="run records controller_before.tool_frame (Resource/runs/tracks)",
        runs=ranked[0]["runs"], unanimous=True,
        centroid_units_as_recorded=tf.get("centroid_units"),
        ik_frame=ik or None,
        ik_frame_is=("the task's AUTHORED frame: it selects the HAND STATE for this "
                     "stroke from glove_frames.yaml, with the contact axis and pad "
                     "footprint. Settled per task; not a controller setting."),
        controller_tool_frame=tf["name"],
        controller_tool_frame_is=("the CONTROLLER-side supervision frame this arm was "
                                  "observed holding: it fixes where the Cartesian caps "
                                  "are evaluated and carries the payload gravity "
                                  "compensation uses."),
        tool_frame_affects=TOOL_FRAME_NOTE,
        tool_frame_does_not_affect=("the executed joint path — the replay streams joint "
                                    "millidegrees verbatim — nor any inter-arm clearance "
                                    "figure, which is URDF FK on those same joint angles"),
        note=("the controller's frame is what the last session left, not something the "
              "track channel sets: nothing here has ever called rm_change_tool_frame. "
              "This row records what the arm was ACTUALLY supervised at, so `run` and "
              "`preflight` can record a CHANGE since these runs (never a refusal since "
              "2026-09-15)."))
    return out


# ---------------------------------------------------------------------- emit
RATED_DEG_S = (180.0, 180.0, 225.0, 225.0, 225.0, 225.0, 225.0)


#: THE RATE CEILING CLOCK (2026-09-17). The upload player advances at most ONE point per 5 ms
#: controller cycle and holds otherwise (Resource/reports/2026-09-17_upload_player_clock), so
#: between holds a track plays at 5.0 ms/pt — 11 % faster than the 5.5545 ms average (PLAYER_CLOCK_S,
#: which was also the emit clock until 2026-10-02). Every rating check is judged here. Judged at that
#: 5.5545 ms figure, 22 of 24 emitted tracks rode their ratings and the arm clamped
#: at them (logged joint_speed 183-185 deg/s on J1, 228-231 on J4), where the median departure is
#: ~1.7x the unclamped one. Same instrument as the taught lane's G2 (20 Hz band-limited).
RATE_CEILING_CLOCK_S = 0.005


def rating_report_at_ceiling(q_deg, clock_s: float = RATE_CEILING_CLOCK_S) -> dict:
    """G2's instrument on a point list: band-limited ticks over a joint rating at the player's
    fastest clock, the peak utilisation and its joint."""
    import numpy as np
    import taught_tracks as tt
    ptr = tt._ptr()
    vmax = np.array(ptr.K.QDOT_MAX_DEG_S, dtype=float)
    rr = ptr.rate_report(np.asarray(q_deg, dtype=float), clock_s, vmax)
    bl, raw = rr["band_limited_20hz"], rr["raw"]
    return dict(clock_s=float(clock_s), ticks_over_rating=int(bl["ticks_over_rating"]),
                peak_util=round(float(bl["peak_util"]), 4), peak_joint=int(bl["peak_joint"]),
                per_joint_peak_deg_s=[round(float(x), 1) for x in bl["per_joint_peak_deg_s"]],
                # the RAW instrument alongside: a drag-teach file's ripple spikes are over a rating
                # tick by tick even when the band-limited rate is not, and that is worth recording
                ticks_over_rating_raw=int(raw["ticks_over_rating"]),
                peak_util_raw=round(float(raw["peak_util"]), 4),
                per_joint_peak_deg_s_raw=[round(float(x), 1) for x in raw["per_joint_peak_deg_s"]])


def over_limit_ticks(track: pathlib.Path) -> tuple:
    """(ticks whose commanded joint speed exceeds the RATED limit at the player's 5.0 ms ceiling,
    20 Hz band-limited — the taught lane's G2 — and the peak per joint) on the written integers:
    error 4105 territory (F111), and the clamp."""
    import numpy as np
    q = np.array([json.loads(l)["point"] for l in track.open() if l.strip()], float) / 1000.0
    r = rating_report_at_ceiling(q)
    return r["ticks_over_rating"], r["per_joint_peak_deg_s_raw"]


def rate_fit_track(track: pathlib.Path, clock_s: float = RATE_CEILING_CLOCK_S, *, write: bool = True,
                   max_iter: int = 6) -> dict:
    """Re-time a track so no joint exceeds its rating at the player's fastest clock, slowing ONLY the
    stretches that need it (`taught_tracks.capped_millideg` with lambda_fast 1: every point stays on
    the track's own chords, so the path cannot move). A track already inside is left untouched.
    The design allowance starts at the taught lane's RATING_FILTER_ALLOWANCE and tightens 2 % per
    pass until the band-limited check reads zero. Returns what it did; `write` replaces the file."""
    import numpy as np
    import taught_tracks as tt
    q = np.array([json.loads(l)["point"] for l in track.open() if l.strip()], float) / 1000.0
    before = rating_report_at_ceiling(q, clock_s)
    own = track_clock_s(track)          # the file's own sample clock (durations), not the rating clock
    out = dict(applied=False, clock_s=float(clock_s), lines_before=int(q.shape[0]),
               duration_before_s=round((q.shape[0] - 1) * own, 3), before=before)
    if before["ticks_over_rating"] == 0:
        out.update(lines_after=out["lines_before"], duration_after_s=out["duration_before_s"], after=before)
        return out
    rating = np.array(tt._ptr().K.QDOT_MAX_DEG_S, dtype=float)
    allowance = float(tt.RATING_FILTER_ALLOWANCE)
    md = after = None
    for it in range(1, max_iter + 1):
        caps = np.tile(allowance * rating, (q.shape[0], 1))
        md = tt.capped_millideg(q, 1.0, caps, ramp=tt.GUARD_RAMP, clock_s=clock_s,
                                smooth_lines=int(np.ceil(0.10 / clock_s)))
        after = rating_report_at_ceiling(md / 1000.0, clock_s)
        out.update(iterations=it, design_allowance=round(allowance, 4))
        if after["ticks_over_rating"] == 0:
            break
        allowance *= 0.98
    out.update(applied=True, lines_after=int(md.shape[0]),
               duration_after_s=round((md.shape[0] - 1) * own, 3), after=after,
               slowdown=round((md.shape[0] - 1) / max(q.shape[0] - 1, 1), 4))
    if after["ticks_over_rating"] and write:
        raise SystemExit(f"{track.name}: the rate fit still leaves {after['ticks_over_rating']} tick(s) over "
                         f"a rating at {clock_s * 1e3:.1f} ms after {max_iter} passes")
    if write:
        with open(track, "w", newline="") as fh:
            for row in md:
                fh.write('{"point":[' + ",".join(str(int(v)) for v in row) + "]}\n")
    return out


def refresh_track_sidecar(track: pathlib.Path, ik_frame: str, fit: dict) -> dict:
    """After a rate fit the emitter's sidecar describes the OLD bytes: recompute the figures a row
    reads from it, with the same instruments `emit_fastpath` uses, and record the fit."""
    import numpy as np
    sc_path = track.with_suffix(".json")
    sc = json.load(open(sc_path)) if sc_path.exists() else {}
    q = np.array([json.loads(l)["point"] for l in track.open() if l.strip()], float) / 1000.0
    sys.path.insert(0, str(REPO / "tools"))
    import plan_min_time as pmt
    qrad = np.radians(q)
    side = "R" if str(ik_frame).startswith("R") else "L"
    _names, vlim, alim, jlim = pmt.load_limits(side)
    clk = float(sc.get("clock_s") or CLOCK_S)     # the file's own sample clock (2026-10-02)
    uv, ua, uj = pmt.band_limited_util(qrad, clk, vlim, alim, float(jlim.min()), 20.0)
    worst_rate, n_over, _per_joint = pmt.check_rate(qrad, clk, vlim)
    pos_ok = pmt.check_positions(qrad, side)
    sc.update(points=int(q.shape[0]), track_duration_s=(q.shape[0] - 1) * clk,
              band_limited_util=dict(fc_hz=20.0, vel=uv, acc=ua, jerk=uj,
                                     measured_on="the integer millidegrees written to the file"),
              max_joint_rate_deg_s=worst_rate, ticks_over_180_deg_s=n_over,
              canfd_streamable=bool(n_over == 0), **position_margin_record(pos_ok),   # 2026-10-02 advisory
              rate_fit=fit)
    sc_path.write_text(json.dumps(sc, indent=1))
    return sc


def tick_step_check(track: pathlib.Path) -> float:
    """Max one-tick commanded joint-speed step [deg/s] on the written integers
    — the raw self-consistency check (F103 R3): catches a broken sampler or an
    F101-class source plan without band-limit ambiguity."""
    import numpy as np
    q = np.array([json.loads(l)["point"] for l in track.open() if l.strip()], float) / 1000.0
    v = np.diff(q, axis=0) / track_clock_s(track)   # commanded joint speed per tick at the file's clock, deg/s
    return float(np.abs(np.diff(v, axis=0)).max())  # max ONE-TICK speed step, deg/s


def tick_step_gate(step: float, scale: float, plan, track, *, fastpath: bool, refit: bool) -> dict:
    """The one-tick speed-step self-check's limit for one emitted track, and the record of it.

    RE-SOLVE (plan_min_time): TICK_STEP_PER_ACC_DEG_S x max(acc_scale, 1), calibrated on that emitter,
    whose step is acc x clock ("3.3 deg/s at 1.0").
    FASTPATH = THE PLANNED TIMING (2026-10-02): a fastpath track's step is the PLAN's own velocity change at
    its waypoints, which `acc_scale` does not govern. At the 5.0 ms emit clock the samples are commensurate
    with a plan's 10 ms waypoints, so a sample lands ON every knot and the knot's whole velocity change
    shows in ONE tick; at 5.5545 ms it was split over two ticks and read lower for the same time law
    (MEASURED, Resource/reports/2026-10-02_tracks5ms: cspace side_right_toppra 23.60 vs 19.26 deg/s,
    toplid_right 17.00 vs 14.76). What a resample can get wrong is a step the PLAN does not have, so the
    sampler is judged against the plan's own largest knot step plus two millidegree LSBs per tick (the
    rounding); the re-solve limit is RECORDED beside it as an advisory -- the planned timing's
    acceleration is the planner's, and on the track route the acceleration gate is advisory (Newton
    2026-09-07, revised ruling 1). A track the rate fit re-timed is no longer the plan's timing and takes
    the re-solve limit, as before."""
    resolve = TICK_STEP_PER_ACC_DEG_S * max(float(scale), 1.0)
    if not fastpath or refit:
        return dict(kind="re-solve threshold", limit_deg_s=round(resolve, 2), limit_deg_s_exact=resolve)
    src = plan_knot_speed_step(plan)
    clk = track_clock_s(track)
    lim = src + 2 * 0.001 / clk
    return dict(kind="fastpath sampler (track step <= the plan's own knot step + 2 LSB/tick)",
                limit_deg_s=round(lim, 2), limit_deg_s_exact=lim, plan_knot_step_deg_s=round(src, 2),
                clock_s=clk, advisory_re_solve_limit_deg_s=round(resolve, 2),
                advisory_over=bool(float(step) > resolve))


def plan_knot_speed_step(plan_path, stage: str = "execute_path") -> float:
    """The largest change of commanded joint speed [deg/s] between consecutive waypoint intervals of the
    plan's own `stage` -- the velocity discontinuity of the piecewise-linear q(t) that `emit_fastpath`
    resamples. A resample at any clock reproduces at most this (plus rounding)."""
    import numpy as np
    d = json.load(open(plan_path))
    st = next(x for x in d["sub_trajectories"] if (x.get("stage_name") or x.get("name")) == stage)
    t = np.array([float(w["time_from_start"]) for w in st["waypoints"]])
    q = np.degrees(np.array([w["positions"] for w in st["waypoints"]], float))
    dt = np.diff(t)
    keep = dt > 1e-9
    v = np.diff(q, axis=0)[keep] / dt[keep, None]
    return float(np.abs(np.diff(v, axis=0)).max()) if len(v) > 1 else 0.0


def emit_fastpath(plan_path: pathlib.Path, track: pathlib.Path, ik_frame: str):
    """Downstream optimization (Newton, 2026-09-01): a plan authored with
    cleaning_path_mode=toppra is ALREADY time-optimal under the same limits —
    re-solving would be the A->D chain again. Resample the plan's own q(t)
    onto the emit clock (CLOCK_S, 5.0 ms since Newton's 2026-10-02 ruling),
    duration-preserving, and gate as usual.
    Returns a sidecar-shaped dict, or None if the plan is not toppra-timed."""
    import numpy as np
    d = json.load(open(plan_path))
    if d.get("cleaning_path_mode") != "toppra":
        return None
    st = next((x for x in d["sub_trajectories"]
               if (x.get("stage_name") or x.get("name")) == "execute_path"), None)
    if st is None:
        return None
    wp = st["waypoints"]
    t = np.array([float(w["time_from_start"]) for w in wp])
    q = np.degrees(np.array([w["positions"] for w in wp], float))
    T = t[-1]
    n = int(np.ceil(T / CLOCK_S)) + 1
    tt = np.minimum(np.arange(n) * CLOCK_S, T)
    qq = np.column_stack([np.interp(tt, t, q[:, j]) for j in range(q.shape[1])])
    md = np.round(qq * 1000.0).astype(np.int64)
    with open(track, "w", newline="") as fh:
        for row in md:
            fh.write('{"point":[' + ",".join(str(int(v)) for v in row) + "]}\n")
    # gates on the written integers, same instruments as the solver path
    sys.path.insert(0, str(REPO / "tools"))
    import plan_min_time as pmt
    qrad = np.radians(md / 1000.0)
    side = "R" if ik_frame.startswith("R") else "L"
    _names, vlim, alim, jlim = pmt.load_limits(side)
    jceil = float(jlim.min())          # same convention as plan_min_time main
    uv, ua, uj = pmt.band_limited_util(qrad, CLOCK_S, vlim, alim, jceil, 20.0)
    worst_rate, n_over, per_joint = pmt.check_rate(qrad, CLOCK_S, vlim)
    pos_ok = pmt.check_positions(qrad, side)
    # Newton 2026-10-02 "Relax the emit gate": the 5 deg margin RECORDS, it no longer refuses (was:
    # `min(pos_ok) < 5.0` -> refused; front_left J1 0.716 deg). Beyond a limit still refuses.
    pos_rec = position_margin_record(pos_ok)
    if pos_rec["position_margin_min_deg"] < POSITION_MARGIN_REFUSE_BELOW_DEG:
        track.unlink()
        raise SystemExit(f"fastpath: J{pos_rec['position_margin_min_joint']} is "
                         f"{-pos_rec['position_margin_min_deg']:.3f} deg BEYOND its joint-position limit")
    side = json.dumps(dict(
        source=str(plan_path.resolve()), emitter="cleaning_tracks.py emit --fastpath "
        "(plan is toppra-timed; duration-preserving resample onto the 5.0 ms emit clock = the CANFD stream period)",
        points=n, track_duration_s=(n - 1) * CLOCK_S, clock_s=CLOCK_S, acc_scale=1.0,
        band_limited_util=dict(fc_hz=20.0, vel=uv, acc=ua, jerk=uj,
                               measured_on="the integer millidegrees written to the file"),
        max_joint_rate_deg_s=worst_rate, ticks_over_180_deg_s=n_over,
        canfd_streamable=bool(n_over == 0),
        **pos_rec), indent=1)
    track.with_suffix(".json").write_text(side)
    return json.loads(side)


# ------------------------------------------------------------ one set of bytes
#: Newton 2026-10-02 (verbatim): "As I said before any plans can be channeled to CANFD or track upload
#: channel irregardless of the channel set during generation". A plan therefore carries ONE set of stroke
#: bytes, whichever lane plays it: the C++ executives play the plan's OWN track group (arm_executive's CANFD
#: lane streams `<stem>.g<k>.txt` and its upload lane pins or uploads the group's .proj), while the Python
#: upload lane and the dual-arm sweeps (tools/dual_arm_verdicts.py, PAIR_VERDICTS.json) read the MANIFEST
#: track this emit writes. When the emit's bytes differ from the plan's group -- the upload-player rate fit
#: below re-timed them (MEASURED 2026-10-02: front_left slot 114, +1 line with J5 at its 225 deg/s rating;
#: the plan's g0 11e6cf6e, 2227 lines, the manifest track 8092c51c, 2228 lines, so the C++ lane's
#: certify_pair read STALE on row 1) -- the plan's group is rewritten to the emitted bytes, with the
#: planner's own figures kept as provenance. Never a 'CANFD-only' exemption from the rate fit: CANFD-only
#: slots (101+) are a numbering rule only.
GROUP_REWRITE_KEY = "emit_rewrite"
#: the C++ gateTrack constants (track_emit.hpp kStepMaxDegPerTick, kPositionMarginGateDeg, kNearLimitBandDeg,
#: kApproachWindowTicks, kBandLimitFcHz), so a rewritten group's `gates` block reads as the planner's would
GATE_STEP_MAX_DEG_PER_TICK = 10.0
GATE_POSITION_MARGIN_DEG = 5.0
GATE_NEAR_LIMIT_BAND_DEG = 20.0
GATE_APPROACH_WINDOW_TICKS = 18
GATE_BAND_LIMIT_FC_HZ = 20.0


def _urdf_position_limits_rad(side: str):
    """(lower, upper) per arm joint, radians, from the expanded URDF plan_min_time.check_positions reads
    (the same file the C++ track_emit_cli is handed by tools/track_emit_parity.py)."""
    import xml.etree.ElementTree as ET
    sys.path.insert(0, str(REPO / "tools"))
    import plan_min_time as pmt
    root = ET.parse(pmt.URDF).getroot()
    lim = {}
    for j in root.findall("joint"):
        e = j.find("limit")
        if j.get("name") and e is not None:
            lim[j.get("name")] = (float(e.get("lower")), float(e.get("upper")))
    lo, hi = [], []
    for i in range(1, 8):
        a, b = lim[f"{side}_joint{i}"]
        lo.append(a)
        hi.append(b)
    return lo, hi


def track_gates(rows, side: str, clock_s: float = None) -> dict:
    """The planner's group `gates` block for a list of integer-millidegree rows: a line-for-line Python twin
    of the C++ `track_emit::gateTrack` (src/common/stroke/track_emit.cpp) with the same instruments the
    Python sidecars use (plan_min_time.check_positions / band_limited_util / check_rate). Used when the emit
    rewrites a plan's group, so the block describes the bytes it sits beside. Parity with the C++ block is
    pinned by core/python/alix_dispatch/test/test_one_set_of_bytes.py."""
    import numpy as np
    sys.path.insert(0, str(REPO / "tools"))
    import plan_min_time as pmt
    clk = float(clock_s or CLOCK_S)
    md = np.asarray(rows, dtype=np.int64)
    n = md.shape[0]
    if n < 2:
        raise SystemExit("track_gates: a track needs at least 2 points")
    qdeg = md / 1000.0
    qrad = qdeg * (np.pi / 180.0)
    rated = np.array(RATED_DEG_S)
    lo_rad, hi_rad = _urdf_position_limits_rad(side)
    margins = [float(min(qrad[:, j].min() - lo_rad[j], hi_rad[j] - qrad[:, j].max()) * (180.0 / np.pi))
               for j in range(7)]
    jmin = int(np.argmin(margins))                     # the FIRST minimum, as the C++ strict `<`
    d = np.diff(qdeg, axis=0)
    vraw = d / clk
    over_rating = (np.abs(d) / clk > rated).any(axis=1)
    speed_step = float(np.abs(np.diff(vraw, axis=0)).max()) if n > 2 else 0.0
    _names, vlim, alim, jlim = pmt.load_limits("R" if side == "R" else "L")
    uv, ua, uj = pmt.band_limited_util(qrad, clk, vlim, alim, float(jlim.min()), GATE_BAND_LIMIT_FC_HZ)
    worst, n180, per_joint = pmt.check_rate(qrad, clk, vlim)
    extrema = []
    for j in range(7):
        lo, hi = lo_rad[j] * (180.0 / np.pi), hi_rad[j] * (180.0 / np.pi)
        found, prev = [], 0
        for i in range(1, n):
            s = int(np.sign(d[i - 1, j]))
            if s == 0:
                continue
            if prev != 0 and s != prev:
                idx = i - 1
                q = float(qdeg[idx, j])
                margin = min(q - lo, hi - q)
                if margin < GATE_NEAR_LIMIT_BAND_DEG:
                    frm = idx + 1 - GATE_APPROACH_WINDOW_TICKS if idx + 1 > GATE_APPROACH_WINDOW_TICKS else 1
                    approach = max((abs(float(vraw[m - 1, j])) for m in range(max(frm, 1), idx + 1)), default=0.0)
                    found.append(dict(joint=j + 1, index=idx, deg=q, margin_deg=margin, approach_deg_s=approach))
            prev = s
        found.sort(key=lambda e: e["margin_deg"])      # stable, as std::stable_sort
        extrema += found[:8]
    return {
        "position_margin_deg": margins[jmin],
        "position_margin_min_joint": jmin + 1,
        "position_margin_per_joint_deg": margins,
        "position_margin_gate_deg": GATE_POSITION_MARGIN_DEG,
        "position_margin_ok": bool(margins[jmin] >= GATE_POSITION_MARGIN_DEG),
        "step_max_deg": float(np.abs(d).max()),
        "step_gate_deg": GATE_STEP_MAX_DEG_PER_TICK,
        "step_ok": bool(float(np.abs(d).max()) <= GATE_STEP_MAX_DEG_PER_TICK),
        "speed_step_max_deg_s": speed_step,
        "rate_peak_raw_deg_s": [float(x) for x in (np.abs(d) / clk).max(axis=0)],
        "rated_deg_s": list(RATED_DEG_S),
        "ticks_over_rating": int(over_rating.sum()),
        "ticks_over_180": int(n180),
        "rate_max_deg_s": float(worst),
        "per_joint_rate_deg_s": [float(x) for x in per_joint],
        "band_limited_util": {"fc_hz": GATE_BAND_LIMIT_FC_HZ, "vel": float(uv), "acc": float(ua), "jerk": float(uj),
                              "valid": True},
        "near_limit_band_deg": GATE_NEAR_LIMIT_BAND_DEG,
        "approach_window_ticks": GATE_APPROACH_WINDOW_TICKS,
        "near_limit_extrema": extrema,
    }


def _group_entry_json(g: dict, pad: str = "    ") -> str:
    """One `track_groups` entry in the planner's layout (track_emit.cpp groupPlanJson: a key per line,
    arrays inline, the gates block a key per line, one near-limit extremum per line)."""
    inl = lambda v: json.dumps(v, separators=(", ", ": "))           # noqa: E731
    i1, i2, i3 = pad + "  ", pad + "    ", pad + "      "
    lines = []
    for k, v in g.items():
        if k == "gates" and isinstance(v, dict):
            sub = []
            for gk, gv in v.items():
                if gk == "near_limit_extrema" and isinstance(gv, list) and gv:
                    sub.append(f'{i2}"{gk}": [\n' + ",\n".join(i3 + inl(e) for e in gv) + f"\n{i2}]")
                else:
                    sub.append(f'{i2}"{gk}": {inl(gv)}')
            lines.append(f'{i1}"{k}": {{\n' + ",\n".join(sub) + f"\n{i1}}}")
        elif k == GROUP_REWRITE_KEY and isinstance(v, dict):
            sub = [f'{i2}"{gk}": {inl(gv)}' for gk, gv in v.items()]
            lines.append(f'{i1}"{k}": {{\n' + ",\n".join(sub) + f"\n{i1}}}")
        else:
            lines.append(f'{i1}"{k}": {inl(v)}')
    return pad + "{\n" + ",\n".join(lines) + "\n" + pad + "}"


def _splice_track_groups(raw: str, groups: list) -> str:
    """The plan text with ONLY its top-level `track_groups` array replaced: every other byte of the plan
    (its sub-trajectories, the planner's number formatting) is kept, so the rewrite is visible as exactly
    the group change. Falls back to a whole re-serialisation only when the planner's layout is not found."""
    doc = json.loads(raw)
    key = '\n  "track_groups": '
    at = raw.find(key)
    if at >= 0:
        start = at + len(key)
        try:
            _old, length = json.JSONDecoder().raw_decode(raw[start:])
        except ValueError:
            at = -1
        else:
            body = ("[\n" + ",\n".join(_group_entry_json(g) for g in groups) + "\n  ]") if groups else "[]"
            out = raw[:start] + body + raw[start + length:]
            new = json.loads(out)
            doc["track_groups"] = groups
            if new == json.loads(json.dumps(doc)):
                return out
    doc["track_groups"] = groups
    return json.dumps(doc, indent=2) + "\n"


def sync_plan_track_group(plan_path: pathlib.Path, track: pathlib.Path, *, cause: str,
                          fit: dict = None, stage: str = "execute_path") -> dict:
    """ONE SET OF BYTES (Newton 2026-10-02, see GROUP_REWRITE_KEY): make the plan's own track group for
    `stage` the bytes `track` holds -- the .g<k>.txt, its .proj, its sidecar and the plan's `track_groups`
    entry (sha256, proj_sha256, points, duration_nominal_s, band_s, first/last rows, gates) -- recording the
    planner's own figures and the cause under `emit_rewrite`. A group that already holds these bytes is
    left byte-identical. Returns what it found / did (the manifest row records it as `plan_group`)."""
    from alix_dispatch import track as tr
    plan_path = pathlib.Path(plan_path)
    raw = plan_path.read_text()
    doc = json.loads(raw)
    groups = doc.get("track_groups") or []
    tbytes = pathlib.Path(track).read_bytes()
    tsha = hashlib.sha256(tbytes).hexdigest()
    out = dict(plan=repo_rel(plan_path), track_sha256=tsha, group=None, rewritten=False)
    if not groups:
        out.update(equals_track=False, why="the plan carries no track group: the C++ lanes have no bytes of "
                                            "their own to stream for it (only this manifest track)")
        return out
    names = [s.get("stage_name") or s.get("name") for s in doc.get("sub_trajectories") or []]
    k = next((i for i, g in enumerate(groups)
              if any(0 <= int(si) < len(names) and names[int(si)] == stage for si in g.get("stage_indices") or [])),
             None)
    if k is None:
        out.update(equals_track=False, why=f"no track group covers the plan's {stage} stage")
        return out
    g = groups[k]
    out["group"] = int(g.get("group", k))
    if [names[int(si)] for si in g.get("stage_indices") or []] != [stage]:
        out.update(equals_track=False, why=f"group g{out['group']} covers {g.get('stage_names')}, not {stage} "
                                            "alone: the emitted track is not that group's bytes (left as planned)")
        return out
    if g.get("sha256") == tsha:
        out.update(equals_track=True)
        return out
    pdir = plan_path.parent
    g_txt, g_proj = pdir / g["track_path"], pdir / g["proj_path"]
    g_json = pdir / g["sidecar_path"] if g.get("sidecar_path") else None
    rows = tr.parse_track(track)
    n = len(rows)
    clk = track_clock_s(track)
    side = "R" if str(g.get("side") or "").upper().startswith("R") else None
    if side is None:
        ex = next((s for s in doc.get("sub_trajectories") or [] if (s.get("stage_name") or s.get("name")) == stage), {})
        side = (ex.get("joint_names") or ["R"])[0][0]
    planner = {kk: g.get(kk) for kk in ("sha256", "proj_sha256", "points", "clock_s", "duration_nominal_s",
                                         "band_s", "last_row_millideg")}
    prev = g.get(GROUP_REWRITE_KEY)
    if isinstance(prev, dict) and prev.get("planner_group"):
        planner = prev["planner_group"]              # a second rewrite keeps the PLANNER's figures
    g_txt.write_bytes(tbytes)
    tr.build_project_file(g_txt, g_proj)
    proj_sha = hashlib.sha256(g_proj.read_bytes()).hexdigest()
    gates = track_gates(rows, side, clk)
    when = datetime.datetime.now().astimezone().isoformat(timespec="seconds")
    rec = {
        "by": "tools/cleaning_tracks.py emit (sync_plan_track_group)",
        "ruling": "Newton 2026-10-02: any plan can be channeled to CANFD or the track upload channel "
                  "irrespective of the channel set at generation -> ONE set of bytes for both",
        "cause": cause,
        "when": when,
        "manifest_track": repo_rel(track),
        "planner_group": planner,
        "rate_fit": ({kk: fit.get(kk) for kk in ("applied", "slowdown", "lines_before", "lines_after",
                                                "duration_before_s", "duration_after_s", "iterations",
                                                "design_allowance", "clock_s") if kk in fit}
                     if fit and fit.get("applied") else None),
    }
    g.update(sha256=tsha, proj_sha256=proj_sha, points=n, clock_s=clk, duration_nominal_s=(n - 1) * clk,
             band_s=list(tr.expected_band_s(n)), first_row_millideg=[int(v) for v in rows[0]],
             last_row_millideg=[int(v) for v in rows[-1]], gates=gates)
    g[GROUP_REWRITE_KEY] = rec
    plan_path.write_text(_splice_track_groups(raw, groups))
    if g_json is not None:
        sc = json.loads(g_json.read_text()) if g_json.is_file() else {}
        sc.update(clock_s=clk, points=n, track_duration_s=(n - 1) * clk, band_s=list(tr.expected_band_s(n)),
                  sha256=tsha, proj_sha256=proj_sha, track_bytes=len(tbytes), proj_bytes=g_proj.stat().st_size,
                  first_row_millideg=g["first_row_millideg"], last_row_millideg=g["last_row_millideg"],
                  band_limited_util={kk: gates["band_limited_util"][kk] for kk in ("fc_hz", "vel", "acc", "jerk")}
                  | {"measured_on": "the integer millidegrees written to the file"},
                  max_joint_rate_deg_s=gates["rate_max_deg_s"], ticks_over_180_deg_s=gates["ticks_over_180"],
                  canfd_streamable=bool(gates["ticks_over_180"] == 0),
                  per_joint_rate_deg_s=gates["per_joint_rate_deg_s"],
                  position_margin_deg=gates["position_margin_per_joint_deg"],
                  position_margin_min_deg=gates["position_margin_deg"],
                  position_margin_min_joint=gates["position_margin_min_joint"], gates=gates)
        if isinstance(sc.get("stages"), list) and len(sc["stages"]) == 1:
            sc["stages"][0]["points"] = [0, n]
        sc[GROUP_REWRITE_KEY] = rec
        g_json.write_text(json.dumps(sc, indent=2) + "\n")
    out.update(rewritten=True, equals_track=True, cause=cause, planner_group=planner,
               group_track=repo_rel(g_txt), points=n)
    return out


def emit_overrides() -> dict:
    """Per-region emit policy (Resource/tracks/<fixture>/EMIT_OVERRIDES.json),
    keyed by manifest name: 'plan' (repo-relative) replaces the hardware/
    lookup, 'acc_scale' replaces the side's scale, 'ik_frame' the config's,
    'no_fastpath' forces the re-solve."""
    p = TRACKS / "EMIT_OVERRIDES.json"
    if not p.exists():
        return {}
    return {k: v for k, v in json.load(open(p)).items() if not k.startswith("_")}


def is_section_plan_name(name: str) -> bool:
    """A section plan's file name (plan/P17 L5), sealed or not."""
    scripts = str(REPO / "ros/orchestration/alix_tasks/scripts")
    if scripts not in sys.path:
        sys.path.insert(0, scripts)
    from section_plans import is_section_name
    return is_section_name(name)


def plan_for(key: str, arm: str, ov: dict):
    """(plan path | None, reason). The ruckig_pro_only plan is the canonical
    source (Newton 2026-09-02: byte-identical to the butterfli_ws copies)."""
    if ov.get("plan"):
        plan = REPO / ov["plan"]
        return (plan, "") if plan.exists() else (None, f"override plan missing: {ov['plan']}")
    plan = PLANS / f"{key}_{arm}_ruckig_pro_only.json"
    if plan.exists():
        return plan, ""
    alt = sorted(PLANS.glob(f"{key}_{arm}_*.json"))
    return (alt[0], "") if alt else (None, "NO PLAN in hardware/")


def section_plans_of(key: str, arm: str) -> list:
    """Every SECTION plan of (key, arm) in hardware/ -- sealed or not, any mode
    (plan/P17 L5; the name rules live in the executive's pure
    `section_plans`, read here so the two lanes cannot disagree on a name)."""
    scripts = str(REPO / "ros/orchestration/alix_tasks/scripts")
    if scripts not in sys.path:
        sys.path.insert(0, scripts)
    from section_plans import section_plans_for
    return [p for p, _ in section_plans_for(PLANS, key, arm)]


def emit_one(key: str, arm: str, slot: int, ov: dict, acc_scale: float) -> dict:
    """Emit one task into *slot*; returns its manifest row (any status)."""
    name = f"{FIXTURE}_{key}_{arm}"
    row = dict(slot=None, name=name, side=arm, status="REFUSED")
    # A SECTIONED region (plan/P17 L5; review S2, 2026-09-29): N section plans,
    # ONE slot per region here -- this lane has no section route. The row is
    # SECTIONED with NO plan and NO slot (not a reserved one either: the C++
    # `Manifest::find` reads rows whatever their status, and refuses only a row
    # without a slot (--stroke track) or without a plan (--sequence)), so every
    # reader refuses it: Python readers take EMITTED rows only. A single plan
    # beside the sections is the other generation's leftover (today:
    # hinge_area_left_ruckig_pro_only.json, slot 72) and is NOT emitted -- it
    # used to be, into the region's slot.
    secs = section_plans_of(key, arm)
    if secs:
        stale, _w = plan_for(key, arm, ov)
        why = (f"SECTIONED: {len(secs)} section plan(s) in hardware/ ({', '.join(p.name for p in secs)}) -- "
               "the upload lane has one slot per region and no section route (plan/P17 L5): no plan, no slot"
               + (f"; the single plan {stale.name} beside them is the other generation's leftover and is "
                  "NOT emitted" if stale is not None and not is_section_plan_name(stale.name) else ""))
        row.update(status="SECTIONED", reason=why, sections=[p.name for p in secs])
        print(f"{'--':>4} {name:34s} {'-':>7} {'-':>8} {'-':>6}  {why[:160]}")
        return row
    plan, why = plan_for(key, arm, ov)
    if plan is None:
        row.update(status="NO_PLAN" if "NO PLAN" in why else "REFUSED", reason=why)
        print(f"{'--':>4} {name:34s} {'-':>7} {'-':>8} {'-':>6}  {why}")
        return row
    scale = float(ov.get("acc_scale", acc_scale))
    frame_prov = {}
    frame = ov.get("ik_frame") or ik_frame_for(key, arm, frame_prov)
    if ov.get("ik_frame"):
        frame_prov = dict(ik_frame_source="emit_override", ik_frame_declared=ov["ik_frame"])
    track = TRACKS / f"{slot:02d}_{name}_execute_path.txt"
    for stale in (track, track.with_suffix(".json"), track.with_suffix(".proj")):
        if stale.exists():
            stale.unlink()
    fp, r_out = None, ""
    try:
        # no_fastpath: the plan's own timing is NOT channel-optimal (e.g. an
        # in-server toppra plan timed under CONTACT limits, 0.32 m/s) — the
        # channel policy is shortest time under the acc policy, so re-solve.
        fp = None if ov.get("no_fastpath") else emit_fastpath(plan, track, frame)
    except SystemExit as e:
        row.update(reason=str(e)[:160])
        print(f"{'--':>4} {name:34s}  REFUSED (fastpath): {str(e)[:60]}")
        return row
    if fp is None:
        # the re-solve route takes the same relaxed margin (Newton 2026-10-02): plan_min_time refuses
        # only a sample beyond a limit; its own 5.0 default stays for its standalone use
        r = subprocess.run([sys.executable, str(REPO / "tools/plan_min_time.py"), str(plan),
                            *emit_flags(scale), "--ik-frame", frame,
                            "--min-position-margin-deg", f"{POSITION_MARGIN_REFUSE_BELOW_DEG:g}",
                            "--emit-track", str(track)],
                           capture_output=True, text=True, timeout=900)
        r_out = r.stdout + r.stderr
    if not track.exists():
        tail = r_out.strip().splitlines() or ["fastpath produced no track"]
        reason = next((l for l in reversed(tail) if "*" in l or "REFUSED" in l), "?")
        row.update(reason=reason.strip()[:160])
        print(f"{'--':>4} {name:34s} {'-':>7} {'-':>8} {'-':>6}  REFUSED: {reason.strip()[:70]}")
        return row
    # FIT THE RATINGS AT THE PLAYER'S CEILING (2026-09-17): slow only the stretches a joint would
    # clamp in at 5.0 ms/pt, points kept on the emitted chords. emit_fastpath itself is untouched —
    # it is the C++ track_emit parity reference — so the fit is its own recorded step here.
    try:
        fit = rate_fit_track(track)
    except SystemExit as e:
        track.unlink(missing_ok=True)
        track.with_suffix(".json").unlink(missing_ok=True)
        row.update(reason=str(e)[:160])
        print(f"{'--':>4} {name:34s}  REFUSED (rate fit): {str(e)[:60]}")
        return row
    if fit["applied"]:
        refresh_track_sidecar(track, frame, fit)
    step = tick_step_check(track)
    step_gate = tick_step_gate(step, scale, plan, track, fastpath=fp is not None, refit=bool(fit["applied"]))
    step_max = step_gate["limit_deg_s_exact"]
    if step > step_max:
        track.unlink()
        track.with_suffix(".json").unlink(missing_ok=True)
        row.update(reason=f"tick step {step:.2f} deg/s > {step_max:.1f} "
                          f"(broken sampler / F101-class source; {step_gate['kind']})")
        print(f"{'--':>4} {name:34s}  REFUSED: tick step {step:.2f}")
        return row
    n_over, peaks = over_limit_ticks(track)
    if n_over:
        track.unlink()
        track.with_suffix(".json").unlink(missing_ok=True)
        row.update(reason=f"{n_over} tick(s) command a joint over its RATED speed at the player's "
                          f"5.0 ms ceiling (peaks {peaks}) — error 4105 and the clamp (F111)")
        print(f"{'--':>4} {name:34s}  REFUSED: {n_over} ticks over the rated speed")
        return row
    sc = json.load(open(track.with_suffix(".json")))
    if "position_margin_advisory" not in sc and sc.get("position_margin_deg"):
        # the re-solve route's sidecar (plan_min_time) carries the per-joint margins but not the
        # 2026-10-02 advisory: record it there too, so every emitted row reads it the same way
        sc.update(position_margin_record(sc["position_margin_deg"]))
        track.with_suffix(".json").write_text(json.dumps(sc, indent=1))
    pos_adv = sc.get("position_margin_advisory")
    # THE ACCELERATION FLAG (Newton 2026-10-02 "Keep advisory, flag it"; ACC_ADVISORY_KEY): the commanded 20 Hz
    # acceleration of the FINAL bytes (after the rate fit), recorded in the sidecar and the row. Never a refusal,
    # never a re-time: a measurement failure is recorded as such and the row is emitted exactly as before.
    try:
        acc_adv = acceleration_advisory(track)
    except Exception as exc:                                            # noqa: BLE001 -- advisory only
        acc_adv = dict(advisory=True, refuses=False, flagged=None, rule=ACC_ADVISORY_RULE,
                       error=f"{type(exc).__name__}: {str(exc)[:160]}")
    sc[ACC_ADVISORY_KEY] = acc_adv
    track.with_suffix(".json").write_text(json.dumps(sc, indent=1))
    # ONE SET OF BYTES (Newton 2026-10-02, GROUP_REWRITE_KEY): the bytes written above are what the upload
    # player plays and what the sweeps measure; the plan's own track group (what the C++ CANFD lane streams)
    # becomes the same bytes when they differ. Done BEFORE the row reads the plan, so `plan_sha256` pins the
    # plan as rewritten. A sync that cannot complete refuses the row: two sets of bytes are never emitted.
    cause = ("rate fit: the upload player's 5.0 ms ceiling re-timed the track (rate_fit_track, "
             f"slowdown {fit.get('slowdown')}, {fit.get('lines_before')} -> {fit.get('lines_after')} lines)"
             if fit["applied"] else
             "re-solve: plan_min_time re-timed the plan under the channel acc policy (no fastpath)"
             if fp is None else
             "resample: the plan's group was not the 5.0 ms fastpath of its own execute_path")
    try:
        group_sync = sync_plan_track_group(plan, track, cause=cause, fit=fit)
    except Exception as exc:                                            # noqa: BLE001
        track.unlink(missing_ok=True)
        track.with_suffix(".json").unlink(missing_ok=True)
        row.update(reason=f"the plan's track group could not be set to the emitted bytes "
                          f"({type(exc).__name__}: {str(exc)[:120]})")
        print(f"{'--':>4} {name:34s}  REFUSED: plan group sync failed: {str(exc)[:60]}")
        return row
    d = json.load(open(plan))
    st = next(x for x in d["sub_trajectories"]
              if (x.get("stage_name") or x.get("name")) == "execute_path")
    plan_s = float(st["waypoints"][-1]["time_from_start"])
    n = sc["points"]
    from alix_dispatch import peripherals as per
    pre = per.pre_stroke_state(d, arm)
    ret = per.retreat_pose(d)
    # THE PAD DECIDES THE HAND (Newton, 2026-09-05). glove_frames.yaml is the
    # authority for which state each frame wipes with; the plan's own hand
    # stage is kept alongside as provenance, not used. They disagreed on 4 of
    # 23 tasks — bottomlid R and L stroked at `grasp`, hinge_area_left at
    # `quarter_grasp`, bowl_inside_rim_left at `half_grasp`, where the frame
    # declares `open_tenth`.
    hand_state, hand_counts = hand_from_frame(frame, arm)
    hand_prov = dict(source="glove_frames.yaml", frame=frame,
                     plan_stage=pre.get("hand_state"),
                     plan_counts=pre.get("hand_counts"),
                     agrees=(pre.get("hand_counts") == hand_counts))
    row.update(
        slot=slot, status="EMITTED",
        plan=repo_rel(plan, REPO), plan_sha256=hashlib.sha256(plan.read_bytes()).hexdigest(),
        plan_mode=d.get("cleaning_path_mode"), track=repo_rel(track, REPO),
        points=n, plan_execute_s=round(plan_s, 3),
        track_s=round(sc["track_duration_s"], 4),
        speedup=round(plan_s / sc["track_duration_s"], 3),
        band_5p5_5p6_s=[round((n - 1) * 0.0055, 3), round((n - 1) * 0.0056, 3)],
        u_vel=round(sc["band_limited_util"]["vel"], 3),
        u_acc=round(sc["band_limited_util"]["acc"], 3),
        u_jerk=round(sc["band_limited_util"]["jerk"], 3),
        peak_rate_deg_s=round(sc["max_joint_rate_deg_s"], 1),
        pos_margin_deg=round(sc["position_margin_min_deg"], 3),
        **({} if pos_adv is None else dict(pos_margin_joint=pos_adv["joint"], pos_margin_advisory=pos_adv)),
        **{ACC_ADVISORY_KEY: (acceleration_advisory_row(acc_adv) if "error" not in acc_adv else acc_adv)},
        tick_step_deg_s=round(step, 2),
        tick_step_gate={k: v for k, v in step_gate.items() if k != "limit_deg_s_exact"},
        ik_frame=frame, **frame_prov,
        acc_scale=float(sc.get("acc_scale", scale)), vel_scale=float(sc.get("vel_scale", 1.0)),
        ticks_over_rated=n_over, rate_ceiling_clock_s=RATE_CEILING_CLOCK_S,
        rate_fit=({k: fit[k] for k in ("applied", "slowdown", "lines_before", "lines_after",
                                       "duration_before_s", "duration_after_s", "iterations",
                                       "design_allowance") if k in fit} if fit["applied"] else None),
        pre_stroke=dict(pole_m=pre["pole_m"], pole_stage=pre["pole_stage"],
                        hand_state=hand_state, hand_counts=hand_counts,
                        hand_provenance=hand_prov,
                        hand_release_counts=pre["hand_release_counts"]),
        retreat_stage=(ret or {}).get("stage"), retreat_deg=(ret or {}).get("deg"),
        plan_group={k: group_sync.get(k) for k in ("group", "equals_track", "rewritten", "cause", "why")
                    if group_sync.get(k) is not None})
    # PIN THE BYTES (B15). The .proj is built HERE, not first at upload: a row
    # that reaches the controller with no local .proj has nothing for the
    # stored-size gate to compare against, which is exactly how `toplid_left`
    # ran four times on bytes that are not in this tree. build_project_file is
    # a pure function of the track, so building it at emit changes nothing
    # about what is uploaded — it just means the comparison always exists.
    d = disk_provenance(row["track"], repo=REPO, build=True)
    row.update(track_sha256=d["track_sha256"], track_bytes=d["track_bytes"],
               points_on_disk=d["points_on_disk"], proj=d["proj"],
               proj_sha256=d["proj_sha256"], proj_bytes=d["proj_bytes"])
    print(f"{slot:>4} {name:34s} {plan_s:7.2f} {sc['track_duration_s']:8.3f} "
          f"{plan_s / sc['track_duration_s']:5.2f}x  OK  (acc {scale:g}, pole "
          f"{pre['pole_m']}, hand {pre['hand_state']}, retreat {(ret or {}).get('stage')})")
    return row


def cmd_emit(args) -> int:
    TRACKS.mkdir(parents=True, exist_ok=True)
    side = args.side
    want = set(args.region or [])
    overrides = emit_overrides()
    man = load_manifest() if (TRACKS / "MANIFEST.json").exists() else dict(tasks=[])
    if "sides" not in man:
        # a pre-2026-09-02 manifest (one slot counter across both arms): its rows
        # are wrong under the per-arm convention — start clean, both sides
        man = dict(tasks=[])
    print(f"{'slot':>4} {'task':34s} {'plan':>7} {'emit':>8} {'gain':>6}  result")
    if want:
        # partial: re-emit named regions INTO their existing slots; rows replaced in place
        for key in sorted(want):
            name = f"{FIXTURE}_{key}_{side}"
            cur = next((t for t in man["tasks"] if t["name"] == name), None)
            # a SECTIONED row keeps its number as `reservedSlot` (emit_one)
            cur_slot = None if cur is None else (cur.get("slot") if cur.get("slot") is not None
                                                 else cur.get("reservedSlot"))
            # a row outside the upload task list takes its DECLARED CANFD-only number (2026-10-02)
            canfd = declared_canfd_slot(name, side, overrides.get(name, {}), man)
            if cur_slot is None and canfd is not None:
                cur_slot = canfd
            if cur_slot is None:
                raise SystemExit(f"{name}: not in the manifest with a slot — run a full "
                                 f"`emit --side {side}` first (or declare a CANFD-only `canfd_slot` "
                                 f"in EMIT_OVERRIDES.json)")
            row = emit_one(key, side, cur_slot, overrides.get(name, {}), args.acc_scale)
            if canfd is not None and cur_slot == canfd:
                row["slot_source"] = ("EMIT_OVERRIDES.json canfd_slot: CANFD-only (Newton 2026-09-22; "
                                      "the front regions brought in 2026-10-02)")
            if row["status"] == "SECTIONED":
                row["reservedSlot"] = cur_slot   # no slot on a SECTIONED row (emit_one)
            elif row["status"] != "EMITTED":
                row["slot"] = cur_slot         # the slot stays reserved for it
            if cur is not None and cur.get("tool_frame") and not row.get("tool_frame"):
                row["tool_frame"] = cur["tool_frame"]   # survives a re-emit (H35)
            man["tasks"] = [t for t in man["tasks"] if t["name"] != name] + [row]
            attach_evidence([row])
        refresh_not_ready_to_pair(man)      # new bytes are not ready to pair, and the manifest says so
        save_manifest(man)
        print(f"\nmanifest updated in place -> {TRACKS / 'MANIFEST.json'}")
        return 0
    # full emit for ONE side: this side's files are ours — clear them first so a
    # renumbering never leaves a stale twin beside a live track (2026-09-01
    # left 22 orphans from a refused-row shift).
    # THE OLD FILES ARE MOVED ASIDE, NOT DELETED, UNTIL THE EMIT SUCCEEDS (2026-09-17). They were
    # unlinked here, up front, so a refusal partway through — one region whose config could not be
    # resolved — left the side with the emitted rows rewritten, the rest of the files GONE and the
    # manifest still pinning them. That happened on the right side today: 2 rows re-emitted, 10 rows'
    # files removed, `emit` dead on a SystemExit, and only the git index to recover from. Now the
    # sweep is reversible and the failure path restores it.
    import shutil as _sh
    import tempfile as _tf
    stale = sorted(p for p in TRACKS.glob(f"*_{FIXTURE}_*_{side}_execute_path.*"))
    _hold = pathlib.Path(_tf.mkdtemp(prefix=f"emit_{side}_", dir=str(TRACKS)))
    for p in stale:
        _sh.move(str(p), str(_hold / p.name))
    if stale:
        print(f"(moved {len(stale)} previous {side}-side track files aside; they are restored if this "
              f"emit cannot finish)")

    def _restore_stale():
        # PUT THE SIDE BACK EXACTLY AS IT WAS. An earlier version skipped a target that already
        # existed, which left the rows the emit HAD rewritten carrying new bytes against the manifest's
        # old pins — 5 inconsistent rows, the same fault in a smaller form. A restore that is not
        # total is not a restore.
        for q in sorted(_hold.glob("*")):
            tgt = TRACKS / q.name
            tgt.unlink(missing_ok=True)
            _sh.move(str(q), str(tgt))
        # anything the emit wrote for a row it had NOT swept aside (a new slot) is also undone
        _sh.rmtree(_hold, ignore_errors=True)

    def _drop_stale():
        _sh.rmtree(_hold, ignore_errors=True)
    keep = [t for t in man["tasks"] if t.get("side", side_of(t["name"])) != side]
    rows, slot = [], SLOT_BASE
    try:
        return _emit_side_rows(args, man, keep, rows, slot, side, overrides, _drop_stale)
    except BaseException:
        _restore_stale()
        print(f"** emit did not finish — the {len(stale)} previous {side}-side track files were put back, "
              f"so the manifest still describes what is on disk. Nothing was uploaded.")
        raise


def _emit_side_rows(args, man, keep, rows, slot, side, overrides, _drop_stale):
    """The whole-side emit loop, split out so its caller can restore the swept files on any failure."""
    for key, included in side_tasks(side):
        name = f"{FIXTURE}_{key}_{side}"
        if not included:
            # A row outside the upload task list with a DECLARED CANFD-only number (2026-10-02,
            # `declared_canfd_slot`) is emitted there; the production counter does not move.
            canfd = declared_canfd_slot(name, side, overrides.get(name, {}), man)
            if canfd is not None:
                row = emit_one(key, side, canfd, overrides.get(name, {}), args.acc_scale)
                if row.get("slot") is None and row.get("status") != "SECTIONED":
                    row["slot"] = canfd          # reserved, even when refused
                row["slot_source"] = ("EMIT_OVERRIDES.json canfd_slot: CANFD-only (Newton 2026-09-22; "
                                      "the front regions brought in 2026-10-02)")
                rows.append(row)
                continue
            rows.append(dict(slot=None, name=name, side=side, status="EXCLUDED",
                             reason="not in Newton's task list (2026-09-02)"))
            print(f"{'--':>4} {name:34s} {'-':>7} {'-':>8} {'-':>6}  EXCLUDED (not in the task list)")
            continue
        row = emit_one(key, side, slot, overrides.get(name, {}), args.acc_scale)
        rows.append(row)
        if row.get("status") == "SECTIONED":
            # the NUMBER stays reserved (the invariant below: nothing after it
            # moves), but not as `slot` -- a SECTIONED row carries no slot at all
            # (emit_one: the C++ manifest reader does not filter on status)
            row["reservedSlot"] = slot
        # RESERVE the slot for every task in the list, emitted or refused.
        # A refused row used to consume no slot, so everything after it moved
        # down one — and fixing that one region later renumbered the whole
        # side. A slot whose meaning changes is the exact fault that made 21 of
        # 116 run records reconstruct at another region's ik_frame (2026-09-05
        # audit); the ledger keys on the region NAME now, but the slot should
        # not be moving underneath it either. `rows_for(..., emitted_only=True)`
        # still hides a refused row from upload, preflight and the flow, so the
        # reservation costs a gap in the controller's slot list and nothing else.
        if row.get("slot") is None and row.get("status") != "SECTIONED":
            row["slot"] = slot                 # reserved, even when refused
        slot += 1
        if slot > SLOT_MAX:
            raise SystemExit(f"slot allocation exceeded {SLOT_MAX} — the reserved range is full")
    # The tool-frame expectation is EVIDENCE (run records), not an emit
    # product, so a re-emit must not silently drop it (H35).
    prev = {t["name"]: t for t in man["tasks"]}
    for row in rows:
        keep_tf = prev.get(row["name"], {}).get("tool_frame")
        if keep_tf and not row.get("tool_frame"):
            row["tool_frame"] = keep_tf
    attach_evidence(rows)
    man["tasks"] = keep + rows
    man.setdefault("sides", {})[side] = dict(
        emitted_at=datetime.datetime.now().isoformat(timespec="seconds"),
        acc_scale=args.acc_scale, slots=[t["slot"] for t in rows if t["status"] == "EMITTED"])
    refresh_not_ready_to_pair(man)      # new bytes are not ready to pair, and the manifest says so
    save_manifest(man)
    _drop_stale()                  # the emit stands: the swept files are not coming back
    n_ok = sum(1 for t in rows if t["status"] == "EMITTED")
    print(f"\n{side}: {n_ok} tracks in slots {SLOT_BASE}-{slot - 1} at acc-scale {args.acc_scale:g}"
          f"   manifest -> {TRACKS / 'MANIFEST.json'}")
    return 0


# ------------------------------------------------------------------- status
def cmd_status(args) -> int:
    man = load_manifest()
    for side in SIDES:
        em = rows_for(man, side)
        others = [t for t in rows_for(man, side, emitted_only=False) if t["status"] != "EMITTED"]
        if not em and not others:
            continue
        meta = man.get("sides", {}).get(side, {})
        span = f", slots {em[0]['slot']}-{em[-1]['slot']}" if em else ""
        when = (f"  emitted {meta.get('emitted_at')} @ acc-scale {meta.get('acc_scale')}"
                if meta else "")
        print(f"\n{man['fixture']} {side.upper()} arm: {len(em)} tracks{span} "
              f"(reserved {SLOT_BASE}-{SLOT_MAX} on this arm's controller){when}")
        print(f"{'slot':>4} {'name':34s} {'pts':>5} {'emit s':>8} {'band s':>16} {'margin':>7} "
              f"{'acc':>4} {'pole':>6} hand")
        for t in em:
            ps = t.get("pre_stroke", {})
            print(f"{t['slot']:>4} {t['name']:34s} {t['points']:>5} {t['track_s']:>8.3f} "
                  f"[{t['band_5p5_5p6_s'][0]:6.2f},{t['band_5p5_5p6_s'][1]:6.2f}] "
                  f"{t['pos_margin_deg']:>6.2f}° {t.get('acc_scale', 1.0):>4g} "
                  f"{(ps.get('pole_m') if ps.get('pole_m') is not None else '-')!s:>6} "
                  f"{ps.get('hand_state') or '-'}")
        for t in others:
            print(f"{'--':>4} {t['name']:34s} {t['status']}: {t.get('reason', '')[:60]}")
    never = [t["name"] for t in man["tasks"]
             if (t.get("observed_runs") or {}).get("never_run_on_disk_bytes")]
    drift = [t["name"] for t in man["tasks"] if t.get("status") == "EMITTED"
             and not check_row_provenance(t, REPO)["ok"]]
    if never:
        print(f"\n!! the bytes on disk have NEVER been run for: {never} (B15) — the "
              f"controller was fed something else on every recorded run of that slot")
    if drift:
        print(f"\n** rows that no longer match their pins: {drift} — `upload` and `run` "
              f"REFUSE these (see `provenance`)")
    if args.robot:
        from alix_dispatch import connect, track as tr
        h = open_or_die(args.side, args.ip)
        try:
            ret, items = tr.program_list(h.robot)
            print(f"\n{args.side} controller slot list (ret={ret}):")
            local = {}
            for t in rows_for(man, args.side):
                pj = (REPO / t["track"]).with_suffix(".proj")
                if pj.is_file():
                    local[t["slot"]] = pj.stat().st_size
            for it in items:
                # vendor rm_trajectory_data_t: id/size/speed/trajectory_name
                # (aliased 'name' by program_list; V1.7.4 carries no point count)
                size = it.get("size") or 0
                want = local.get(it.get("id"))
                cmp_ = ""
                if want:
                    d = size - want
                    cmp_ = (f"  local {want / 1024:6.1f}KB delta {d:+7d} B"
                            + ("" if abs(d) <= max(64, want // 1000) else "  ** MISMATCH"))
                print(f"   id {it.get('id'):>3}: {it.get('name')}  "
                      f"{size / 1024:6.1f}KB  stored speed {it.get('speed')}{cmp_}")
            print("   (a slot storing fewer bytes than were sent would end its program "
                  "early with no error — the F112 signature. The .proj must be present "
                  "locally for the comparison; `upload` writes it.)")
        finally:
            connect.close_arm(h)
    return 0


# ------------------------------------------------------- the provenance verb
def refresh_not_ready_to_pair(man: dict) -> list:
    """Re-derive the manifest's OWN statement of which rows may not be paired yet, from the rows.

    B15's precondition is a clean single-arm run on the bytes NOW ON DISK, so the moment an emit gives a
    row new bytes that row is not ready to pair — and the list has to say so. It was written only by
    `provenance --write`, so a re-emit left the manifest asserting that rows were ready when their bytes
    had never been run: after today's rate-fit re-emit, 20 of 24 rows were in that state and the list
    still named none of them (2026-09-17). Called by every emit path and by `provenance --write`.
    """
    names = sorted(t["name"] for t in man["tasks"]
                   if t.get("status") == "EMITTED"
                   and not (t.get("observed_runs") or {}).get("clean_single_arm_run_on_disk_bytes"))
    prov = man.setdefault("provenance", {})
    prov["not_ready_to_pair"] = names
    prov["not_ready_to_pair_means"] = (
        "no clean single-arm run exists on the bytes now on disk (B15). Run each "
        "of these alone, to completion, before it is paired with anything. The "
        "same question in code, for the dual executive: "
        "cleaning_tracks.pairing_gate(row) -> {'ok', 'reasons', ...}.")
    return names


def manifest_for(args) -> dict:
    """The row source for a verb: the production manifest, the dry registry
    when `--dry` is given, or the taught lane's registry when `--taught` is.
    All three are manifest-shaped on purpose, so every consumer downstream —
    `rows_for`, `by_name`, `check_row_provenance`, `controller_slot_check`,
    `preflight_report`, `run_one` — is unchanged (H37, and the taught lane's
    plan of 2026-09-14 section 8)."""
    if getattr(args, "taught", False):
        return load_taught_registry()
    if getattr(args, "experiment", False):
        return load_experiment_registry()
    return load_dry_registry() if getattr(args, "dry", False) else load_manifest()


def cmd_provenance(args) -> int:
    """Pin every emitted row to the bytes on disk, and say what the run ledger
    says was actually played (B15, H35). OFFLINE and read-only unless --write.
    """
    man = load_manifest()
    ev = run_record_evidence()
    rows = [t for t in man["tasks"] if t.get("status") == "EMITTED"]
    if args.side:
        rows = [t for t in rows if t.get("side", side_of(t["name"])) == args.side]
    changed, flagged, unwritable, drifted, repinned = 0, [], [], [], []
    # TWO FRAME COLUMNS, SIDE BY SIDE, AND NO COMPARISON BETWEEN THEM
    # (2026-09-09). `ik frame` is the task's AUTHORED CONTACT frame — Newton
    # has ruled that association settled — and `tool frame` is the
    # CONTROLLER-side supervision setting the arm was observed holding. They
    # are different namespaces; a joint-space replay cannot be moved by the
    # latter, which reaches the Cartesian caps, gravity compensation and the
    # controller's own collision stage (F31) — but never the joint path.
    # Printing both as facts is provenance. Deriving one from the other and
    # calling the difference an open question was the defect.
    print("  ik frame   = the task's AUTHORED frame: it selects the HAND STATE from "
          "glove_frames.yaml,\n               with the contact axis and pad.")
    print("  tool frame = the CONTROLLER's supervision setting: it fixes where the "
          "Cartesian caps are\n               evaluated and carries the payload "
          "gravity compensation reads. 157 runs,\n               two different frames "
          "throughout, no collision-stage abort in any of them.")
    print("  Two namespaces, two facts. They are NOT required to be equal, and a "
          "joint-space replay is\n  unaffected by the second one.\n")
    print(f"{'slot':>4} {'task':38s} {'pts':>5} {'track sha':>10} {'proj':>10} "
          f"{'ik frame':16s} {'tool frame':11s} runs")
    for t in rows:
        d = disk_provenance(t["track"], build=args.write)
        obs = observed_runs_for(t, ev, d["track_sha256"])
        tf = tool_frame_from_records(t, ev)
        if args.write:
            if d["track_sha256"] is None:
                # NEVER unpin a row by writing a null over a good pin. A file
                # that is temporarily absent (a half-finished move, a bad
                # checkout) would otherwise erase the only record of which
                # bytes the row means, which is B15's own failure in reverse.
                print(f"  ** {t['name']}: {t['track']} is missing or unreadable — its "
                      f"existing pin is KEPT and nothing was written for it")
                unwritable.append(t["name"])
                continue
            # SAY WHICH PINS MOVED (hardening review 2026-09-08). `--write` is
            # the one command in this tool that can erase a B15: it adopts
            # whatever is on disk as the truth. Adopting is often right — a
            # re-emit — but it must never be silent, because after it runs the
            # old sha exists nowhere.
            if t.get("track_sha256") and t["track_sha256"] != d["track_sha256"]:
                repinned.append(t["name"])
                print(f"  ** {t['name']}: RE-PINNED {t['track_sha256'][:10]} -> "
                      f"{(d['track_sha256'] or '')[:10]} ({t.get('points')} -> "
                      f"{d['points_on_disk']} pts). The old bytes are now recorded "
                      f"nowhere; the controller may still be holding them.")
            for k in ("track_sha256", "track_bytes", "points_on_disk", "proj",
                      "proj_sha256", "proj_bytes"):
                t[k] = d[k]
            t["observed_runs"] = obs
            if tf is not None:
                t["tool_frame"] = tf
            changed += 1
        tfn = (tf or {}).get("name") or "-"
        flag = ""
        # THE VERB THAT REPORTS THE PINS MUST COMPARE THEM (hardening review
        # 2026-09-08). Read-only, this loop printed the sha it had just read
        # OFF THE DISK in the column headed "track sha", so a row whose pin no
        # longer matched its file looked exactly like one that did — the
        # operator's own check for B15 could not see B15. `upload`, `run` and
        # `status` all compared; the dedicated verb did not.
        if not args.write:
            chk = check_row_provenance(t, REPO)
            for why in chk["reasons"]:
                flag += f"  ** {why}"
            if not chk["ok"]:
                drifted.append(t["name"])
        if d["points_on_disk"] != t.get("points"):
            flag += "  ** points differ from the row"
        if obs.get("never_run_on_disk_bytes"):
            flag += "  ** these bytes have NEVER been run"
        if obs.get("other_bytes"):
            flag += (f"  ** {obs['n_runs_other']} run(s) on other bytes "
                     f"{[o['track_sha256'][:8] for o in obs['other_bytes']]}")
        # NOTE what is NOT here any more: a flag comparing `ik_frame` to the
        # controller's frame. Both are printed below as columns; neither is a
        # question, so neither may enter `flagged`, which is reserved for the
        # byte-level questions (B15) that really are open.
        if flag:
            flagged.append((t["name"], flag.strip()))
        print(f"{t['slot']:>4} {t['name']:38s} {d['points_on_disk'] or 0:>5} "
              f"{(d['track_sha256'] or '-')[:10]:>10} {(d['proj_sha256'] or '-')[:10]:>10} "
              f"{(t.get('ik_frame') or '-'):16s} {tfn:11s} "
              f"{obs.get('n_runs', 0):>3}{flag}")
    if args.write:
        man["provenance"] = dict(
            written=datetime.datetime.now().isoformat(timespec="seconds"),
            what=("per-row sha256 of the track and of the .proj that is uploaded from it, "
                  "the point count read back off the disk file, the tool frame the "
                  "controller was OBSERVED holding for that task, and which bytes the run "
                  "ledger says the slot has actually played (B15/H35, 2026-09-08 design)."),
            runs_root=str(RUNS_ROOT.relative_to(REPO)),
            gates=("`upload` refuses a row whose on-disk track no longer hashes to "
                   "track_sha256, or that is unpinned; `preflight` compares the "
                   "controller's stored size and trajectory_name against the row and reads "
                   "the tool frame back; `run` records (never refuses, since 2026-09-15) "
                   "a controller tool frame that disagrees with the row, and refuses a "
                   "slot whose STORED size is not this row's .proj (the disk pin cannot "
                   "see the arm)."),
            # DERIVED, never asserted: which rows may be run beside a live
            # partner. `complete` is the finish-event label, so a row whose
            # disk bytes have run and never completed is listed here too — a
            # run count cannot answer B15's precondition.
            )
        refresh_not_ready_to_pair(man)
        save_manifest(man)
        print(f"\n{changed} row(s) pinned -> {(TRACKS / 'MANIFEST.json').relative_to(REPO)}"
              + (f"   ** {len(repinned)} row(s) CHANGED their pin: {repinned}"
                 if repinned else ""))
    if flagged:
        # These are the BYTE questions — which bytes a slot has actually played
        # (B15) — and they are genuinely open. The frame columns above are not
        # in here: they are two recorded facts, not a disagreement to resolve.
        print("\nOPEN QUESTIONS — which BYTES a slot has played (B15). Stated, not "
              "resolved:")
        for name, why in flagged:
            print(f"  {name}: {why}")
    if unwritable:
        print(f"\n** {len(unwritable)} row(s) could not be pinned: {unwritable}. Their old "
              f"pins stand, so every gate still refuses them until the files are back.")
        return 4
    if drifted:
        print(f"\n** {len(drifted)} row(s) do NOT match their pins: {drifted}. `upload` and "
              f"`run` REFUSE these. Re-emit the region, or `provenance --write` if the disk "
              f"bytes are the ones you mean to pin — and say so in the record.")
        return 4
    print(f"\n{len(rows)} row(s) match their pins"
          f"{' (nothing was written; --write to re-pin)' if not args.write else ''}")
    return 0


# ------------------------------------------------------- the DRY track (H37)
# H37 (2026-09-08 audit). The first-run ladder's zero-exposure rungs D1/D2 —
# "two arms move at once, but neither leaves its rest pose" — could not be
# executed at all: every upload/run verb resolves through MANIFEST and no verb
# takes a file plus a slot, so D1 and D2 get skipped or improvised and the
# first two arms ever moved together do so with a real stroke on each side.
# `dry` synthesises that track. It holds the side's own `move_to_rest` pose and
# dithers ONE joint by at most the roll angle, at the player's 5.5545 ms clock,
# and it is registered in DRY_TRACKS.json — a sibling of the manifest — so the
# production manifest stays clean while `by_name`, `preflight_slots`,
# `run_verified` and `run_one` all work unmodified.
#
# SAFE BY CONSTRUCTION, not by inspection: the points are built as
# rest_millideg + round(A * sin(...)) on one joint and rest_millideg exactly on
# the other six, with A the integer millidegree roll. The deviation from rest
# is therefore <= the roll angle on every joint of every point as an ARITHMETIC
# property, and `test_track_provenance.py` asserts it over the generated file.
DRY_DIR = TRACKS / "dry"
DRY_REGISTRY = TRACKS / "DRY_TRACKS.json"
#: THE TAUGHT-TRACK LANE's namespace (Resource/tracks/taught/, 2026-09-14).
#: Same contract as the dry registry: the rows are MANIFEST-SHAPED and PINNED,
#: so `rows_for`, `by_name`, `check_row_provenance`, `controller_slot_check`,
#: `trajectory_name_check`, `preflight_report` and `run_one` read them with no
#: changes — `manifest_for` is the single row-source choke point. It exists
#: because a taught motion is not a cleaning task and must never be able to
#: overwrite one: its slots are 40-59 (upload-capable) or 101 and up (CANFD-only), both OUTSIDE
#: the 70-100 production range.
TAUGHT_DIR = REPO / "Resource" / "tracks" / "taught"
TAUGHT_REGISTRY = TAUGHT_DIR / "TAUGHT_TRACKS.json"
TAUGHT_SOURCE_DIR = TAUGHT_DIR / "source"
TAUGHT_RUNGS_DIR = TAUGHT_DIR / "rungs"
#: Inclusive. A taught row inside 70-100 is refused by name — the same rule
#: `build_dry_row` applies — so a lane bug can never overwrite a task.
TAUGHT_SLOT_RANGE = (40, 59)
#: CANFD-ONLY TAUGHT ROWS (Newton, 2026-09-22: "canfd is not limited to 100 slots, because nothing
#: need to be uploaded to the robot controller"). A CANFD stream reads the row's .txt from disk; the
#: controller's program slots 1-100 are the UPLOAD channel's store. A taught row numbered from here
#: up has no controller slot at all — `upload`, and `run`/`flow` on the upload channel, refuse it —
#: and the number is only the row's identity (its file names and registry key).
TAUGHT_CANFD_SLOT_MIN = 101


def taught_slot_ok(slot) -> bool:
    """A taught row's number: the upload-capable 40-59, or a CANFD-only one from 101 up."""
    return slot is not None and (TAUGHT_SLOT_RANGE[0] <= int(slot) <= TAUGHT_SLOT_RANGE[1]
                                 or int(slot) >= TAUGHT_CANFD_SLOT_MIN)


def canfd_only_slot(slot) -> bool:
    """True for a row beyond the controller's 1-100: it can only be streamed over CANFD."""
    return slot is not None and int(slot) > SLOT_MAX


def refuse_canfd_only_on_upload(rows: list, channel: str) -> None:
    """Nothing to upload or play from the controller for a CANFD-only row — refuse before connecting."""
    if (channel or "upload") == "canfd":
        return
    bad = [t for t in rows if canfd_only_slot(t.get("slot"))]
    if bad:
        raise SystemExit(f"{', '.join(t['name'] for t in bad)}: CANFD-only row(s) (slot > {SLOT_MAX}, "
                         f"no controller slot) — never uploaded; stream them with `flow`/`run ... "
                         f"--channel canfd`")


def drop_canfd_only(rows: list, verb: str) -> list:
    """A WHOLE-SIDE selection on the upload channel leaves CANFD-only rows out, and says so; only a
    row the operator NAMED is refused (review 2026-09-22 — refusing the whole arm because 101+
    exist would have stopped every left-arm upload)."""
    skipped = [t for t in rows if canfd_only_slot(t.get("slot"))]
    if skipped:
        print(f"   {verb}: skipping CANFD-only row(s) {[t['slot'] for t in skipped]} (no controller "
              f"slot; stream them with --channel canfd)")
    return [t for t in rows if not canfd_only_slot(t.get("slot"))]


def declared_canfd_slot(name: str, side: str, ov: dict, man: dict):
    """The CANFD-ONLY slot EMIT_OVERRIDES.json declares for a row (`canfd_slot`), or None.

    Newton 2026-10-02 ("Sure, for the front task configs and plans") brought the FRONT regions into the
    re-plan + re-emit. They were never in Newton's 2026-09-02 upload task list (CLEANING_TASKS), so they
    have no number in the 70-100 production range, and one there would renumber every later slot of the
    side ("+1 per task in schedule order") -- every pair verdict pins its slots. Newton 2026-09-22: "canfd
    is not limited to 100 slots, because nothing need to be uploaded to the robot controller": a slot over
    SLOT_MAX is the row's identity only (file names, the driver's pin, the canfd_slot<N> record) and the
    upload lane refuses it (`refuse_canfd_only_on_upload`). So such a row takes a DECLARED number from 101
    up that no other row on that arm holds in any registry (manifest, taught, experiment, dry), and a full
    `emit` keeps it there without consuming a production slot. A declaration in 70-100, or one another row
    already holds on that arm, is refused.
    """
    v = ov.get("canfd_slot")
    if v is None:
        return None
    slot = int(v)
    if not canfd_only_slot(slot):
        raise SystemExit(f"{name}: EMIT_OVERRIDES canfd_slot {slot} is not CANFD-only (must be > {SLOT_MAX}): "
                         f"{SLOT_BASE}-{SLOT_MAX} is the production upload range, numbered in schedule order")
    holders = []
    regs = [("MANIFEST", man)]
    for label, path in (("TAUGHT", TAUGHT_REGISTRY), ("EXPERIMENT", EXPERIMENT_REGISTRY), ("DRY", DRY_REGISTRY)):
        try:
            regs.append((label, json.load(open(path))))
        except (OSError, ValueError):
            continue
    for label, reg in regs:
        for t in reg.get("tasks", []):
            if label == "MANIFEST" and t.get("name") == name:
                continue
            if t.get("side", side_of(str(t.get("name", "")))) != side:
                continue
            if slot in (t.get("slot"), t.get("reservedSlot")):
                holders.append(f"{label}:{t.get('name')}")
    if holders:
        raise SystemExit(f"{name}: EMIT_OVERRIDES canfd_slot {slot} is already held on the {side} arm by "
                         f"{holders} -- declare a free number above {SLOT_MAX}")
    return slot


#: Below the 70-100 production range on purpose: a dry track must never be able
#: to land on a slot a cleaning task owns.
DRY_SLOT_DEFAULT = 60
#: EXPERIMENT TRACKS (2026-09-17). A plan's emitted track, run EXACTLY as given —
#: the first C-space + spline plan landed as files in a report folder, and the
#: question was how it runs on the arm, on both channels, not how `emit` would
#: re-time it. Same contract as the dry and taught registries (manifest-shaped,
#: pinned rows; `manifest_for` is the choke point, selected with --experiment),
#: and outside every other range: 40-59 taught, 60 dry, 70-100 production, whose
#: full `emit` renumbers from 70 and would overwrite anything placed there.
EXPERIMENT_DIR = REPO / "Resource" / "tracks" / "experiments"
EXPERIMENT_REGISTRY = EXPERIMENT_DIR / "EXPERIMENT_TRACKS.json"
#: Inclusive bands. 61-69 first (2026-09-17, the spline plan); 21-39 added the same day for the
#: J1-J7 single-joint sweeps (21 upload files, one arm). Slots below 40 are the band the pendant
#: may also use — a pendant-saved track lives in the same 1-100 program list — so an
#: `upload --experiment` refuses to overwrite any slot holding a program that is not one of ours
#: (`foreign_programs`).
EXPERIMENT_SLOT_RANGES = ((21, 39), (61, 69))
EXPERIMENT_SLOT_RANGE = EXPERIMENT_SLOT_RANGES[-1]


def experiment_slot_ok(slot) -> bool:
    return any(lo <= int(slot) <= hi for lo, hi in EXPERIMENT_SLOT_RANGES)
#: The dithered joint. J7 is the wrist roll: it spins the pad about its own
#: press axis, so the TCP does not translate and the arm's silhouette barely
#: changes ("wipe yaw is the unchosen parameter").
DRY_JOINT_DEFAULT = 7
#: A dry track exists to be boring, and "boring" is a MEASURED number here, not
#: a round one (hardening review 2026-09-08; it was 30.0). F103 separated the
#: four right_test J4 aborts from the two tolerated crossings on APPROACH RATE:
#: >=32.9 deg/s tripped, <=13.4 deg/s was tolerated (docs/sources.md, "J4 trip
#: separates on APPROACH RATE"). A 30 deg/s ceiling therefore sat INSIDE the
#: band that has actually aborted an arm. The ladder's zero-exposure rung must
#: not be built out of speeds we have seen trip a controller, so the ceiling is
#: the fastest rate ever observed to be tolerated. The committed dry tracks peak
#: at 4.68 deg/s, so nothing in use changes.
DRY_MAX_RATE_DEG_S = 13.4
#: RM75 joint POSITION limits in degrees, joint1..joint7, taken from the single
#: source of truth for this arm's kinematics —
#: ros/description/rm_description/urdf/rm_75_macro.xacro, the `<limit
#: lower=.. upper=..>` of each joint ("Kinematics single source of truth": edit
#: the xacro only). They are mirrored here rather than parsed because this tool
#: is offline and must not pull in the description stack;
#: `test_track_provenance.py` re-derives them FROM THE XACRO and fails if this
#: copy drifts, so it is a cached read, not an invented number.
#:
#: The dry verb lets the operator choose the joint and the roll, and a
#: commanded point outside a joint's range is a controller-side refusal (or a
#: fault) on the ladder's very first rung. The rest poses in use today sit far
#: inside — J7 at 69 deg (right) / 111 deg (left) against +-360 — so this gate
#: fires on nothing that exists; it is there because the operator picks the
#: arguments (hardening review 2026-09-08).
JOINT_LIMIT_DEG = ((-178.0, 178.0), (-130.0, 130.0), (-178.0, 178.0), (-135.0, 135.0),
                   (-178.0, 178.0), (-128.0, 128.0), (-360.0, 360.0))


def joint_limit_margin_deg(rows: list) -> list:
    """Per joint, how close the WRITTEN integers come to that joint's position
    limit, in degrees. Negative means the track commands a pose the arm cannot
    reach."""
    out = []
    for j, (lo, hi) in enumerate(JOINT_LIMIT_DEG):
        vals = [r[j] / 1000.0 for r in rows]
        out.append(round(min(min(v - lo for v in vals), min(hi - v for v in vals)), 4))
    return out


#: HOLD PROBES (2026-09-17, Newton agreed). The upload player's holds can only be measured window by
#: window where the command moves >= 0.25 deg/line (~50 deg/s at the 5.0 ms ceiling) — far above the
#: dry ceiling, which stays exactly as it is for the ladder's zero-exposure rung. A probe is its own
#: shape, opted into with --probe, and still an in-place track: the rest pose, ONE wrist joint moving
#: (the joints whose motion barely moves the hand: F103's aborts were J4 approaching a limit), within
#: PROBE_MAX_AMPLITUDE_DEG of rest, at most PROBE_MAX_RATE_DEG_S judged at the 5.0 ms ceiling, and
#: forced strict like any dry row (no transit).
PROBE_JOINTS = (6, 7)
PROBE_MAX_RATE_DEG_S = 120.0
PROBE_MAX_AMPLITUDE_DEG = 15.0
PROBE_MAX_ACCEL_DEG_S2 = 3000.0
PROBE_CLOCK_S = 0.005

#: RATING SWEEPS (2026-09-17). A hold probe measures holds BELOW the ratings; it cannot answer "does the
#: arm depart more where the file asks for more than the joint can do", because that needs the commanded
#: speed taken THROUGH the rating, sustained long enough to accumulate 100 ms blocks, on a joint whose
#: clamp can also be measured WITHOUT the player — and that last condition picks the joint for us:
#:
#:   * our CANFD preflight refuses above 180 deg/s by default (the vendor's documented flat figure;
#:     the vendor's own stated consequence is non-response, not a refusal), and J1/J2 are rated 180
#:     while J3-J7 are rated 225.
#:     So J1 and J2 are the ONLY joints whose rating the hold-free channel can reach: at exactly 1.0x,
#:     legally (the vendor's rule is "must not exceed"), the same bytes can go down both channels.
#:   * on J3-J7 a clamp condition cannot have a CANFD control at all. Slowing the stream until it is
#:     legal removes the clamp, which is the thing being measured. The 24 installed sweeps all fail this
#:     way: every one of their 9 over-rating rows needs 6.07-11.76 ms to stream (`sweep_design_audit.py`).
#:
#: Sustaining 180 deg/s needs room: braking from it at 600 deg/s^2 alone takes 27 deg, so the amplitude
#: is large and THIS IS NOT A WRIST DITHER — J1 swings the whole arm through 2x the amplitude. That is a
#: hardware-exposure decision, not a measurement one, so the envelope here is deliberately narrow and the
#: row still has to be built and run deliberately.
RATING_SWEEP_JOINTS = (1, 2)
RATING_SWEEP_MAX_FRAC = 1.15          # of the joint's rating, at the 5.0 ms ceiling
RATING_SWEEP_MAX_AMPLITUDE_DEG = 75.0
RATING_SWEEP_MAX_ACCEL_DEG_S2 = 1200.0
RATING_SWEEP_MAX_DURATION_S = 30.0
#: A block is 20 lines (100 ms at the ceiling); the per-position analysis reports nothing below this many.
RATING_SWEEP_MIN_BLOCKS = 8


def _return_to_rest_rows(base: list, j: int, x0: float, v: float, a: float, clock_s: float) -> list:
    """Lines that carry joint *j* from `x0` (at rest velocity) back to the rest pose: accelerate at
    `a` to at most `v`, cruise, decelerate to zero, last line exactly at rest."""
    import math as _m
    D = abs(float(x0))
    if D < 1e-9:
        return [list(base)]
    sgn = -1.0 if x0 > 0 else 1.0
    vp = min(float(v), _m.sqrt(float(a) * D))
    t_acc = vp / float(a)
    d_acc = 0.5 * float(a) * t_acc * t_acc
    d_cruise = max(0.0, D - 2.0 * d_acc)
    t_cruise = d_cruise / vp if vp > 0 else 0.0
    T = 2.0 * t_acc + t_cruise
    out = []
    n = int(_m.ceil(T / clock_s))
    for i in range(1, n + 1):
        t = min(i * clock_s, T)
        if t <= t_acc:
            d = 0.5 * float(a) * t * t
        elif t <= t_acc + t_cruise:
            d = d_acc + vp * (t - t_acc)
        else:
            td = t - t_acc - t_cruise
            d = d_acc + d_cruise + vp * td - 0.5 * float(a) * td * td
        row = list(base)
        row[j] = base[j] + int(round((x0 + sgn * min(d, D)) * 1000.0))
        out.append(row)
    out[-1] = list(base)
    return out


def probe_track_rows(rest_deg, joint: int, shape: str, *, speed_deg_s: float = 0.0,
                     accel_deg_s2: float = 600.0, amplitude_deg: float = 10.0, duration_s: float = 20.0,
                     clock_s: float = PROBE_CLOCK_S) -> list:
    """A hold-probe track sampled at `clock_s` (the player's one point per 5 ms):

    * `still`  — the rest pose on every line, for `duration_s`: any time the player takes beyond
      lines x 5 ms is held time, with no motion at all to explain it.
    * `cruise` — `joint` from rest, accelerating at `accel_deg_s2` to `speed_deg_s`, cruising, and
      reversing at `accel_deg_s2` at +-`amplitude_deg`, for about `duration_s`; it then returns to
      rest and stops there. Constant speed between reversals is what makes the hold measure valid.
    First and last line are the rest pose exactly."""
    import math as _m
    if shape not in ("still", "cruise"):
        raise ValueError("probe shape must be still or cruise")
    if not 1 <= int(joint) <= 7 or len(rest_deg) != 7:
        raise ValueError("a 7-joint rest pose and a joint 1..7 are needed")
    base = [int(round(float(v) * 1000.0)) for v in rest_deg]
    n_total = max(2, int(round(float(duration_s) / clock_s)) + 1)
    if shape == "still":
        return [list(base) for _ in range(n_total)]
    v, a, A = float(speed_deg_s), float(accel_deg_s2), float(amplitude_deg)
    if v <= 0 or a <= 0 or A <= 0:
        raise ValueError("cruise needs a positive speed, acceleration and amplitude")
    if v * v / (2.0 * a) >= A:
        raise ValueError(f"speed {v} at {a} deg/s^2 needs {v * v / (2.0 * a):.2f} deg to reverse, "
                         f"more than the {A} deg amplitude")
    j = int(joint) - 1
    # One integrator, three phases: OSCILLATE (cruise between reversals) until the duration is up,
    # RETURN (an acceleration-limited move back to rest, so the track never jumps), then STOP at rest.
    # Every phase changes velocity by at most a*clock per line, so no line can exceed `speed_deg_s`.
    x, vel, target, t, rows = 0.0, 0.0, +v, 0.0, []
    phase = "oscillate"                          # -> "return" (computed move back to rest) -> done
    guard = 20 * n_total + 20000
    while True:
        row = list(base)
        row[j] = base[j] + int(round(x * 1000.0))
        rows.append(row)
        if len(rows) > guard:
            raise ValueError("probe generation did not terminate")
        # slack on the braking distance: the trigger is tested BEFORE a step and a discrete
        # deceleration covers a little more ground than v^2/2a (overshoots of 0.27-0.47 deg without it)
        brake = vel * vel / (2 * a) + 2.0 * abs(vel) * clock_s
        if phase == "oscillate":
            if t >= float(duration_s):
                phase = "return"
            elif target > 0 and x >= A - brake:
                target = -v
            elif target < 0 and x <= -A + brake:
                target = +v
        if phase == "return":
            # COMPUTED, not steered: a feedback return oscillated around rest and never terminated
            # (caught offline 2026-09-17 at 25 deg/s and at 110 deg/s). Brake to a stop, then walk an
            # accelerate-cruise-decelerate profile straight back to rest and land there exactly.
            if abs(vel) > 1e-9:
                target = 0.0
            else:
                rows.extend(_return_to_rest_rows(base, j, x, v, a, clock_s))
                break                            # the return rows END at rest: nothing more to append
        dv = max(-a * clock_s, min(a * clock_s, target - vel))
        vel_next = vel + dv
        x += 0.5 * (vel + vel_next) * clock_s
        vel = vel_next
        t += clock_s
    peak = max(abs(r[j] - base[j]) for r in rows) / 1000.0
    if peak > A + 1e-3:
        raise ValueError(f"BUG: the cruise probe reached {peak:.3f} deg, past its {A} deg amplitude")
    jump = max((max(abs(p - q) for p, q in zip(a_, b_)) for a_, b_ in zip(rows, rows[1:])), default=0) / 1000.0
    if jump > v * clock_s + 2e-3:
        raise ValueError(f"BUG: the cruise probe steps {jump:.3f} deg in one line "
                         f"({jump / clock_s:.0f} deg/s), past its {v} deg/s speed")
    if rows[0] != base or rows[-1] != base:
        raise ValueError("BUG: a probe must start and end at the rest pose")
    return rows


class DryTrackRefused(Exception):
    """A dry track that would not be safe, or cannot be built.

    An EXCEPTION, not SystemExit (hardening review 2026-09-08). SystemExit is
    a BaseException: it walks straight past a caller's ``except Exception``,
    and inside a worker thread — which is where the dual executive will build
    and run these — Python swallows it and the thread just stops with no
    verdict anywhere. The CLI catches it and returns the same refusal rc as
    every other gate here.
    """


def side_rest_pose(man: dict, side: str, tol_deg: float = 0.01) -> dict:
    """The side's `move_to_rest` pose, read from the PLANS of that side's
    emitted rows — never invented here, and never taken from one plan without
    checking the others agree. Returns {'deg', 'sources', 'spread_deg'}."""
    from alix_dispatch import peripherals as per
    rows = rows_for(man, side)
    poses, sources = [], []
    for t in rows:
        if not t.get("plan"):
            continue
        try:
            rp = per.rest_pose(json.loads((REPO / t["plan"]).read_text()))
        except Exception:                                              # noqa: BLE001
            continue
        if rp:
            poses.append([float(v) for v in rp["deg"]])
            sources.append(t["name"])
    if not poses:
        raise DryTrackRefused(f"no {side} plan carries a move_to_rest stage — cannot build "
                              f"a dry track without a rest pose (it is read, never invented)")
    spread = max(max(abs(q[j] - poses[0][j]) for q in poses) for j in range(len(poses[0])))
    if spread > tol_deg:
        raise DryTrackRefused(f"the {side} plans disagree about move_to_rest by "
                              f"{spread:.4f} deg (> {tol_deg}) — refusing to pick one")
    return dict(deg=poses[0], sources=sources, spread_deg=round(spread, 6))


def dry_track_rows(rest_deg, points: int, roll_deg: float, cycles: int = 2,
                   joint: int = DRY_JOINT_DEFAULT) -> list:
    """The synthetic in-place track: every point is the rest pose, with *joint*
    dithered by a whole number of sine cycles of amplitude *roll_deg*.

    Whole cycles mean the first and last point ARE the rest pose, so the run's
    movej bridge to point 1 is a no-op when the arm is already parked and the
    replay ends where it started.
    """
    import math as _m
    if len(rest_deg) != 7:
        raise ValueError(f"rest pose must have 7 joints, got {len(rest_deg)}")
    if points < 2:
        raise ValueError("a track needs at least 2 points")
    if not 1 <= int(joint) <= 7:
        raise ValueError("joint must be 1..7")
    if roll_deg < 0:
        raise ValueError("roll must be >= 0")
    if cycles < 1:
        raise ValueError("cycles must be >= 1")
    base = [int(round(float(v) * 1000.0)) for v in rest_deg]
    amp = int(round(float(roll_deg) * 1000.0))
    rows = []
    n = int(points)
    for i in range(n):
        row = list(base)
        row[int(joint) - 1] = base[int(joint) - 1] + int(round(
            amp * _m.sin(2.0 * _m.pi * int(cycles) * i / (n - 1))))
        rows.append(row)
    return rows


def dry_track_facts(rows: list, base_millideg: list) -> dict:
    """The properties a dry track must have, measured on the integers that will
    be uploaded (never on the floats that made them)."""
    dev = [max(abs(r[j] - base_millideg[j]) for r in rows) for j in range(7)]
    peak = 0.0
    for a, b in zip(rows, rows[1:]):
        # a dry track is an UPLOAD-lane rung played at the player's pace: unchanged by the 5.0 ms emit clock
        peak = max(peak, max(abs(x - y) for x, y in zip(a, b)) / 1000.0 / PLAYER_CLOCK_S)
    return dict(max_dev_millideg=dev, max_dev_deg=[round(v / 1000.0, 6) for v in dev],
                moving_joints=[j + 1 for j, v in enumerate(dev) if v],
                peak_rate_deg_s=round(peak, 3),
                first_is_rest=all(r == b for r, b in zip(rows[0], base_millideg)),
                last_is_rest=all(r == b for r, b in zip(rows[-1], base_millideg)))


def write_dry_track(path: pathlib.Path, rows: list) -> pathlib.Path:
    path = pathlib.Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", newline="") as fh:
        for r in rows:
            fh.write('{"point":[' + ",".join(str(int(v)) for v in r) + "]}\n")
    return path


def load_dry_registry(path: pathlib.Path = None) -> dict:
    path = pathlib.Path(path or DRY_REGISTRY)
    if not path.exists():
        return dict(fixture=FIXTURE, kind="dry", tasks=[])
    return json.load(open(path))


def save_dry_registry(reg: dict, path: pathlib.Path = None) -> None:
    path = pathlib.Path(path or DRY_REGISTRY)
    reg["kind"] = "dry"
    reg["what"] = (
        "SYNTHETIC in-place tracks for the first-run ladder's zero-exposure rungs "
        "(D1/D2, 2026-09-08 design section 5). Each holds its side's move_to_rest pose "
        "and dithers ONE joint by at most `roll_deg`; no point of any of them is more "
        "than that from rest. NOT cleaning tasks: they live outside MANIFEST.json and "
        "outside the 70-100 slot range, and nothing in the production flow reads this "
        "file. Run them with `cleaning_tracks.py run --dry <slot>`.")
    reg["generated"] = datetime.datetime.now().isoformat(timespec="seconds")
    reg["tasks"].sort(key=lambda t: (t.get("side", ""), t.get("slot") or 999))
    path.parent.mkdir(parents=True, exist_ok=True)
    json.dump(reg, open(path, "w"), indent=1)


def load_taught_registry(path: pathlib.Path = None) -> dict:
    """The taught lane's rows (Resource/tracks/taught/TAUGHT_TRACKS.json).
    Manifest-shaped on purpose: `manifest_for` returns this instead of the
    production manifest when a verb is given `--taught`, and every consumer
    downstream is unchanged (the DRY_TRACKS.json precedent, H37)."""
    path = pathlib.Path(path or TAUGHT_REGISTRY)
    if not path.exists():
        return dict(kind="taught", slots=dict(range=list(TAUGHT_SLOT_RANGE)),
                    convention={}, gates={}, tasks=[])
    return json.load(open(path))


def save_taught_registry(reg: dict, path: pathlib.Path = None) -> None:
    """Write the lane's registry. `tools/taught_tracks.py` owns its rows; this
    is here, and not there, only so `manifest_for` can resolve the file without
    importing the lane (which imports this module)."""
    path = pathlib.Path(path or TAUGHT_REGISTRY)
    reg["kind"] = "taught"
    reg.setdefault("generated", datetime.datetime.now().isoformat(timespec="seconds"))
    reg["generated"] = datetime.datetime.now().isoformat(timespec="seconds")
    reg["tasks"].sort(key=lambda t: (SIDES.index(t.get("side", "right")),
                                     t.get("slot") if t.get("slot") is not None else 999))
    path.parent.mkdir(parents=True, exist_ok=True)
    json.dump(reg, open(path, "w"), indent=1)


def load_experiment_registry(path: pathlib.Path = None) -> dict:
    """Experiment rows (Resource/tracks/experiments/EXPERIMENT_TRACKS.json),
    manifest-shaped; `manifest_for` returns this for a verb given --experiment."""
    path = pathlib.Path(path or EXPERIMENT_REGISTRY)
    if not path.exists():
        return dict(kind="experiment", slots=dict(ranges=[list(r) for r in EXPERIMENT_SLOT_RANGES]),
                    tasks=[])
    return json.load(open(path))


def save_experiment_registry(reg: dict, path: pathlib.Path = None) -> None:
    path = pathlib.Path(path or EXPERIMENT_REGISTRY)
    reg["kind"] = "experiment"
    reg["slots"] = dict(ranges=[list(r) for r in EXPERIMENT_SLOT_RANGES])
    reg["what"] = (
        "Plan-emitted or recorded tracks installed BYTE FOR BYTE as given, to be run and measured "
        "on both channels (--channel upload | canfd). NOT production tasks: slots 21-39 and 61-69, "
        "outside MANIFEST.json and the 70-100 range, and no production flow reads this "
        "file. Install with `cleaning_tracks.py experiment`, run with --experiment.")
    reg["generated"] = datetime.datetime.now().isoformat(timespec="seconds")
    reg["tasks"].sort(key=lambda t: (SIDES.index(t.get("side", "right")),
                                     t.get("slot") if t.get("slot") is not None else 999))
    path.parent.mkdir(parents=True, exist_ok=True)
    json.dump(reg, open(path, "w"), indent=1)


def build_experiment_row(plan, track_src: pathlib.Path, side: str, slot: int,
                         name: str, *, tool_frame_from: str = None, evidence: dict = None,
                         out_dir: pathlib.Path = None, allow_over_rating: bool = False,
                         ik_frame: str = None, note: str = None, derived_from: dict = None) -> dict:
    """Install ONE plan-emitted track as an experiment row, byte for byte.

    The track is copied, never re-emitted, and its sha is checked against the
    emitter's sidecar (`<track>.json`) when there is one; a `.proj` beside it
    must equal the one `build_project_file` makes from those bytes (it is a pure
    function of them), or the install refuses. Everything else comes from the
    same helpers `emit_one` uses: the pole and the plan's hand stage
    (`peripherals.pre_stroke_state`), the hand from the pad (`hand_from_frame`,
    Newton 2026-09-05), the retreat pose, and the same two rate gates. The tool
    frame is EVIDENCE: taken from the run records of *tool_frame_from* (the
    production row of the same region), never invented.

    *plan* may be None for a RECORDED track (a drag-teach file, 2026-09-17): the row then has no
    pole/hand staging, no retreat and no park — run it with `--no-devices`. *allow_over_rating*
    installs a track whose ticks command a joint over its rating, recording how many and the
    peaks (Newton 2026-09-17: the FAST sweeps at recorded speed on the upload channel); it is
    never the default."""
    import shutil
    import numpy as np                    # this module imports numpy per function, never at import
    from alix_dispatch import peripherals as per, track as tr
    if not experiment_slot_ok(slot):
        raise SystemExit(f"slot {slot} is outside the experiment ranges "
                         f"{', '.join(f'{lo}-{hi}' for lo, hi in EXPERIMENT_SLOT_RANGES)}")
    if side not in SIDES:
        raise SystemExit(f"side must be one of {SIDES}")
    # another registry's row on this arm's slot (a dry track sits on right 69) — its stored name is
    # "ours", so the upload guard would let the experiment overwrite it: refuse here instead
    for label, loader in (("production manifest", load_manifest), ("taught registry", load_taught_registry),
                          ("dry registry", load_dry_registry)):
        try:
            other = [t for t in loader().get("tasks", []) if t.get("side") == side and t.get("slot") == int(slot)
                     and t.get("status") == "EMITTED"]
        except Exception:                                              # noqa: BLE001
            other = []
        if other:
            raise SystemExit(f"{side} slot {slot} is used by {other[0]['name']} in the {label}")
    track_src = pathlib.Path(track_src)
    plan = pathlib.Path(plan) if plan else None
    for f in ([plan] if plan else []) + [track_src]:
        if not f.is_file():
            raise SystemExit(f"not a file: {f}")
    d = json.load(open(plan)) if plan else None
    if d is not None and not any((x.get("stage_name") or x.get("name")) == "execute_path"
                                 for x in d.get("sub_trajectories", [])):
        raise SystemExit(f"{plan.name}: no execute_path stage")
    src_sha = sha256_of(track_src)
    sidecar = track_src.with_suffix(".json")
    sc = json.load(open(sidecar)) if sidecar.is_file() else {}
    if sc.get("sha256") and sc["sha256"] != src_sha:
        raise SystemExit(f"{track_src.name} hashes to {src_sha[:12]} but its sidecar says "
                         f"{sc['sha256'][:12]} — not the file the emitter wrote")
    sc_side = {"R": "right", "L": "left"}.get(str(sc.get("side") or ""), None)
    if sc_side and sc_side != side:
        raise SystemExit(f"the emitter wrote this track for the {sc_side} arm, not {side}")
    out_dir = pathlib.Path(out_dir or EXPERIMENT_DIR)
    out_dir.mkdir(parents=True, exist_ok=True)
    stem = f"{int(slot):02d}_{name}_execute_path"
    track = out_dir / f"{stem}.txt"
    # THE STEM DOES NOT CARRY THE SIDE, and an experiment slot number is not per-arm: a sweep named
    # `sweep_j1_slow_raw` installed on the right at the same slot would write these very bytes over the
    # left arm's file, and both rows would then point at one track (2026-09-17 review, B3). The 25 rows
    # already on disk are not renamed — a name that is already another arm's is refused instead.
    if track.is_file():
        rel = repo_rel(track)
        clash = [t for t in load_experiment_registry().get("tasks", [])
                 if t.get("track") == rel and t.get("side") and t.get("side") != side]
        if clash:
            raise SystemExit(f"{track.name} is the {clash[0]['side']} arm's track for "
                             f"{clash[0]['name']} (slot {clash[0]['slot']}) — the stem carries no side, "
                             f"so emitting it for the {side} arm would overwrite it. Use a name that "
                             f"says which arm it is (e.g. {name}_{side}).")
    shutil.copyfile(track_src, track)
    if sidecar.is_file():
        shutil.copyfile(sidecar, track.with_suffix(".json"))
    plan_copy = out_dir / f"{int(slot):02d}_{name}_plan.json" if plan else None
    if plan:
        shutil.copyfile(plan, plan_copy)
    try:
        rr = rating_report_at_ceiling(np.array(tr.parse_track(track), float) / 1000.0)
        n_over, peaks = rr["ticks_over_rating"], rr["per_joint_peak_deg_s_raw"]
        if n_over and not allow_over_rating:
            raise SystemExit(f"{n_over} tick(s) command a joint over its RATED speed at the player's "
                             f"{RATE_CEILING_CLOCK_S * 1e3:.1f} ms ceiling, 20 Hz band-limited (raw "
                             f"peaks {peaks}) — error 4105 and the clamp (F111)")
        step = tick_step_check(track)
        disk = disk_provenance(track, build=True)
        src_proj = track_src.with_suffix(".proj")
        if src_proj.is_file() and sha256_of(src_proj) != disk["proj_sha256"]:
            raise SystemExit(f"{src_proj.name} is not the .proj these bytes make "
                             f"({sha256_of(src_proj)[:12]} vs {disk['proj_sha256'][:12]})")
    except BaseException:
        # ANY failure must roll back, not just a refusal. `except SystemExit` left the copied .txt/.json/
        # .proj behind whenever a check raised something else — a malformed track makes `parse_track`
        # raise ValueError — and a half-installed slot is what the registry cannot see (2026-09-17
        # review, B3). BaseException so a Ctrl-C mid-install also cleans up.
        for f in (track, track.with_suffix(".json"), track.with_suffix(".proj"), plan_copy):
            if f is not None:
                f.unlink(missing_ok=True)
        raise
    q = [[v / 1000.0 for v in r] for r in tr.parse_track(track)]
    n = len(q)
    frame = ik_frame or sc.get("frame") or (ik_frame_for(name, side, {}) if d is not None else None)
    track_s = (n - 1) * track_clock_s(track)        # the file's own sample clock (2026-10-02)
    if d is not None:
        pre = per.pre_stroke_state(d, side)
        ret = per.retreat_pose(d)
        hand_state, hand_counts = hand_from_frame(frame, side)
        st = next(x for x in d["sub_trajectories"]
                  if (x.get("stage_name") or x.get("name")) == "execute_path")
        plan_s = float(st["waypoints"][-1]["time_from_start"])
    else:
        pre = ret = None
        plan_s = None
    ev = evidence if evidence is not None else (run_record_evidence() if RUNS_ROOT.is_dir() else {})
    row = dict(
        slot=int(slot), name=name, side=side, status="EMITTED", kind="experiment",
        plan=(repo_rel(plan_copy) if plan else None),
        plan_sha256=(sha256_of(plan_copy) if plan else None),
        plan_mode=(d.get("cleaning_path_mode") if d else None),
        path_source=(d.get("path_source") if d else "recorded"),
        track=repo_rel(track), points=n,
        plan_execute_s=(round(plan_s, 3) if plan_s is not None else None),
        track_s=round(track_s, 4),
        speedup=(round(plan_s / track_s, 3) if (plan_s is not None and track_s) else None),
        band_5p5_5p6_s=[round((n - 1) * 0.0055, 3), round((n - 1) * 0.0056, 3)],
        u_vel=(sc.get("band_limited_util") or {}).get("vel"),
        u_acc=(sc.get("band_limited_util") or {}).get("acc"),
        u_jerk=(sc.get("band_limited_util") or {}).get("jerk"),
        peak_rate_deg_s=sc.get("max_joint_rate_deg_s"),
        pos_margin_deg=sc.get("position_margin_min_deg"),
        tick_step_deg_s=round(step, 2), ik_frame=frame, ticks_over_rated=n_over,
        over_rating=(dict(allowed=True, ticks=n_over, peaks=peaks,
                          peak_util=rr["peak_util"], peak_joint=rr["peak_joint"],
                          ticks_raw=rr["ticks_over_rating_raw"], peak_util_raw=rr["peak_util_raw"],
                          clock_s=RATE_CEILING_CLOCK_S) if n_over else None),
        rating_at_ceiling=rr,
        pre_stroke=(dict(pole_m=pre["pole_m"], pole_stage=pre["pole_stage"],
                         hand_state=hand_state, hand_counts=hand_counts,
                         hand_provenance=dict(source="glove_frames.yaml", frame=frame,
                                              plan_stage=pre.get("hand_state"),
                                              plan_counts=pre.get("hand_counts"),
                                              agrees=(pre.get("hand_counts") == hand_counts)),
                         hand_release_counts=pre["hand_release_counts"]) if pre else None),
        retreat_stage=(ret or {}).get("stage"), retreat_deg=(ret or {}).get("deg"),
        track_sha256=disk["track_sha256"], track_bytes=disk["track_bytes"],
        points_on_disk=disk["points_on_disk"], proj=disk["proj"],
        proj_sha256=disk["proj_sha256"], proj_bytes=disk["proj_bytes"],
        installed_from=dict(plan=(repo_rel(plan) if plan else None), track=repo_rel(track_src),
                            track_sha256=src_sha, emitter=sc.get("emitter"),
                            derived_from=derived_from, note=note,
                            installed=datetime.datetime.now().isoformat(timespec="seconds")))
    row["observed_runs"] = observed_runs_for(row, ev, row["track_sha256"])
    if tool_frame_from:
        tf = tool_frame_from_records(dict(name=tool_frame_from, ik_frame=frame), ev)
        if tf is not None:
            tf = dict(tf)
            tf["provenance"] = dict(tf.get("provenance") or {},
                                    inherited_from=tool_frame_from,
                                    note="the controller frame the production row of the same "
                                         "region ran under; evidence, not a choice")
            row["tool_frame"] = tf
    return row


def known_trajectory_names() -> list:
    """The stored names every row of every registry would carry on a controller."""
    names = []
    for loader in (load_manifest, load_taught_registry, load_dry_registry, load_experiment_registry):
        try:
            reg = loader()
        except Exception:                                              # noqa: BLE001
            continue
        for t in reg.get("tasks", []):
            n = expected_trajectory_name(t) if t.get("track") else None
            if n:
                names.append(n)
    return names


def foreign_programs(stored_names: dict, rows: list, known: list = None) -> list:
    """The slots about to be written that hold a program which is NOT one of ours.

    A pendant-saved track lives in the same 1-100 program list, and `send_project` overwrites
    whatever is there. A slot is safe when it is empty or its stored name is the truncation of a
    name one of our registries would upload. Returns [{slot, stored, row}]."""
    known = known if known is not None else known_trajectory_names()
    out = []
    for t in rows:
        stored = stored_names.get(t["slot"])
        if stored is None:
            continue
        s = str(stored).strip().rstrip("\x00")
        if not s:
            continue
        if any(k.startswith(s) or s.startswith(k) for k in known):
            continue
        out.append(dict(slot=t["slot"], stored=s, row=t["name"]))
    return out


def cmd_ratecheck(args) -> int:
    """Every row of a registry against the ratings at the player's 5.0 ms ceiling, and what `emit`'s
    rate fit would do to it — READ-ONLY: nothing is re-timed or written."""
    import tempfile, shutil
    man = manifest_for(args)
    rows = [t for t in man["tasks"] if t.get("status") == "EMITTED" and t.get("track")
            and (args.side is None or t.get("side") == args.side)]
    print(f"{'side':5s} {'slot':>4s} {'row':40s} {'over @5.0':>9s} {'peak':>6s} {'joint':>5s} "
          f"{'fit: s before -> after':>24s} {'slower':>7s}")
    over = 0
    for t in sorted(rows, key=lambda t: (t.get("side", ""), t.get("slot") or 999)):
        src = REPO / t["track"]
        with tempfile.TemporaryDirectory() as td:
            tmp = pathlib.Path(td) / src.name
            shutil.copyfile(src, tmp)
            f = rate_fit_track(tmp, write=False)
        b = f["before"]
        over += bool(b["ticks_over_rating"])
        fit_txt = (f"{f['duration_before_s']:8.2f} -> {f['duration_after_s']:8.2f}" if f["applied"] else "unchanged")
        print(f"{t.get('side', ''):5s} {t.get('slot')!s:>4s} {t['name'][:40]:40s} {b['ticks_over_rating']:>9d} "
              f"{b['peak_util']:>6.3f} {('J' + str(b['peak_joint'])):>5s} {fit_txt:>24s} "
              f"{(str(round(100 * (f['slowdown'] - 1), 1)) + '%') if f['applied'] else '':>7s}")
    print(f"\n{over} of {len(rows)} row(s) exceed a joint rating at the player's "
          f"{RATE_CEILING_CLOCK_S * 1e3:.1f} ms ceiling (20 Hz band-limited). `emit` now fits them; "
          f"taught rows re-certify with `taught_tracks.py shape`.")
    return 0


def cmd_accflag(args) -> int:
    """THE ACCELERATION FLAG per row (Newton 2026-10-02 "Keep advisory, flag it"; ACC_ADVISORY_KEY): the commanded
    20 Hz peak joint acceleration and the % of moving time above 600 / 900 deg/s^2, from the bytes on disk.
    Read-only unless --write, which records the block in every EMITTED manifest row and in each track's sidecar
    (no track byte changes; nothing is re-timed; nothing is refused)."""
    man = load_manifest()
    rows = [t for t in man["tasks"] if t.get("status") == "EMITTED"
            and (args.side is None or t.get("side", side_of(t["name"])) == args.side)]
    res = stamp_acceleration_advisory(rows, write_sidecars=bool(args.write))
    print(f"{'side':5s} {'slot':>4s} {'row':40s} {'peak':>6s} {'joint':>5s} {'>600 %':>7s} {'>900 %':>7s} "
          f"{'p90':>5s}  flag")
    for r in sorted(res, key=lambda r: (str(r.get("side")), r.get("slot") or 999)):
        if r.get("missing"):
            print(f"  ** {r['name']}: {r['missing']} is missing — nothing recorded")
            continue
        pct = r["pct_moving_above"]
        print(f"{str(r['side']):5s} {r['slot']!s:>4s} {r['name'][:40]:40s} {r['peak_deg_s2']!s:>6s} "
              f"{('J' + str(r['peak_joint'])):>5s} {pct.get('600')!s:>7s} {pct.get('900')!s:>7s} {r['p90']!s:>5s}  "
              f"{'FLAGGED (advisory): ' + str(r['corpus_band']) if r['flagged'] else '-'}")
    n_flag = sum(1 for r in res if r.get("flagged"))
    print(f"\n{n_flag} of {len(res)} row(s) command moving time above {ACC_ADVISORY_FLAG_DEG_S2:g} deg/s^2 "
          f"(20 Hz). ADVISORY: {ACC_ADVISORY_RULE}.")
    if args.write:
        save_manifest(man)
        print(f"recorded under '{ACC_ADVISORY_KEY}' in {len(res)} row(s) of {TRACKS / 'MANIFEST.json'} and their "
              "track sidecars (no track byte changed)")
    return 0


def cmd_experiment(args) -> int:
    """Install a plan-emitted track as an experiment row, or list the rows."""
    reg = load_experiment_registry()
    if args.list or not args.track:
        rows = sorted(reg["tasks"], key=lambda t: (t.get("side", ""), t.get("slot") or 999))
        if not rows:
            print("no experiment rows")
        for t in rows:
            ps = t.get("pre_stroke") or {}
            print(f"  {t['side']:5s} slot {t['slot']}: {t['name']}  {t['points']} pts "
                  f"{t['track_s']:.2f} s  pole {ps.get('pole_m')}  hand {ps.get('hand_state')}  "
                  f"{'over-rating ' if t.get('over_rating') else ''}"
                  f"frame {(t.get('tool_frame') or {}).get('name')}  sha {t['track_sha256'][:12]}")
        return 0
    if not args.side or args.slot is None or not args.name:
        raise SystemExit("installing needs the track, --side, --slot and --name (and --plan for a "
                         "plan-emitted track)")
    clash = [t for t in reg["tasks"] if t.get("side") == args.side and t.get("slot") == args.slot]
    if clash and not args.replace:
        raise SystemExit(f"{args.side} slot {args.slot} already holds {clash[0]['name']} — "
                         f"--replace to overwrite it")
    row = build_experiment_row((pathlib.Path(args.plan) if args.plan else None),
                               pathlib.Path(args.track), args.side, args.slot, args.name,
                               tool_frame_from=args.tool_frame_from,
                               allow_over_rating=bool(getattr(args, "allow_over_rating", False)),
                               ik_frame=getattr(args, "ik_frame", None))
    reg["tasks"] = [t for t in reg["tasks"]
                    if not (t.get("side") == args.side and t.get("slot") == args.slot)] + [row]
    save_experiment_registry(reg)
    ps = row["pre_stroke"] or {}
    print(f"installed {args.side} slot {row['slot']}: {row['name']}")
    print(f"  track {row['track']}  ({row['points']} pts, {row['track_s']:.3f} s, sha "
          f"{row['track_sha256'][:12]} = the given file)")
    print(f"  proj  {row['proj']}  (sha {row['proj_sha256'][:12]})")
    print(f"  pole {ps.get('pole_m')} m ({ps.get('pole_stage')}), hand {ps.get('hand_state')} "
          f"{ps.get('hand_counts')}, retreat {row['retreat_stage']}, ik frame {row['ik_frame']}, "
          f"tool frame {(row.get('tool_frame') or {}).get('name')}"
          + ("" if row["pre_stroke"] else "  (recorded track: no staging, run with --no-devices)"))
    if row.get("over_rating"):
        print(f"  ** over rating ALLOWED: {row['over_rating']['ticks']} tick(s), peaks "
              f"{row['over_rating']['peaks']}")
    print(f"  registry -> {repo_rel(EXPERIMENT_REGISTRY)}")
    return 0


def build_dry_row(man: dict, side: str, slot: int, points: int, roll_deg: float,
                  cycles: int, joint: int, out_dir: pathlib.Path = None,
                  evidence: dict = None, probe: dict = None) -> dict:
    """Synthesise one dry track and return its registry row (manifest-shaped,
    so `by_name` / `run_one` / `preflight_slots` need no changes)."""
    from alix_dispatch import track as tr     # this module imports its deps per function
    out_dir = pathlib.Path(out_dir or DRY_DIR)
    if SLOT_BASE <= int(slot) <= SLOT_MAX:
        raise DryTrackRefused(f"slot {slot} is inside the production range "
                              f"{SLOT_BASE}-{SLOT_MAX} — a dry track must never be able to "
                              f"overwrite a cleaning task (default {DRY_SLOT_DEFAULT})")
    if not 1 <= int(slot) <= 100:
        raise DryTrackRefused("the controller stores programs in slots 1..100")
    # ANOTHER REGISTRY'S ROW ON THIS ARM'S SLOT. The production range is refused above, but the taught
    # rows (40-59) and the experiment rows (21-39, 61-69) are not — and a dry track installed on one of
    # those slots would overwrite a certified file whose stored name the upload guard reads as "ours"
    # (2026-09-17 review, C6; `build_experiment_row` has had this check since the registry was added).
    for label, loader in (("taught registry", load_taught_registry),
                          ("experiment registry", load_experiment_registry)):
        try:
            other = [t for t in loader().get("tasks", [])
                     if t.get("side") == side and t.get("slot") == int(slot)
                     and t.get("status") == "EMITTED"]
        except Exception:                                              # noqa: BLE001
            other = []
        if other:
            raise DryTrackRefused(f"{side} slot {slot} is used by {other[0]['name']} in the {label}")
    rest = side_rest_pose(man, side)
    if probe and probe.get("rating_sweep"):
        # A RATING SWEEP, not a wrist dither: the speed is a FRACTION OF THIS JOINT'S RATING so the
        # commanded demand is the experimental variable, and the joint is restricted to the two whose
        # rating the CANFD channel can also reach (see RATING_SWEEP_JOINTS).
        j = int(joint)
        if j not in RATING_SWEEP_JOINTS:
            raise DryTrackRefused(
                f"a rating sweep runs on J{' or J'.join(str(x) for x in RATING_SWEEP_JOINTS)} only, not "
                f"J{j}: those are rated {RATED_DEG_S[0]} deg/s, the same as the CANFD pass-through "
                f"limit, so the SAME bytes can be streamed hold-free as a control. J{j} is rated "
                f"{RATED_DEG_S[j - 1]:g} deg/s and CANFD refuses above {tr.CANFD_MAX_RATE_DEG_S:g}, so a "
                f"clamp on it cannot be measured against a hold-free reference at all.")
        frac = float(probe.get("speed_frac") or 0.0)
        if not 0.0 < frac <= RATING_SWEEP_MAX_FRAC:
            raise DryTrackRefused(f"rating-sweep speed fraction {frac} must be in "
                                  f"(0, {RATING_SWEEP_MAX_FRAC}] of the joint's rating")
        probe = dict(probe, speed_deg_s=frac * RATED_DEG_S[j - 1])
        if float(probe.get("amplitude_deg", 0.0)) > RATING_SWEEP_MAX_AMPLITUDE_DEG:
            raise DryTrackRefused(f"rating-sweep amplitude {probe['amplitude_deg']} deg > "
                                  f"{RATING_SWEEP_MAX_AMPLITUDE_DEG} — J{j} swings the whole arm through "
                                  f"twice this, and the exposure is the operator's call, not the "
                                  f"measurement's")
        if float(probe.get("accel_deg_s2", 0.0)) > RATING_SWEEP_MAX_ACCEL_DEG_S2:
            raise DryTrackRefused(f"rating-sweep acceleration {probe['accel_deg_s2']} > "
                                  f"{RATING_SWEEP_MAX_ACCEL_DEG_S2} deg/s^2")
        if float(probe.get("duration_s", 0.0)) > RATING_SWEEP_MAX_DURATION_S:
            raise DryTrackRefused(f"rating-sweep duration {probe['duration_s']} s > "
                                  f"{RATING_SWEEP_MAX_DURATION_S}")
    elif probe:
        if int(joint) not in PROBE_JOINTS:
            raise DryTrackRefused(f"a hold probe moves a wrist joint {PROBE_JOINTS} only, not J{joint}")
        if float(probe.get("amplitude_deg", 0.0)) > PROBE_MAX_AMPLITUDE_DEG:
            raise DryTrackRefused(f"probe amplitude {probe['amplitude_deg']} deg > {PROBE_MAX_AMPLITUDE_DEG}")
        if float(probe.get("accel_deg_s2", 0.0)) > PROBE_MAX_ACCEL_DEG_S2:
            raise DryTrackRefused(f"probe acceleration {probe['accel_deg_s2']} > {PROBE_MAX_ACCEL_DEG_S2} deg/s^2")
    if probe:
        rows = probe_track_rows(rest["deg"], joint, probe["shape"], speed_deg_s=probe.get("speed_deg_s", 0.0),
                                accel_deg_s2=probe.get("accel_deg_s2", 600.0),
                                amplitude_deg=probe.get("amplitude_deg", 10.0),
                                duration_s=probe.get("duration_s", 20.0))
        roll_deg = float(probe.get("amplitude_deg", 0.0)) if probe["shape"] == "cruise" else 0.0
    else:
        rows = dry_track_rows(rest["deg"], points, roll_deg, cycles, joint)
    base = [int(round(v * 1000.0)) for v in rest["deg"]]
    facts = dry_track_facts(rows, base)
    if probe:
        # judged at the player's ceiling: a probe line lasts 5 ms, not the nominal clock
        pk = 0.0
        for a_, b_ in zip(rows, rows[1:]):
            pk = max(pk, max(abs(x - y) for x, y in zip(a_, b_)) / 1000.0 / PROBE_CLOCK_S)
        facts["peak_rate_deg_s_at_5ms"] = round(pk, 3)
    if max(facts["max_dev_millideg"]) > int(round(roll_deg * 1000.0)) + 1:
        raise DryTrackRefused("BUG: a dry track left the rest pose by more than the roll "
                              "angle")
    margins = joint_limit_margin_deg(rows)
    facts["joint_limit_margin_deg"] = margins
    bad = [(j + 1, margins[j], JOINT_LIMIT_DEG[j]) for j in range(7) if margins[j] < 0]
    if bad:
        raise DryTrackRefused(
            f"the dry track commands joint(s) outside their position limits {bad} "
            f"(rm_75_macro.xacro) — the rest pose plus the roll does not fit. Lower "
            f"--roll-deg, or dither another joint.")
    if probe and probe.get("rating_sweep"):
        # JUDGED AGAINST THIS JOINT'S RATING, not the wrist probe's cap: going through the rating is the
        # whole point, so the ceiling is the envelope's fraction of it.
        cap = RATING_SWEEP_MAX_FRAC * RATED_DEG_S[int(joint) - 1]
        if facts["peak_rate_deg_s_at_5ms"] > cap + 0.5:
            raise DryTrackRefused(f"rating sweep peaks at {facts['peak_rate_deg_s_at_5ms']:.1f} deg/s at "
                                  f"5.0 ms (> {cap:.1f} = {RATING_SWEEP_MAX_FRAC}x the J{int(joint)} rating)")
        blocks = (len(rows) - 1) // 20
        over = sum(1 for i in range(0, len(rows) - 20, 20)
                   if max(abs(x - y) for a_, b_ in zip(rows[i:i + 20], rows[i + 1:i + 21])
                          for x, y in zip(a_, b_)) / 1000.0 / PROBE_CLOCK_S
                   > RATED_DEG_S[int(joint) - 1])
        facts["blocks_20line"] = int(blocks)
        facts["blocks_over_rating"] = int(over)
        # A SWEEP THAT CANNOT BE ANALYSED IS NOT WORTH RUNNING. The per-position comparison needs blocks
        # at the condition, and a sweep asked for >= 1.0x that produces almost none is a bad parameter
        # set, not a result (the 24 installed J1-J7 sweeps carry 4-9 clamp blocks each; see
        # `sweep_design_audit.py`).
        # EXACTLY 1.0x IS THE AT-RATING CONTROL, not a clamp: no block is OVER the rating there by
        # construction, and that is the point — it is the fastest condition the CANFD channel can still
        # carry, so it is the matched hold-free reference for the 1.05x/1.10x clamp runs. Only a sweep
        # asked for more than the rating has to produce over-rating blocks.
        if float(probe.get("speed_frac") or 0) > 1.0 and over < RATING_SWEEP_MIN_BLOCKS:
            raise DryTrackRefused(
                f"this rating sweep puts only {over} of {blocks} 20-line blocks over the J{int(joint)} "
                f"rating, below the {RATING_SWEEP_MIN_BLOCKS} the per-position analysis needs — raise "
                f"--amplitude-deg or --duration-s so the joint spends longer at speed")
    elif probe:
        if facts["peak_rate_deg_s_at_5ms"] > PROBE_MAX_RATE_DEG_S + 0.5:
            raise DryTrackRefused(f"hold probe peaks at {facts['peak_rate_deg_s_at_5ms']:.1f} deg/s at 5.0 ms "
                                  f"(> {PROBE_MAX_RATE_DEG_S})")
    elif facts["peak_rate_deg_s"] > DRY_MAX_RATE_DEG_S:
        raise DryTrackRefused(f"dry track peaks at {facts['peak_rate_deg_s']:.1f} deg/s "
                              f"(> {DRY_MAX_RATE_DEG_S}) — lower --roll-deg / --cycles or "
                              f"raise --points; a dry track exists to be boring")
    if probe and probe.get("rating_sweep"):
        name = (f"ratesweep_{side}_j{int(joint)}_"
                f"{int(round(float(probe['speed_frac']) * 100))}pct_"
                f"a{int(round(probe['accel_deg_s2']))}")
    elif probe:
        name = (f"probe_{side}_j{int(joint)}_{probe['shape']}"
                + (f"_v{int(round(probe['speed_deg_s']))}_a{int(round(probe['accel_deg_s2']))}"
                   if probe["shape"] == "cruise" else f"_{int(round(probe.get('duration_s', 20)))}s"))
    else:
        name = f"dry_{side}_rest_j{int(joint)}_{int(round(roll_deg * 10)):03d}"
    track = out_dir / f"{int(slot):02d}_{name}.txt"
    write_dry_track(track, rows)
    proj = build_proj_for(track)
    n = len(rows)
    row = dict(
        slot=int(slot), name=name, side=side, status="EMITTED", kind="dry",
        plan=None, track=repo_rel(track), points=n,
        track_s=round((n - 1) * PLAYER_CLOCK_S, 4),
        band_5p5_5p6_s=[round((n - 1) * 0.0055, 3), round((n - 1) * 0.0056, 3)],
        acc_scale=None,
        rest_pose_deg=rest["deg"], rest_pose_sources=rest["sources"],
        roll_deg=float(roll_deg), cycles=(None if probe else int(cycles)), joint=int(joint),
        clock_s=(PROBE_CLOCK_S if probe else PLAYER_CLOCK_S), dry_facts=facts,
        probe=(dict(probe, clock_s=PROBE_CLOCK_S, ceiling_deg_s=PROBE_MAX_RATE_DEG_S) if probe else None),
        # A dry track commands NO devices: `run` reads pre_stroke and, with no
        # pole height in it, says so and compares nothing (it never COMMANDS
        # the pole anyway — the flow does).
        pre_stroke=dict(pole_m=None, pole_stage=None, hand_state=None,
                        hand_counts=None, hand_release_counts=None,
                        note="a dry track is run at whatever standoff the arm is already "
                             "at; it does not go near the fixture"),
        retreat_stage=None, retreat_deg=None,
        what=(("synthetic HOLD PROBE: this side's move_to_rest pose on every line"
               + (f", joint {int(joint)} cruising at {probe.get('speed_deg_s')} deg/s and reversing at "
                  f"{probe.get('accel_deg_s2')} deg/s^2 within +-{roll_deg} deg" if probe["shape"] == "cruise"
                  else ", nothing moving")
               + ", sampled at the player's 5.0 ms ceiling. The arm cannot leave rest by more than the "
                 "amplitude.") if probe else
              ("synthetic: this side's move_to_rest pose held for every point, joint "
               f"{int(joint)} dithered +-{roll_deg} deg over {int(cycles)} whole sine "
               "cycles at the player clock. Neither arm can leave rest by more than the "
               "roll angle.")))
    d = disk_provenance(row["track"], build=False)
    row.update(track_sha256=d["track_sha256"], track_bytes=d["track_bytes"],
               points_on_disk=d["points_on_disk"], proj=d["proj"],
               proj_sha256=d["proj_sha256"], proj_bytes=d["proj_bytes"])
    row["proj_points"] = proj["points"]
    # The tool frame is pinned the same way a cleaning task's is: from what
    # THIS side's controller was actually observed holding. A dry track does
    # not go near the fixture, but running it under a frame nobody expects is
    # exactly the state H35 is about, and the gate must not have a hole in it.
    ev = evidence if evidence is not None else run_record_evidence()
    src = next((t for t in rows_for(man, side) if tool_frame_from_records(t, ev)), None)
    row["tool_frame"] = tool_frame_from_records(src, ev) if src else None
    if row["tool_frame"]:
        row["tool_frame"]["provenance"]["note"] = (
            f"inherited from {src['name']}: the {side} controller's OBSERVED frame. A dry "
            f"track does not choose a TCP; it must simply not run under a surprise one.")
    return row


def cmd_dry(args) -> int:
    """Synthesise (and by default upload) a zero-exposure in-place track."""
    man = load_manifest()
    reg = load_dry_registry()
    try:
        probe = None
        if getattr(args, "shape", "sine") != "sine":
            if not getattr(args, "probe", False):
                raise DryTrackRefused(f"--shape {args.shape} is a hold probe: pass --probe (J6/J7 only, "
                                      f"<= {PROBE_MAX_RATE_DEG_S:g} deg/s at 5.0 ms, <= {PROBE_MAX_AMPLITUDE_DEG:g} deg)")
            probe = dict(shape=args.shape, speed_deg_s=float(args.speed_deg_s or 0.0),
                         accel_deg_s2=float(args.accel_deg_s2), amplitude_deg=float(args.amplitude_deg),
                         duration_s=float(args.duration_s))
            if getattr(args, "rating_sweep", False):
                if args.shape != "cruise":
                    raise DryTrackRefused("--rating-sweep is a cruise: pass --shape cruise")
                if getattr(args, "speed_frac", None) is None:
                    raise DryTrackRefused("--rating-sweep needs --speed-frac (of the joint's rating)")
                if getattr(args, "speed_deg_s", None):
                    raise DryTrackRefused("--rating-sweep sets the speed from --speed-frac; "
                                          "do not also pass --speed-deg-s")
                probe["rating_sweep"] = True
                probe["speed_frac"] = float(args.speed_frac)
        row = build_dry_row(man, args.side, args.slot, args.points, args.roll_deg,
                            args.cycles, args.joint,
                            evidence=run_record_evidence() if RUNS_ROOT.is_dir() else {}, probe=probe)
    except (DryTrackRefused, ValueError) as exc:
        # Every one of these is a SAFETY refusal (a production slot, a track
        # that is not boring, plans that disagree about rest). rc 4 is the
        # refusal code the rest of this tool uses; nothing was written.
        print(f"** dry track REFUSED: {exc}")
        return 4
    reg["tasks"] = [t for t in reg["tasks"]
                    if not (t.get("side") == row["side"] and t.get("slot") == row["slot"])]
    reg["tasks"].append(row)
    save_dry_registry(reg)
    f = row["dry_facts"]
    print(f"dry track -> {row['track']}")
    print(f"  {row['points']} points, {row['track_s']:.3f} s at the {PLAYER_CLOCK_S * 1e3:.4f} ms clock"
          + (f" ({(row['points'] - 1) * PROBE_CLOCK_S:.2f} s at one point per 5 ms)" if row.get("probe") else ""))
    print(f"  rest pose from {len(row['rest_pose_sources'])} {args.side} plan(s): "
          f"{[round(v, 3) for v in row['rest_pose_deg']]}")
    print(f"  moving joints {f['moving_joints']}, max deviation from rest "
          f"{max(f['max_dev_deg']):.3f} deg (roll {row['roll_deg']} deg), peak rate "
          f"{f['peak_rate_deg_s']:.2f} deg/s")
    print(f"  starts at rest: {f['first_is_rest']}   ends at rest: {f['last_is_rest']}")
    print(f"  track sha {row['track_sha256'][:12]}  .proj sha {row['proj_sha256'][:12]} "
          f"({row['proj_bytes'] / 1024:.1f} KB)")
    print(f"  registry -> {repo_rel(DRY_REGISTRY)}  (NOT the production manifest)")
    if args.no_upload:
        print("  --no-upload: nothing was sent")
        return 0
    from alix_dispatch import connect, track as tr
    confirm(f"upload the DRY track to slot {row['slot']} (only_save — NO MOTION)",
            args.side, args.ip or "default", assume_yes=args.yes)
    h = open_or_die(args.side, args.ip)
    try:
        # A DRY SLOT IS SHARED GROUND TOO (2026-09-18). `send_project` overwrites whatever is in the
        # slot, and a dry track can be put anywhere in 1..69 — the same program list the pendant saves
        # into. `upload --experiment` has guarded this since the experiment registry was added; this
        # lane never did, so a probe on slot 20 would have silently destroyed a pendant file there
        # (Newton: "there exist a file"). Same rule, same override.
        rc0, items0 = tr.program_list(h.robot)
        if rc0 == 0 and items0 is not None:
            stored0 = {it.get("id"): it.get("name") for it in items0}
            foreign = foreign_programs(stored0, [row])
            if foreign and not getattr(args, "overwrite_foreign", False):
                for f in foreign:
                    print(f"  ** slot {f['slot']} holds {f['stored']!r}, which is not one of ours")
                print("  Refusing. Choose a free slot (--slot N), or pass --overwrite-foreign if that "
                      "program may go.")
                return 4
            if foreign:
                for f in foreign:
                    print(f"  !! --overwrite-foreign: slot {f['slot']} held {f['stored']!r} and is "
                          f"being overwritten by {f['row']}")
        else:
            print(f"  !! cannot read the controller's program list (ret={rc0}) — writing slot "
                  f"{row['slot']} without the foreign-program check")
        ret, err, secs = tr.send_project(h.robot, REPO / row["proj"], row["slot"],
                                         only_save=True)
        print(f"  slot {row['slot']}: ret={ret} err_line={err} {secs:.1f}s")
        if not (ret == 0 and err == -1):
            return 4
        rc, items = tr.program_list(h.robot)
        stored = {it.get("id"): it for it in items}
        it = stored.get(row["slot"])
        if it is None:
            print(f"  ** slot {row['slot']} is NOT in the controller's list after upload")
            return 4
        # The RIVAL is the OTHER arm's dry row: both sides use slot 60, so a
        # swapped cable puts `60_60_dry_left_...` where `60_60_dry_right_...`
        # is expected. Those two differ at character 11, well inside the
        # 32-char field, so the name alone separates them — but the check can
        # only say so if it is given the rival, and this was the one call site
        # of five that passed none (workflow audit, 2026-09-10).
        other_dry = [r for r in (load_dry_registry().get("tasks") or [])
                     if r.get("side") != args.side]
        nc = trajectory_name_check(it.get("name"), row, other_side_rows=other_dry,
                                   stored_size=it.get("size"))
        # advisory where the controller does not use our naming model at all
        # (see `name_convention_confirmed`); the size gate is the hard one.
        # every one of this side's production rows counts as evidence for the
        # model too: if the controller names THEM our way, it names this one.
        name_model = name_convention_confirmed([i.get("name") for i in items],
                                               rows_for(man, args.side) + [row])
        delta = int(it.get("size") or 0) - row["proj_bytes"]
        print(f"  stored: {it.get('name')!r} {int(it.get('size') or 0) / 1024:.1f}KB "
              f"delta {delta:+d} B  name "
              f"{'ok' if nc['ok'] else ('** ' if name_model else '!! advisory ') + '; '.join(nc['reasons'])}")
        if (not nc["ok"] and name_model) or abs(delta) > max(64, row["proj_bytes"] // 1000):
            return 4
    finally:
        connect.close_arm(h)
    return 0


# ------------------------------------------------- the blocking probe (L50)
class BlockingProbe:
    """One flag, two consumers — and a mismatch between them is a REFUSAL.

    L50. `dual_arm_flow.py:198` hardcoded `blocking_probe=True`; the proven
    single-arm flow makes it opt-in. Two things read the flag: the program
    start (`track.run_verified(blocking_probe=)`) and the pole
    (`peripherals.Pole.move(block=)`). They must agree, because a BLOCKING
    `rm_set_program_id_run` consumes the next arrival event on the handle: if
    the pole moved non-blocking just before it, the program start eats the
    LIFT's arrival and returns -4 — indistinguishable from a real cut, and 7
    of 125 probe runs have returned -4 already. So the two values are recorded
    as they are handed out and checked before the program starts.
    """

    def __init__(self, enabled: bool):
        self.enabled = bool(enabled)
        self.seen = {}

    def for_consumer(self, who: str) -> bool:
        self.seen[who] = self.enabled
        return self.enabled

    def observe(self, who: str, value) -> None:
        """Record what a consumer ACTUALLY did (e.g. `Pole.move`'s returned
        `block`), which is the value that matters — not the one we asked for."""
        if value is not None:
            self.seen[who] = bool(value)

    def observe_pole(self, result) -> None:
        """Read a pole result of ANY shape — `Pole.move`'s, or a staging
        call's — and say what it did with the arrival.

        A result that shows a MOVE but does not say whether it blocked leaves
        the question open, and an open question here is not a pass: with the
        probe on, an unconsumed lift arrival is eaten by the blocking program
        start and returns a -4 that reads exactly like a cut. Unknown is
        recorded as unknown and `check` refuses it.

        THREE SHAPES, on purpose (hardening review 2026-09-08). `Pole.move`
        returns the answer directly in `block`; the staging contract's result
        (lane A) NESTS the pole under a `pole` key and carries no `block` at
        all, and reading only the outer dict there passed a real move silently
        — the exact false pass this probe exists to prevent. And a pole that
        did NOT move consumes no arrival: `verified_by == "no_op"` (or
        `no_op`/zero travel) is a fact, not an open question, so it must not
        be refused either.
        """
        if not isinstance(result, dict):
            return
        if "block" in result:
            self.observe("pole", result["block"])
            return
        inner = result.get("pole")
        if isinstance(inner, dict):
            # the staging contract: {'ok', 'side', 'pole': {...}, 'hand': ...}
            self.observe_pole(inner)
            return
        if result.get("no_op") is True or result.get("verified_by") == "no_op":
            return                              # nothing moved, no arrival exists
        moved = any(result.get(k) is not None
                    for k in ("ret", "target_m", "travel_s", "t_s"))
        if moved and (result.get("travel_s") == 0 or result.get("t_s") == 0):
            return                              # a zero-length move is not a move
        if moved and self.enabled:
            self.seen["pole"] = None            # a move whose arrival owner is unknown

    def check(self) -> dict:
        vals = set(self.seen.values())
        unknown = [k for k, v in self.seen.items() if v is None]
        ok = len(vals) <= 1 and not unknown
        if unknown:
            why = (f"the blocking probe is ON and {unknown} moved without saying whether it "
                   f"consumed its own arrival — a blocking program start would eat it and "
                   f"return a FALSE -4 (L50/F89). Refusing.")
        elif not ok:
            why = (f"blocking-probe mismatch between consumers {self.seen} — a blocking "
                   f"program start would consume an arrival that another device left "
                   f"unconsumed and return a FALSE -4 (L50/F89). Refusing.")
        else:
            why = ""
        return dict(ok=ok, enabled=self.enabled, seen=dict(self.seen), unknown=unknown,
                    why=why)


# ----------------------------------------------------------- hardware verbs

def cmd_push(args) -> int:
    """Read — and optionally repoint or disable — the controller's ONE realtime
    UDP push destination, without running anything.

    A controller has exactly one push target. A session that dies between
    `start` and `stop` leaves it pointing at itself, and from then on every
    capture demands --steal-push with nothing to say whether the owner is a
    live driver or that corpse (Newton, 2026-09-03: the flag was needed on
    every run of the session). This verb is how you look, and how you put it
    back, without touching the arm.
    """
    from alix_dispatch import connect
    h = open_or_die(args.side, args.ip)
    try:
        ret, cur = h.robot.rm_get_realtime_push()
        cur = cur if isinstance(cur, dict) else {}
        print(f"\n{args.side} controller push config (read ret={ret}):")
        for k in ("enable", "ip", "port", "cycle", "force_coordinate"):
            print(f"   {k:17s} {cur.get(k)}")
        cc = cur.get("custom_config")
        if isinstance(cc, dict):
            print(f"   {'custom_config':17s} " + ", ".join(f"{k}={v}" for k, v in cc.items()))
        mine = connect.host_ip_for(h.ip)
        port = args.udp_port or connect.DEFAULT_UDP_PORT
        if cur.get("enable"):
            who = ("THIS host" if str(cur.get("ip")) == str(mine)
                   else f"ANOTHER host (this one is {mine})")
            print(f"   -> enabled toward {who}. A capture here would need --steal-push.")
        else:
            print("   -> disabled; a capture can take it with no --steal-push.")
        if not (args.clear or args.set):
            return 0
        confirm(f"REPOINT the realtime push ({'disable' if args.clear else args.set})",
                args.side, h.ip, args.yes)
        from Robotic_Arm.rm_robot_interface import (rm_realtime_push_config_t,
                                                    rm_udp_custom_config_t)
        if args.clear:
            ip_to, port_to, enable = str(cur.get("ip") or mine), int(cur.get("port") or port), False
        else:
            ip_to, _, p_to = args.set.partition(":")
            port_to, enable = int(p_to or port), True
        cfg = rm_realtime_push_config_t(
            cycle=int(cur.get("cycle", 1) or 1), enable=enable, port=port_to,
            force_coordinate=int(cur.get("force_coordinate", -1) or -1), ip=ip_to,
            custom_config=rm_udp_custom_config_t(
                joint_speed=1, lift_state=1, expand_state=1, hand_state=1,
                arm_current_status=1, aloha_state=1))
        # -2/-3 are "the reply was lost", not "refused" (rm_interface.h:3456),
        # so the read-back is the answer, not the return code — and a write the
        # controller genuinely never took is simply repeated, the way the
        # capture path does it (track.UdpRecorder._apply_push).
        ok, sret, rb, now = False, None, None, {}
        for attempt in range(1, 4):
            sret = h.robot.rm_set_realtime_push(cfg)
            rb, now = h.robot.rm_get_realtime_push()
            now = now if isinstance(now, dict) else {}
            ok = (rb == 0 and bool(now.get("enable")) == enable
                  and (not enable or (str(now.get("ip")) == ip_to
                                      and int(now.get("port") or 0) == port_to)))
            if ok or sret == 0:
                break
            print(f"   set ret={sret} and it did not take — retrying ({attempt + 1}/3)")
            time.sleep(0.25)
        print(f"set ret={sret}; controller now reads {now.get('ip')}:{now.get('port')} "
              f"enable={now.get('enable')} (read ret={rb})")
        print("   -> applied" if ok else "   ** NOT applied — repeat, or set it from the pendant")
        return 0 if ok else 1
    finally:
        from alix_dispatch import connect as _c
        _c.close_arm(h)


def open_or_die(side: str, ip):
    """open_arm returns None as a SKIP ("no arm plugged in"); every verb here
    needs a hard refusal instead, with the side asserted before any use."""
    from alix_dispatch import connect, track as tr
    h = connect.open_arm(side, ip)
    if h is None:
        raise SystemExit(f"no usable connection to the {side} arm — aborting")
    tr.assert_side(side, h.side)
    return h


def confirm(action: str, side: str, ip: str, assume_yes: bool = False) -> None:
    """Interactive supervision prompt. --yes (assume_yes) skips the typing —
    for the automated executive, where the same protection is the MACHINE
    side assert (track.assert_side: manifest side vs connection side); the
    banner is still printed so logs show what ran where."""
    print(f"\n*** LIVE: {action} — {side.upper()} arm at {ip} ***")
    print("Preconditions: e-stop released and not latched, all joints enabled, workspace clear.")
    print("Driver coexistence: if an alix_hw driver is up it is detected and custody is taken")
    print("through its own choreography (hold_release + JTC cycle) and verified on handback;")
    print("if ROS is unavailable here, the driver MUST be stopped (cannot be checked).")
    if assume_yes:
        print(f"--yes: proceeding on the {side.upper()} arm (side verified by assert, not by prompt)")
        return
    if input(f"Type the arm side to proceed [{side}]: ").strip().lower() != side:
        raise SystemExit("aborted")


def driver_present(side: str) -> bool:
    """True when an alix_hw driver serves this side; None-ish (False) when ROS
    is unavailable here — announced, since it then cannot be checked."""
    try:
        sys.path.insert(0, str(REPO / "ros/orchestration/alix_tasks/scripts"))
        import arm_custody
        return bool(arm_custody.driver_present(side))
    except Exception as exc:                                           # noqa: BLE001
        print(f"(driver detection unavailable: {exc} — proceeding as OFF-DRIVER; "
              f"ensure no driver is running)")
        return False


def cmd_upload(args) -> int:
    from alix_dispatch import connect, track as tr
    man = manifest_for(args)
    side = args.side
    rows = [by_name(man, t, side) for t in args.track] if args.track else rows_for(man, side)
    if not args.track:
        rows = drop_canfd_only(rows, "upload")
    if not rows:
        raise SystemExit(f"no emitted {side} tracks in the manifest")
    refuse_canfd_only_on_upload(rows, "upload")
    ip = args.ip
    # B15: never upload bytes the manifest did not pin. A row whose track no
    # longer hashes to `track_sha256` is a different file than the one the
    # bands, the collision sweep and the schedule were computed from, and an
    # UNPINNED row is the state that let a 2218-point stroke run four times
    # from a slot whose disk file holds 1891.
    print("provenance gate (manifest pins vs the bytes on disk):")
    if not provenance_gate(rows, REPO):
        print("\n  ** refusing to upload. Re-emit the offending region, or "
              "`cleaning_tracks.py provenance --write` if the disk bytes are the "
              "ones you mean to pin — and say so in the record.")
        return 4
    print(f"   {len(rows)} row(s) match their pins")
    confirm(f"upload {len(rows)} track(s) to slots {[t['slot'] for t in rows]} "
            f"(only_save — NO MOTION)", side, ip or "default", assume_yes=args.yes)
    h = open_or_die(side, ip)
    sent = {}
    try:
        if getattr(args, "experiment", False) and not getattr(args, "overwrite_foreign", False):
            # experiment slots 21-39 are shared ground with the pendant's own programs
            rc0, items0 = tr.program_list(h.robot)
            if rc0 != 0:
                print(f"  ** cannot read the controller's program list (ret={rc0}) — refusing to "
                      f"write experiment slots blind; --overwrite-foreign overrides")
                return 4
            foreign = foreign_programs({it.get("id"): it.get("name") for it in items0}, rows)
            if foreign:
                for f in foreign:
                    print(f"  ** slot {f['slot']} holds {f['stored']!r}, which is not one of ours "
                          f"— uploading {f['row']} would overwrite it")
                print("  Refusing. Choose free slots (`experiment --replace --slot N`), or pass "
                      "--overwrite-foreign if those programs may go.")
                return 4
        for t in rows:
            proj = tr.build_project_file(REPO / t["track"],
                                         (REPO / t["track"]).with_suffix(".proj"))
            # the file we are about to send, hashed as it leaves
            proj_sha = sha256_of(proj["path"])
            if t.get("proj_sha256") and t["proj_sha256"] != proj_sha:
                print(f"  ** slot {t['slot']} {t['name']}: the .proj built from this track "
                      f"hashes {proj_sha[:8]}, the manifest pins {t['proj_sha256'][:8]} — "
                      f"refusing (the uploader and the pin disagree about the bytes)")
                return 4
            ret, err, secs = tr.send_project(h.robot, pathlib.Path(proj["path"]), t["slot"],
                                             only_save=True)
            verdict = "OK" if (ret == 0 and err == -1) else "FAILED"
            print(f"  slot {t['slot']:>3} {t['name']:34s} ret={ret} err_line={err} "
                  f"{secs:5.1f}s {proj['bytes'] / 1024:6.1f}KB  {verdict}")
            if verdict == "FAILED":
                return 4
            sent[t["slot"]] = proj["bytes"]
        # ret=0/err_line=-1 says the controller ACCEPTED the file, not that it
        # kept all of it. rm_get_program_trajectory_list is the only vendor
        # readback of what is actually stored, so compare it: a short slot
        # would end its program early with no error at all, which is exactly
        # the signature we are chasing (F112).
        rc, items = tr.program_list(h.robot)
        stored = {it.get("id"): it.get("size") for it in items}
        stored_names = {it.get("id"): it.get("name") for it in items}
        print(f"\n  stored-size readback (rm_get_program_trajectory_list ret={rc}):")
        # IS THE NAMING MODEL TRUE OF THIS CONTROLLER? No hardware record
        # carries a program list, so "the stored name is our .proj basename"
        # has only ever been seen in the emulated double. `preflight` already
        # downgrades the name to advisory where nothing on the controller
        # resembles our files; `upload` was refusing outright, which would
        # have made a getter that spells its slots `Project_70` block the
        # entire bringup — a worse failure than the one it prevents
        # (hardening review 2026-09-08). The SIZE gate is confirmed hardware
        # behaviour and carries the check alone in that branch.
        others = rows_for(man, "left" if side == "right" else "right")
        name_model = name_convention_confirmed(stored_names.values(), rows + others)
        if not name_model:
            print(f"   !! none of this controller's stored names "
                  f"{sorted({str(n) for n in stored_names.values() if n})[:4]} look like the "
                  f"files we upload — name findings below are ADVISORY, the stored-size "
                  f"comparison is the gate. Record the real names.")
        bad = 0
        for slot, nbytes in sorted(sent.items()):
            got = stored.get(slot)
            if got is None:
                print(f"   slot {slot:>3}: NOT IN THE CONTROLLER'S LIST  (sent "
                      f"{nbytes / 1024:.1f}KB)")
                bad += 1
                continue
            delta = got - nbytes
            flag = "ok" if abs(delta) <= max(64, nbytes // 1000) else "** MISMATCH"
            if flag != "ok":
                bad += 1
            row = next(t for t in rows if t["slot"] == slot)
            nc = trajectory_name_check(stored_names.get(slot), row, stored_size=got,
                                        other_side_rows=others)
            if not nc["ok"]:
                if name_model:
                    bad += 1
                    flag += "  ** NAME: " + "; ".join(nc["reasons"])
                else:
                    flag += "  !! name (advisory): " + "; ".join(nc["reasons"])
            print(f"   slot {slot:>3}: sent {nbytes / 1024:7.1f}KB  stored "
                  f"{got / 1024:7.1f}KB  delta {delta:+7d} B  name "
                  f"{str(stored_names.get(slot))!r}  {flag}")
        if bad:
            print(f"\n  ** {bad} slot(s) do not match what was sent — the controller did "
                  f"not keep the whole file. Do NOT run these; re-upload and re-check.")
            return 4
    finally:
        connect.close_arm(h)
    return 0


def _jsonable(v):
    """Coerce an SDK struct (or a dict holding one, e.g. the push config's
    nested custom_config) into something json.dump accepts."""
    if v is None or isinstance(v, (bool, int, float, str)):
        return v
    if isinstance(v, dict):
        return {str(k): _jsonable(x) for k, x in v.items()}
    if isinstance(v, (list, tuple)):
        return [_jsonable(x) for x in v]
    d = getattr(v, "__dict__", None)
    if d:
        return {str(k): _jsonable(x) for k, x in d.items() if not k.startswith("_")}
    return repr(v)


_REST_CACHE: dict = {}


_SIDE_REST_CACHE: dict = {}


def taught_rest_setting(t: dict) -> dict:
    """Which rest legs a planless row takes in `flow`: {before, after[, default]}.

    A TAUGHT row that never set it takes BOTH (Newton, 2026-09-22: "the rest pose, that is flow,
    should be the default behaviour"); `taught_tracks.py rest --none` stores an explicit opt-out,
    which is honoured. A dry or experiment row moves to rest only if its row asks. A plan row keeps
    its own plan's move_to_rest stage (`rest_for`), never this rule."""
    if t.get("plan"):
        return {}
    want = t.get("rest")
    if want is None and t.get("kind") == "taught":
        return dict(before=True, after=True, default=True)
    return want or {}


def taught_rest(t: dict, when: str):
    """The side's move_to_rest pose for a TAUGHT row that takes a rest move `when` ("before" or
    "after" its stroke), else None.

    Newton, 2026-09-21: "append and prepend with a movej to rest"; 2026-09-22: the default. A taught
    row has no plan, so it has no move_to_rest stage of its own; the pose is the SIDE's, read from
    that side's cleaning plans by `side_rest_pose` (all of them must agree), never invented and
    never stored in the row — the row records at most THAT it opts out (`rest: {before, after}`,
    set by `taught_tracks.py rest`). A plan row keeps its own plan's pose (`rest_for`).

    `rest.at == "start"` (`taught_tracks.py rest --at start`): the pose is instead the row's OWN
    first line — the drag start pose every recording of a fixture begins at, the "START POSE
    (Drag_start_pose) — GLOBAL WAYPOINT" of the operator's flow sheet (Newton, 2026-09-22; ten of
    the eleven commode C recordings begin within 0.04° of one pose, 30.2° (J6) from the side's
    move_to_rest). Read from the certified .txt at run time, so it is never invented and cannot
    drift from the bytes. `rest.at == "pose"`: a pose STORED in the row (`rest --at pose --deg ...`),
    for a recording that does not begin there — FILE_01.2 begins 163.7° away on J7."""
    want = taught_rest_setting(t)
    if not want.get(when):
        return None
    if want.get("at") == "start":
        from alix_dispatch import track as _tr
        first = _tr.parse_track(REPO / t["track"])[0]
        return dict(stage="the stroke's own start (the drag start pose)",
                    deg=[v / 1000.0 for v in first])
    if want.get("at") == "pose":
        # a stored pose (`rest --at pose`): a recording that does not begin at its fixture's drag
        # start pose waits there anyway; the program's bridge then goes to the stroke's first line
        return dict(stage=str(want.get("pose_name") or "the row's stored rest pose"),
                    deg=[float(x) for x in want["deg"]])
    side = t["side"]
    if side not in _SIDE_REST_CACHE:
        rp = side_rest_pose(load_manifest(), side)
        _SIDE_REST_CACHE[side] = dict(
            stage=f"the side's move_to_rest (the {side} cleaning plans, {len(rp['sources'])} agree)",
            deg=[float(x) for x in rp["deg"]])
    return _SIDE_REST_CACHE[side]


#: A TAUGHT row's rest legs are never checked for collisions and can be large (slot 59: 118 deg on
#: J3 from the stroke's end), so they run no faster than the operator's own bridge speed.
def taught_rest_speed(args) -> int:
    # the bridge runs at --speed when --movej-speed is 0, and the legs never outrun the bridge
    bridge = getattr(args, "movej_speed", None) or getattr(args, "speed", None)
    speeds = [int(v) for v in (getattr(args, "rest_speed", None), bridge) if v]
    return min(speeds) if speeds else 20


#: The measured gap under which the arm already IS at rest, so the prepended move is skipped.
REST_AT_TOL_DEG = 1.0

#: Verdicts reached before the stroke moved at all — a taught row's park is skipped after them:
#: either nothing moved, or the prepended move already left the arm at rest (review 2026-09-21).
PRE_MOTION_VERDICTS = ("refused_no_state", "refused_not_at_start", "refused_pre_run",
                       "refused_devices", "refused_provenance", "refused_tool_frame",
                       "refused_blocking_probe", "refused_capture", "refused_canfd_limits",
                       "refused_canfd_state", "refused_movej", "did_not_start")


def taught_rest_legs(t: dict) -> dict:
    """The two joint-space legs a rest-around taught row adds, for the operator's banner:
    rest -> the stroke's first point and its last point -> rest, per-joint excursion in deg."""
    from alix_dispatch import track as _tr
    rows = _tr.parse_track(REPO / t["track"])
    first, last = [v / 1000.0 for v in rows[0]], [v / 1000.0 for v in rows[-1]]
    out = {}
    for when, a_b in (("before", "first"), ("after", "last")):
        pose = taught_rest(t, when)
        if pose is None:
            continue
        q = first if a_b == "first" else last
        exc = [round(b - a, 1) for a, b in zip(pose["deg"], q)] if a_b == "first" else \
              [round(a - b, 1) for a, b in zip(pose["deg"], q)]
        out[when] = dict(pose=pose["deg"], excursion_deg=exc,
                         max_deg=max(abs(x) for x in exc), leg=("rest -> first point" if a_b == "first"
                                                               else "last point -> rest"))
    return out


def rest_for(t: dict):
    """The plan's move_to_rest pose for this task, cached per plan path.

    Read from the PLAN rather than the manifest so no re-emit is needed: the
    tracks on the controller stay exactly as uploaded. All nine commode_c right
    plans carry the same pose, but it is read per task, never assumed.
    """
    from alix_dispatch import peripherals as per
    key = t.get("plan")
    if not key:
        # A taught row has no plan at all (and no move_to_rest); this used to
        # try `REPO / None` and print a TypeError on every taught run. It parks
        # only when the row asks for it (`rest.after`), at its SIDE's pose.
        return taught_rest(t, "after")
    if key not in _REST_CACHE:
        try:
            _REST_CACHE[key] = per.rest_pose(json.loads((REPO / key).read_text()))
        except Exception as exc:                                       # noqa: BLE001
            print(f"   ** rest: cannot read {key}: {exc!r}")
            _REST_CACHE[key] = None
    return _REST_CACHE[key]


def check_devices_as_found(h, t: dict) -> dict:
    """What state the pole and the hand are ACTUALLY in, versus what this task
    was planned with. Read-only and side-effect free.

    A single `run` never commands the devices — the flow owns that. That is a
    deliberate design, but it was also silent: on 2026-09-02 twenty of
    twenty-three runs replayed with whatever the previous task had left, six of
    eight of them with the pole at 150 mm where the plan wanted 75, and the
    console said nothing either way (F112). The pole is read here because
    `rm_get_lift_state` costs nothing and changes nothing. The hand is NOT
    read: that needs the end-port Modbus mode, and switching controller modes
    as a side effect of a check is worse than not knowing — so the expected
    hand state is printed and named as unverified.
    """
    from alix_dispatch import peripherals as per
    ps = t.get("pre_stroke") or {}
    out = {"pole_want_m": ps.get("pole_m"), "pole_found_m": None,
           "hand_want": ps.get("hand_state"), "hand_counts": ps.get("hand_counts"),
           "hand_verified": False, "mismatch": False, "lines": []}
    want = ps.get("pole_m")
    try:
        found = per.Pole(h.robot, log=lambda *_: None).height_m()
    except Exception as exc:                                           # noqa: BLE001
        out["lines"].append(f"   devices: pole height unreadable ({exc!r})")
        return out
    out["pole_found_m"] = found
    if want is None or found is None:
        out["lines"].append("   devices: no pole height to compare (not in pre_stroke)")
    else:
        delta = abs(found - float(want))
        if delta > per.LIFT_TOL_M:
            out["mismatch"] = True
            out["lines"].append(
                f"   ** devices NOT SET: the pole is at {found * 1000:.0f} mm but this "
                f"task was planned at {float(want) * 1000:.0f} mm "
                f"({delta * 1000:.0f} mm off). A single `run` commands no devices.")
        else:
            out["lines"].append(
                f"   devices: pole at {found * 1000:.0f} mm, matches the plan's "
                f"{float(want) * 1000:.0f} mm (not commanded by `run`)")
    out["lines"].append(
        f"   devices: hand should be '{out['hand_want']}' {out['hand_counts']} — NOT "
        f"verified (reading it needs the end-port Modbus mode; `flow` sets it)")
    return out


def caps_for(args):
    """The Cartesian envelope this replay should run under, or None.

    Newton, 2026-09-05: raise the caps for the replay itself. The four values
    are global, persistent controller state, so `track.run_verified` holds them
    only around the program and puts the controller's own back afterwards.
    `--no-caps` runs at whatever the arm is already configured with, which is
    what every run before today did.
    """
    from alix_dispatch import limits as lim
    if getattr(args, "no_caps", False):
        return None
    # argparse defaults are None so that "not given" is distinguishable; the
    # module constants are the actual defaults.
    return lim.replay_caps(
        line_speed=getattr(args, "line_speed", None) or lim.REPLAY_LINE_SPEED,
        angular_speed=getattr(args, "angular_speed", None) or lim.REPLAY_ANGULAR_SPEED,
        ratio=getattr(args, "cap_ratio", None) or lim.REPLAY_ACC_RATIO)


class _CanfdPreRunRefusal(Exception):
    """A CANFD pre-run refusal (no push, or the arm already in a stop state): nothing is streamed."""


def canfd_options(args) -> dict:
    """The CANFD stream's settings from the command line.

    THE DEFAULT PERIOD IS THE CONTROLLER'S CYCLE, 5.0 ms
    (`tr.CANFD_DEFAULT_PERIOD_S`). It was `tr.REPLAY_CLOCK_S` = 5.5545 ms, which
    is NOT a clock: it is the upload player's WHOLE-RUN average (F98's 29.167 s
    over 5251 intervals on one track), and 5.5545/5.000 = 1.1109 — an 11.09 %
    inflation that sits inside the independently measured 10-13 % hold fraction.
    The player advances at most one point per 5 ms cycle and holds stochastically;
    the average is the cycle plus the holds. CANFD has no holds, so here the
    period IS the clock (Newton 2026-09-20: "we have confirmed multiple times
    5.0 ms is the clock, the fluctuation is an artefact of the holds").

    The only thing 5.5545 ever bought was a like-for-like CROSS-CHANNEL
    comparison, which is not a method we use (Newton: "per channel independent
    analysis"). Runs before 2026-09-20 used other periods and each record carries
    its own `canfd.period_s` — group by it, never assume it.
    """
    from alix_dispatch import track as tr
    ms = getattr(args, "canfd_period_ms", None)
    period = float(ms) / 1000.0 if ms else tr.CANFD_DEFAULT_PERIOD_S
    follow = (getattr(args, "canfd_follow", None) or "high") == "high"
    mode = int(getattr(args, "canfd_mode", None) or 0)
    radio = int(getattr(args, "canfd_radio", None) or 0)
    rate_gate = (getattr(args, "canfd_rate_gate", None) or "per_joint").replace("-", "_")
    if rate_gate not in tr.CANFD_RATE_GATES:
        raise SystemExit(f"--canfd-rate-gate {rate_gate}: expected flat or per-joint")
    # the vendor's own bounds, checked here so a typo cannot reach the arm (2026-09-17 review)
    if not 0.002 <= period <= 0.020:
        raise SystemExit(f"--canfd-period-ms {period * 1e3:g}: the vendor's pass-through period is "
                         f"2-20 ms (Gen-3 Ethernet reaches 2 ms)")
    if follow and period > 0.010 + 1e-9:
        raise SystemExit(f"--canfd-period-ms {period * 1e3:g} needs --canfd-follow low: high follow "
                         f"requires a period <= 10 ms (vendor)")
    if mode and not follow:
        raise SystemExit(f"--canfd-mode {mode} takes effect in HIGH follow only (vendor)")
    if mode == 1 and not 0 <= radio <= 100:
        raise SystemExit("--canfd-radio is 0-100 in curve-fit mode (1)")
    if mode == 2 and not 0 <= radio <= 999:
        raise SystemExit("--canfd-radio is 0-999 in filter mode (2)")
    return {"period_s": period, "follow": follow, "trajectory_mode": mode, "radio": radio,
            "rate_gate": rate_gate}


def run_one(h, t: dict, args, rec_dir: pathlib.Path, sdk: str, before_run=None,
            stamp: str = None, after_run=None, limits_cache=None) -> tuple:
    """ONE slot on an open handle: the run policy (gap check, movej-bridge
    default per ruling 2026-09-01, speed guard, band verdict) lives in
    alix_dispatch.track.run_verified so the task executive shares it exactly;
    this adds the UDP capture (ON by default — every section at 5 ms, the
    evidence; --no-record-joints runs unlogged), its completeness/clock
    verdict, and the ledger record. *before_run* (the flow's retreat, device
    moves and a taught row's move to rest before its stroke) executes UNDER the
    capture, before the program; returning
    {'ok': False, ...} refuses the run. Returns (rc, out)."""
    from alix_dispatch import connect as _connect, track as tr
    lo, hi = t["band_5p5_5p6_s"]
    # ONE flag, two consumers (L50). Created here so it is in place before
    # `before_run` moves the pole, and read again before the program starts.
    bp = BlockingProbe(getattr(args, "blocking_probe", False))
    setattr(args, "_bp", bp)
    # THE TOOL-FRAME GUARD'S RECORD EXISTS FROM THE FIRST LINE. It was first
    # created inside the run branch, next to the guard, while the ledger write
    # below reads it unconditionally — so every refusal that happens BEFORE the
    # guard (provenance B15, devices, slot check) raised UnboundLocalError and
    # the refusal's own run record was never written (2026-09-17, slot 43,
    # twice). An empty dict here means "the guard never ran", which is true.
    tf_guard_rec: dict = {}
    # B15: the bytes on disk are what the band, the point count and every
    # collision figure were computed from. If they no longer hash to the pin,
    # the controller is holding something this record cannot describe.
    prov = check_row_provenance(t, REPO)
    # the opposite side's rows, for the ambiguous-name test (two of the twelve
    # shared slots truncate identically). Best effort: a dry row or a manifest
    # that will not load must not stop a run.
    try:
        other_rows = rows_for(manifest_for(args), "left" if h.side == "right" else "right")
    except Exception:                                                  # noqa: BLE001
        other_rows = []
    for w in prov["warnings"]:
        print(f"   !! {w}")
    # The track is read AFTER the provenance verdict, not before (hardening
    # review 2026-09-08). `parse_track` used to run first, so a row whose file
    # is missing or unparsable raised FileNotFoundError out of `run_one` — past
    # every refusal, past the record, and, in a worker thread, past any
    # `except Exception` a dual executive puts around it. The two states
    # check_row_provenance names ("the track file is missing", "does not
    # parse") were therefore unreachable from `run`.
    first_row = None
    if prov["disk"]["track_exists"] and not prov["disk"]["parse_error"]:
        try:
            first_row = tr.parse_track(REPO / t["track"])[0]
        except Exception as exc:                                       # noqa: BLE001
            prov["reasons"].append(f"the track's first point is unreadable: {exc!r}")
            prov["ok"] = False
    stamp = stamp or datetime.datetime.now().strftime("%H%M%S")
    rec_joints = None
    # A CAPTURE THAT CANNOT START IS A REFUSAL, NOT A SystemExit (hardening
    # review 2026-09-08). Both of these used to raise SystemExit from inside
    # `run_one` — which is the function the dual executive calls PER ARM IN A
    # WORKER THREAD. SystemExit is a BaseException: it walks past any
    # `except Exception` the executive wraps the call in, and in a thread
    # Python swallows it and the thread simply stops, leaving the other arm
    # running with no verdict, no run record and no reason anywhere. The
    # behaviour a human sees is unchanged (the run does not start, the rc is
    # non-zero, `flow` stops on it) — but now it is a RECORDED refusal.
    # THE CHANNEL (2026-09-17). `upload` plays the controller's slot; `canfd`
    # streams the SAME disk lines from this host (track.stream_track). A CANFD
    # run's record and capture are named `canfd_slot*`, so no reader of the
    # upload ledger (`run_slot*`: clock envelopes, measured calibration, tool
    # frame evidence, pairing counts) ever takes a host-timed run for a
    # player-timed one; the `channel` field says the same inside the record.
    channel = getattr(args, "channel", None) or "upload"
    is_canfd = (channel == "canfd")
    rec_prefix = "canfd_slot" if is_canfd else "run_slot"
    rows_all_pre = None
    if is_canfd and first_row is not None:
        rows_all_pre = tr.parse_track(REPO / t["track"])
    capture_refusal = None
    if is_canfd and args.record_joints is False:
        capture_refusal = ("CANFD needs the capture: the stream's stop check reads the arm "
                           "from the push, and without it lines keep going to an arm that "
                           "may have stopped. Drop --no-record-joints for --channel canfd.")
    if args.record_joints is not False:
        if not h.host_ip:
            capture_refusal = ("capture: cannot determine this host's address on the route "
                               "to the arm — the push would deliver nothing. Runs are logged "
                               "by default; --no-record-joints runs UNLOGGED.")
        else:
            rec_joints = tr.UdpRecorder(rec_dir / f"{rec_prefix}{t['slot']}_{stamp}_frames.ndjson",
                                        h.host_ip, port=getattr(args, "udp_port", None),
                                        allow_steal=args.steal_push)
            try:
                rec_joints.start(h.robot)
            except Exception as exc:                                   # noqa: BLE001
                rec_joints = None
                capture_refusal = (f"capture could not start: {exc}. Runs are logged by "
                                   f"default; --no-record-joints runs UNLOGGED.")
    pre, out = None, None
    # Per-task host timing. The between-task gap is OUR record-writing chain
    # (measured 2026-09-05: it regresses on the previous capture's frame count
    # at 0.709 s per 1000 frames), and every figure for the ROBOT host so far
    # is a projection through a fitted 2.879x. Recording the splits turns the
    # projection into a measurement on the next real flow.
    timing = {}
    park_t_mono = None
    snap_before = snap_after = settle_out = None
    dev_check = events = evt_out = post = out_tf = None
    # ...and what the CONTROLLER is holding in that slot. The disk pin cannot
    # see the arm: B15 was a file replaced on disk while the slot kept the old
    # bytes, and every disk-side check stays green through that.
    try:
        if is_canfd:
            # A stream plays the disk lines, never the slot: what the slot holds
            # cannot reach this run (the disk pin, B15, still applies).
            slot_chk = dict(ok=True, reasons=[], warnings=[], readable=None, listed=None,
                            stored_size=None, stored_name=None,
                            pinned_bytes=t.get("proj_bytes"),
                            skipped="channel canfd streams the lines on disk; the "
                                    "controller's slot is not used")
        else:
            slot_chk = controller_slot_check(h.robot, t, other_side_rows=other_rows)
    except Exception as exc:                                           # noqa: BLE001
        # A readback that raises is the getter failing, not the arm: say so and
        # let the disk-side gates decide. Never take a run out past its own
        # refusals (and, in a worker thread, out of existence).
        slot_chk = dict(ok=True, reasons=[], readable=False, listed=None,
                        stored_size=None, stored_name=None,
                        pinned_bytes=t.get("proj_bytes"),
                        warnings=[f"the controller's program list raised {exc!r} — the "
                                  f"stored slot was NOT compared with the pin"])
    for w in slot_chk["warnings"]:
        print(f"   !! slot {t['slot']}: {w}")
    for why in slot_chk["reasons"]:
        print(f"   ** slot {t['slot']}: {why}")
    if not slot_chk["ok"] and not getattr(args, "ignore_provenance", False):
        prov = dict(prov, ok=False,
                    reasons=list(prov["reasons"]) + list(slot_chk["reasons"]))
    try:
        if capture_refusal is not None:
            print(f"   ** {capture_refusal}")
            print("   Refusing: the evidence for this run cannot be recorded. Pass "
                  "--no-record-joints to run UNLOGGED, or fix the push (`push --show`).")
            out = {"ok": False, "verdict": "refused_capture", "gap_deg": None,
                   "duration_s": None, "implied_ms_per_pt": None,
                   "wrong_id_seen": False, "transitions": [], "run_ret": None,
                   "capture_refusal": capture_refusal}
        elif not prov["ok"] and not (getattr(args, "ignore_provenance", False)
                                     and first_row is not None):
            for why in prov["reasons"]:
                print(f"   ** provenance: {why}")
            # --ignore-provenance overrides a DISAGREEMENT about the bytes. It
            # cannot override their ABSENCE: with no first point there is
            # nothing to check the arm's start gap against and nothing to
            # bridge to, so the override is not offered on that branch.
            # NAME THE ACTUAL CAUSE (2026-09-18). A slot-check failure is folded into
            # `prov` above, so an EMPTY slot printed "this row is not pinned to the bytes on
            # disk (B15) — re-emit or `provenance --write`", which is the wrong remedy: the
            # pins were fine, the controller simply had nothing in that slot. Newton hit this
            # on all 24 sweep rows. The disk-pin remedy is only offered when the disk pin is
            # what failed.
            not_stored = any("NOT on this controller" in str(w) for w in prov["reasons"])
            why_line = ("this slot is empty on the controller — `upload` it first"
                        if not_stored else
                        "this row is not pinned to the bytes on disk (B15). Re-emit or "
                        "`provenance --write`, then re-upload")
            print(f"   Refusing: {why_line}; --ignore-provenance overrides and is recorded."
                  if first_row is not None else
                  "   Refusing: the track file cannot be read, so the arm's distance from "
                  "its first point cannot be checked. --ignore-provenance does NOT "
                  "override a missing or unparsable track.")
            out = {"ok": False, "verdict": "refused_provenance", "gap_deg": None,
                   "duration_s": None, "implied_ms_per_pt": None,
                   "wrong_id_seen": False, "transitions": [], "run_ret": None}
        elif before_run is not None:
            pre = before_run()
            # Whatever staged the devices, read back what it did with the lift's
            # arrival — the flow's own `before_run` says so directly, and any
            # other stager is held to the same question (L50).
            if isinstance(pre, dict):
                bp.observe_pole(pre.get("pole"))
            if pre is not None and not pre.get("ok", True):
                out = {"ok": False, "verdict": "refused_pre_run", "gap_deg": None,
                       "duration_s": None, "implied_ms_per_pt": None,
                       "wrong_id_seen": False, "transitions": [], "run_ret": None}
            else:
                # THE FLOW READS THE POLE BACK TOO. This check used to live only
                # in the `else` below, so the flow path — the one that actually
                # commands the devices — never ran it: 0 of 68 records on
                # 2026-09-04 carry `devices_as_found`. The flow trusted
                # `state["pole_m"]`, which is the COMMANDED target, and skipped
                # the pole entirely on 6 of 12 tasks per flow, so nothing ever
                # compared the arm's real height against the plan's.
                #
                # It is latent, not broken: measured at the first
                # move_through_joint frame of all 68 runs, the pole sat within
                # 5 microns of its authored height. But the same defect HAS
                # bitten on the `run` path — 19 runs on 2026-09-01/02 replayed
                # up to 75 mm off — and one read costs ~6 ms against a stroke
                # that runs for 4-24 s.
                dev_check = check_devices_as_found(h, t)
                for line in dev_check["lines"]:
                    print(line)
                if dev_check.get("mismatch") and not args.ignore_devices:
                    print("   The flow commanded the devices and the pole still does not "
                          "match the plan. Refusing rather than wipe at the wrong standoff; "
                          "--ignore-devices overrides.")
                    out = {"ok": False, "verdict": "refused_devices", "gap_deg": None,
                           "duration_s": None, "implied_ms_per_pt": None,
                           "wrong_id_seen": False, "transitions": [], "run_ret": None}
        else:
            # A single `run` commands NO devices — by design, the flow owns that
            # choreography. Until 2026-09-03 it also said nothing, so 20 of the 23
            # runs on 2026-09-02 replayed with whatever pole height the previous
            # task left (150 mm where 6 of 8 wanted 75) and nobody could tell from
            # the console. Check what can be checked without side effects, and
            # REFUSE by default rather than run a stroke at the wrong standoff.
            dev_check = check_devices_as_found(h, t)
            if dev_check.get("mismatch") and not args.ignore_devices:
                for line in dev_check["lines"]:
                    print(line)
                print("   Use `flow` (which sets the pole and the hand for each task) or "
                      "pass --ignore-devices to run anyway.")
                out = {"ok": False, "verdict": "refused_devices", "gap_deg": None,
                       "duration_s": None, "implied_ms_per_pt": None,
                       "wrong_id_seen": False, "transitions": [], "run_ret": None}
            else:
                for line in dev_check["lines"]:
                    print(line)
        # F111: the push frames latch only the FIRST error code of a run, so the
        # controller's state is read directly before and right after the program
        snap_before = tr.controller_snapshot(h.robot, limits_cache)
        print("   before: " + tr.snapshot_line(snap_before))
        # THE TOOL FRAME IS RECORDED, NEVER REFUSED (Newton, 2026-09-15: "the
        # frame should never be an issue ... just collect data on the frame
        # set, instead of blocking the run"). The row pins the frame ITS OWN
        # runs were recorded under and nothing in this channel sets the frame,
        # so the controller is read back and a CHANGE is written into the run
        # record (`tool_frame_check`) and said here. Until 2026-09-15 a change
        # was `refused_tool_frame`; that stopped the day's first right taught
        # flow on a pendant holding `Arm_Tip` instead of `R_glove_2`. What a
        # changed frame reaches is still worth the operator's eye — the
        # Cartesian caps `limits.guard` raises (evaluated at the TCP -> a
        # speed-overrun latch), and the payload that gravity compensation and
        # the controller's COLLISION STAGE model (F30/F31) — which is why the
        # reasons are printed in full rather than hidden. The replay itself is
        # a joint-space track: the TCP cannot move the arm off its path.
        tfc = tool_frame_check(snap_before.get("tool_frame"), t)
        out_tf = tfc
        print(f"   tool frame: {tool_frame_line(tfc['found'])}")
        if not tfc["ok"]:
            for why in tfc["reasons"]:
                print(f"   tool frame (recorded, not a refusal): {why}")
            print("   tool frame: the run goes on (Newton, 2026-09-15). A changed frame reaches "
                  "the Cartesian caps, gravity compensation and the collision stage's payload "
                  "model (F30/F31) — written into this run's record as tool_frame_check.")
        # Register the program as a consumer BEFORE checking: the whole point
        # is to compare it against what the pole already did.
        bp.for_consumer("program")
        bpc = bp.check()
        if out is None and not bpc["ok"]:
            print(f"   ** {bpc['why']}")
            out = {"ok": False, "verdict": "refused_blocking_probe", "gap_deg": None,
                   "duration_s": None, "implied_ms_per_pt": None,
                   "wrong_id_seen": False, "transitions": [], "run_ret": None}
        if getattr(args, "log_events", False):
            # h.handle_id, not h.robot.handle.id: the latter is a
            # ctypes.POINTER(rm_robot_handle) which does not proxy struct
            # fields, so it evaluated to None on every run and the log's
            # per-arm filter passed everything. Harmless with one arm, wrong
            # with two.
            events = tr.ArmEventLog(h.handle_id)
            if not events.start(h.robot):
                print("   ** event log: could not register the SDK event callback")
                events = None
        if out is None:
            # A DRY track is the ladder's ZERO-EXPOSURE rung, and its zero
            # exposure is the claim that neither arm leaves its rest pose.
            # `run_verified` bridges any start gap with a collision-unchecked
            # movej by default (ruling 2026-09-01), so an arm parked anywhere
            # else would transit the whole way to rest beside a live partner —
            # a real stroke's worth of motion out of a track that promises
            # none. For a dry row the gap is a REFUSAL, never a transit
            # (hardening review 2026-09-08); park the arm first, then run it.
            _dry = (t.get("kind") == "dry")
            if _dry and not args.require_at_start:
                print("   dry track: --require-at-start is FORCED on. Its safety property "
                      "is 'the arm never leaves rest', so the movej bridge that would "
                      "carry it TO rest is refused instead of taken.")
            # SELECT THE FRAME THIS ROW WAS SUPERVISED AT, AND PUT BACK WHAT THE
            # PENDANT HAD. Newton offered this branch when he ruled on 2026-09-15
            # ("we can either switch the frame implicitly during the run, or just
            # collect data on the frame set, instead of blocking the run"). The
            # payload reaches gravity compensation and the collision stage, so
            # runs are compared under the frame they were measured at. It is NOT
            # what decides whether a track runs: the 2026-09-16 glove-frame aborts
            # were a J2 fault window (4109 at both frames, J2 peaks 19-22 M), and
            # slot 43 completed at L_glove_2 on 2026-09-17. Selecting is not
            # refusing: the never-block ruling stands and this guard refuses nothing.
            # `--tool-frame NAME` replaces the row's frame for THIS run only — the
            # operator's choice, selected and proved the same way, and named in the
            # record so a run at another frame is never read as one at the row's.
            tf_row = tool_frame_wanted(t)
            tf_want = getattr(args, "tool_frame", None) or tf_row
            if tf_want != tf_row:
                print(f"   tool frame: --tool-frame {tf_want!r} replaces this row's "
                      f"{tf_row!r} for this run — the row's runs, and any calibration "
                      f"in its certificate, were measured at {tf_row!r}, not at {tf_want!r}")
            elif tf_want is None:
                print("   tool frame: the manifest records none for this row — "
                      "nothing selected, the run uses what the controller holds")
            canfd_opts = canfd_watch = None
            if is_canfd:
                canfd_opts = canfd_options(args)
                canfd_watch = tr.CanfdWatch(rows_all_pre, canfd_opts["period_s"])
                print(f"   channel: CANFD — {len(rows_all_pre)} lines streamed from this host at "
                      f"{canfd_opts['period_s'] * 1e3:.4f} ms/line, follow "
                      f"{'high' if canfd_opts['follow'] else 'low'}, trajectory_mode "
                      f"{canfd_opts['trajectory_mode']}, rate gate "
                      f"{canfd_opts['rate_gate']}; the controller's slot is not used")
                if bp.enabled:
                    print("   blocking probe: not used on a CANFD stream (no program to block on)")
                if rec_joints is not None:
                    rec_joints.attach_monitor(canfd_watch)
                    # THE STOP CHECK MUST BE SEEING THE ARM BEFORE THE FIRST LINE GOES OUT. The push
                    # accepts a wrong target silently and delivers nothing, and `UdpRecorder.start`
                    # cannot tell; without this the whole file streams unwatched (2026-09-17 review).
                    _t0 = time.monotonic()
                    while canfd_watch.n_frames == 0 and time.monotonic() - _t0 < 1.0:
                        time.sleep(0.01)
                    if canfd_watch.n_frames == 0:
                        print("   ** the push has delivered no frame in 1.0 s — the CANFD stop check "
                              "would be blind. Refusing; fix the push (`push --show`).")
                        out = {"ok": False, "verdict": "refused_capture", "gap_deg": None,
                               "duration_s": None, "implied_ms_per_pt": None, "channel": "canfd",
                               "wrong_id_seen": False, "transitions": [], "run_ret": None,
                               "capture_refusal": "no push frame arrived before the stream"}
                    elif canfd_watch.stop_codes:
                        # a stop-class code is LIVE on the arm right now: streaming lines into an arm
                        # that is not moving is how a 10 deg step jump happens when it clears
                        print(f"   ** the push carries {sorted(canfd_watch.stop_codes)} before the "
                              f"stream starts — the arm is in a stop state. Refusing.")
                        out = {"ok": False, "verdict": "refused_canfd_state", "gap_deg": None,
                               "duration_s": None, "implied_ms_per_pt": None, "channel": "canfd",
                               "wrong_id_seen": False, "transitions": [], "run_ret": None,
                               "cut_reason": f"stop-class code(s) {sorted(canfd_watch.stop_codes)} "
                                             f"in the push before the first line"}
            try:
                if out is not None:
                    raise _CanfdPreRunRefusal
                with tr.tool_frame_guard(h.robot, tf_want, log=print,
                                         verdict=tf_guard_rec) as _tfg:
                    out = tr.run_verified(
                        h.robot, save_id=t["slot"], band_s=(lo, hi), first_row_millideg=first_row,
                        points=t["points"], speed=args.speed, restore_pct=args.restore_speed,
                        start_tolerance_deg=args.start_tolerance_deg,
                        require_at_start=bool(args.require_at_start or _dry),
                        blocking_probe=(False if is_canfd else bp.enabled),
                        movej_speed=getattr(args, "movej_speed", None),
                        caps=caps_for(args),
                        on_transition=lambda x: print(f"   t={x['t']:6.2f}s "
                                                      f"run_state={x.get('run_state')} "
                                                      f"id={x.get('id')}"),
                        channel=channel, track_rows_millideg=rows_all_pre,
                        canfd=canfd_opts, canfd_watch=canfd_watch)
            except _CanfdPreRunRefusal:
                pass                              # `out` already carries the refusal and its reason
            finally:
                if canfd_watch is not None and rec_joints is not None:
                    rec_joints.attach_monitor(None)
    finally:
        try:
            snap_after = tr.controller_snapshot(h.robot, limits_cache)
            print("   after:  " + tr.snapshot_line(snap_after))
        except Exception as exc:                                       # noqa: BLE001
            snap_after = f"unavailable: {exc!r}"
        # THE SETTLE AND THE PARK ARE NOT THE CAPTURE'S. Both of these used to
        # sit inside `if rec_joints is not None:`, so `--no-record-joints` — a
        # flag that promises only "run UNLOGGED" — ALSO skipped the wait for
        # the arm to stop and the park at the plan's own `move_to_rest`. The
        # arm was left wherever the stroke ended, `state["parked"]` stayed
        # False so the NEXT task took the retreat branch from a pose it was
        # never at, and the record carried neither `settle` nor `post_run`, so
        # nothing said the park had not happened. Newton's 2026-09-04 ruling is
        # "end every replay at rest" and it is about the ARM, not the evidence.
        # (Found 2026-09-09 driving the flow verb against the emulator; the
        # proven 12/12 flow captures, so this path never ran on hardware.)
        try:
            # The capture must OUTLIVE the motion. run_state 0 is not "stopped":
            # on 2026-09-02 six runs were still moving at 13-96 deg/s when a flat
            # 0.6 s sleep let the recorder stop, which truncated the evidence and
            # left those runs unjudgeable (F112). Wait for the arm itself.
            settle_out = settle = tr.wait_until_at_rest(h.robot)
        except Exception as exc:                                       # noqa: BLE001
            # This runs in a `finally`: a raise here would REPLACE whatever
            # brought us in and the run would report the settle's failure
            # instead of its own.
            settle_out = {"error": repr(exc)}
            settle = {"was_moving": False, "at_rest": None, "waited_s": 0.0,
                      "peak_deg_s": 0.0}
            print(f"   ** the settle wait raised {exc!r} — arm state after the "
                  f"replay is UNKNOWN")
        if settle["was_moving"]:
            print(f"   ** the arm was STILL MOVING when the program reported its end "
                  f"(settled after {settle['waited_s']:.2f} s, peak "
                  f"{settle['peak_deg_s']:.0f} deg/s) — the program's duration is a "
                  f"lower bound, not the motion time")
        if not settle["at_rest"]:
            print(f"   ** the arm did not come to rest within the settle timeout "
                  f"({settle['waited_s']:.1f} s) — capture stopped anyway")
        # The park move runs UNDER the capture, after the settle, so the
        # frames carry it (Newton, 2026-09-04: end every replay at rest).
        # STAMP IT. The replay window's tail (+0.6 s plus the settle wait)
        # otherwise runs into this move, and then `phase_end` — the
        # assumption-free cross-check on the ordered matcher — is measured
        # while the arm is 3-48 deg off the path, on its way to rest. Seven
        # runs the ledger calls COMPLETE carry a `phase_end` computed that
        # way. The verdict never used it (replay_verdict reads only the
        # ordered pair), so nothing was mislabelled, but the check that
        # would CATCH a regression in the ordered pair was silently gone.
        # A timestamp is exact; the still-stretch heuristic is not — inside
        # a replay the arm holds still for up to 0.226 s at a reversal,
        # against a 0.431 s gap before the park.
        if after_run is not None and out is not None:
            park_t_mono = time.monotonic()
            out["park_t_mono"] = park_t_mono
            try:
                import inspect
                if "verdict" in inspect.signature(after_run).parameters:
                    post = after_run(verdict=out.get("verdict"))
                else:
                    post = after_run()
            except Exception as exc:                               # noqa: BLE001
                post = {"ok": False, "error": repr(exc)}
                print(f"   ** park move raised {exc!r}")
        if rec_joints is not None:
            rec_joints.stop(h.robot)
    out["controller_before"] = snap_before
    out["controller_after"] = snap_after
    out["tool_frame_check"] = out_tf
    if tf_guard_rec:
        tf_guard_rec["wanted_from"] = ("--tool-frame" if getattr(args, "tool_frame", None)
                                       else "manifest")
    # what the guard SELECTED and whether it put the pendant's choice back
    out["tool_frame_guard"] = tf_guard_rec
    out["blocking_probe_policy"] = bp.check()
    out["controller_slot_check"] = slot_chk
    out["provenance_check"] = dict(ok=prov["ok"], reasons=prov["reasons"],
                                   warnings=prov["warnings"],
                                   track_sha256=prov["disk"]["track_sha256"],
                                   proj_sha256=prov["disk"]["proj_sha256"],
                                   points_on_disk=prov["disk"]["points_on_disk"],
                                   ignored=bool(getattr(args, "ignore_provenance", False)))
    if events is not None:
        evt_out = events.stop()
        fin = "finish event SEEN" if evt_out["finish_seen"] else "** NO finish event"
        print(f"   events: {evt_out['n_events']} ({evt_out['arrivals']} arrivals, "
              f"{len(evt_out['failed_arrivals'])} failed) — {fin}")
    out["arm_events"] = evt_out
    out["settle"] = settle_out
    if post is not None:
        out["post_run"] = post
    if dev_check is not None:
        out["devices_as_found"] = dev_check
    if pre is not None:
        out["pre_run"] = pre
    # A REFUSED run never started, so its capture is the arm standing still and
    # the coverage measured from it is meaningless. Scoring it anyway marked a
    # device-guard refusal as `complete: False` and it then read as a CUT in the
    # ledger (caught 2026-09-04 on run_slot72_211316, "cut at 5.3 %").
    if (rec_joints is not None and rec_joints.frames
            and not str(out["verdict"]).startswith("refused")):
        # What the run must PROVE (F108): completeness + path fidelity, measured
        # from the capture. Duration was only ever a proxy and the player's own
        # rate varies +-6% run to run, so it is reported, not gated.
        _t = time.perf_counter()
        rows_all = tr.parse_track(REPO / t["track"])
        timing["parse_track_s"] = round(time.perf_counter() - _t, 4)
        _t = time.perf_counter()
        # ONE parse of the capture, shared by both consumers below. It was being
        # JSON-parsed twice per task, on the critical path between tasks.
        cap = tr.read_capture(rec_joints.out_path)
        # WINDOW IT: without t_start the movej transit counts as "off path" and
        # the deviation stats are meaningless (p99 49 deg on a clean run).
        # The controller's own phase label is exact; positions are the fallback.
        timing["read_capture_s"] = round(time.perf_counter() - _t, 4)
        _t = time.perf_counter()
        cs = out.get("canfd") or {}
        if is_canfd and cs.get("t_first_send") is not None:
            # THE HOST SENT EVERY LINE, on the capture's own clock: the span is
            # exact from the stream, not inferred from a status label this
            # channel has never been seen to report.
            with open(rec_joints.out_path) as _fh:
                _t_cap0 = json.loads(_fh.readline())["t_mono"]
            span0 = {"movej_s": cs["t_first_send"] - _t_cap0, "dwell_s": 0.0,
                     "replay_s": cs.get("replay_s") or 0.0}
        else:
            span0 = (tr.replay_span_from_status(rec_joints.out_path, capture=cap)
                     or tr.replay_span_from_frames(rec_joints.out_path, rows_all[0], rows_all[-1]))
        timing["replay_span_s"] = round(time.perf_counter() - _t, 4)
        t_first = None
        if span0 is not None:
            with open(rec_joints.out_path) as _fh:
                t_first = json.loads(_fh.readline())["t_mono"]
        t_end = None
        if span0:
            t_end = (t_first + span0["movej_s"] + span0["dwell_s"] + span0["replay_s"]
                     + 0.6 + (settle_out or {}).get("waited_s", 0.0))
        if park_t_mono is not None:
            # never past the moment WE started moving the arm again
            t_end = park_t_mono if t_end is None else min(t_end, park_t_mono)
        _t2 = time.perf_counter()
        vr = tr.verify_replay_from_frames(
            rec_joints.out_path, rows_all,
            t_start=(t_first + span0["movej_s"] + span0["dwell_s"]) if span0 else None,
            t_end=t_end, capture=cap,
            clock_s=(cs.get("period_s") or tr.REPLAY_CLOCK_S) if is_canfd else tr.REPLAY_CLOCK_S)
        timing["verify_s"] = round(time.perf_counter() - _t2, 4)
        if vr is not None:
            out["replay_check"] = vr
            # A capture that stopped while the arm was still moving cannot show
            # where the run ended; scoring it as a short run invents a cut that
            # was never observed (F112). The rules live in track.replay_verdict
            # so they can be tested without hardware.
            # THE SECOND WITNESS, supplied here because only the run path knows how the channel
            # measured its own progress: a stream counted the lines it sent, an upload is judged
            # against the fitted player law. Without it a full-length replay whose ordered coverage
            # mis-locked on a self-similar single-joint path records a cut that never happened.
            # `span0` is None when the departure never latched, and then there is no replay LENGTH
            # to compare — only a whole-program duration with the movej transit inside it. The second
            # witness can only withhold a cut, so an inflated length is the one way it could launder
            # a real one; withholding the witness leaves the verdict on coverage, as before.
            played = tr.played_fraction(
                t["points"],
                replay_s=(None if is_canfd else (span0 or {}).get("replay_s")),
                lines_sent=(cs.get("lines_sent") if is_canfd else None))
            vd = tr.replay_verdict(vr, settle_out, played_fraction=played)
            out["played_fraction"] = vd["played_fraction"]
            truncated, tag = vd["capture_truncated"], vd["tag"]
            print(f"replay check: covered {vr['phase_first']*100:.1f}%->"
                  f"{vr['phase_last']*100:.1f}% of the track ({tag}) | "
                  f"ended nearest point {vr['phase_end']*100:.1f}% "
                  f"({vr['phase_end_dev_deg']:.2f}° off, next match "
                  f"{vr['phase_end_margin_deg']:.1f}° away) | "
                  f"path deviation p50 {vr['dev_p50_deg']:.2f}° p99 {vr['dev_p99_deg']:.2f}° "
                  f"max {vr['dev_max_deg']:.2f}° | push cadence "
                  f"{vr['cadence_ms_median']:.3f} ms")
            out["complete"] = vd["complete"]
            out["capture_truncated"] = truncated
            if vd["outcome"] != "complete":
                out["ok"] = False
        # ERROR CODES SEEN IN THE PUSH DURING THE RUN (2026-09-15). The before
        # and after snapshots were both clean on right slot 42 while the frames
        # carried 4105 ("Speed out of limit") on 1,139 of them, 2.3-8.0 s into
        # the replay: the controller flags, keeps playing and clears it, and
        # the record said nothing. Counted from the capture itself; recorded
        # and printed, never a verdict (the run finished).
        try:
            codes = tr.push_error_codes(rec_joints.out_path)
        except Exception as exc:                                       # noqa: BLE001
            codes = {"unreadable": repr(exc)}
        out["codes_in_push"] = codes
        if codes:
            print("   ** codes in the push during this run: " + ", ".join(
                f"{c} on {v['frames']} frames ({v['first_s']:.2f}-{v['last_s']:.2f} s)"
                if isinstance(v, dict) else f"{c}: {v}" for c, v in codes.items()))
        # 5 ms precision beats the 0.1 s online polling: recompute the
        # movej/dwell/replay split from the frames and re-verdict on it.
        # REUSE the span computed above — it is the same call on the same file,
        # and it and parse_track were both being done twice per task on the
        # critical path between tasks.
        span = span0 if span0 is not None else (
            tr.replay_span_from_status(rec_joints.out_path, capture=cap)
            or tr.replay_span_from_frames(rec_joints.out_path, first_row, rows_all[-1]))
        if span is not None:
            out.update(span)
            out["implied_ms_per_pt"] = span["replay_s"] / max(t["points"] - 1, 1) * 1e3
            ms = out["implied_ms_per_pt"]
            env_lo, env_hi = tr.CLOCK_ENVELOPE_MS
            # THE CLOCK IS JUDGED ON THE POINTS THAT WERE PLAYED (2026-09-10).
            # `implied_ms_per_pt` uses the COMMANDED count, so on an incomplete
            # run it is the clock times the completion fraction and not a clock
            # — which is how `in_clock_envelope` came to be an accidental
            # completeness detector. Both fields are kept: the old one for
            # ledger continuity, the new one because it is the actual clock.
            phase_last = (out.get("replay_check") or {}).get("phase_last")
            out["points_played"] = tr.played_points(t["points"], phase_last)
            out["clock_ms_per_pt"] = tr.clock_ms_per_pt(
                span["replay_s"], t["points"], phase_last)
            # THE SCREEN REMOVES THE PLAYER'S FIXED RAMP FIRST (2026-09-17). The flat band is biased by
            # track length — `replay_s/(N-1)` reads 4.3 % higher under 1200 lines, and every one of the
            # 8 rejects in 158 complete runs is a short track. The note below used to explain that bias
            # and then keep the biased number in the verdict. `clock_screen` judges the de-ramped rate;
            # `in_clock_envelope` is still recorded, unchanged, because 190 records carry it.
            out["clock_screen"] = tr.clock_screen(
                span["replay_s"], t["points"], phase_last,
                channel=("canfd" if is_canfd else "upload"),
                period_s=(cs.get("period_s") if is_canfd else None))
            in_env = out["clock_screen"]["ok"]
            in_band = lo <= span["replay_s"] <= hi
            out["in_advisory_band"] = in_band
            out["in_clock_envelope"] = tr.in_clock_envelope(out["clock_ms_per_pt"])
            # AND THE LABEL ONLY MOVES ON A COMPLETE RUN. Relabelling a CUT
            # `in_band` because its played-points clock is normal would read as
            # a pass; on an incomplete run the duration label is left exactly
            # as `run_verified` set it, and completeness is what `ok` carries.
            # AND `None` IS NOT `False` (2026-09-18). `clock_screen` answers None for a CANFD stream
            # — the player's envelope does not apply to a host-timed clock — and this line read that
            # as a failure, so every one of Newton's 42 CANFD sweep runs recorded `out_of_band` while
            # the streams had all completed cleanly with 0 send errors and 1.22 ms worst lateness. The
            # label only moves when the screen actually judged something; "not applicable" leaves the
            # duration label `run_verified` set. Same bug class as `finish_event` and the probe
            # stand-in, both of which this file already warns about.
            # AND `is not False` IS NOT `is True` (2026-09-18, second pass). The comment above says
            # "only on a COMPLETE run" and this line said `is not False`, which was the same thing
            # while `None` only ever meant "not measured". It no longer does: `None` is now a real
            # outcome (the capture was truncated, the track is unresolvable, or the two instruments
            # disagree), and on such a run `implied_ms_per_pt` is the clock times the completion
            # fraction rather than a clock. Letting those through relabelled 2 records `in_band` on
            # sub-ceiling "clocks" of 4.62 and 2.88 ms/pt — below the player's 5.000 ms floor, so not
            # clocks at all. Every one of the 20 sub-ceiling readings in the ledger belongs to a run
            # whose `complete` is False or None; not one is True.
            if (in_env is not None and out.get("complete") is True
                    and out["verdict"] in ("in_band", "out_of_band")):
                out["verdict"] = "in_band" if in_env else "out_of_band"
            # COMPLETENESS is what a run must prove, not duration — the block
            # above says so and then gated on the clock anyway. That stopped the
            # 2026-09-04 18:47 flow at seat_body_hinge_area: 100 % coverage,
            # 4.3 deg deviation, 5.004 ms push cadence, and an implied clock of
            # 6.288 ms/pt against a 6.2 ceiling. Short tracks read high because
            # the fixed ramp-in and ramp-out are a large share of a 4 s replay,
            # so the envelope is a poor gate for them. The clock stays in the
            # verdict and the ledger; it no longer fails a complete run.
            out["ok"] = bool(out.get("complete", True))
            # A STOPPED STREAM IS NEVER OK (2026-09-17 review). `complete` comes from the capture's
            # coverage, and a stream the stop check ended inside the still tail still covers >= 99 %
            # of the file: without this, `ok` contradicted `cut_reason` and the flow carried on.
            if is_canfd and (cs.get("aborted") or not cs.get("completed")):
                out["ok"] = False
                out["cut_reason"] = out.get("cut_reason") or f"the stream stopped: {cs.get('aborted')}"
            # AND `cut_reason` MUST NOT SURVIVE AN `ok` IT NO LONGER DESCRIBES (2026-09-18). `ok` is
            # re-derived from completeness two lines up, but the reason string `run_verified` wrote from
            # its own components stayed — so 8 of Newton's 14 sweep runs recorded
            # `cut_reason: "failed: not_early, probe"` beside `complete: true, ok: true`, which reads as
            # a cut to every later reader of the ledger. The component detail is not lost: it stays in
            # `ok_components`, and the duration note moves to a field that says what it is.
            if out["ok"] and out.get("cut_reason"):
                out["duration_note"] = out.pop("cut_reason")
                out["cut_reason"] = None
            # PRINT THE NUMBER THE VERDICT IS ABOUT. These lines printed
            # `implied_ms_per_pt` beside a verdict now computed on the
            # played-points clock — one number, a judgement about another.
            clk = out["clock_ms_per_pt"]
            played = out.get("points_played")
            over = "" if played in (None, t["points"]) else (
                f" on the {played} of {t['points']} points it played")
            if in_env is False:
                print(f"   (clock {clk:.3f} ms/pt is outside {tr.CLOCK_ENVELOPE_MS}"
                      f"{over} — reported, not gated: the run covered "
                      f"{(out.get('replay_check') or {}).get('phase_last', 0) * 100:.1f}% of "
                      f"its file)")
            print(f"frames split: movej {span['movej_s']:.2f} + dwell {span['dwell_s']:.2f} + "
                  f"replay {span['replay_s']:.3f} s | clock {clk:.4f} ms/pt{over} "
                  f"{'within' if in_env else '** OUTSIDE' if in_env is False else '?'} the "
                  f"hardware envelope [{env_lo}, {env_hi}]"
                  + (f" | {ms:.4f} ms/pt on the commanded count" if over else "")
                  + ("" if in_band else "  (outside the tight advisory band — expected, "
                                        "the player varies ±6%)"))
    # THE TCP EXECUTION GATE (Newton, 2026-09-22): a row with a budget (`tcp_gate.max_mm`, the
    # description's MAX DEV) has its EXECUTED pad path, from this run's own push capture, measured
    # against the RAW recording, both ways — PASS or FAIL in the record and on the console. A FAIL
    # is a run whose executed pad path missed its accuracy — a completed stream judged both ways, or
    # a stopped one judged on its executed part alone (tcp_gate.gate_run): rc 1, so a flow stops,
    # and a rerun of tools/taught_pipeline.py on the folder steps the slot down to a slower rung.
    gate_failed = False
    if (t.get("tcp_gate") and rec_joints is not None and (out.get("canfd") or {}).get("t_first_send")
            and not str(out.get("verdict") or "").startswith("refused")):
        try:
            if str(REPO / "tools") not in sys.path:
                sys.path.insert(0, str(REPO / "tools"))
            import tcp_gate as _tg
            g = _tg.gate_run(t, out, pathlib.Path(rec_joints.out_path))
        except Exception as exc:                                       # noqa: BLE001
            g = dict(max_mm=t["tcp_gate"].get("max_mm"), verdict="NOT MEASURED", why=repr(exc))
        if g is not None and t["tcp_gate"].get("acc_demand") is not None:
            # THE ACCELERATION DEMAND, logged with every run of the row (MAX DEV `+acc`, Newton 2026-09-23)
            g["beyond_ceiling"] = bool(t["tcp_gate"].get("beyond_ceiling"))
            g["acc_demand"] = t["tcp_gate"]["acc_demand"]
        out["tcp_gate"] = g
        if g and g.get("verdict") == "PASS":
            print(f"   TCP GATE PASS: the pad stayed within {g['measured_mm']:.2f} mm of the raw "
                  f"recording (MAX DEV {g['max_mm']:g} mm; executed->raw {g['exe_to_raw_max_mm']:.2f}, "
                  f"raw->executed {g['raw_to_exe_max_mm']:.2f})")
        elif g and g.get("verdict") == "FAIL":
            gate_failed = True
            print(f"** TCP GATE FAIL: the pad departed {g['measured_mm']:.2f} mm from the raw recording "
                  f"(MAX DEV {g['max_mm']:g} mm; executed->raw {g['exe_to_raw_max_mm']:.2f}, "
                  f"raw->executed {g['raw_to_exe_max_mm']:.2f}) — rerun tools/taught_pipeline.py on "
                  f"the folder: it steps this slot to a slower rung")
        elif g:
            print(f"   TCP gate NOT MEASURED: {g.get('why')}")
    # Run record: every hardware run leaves a JSON in the committed ledger
    # (Resource/runs, the C9 convention) — transitions, duration, verdict, the
    # implied clock — so bands/ratios can be judged across sessions.
    rec = rec_dir / f"{rec_prefix}{t['slot']}_{stamp}.json"
    out["log_prefix"] = rec_prefix
    _t = time.perf_counter()
    # `sha256_of`, not a bare read: a run REFUSED because its track file is
    # missing must still leave a record saying so. Hashing it unconditionally
    # raised FileNotFoundError here, after the refusal, and threw the verdict
    # away with it (hardening review 2026-09-08).
    track_sha = sha256_of(REPO / t["track"])
    timing["track_sha_s"] = round(time.perf_counter() - _t, 4)
    _t_write = time.perf_counter()
    json.dump({
        "when": datetime.datetime.now().isoformat(timespec="seconds"),
        "host": _socket.gethostname(), "sdk": sdk, "arm_ip": h.ip, "side": h.side,
        # The SDK reply timeout this process ran with. The vendor default of
        # 500 ms is what turns a busy controller into a -2 "refusal" (F122),
        # so a record that does not say which timeout it used cannot be
        # compared against one that does.
        "ack_timeout_ms": _connect.ACK_TIMEOUT_MS,
        "host_timing_s": timing,
        "slot": t["slot"], "name": t["name"], "acc_scale": t.get("acc_scale"),
        "track": t["track"], "points": t["points"], "band_5p5_5p6_s": [lo, hi],
        "track_sha256": track_sha,
        "channel": channel,
        "speed": args.speed, "restore_speed": args.restore_speed,
        "start_tolerance_deg": args.start_tolerance_deg,
        # what was ENFORCED, not what was asked: a dry row forces it on.
        "require_at_start": bool(args.require_at_start or t.get("kind") == "dry"),
        "require_at_start_requested": bool(args.require_at_start),
        "frames_file": (repo_rel(rec_joints.out_path) if rec_joints else None),
        "frames": (rec_joints.frames if rec_joints else None),
        "steal_push": bool(args.steal_push),
        # RAN PAST A MISSING OR FAILED CERTIFICATE (`taught_tracks.py run/flow --force`).
        # taught_tracks has always said a forced run is "recorded in the run ledger", and until
        # 2026-09-21 nothing was: a forced run's record was indistinguishable from a certified
        # one. `force` only exists on taught_tracks' parsers, so False everywhere else.
        "certificate_forced": bool(getattr(args, "force", False)),
        # Who owned the realtime push before we took it. --steal-push was needed
        # on every run of 2026-09-03 and nothing recorded WHY, so a stale enable
        # left by a crashed session is indistinguishable from a live consumer.
        "push_prev": _jsonable(getattr(rec_joints, "_prev", None)) if rec_joints else None,
        **out,
    }, open(rec, "w"), indent=1)
    timing["record_write_s"] = round(time.perf_counter() - _t_write, 4)
    if timing:
        print("   host timing: " + "  ".join(f"{k[:-2]} {v:.2f}s" for k, v in timing.items()
                                             if isinstance(v, float)))
    print(f"run record -> {repo_rel(rec)}")
    if out["verdict"] in ("refused_no_state", "refused_not_at_start", "refused_pre_run",
                          "refused_devices", "refused_provenance", "refused_tool_frame",
                          "refused_blocking_probe", "refused_capture", "did_not_start",
                          "refused_canfd_limits", "refused_canfd_state", "refused_movej",
                          "no_end", "wrong_id"):
        return 4, out
    return (0 if (out["ok"] or args.accept_out_of_band) and not gate_failed else 1), out


class _Tee:
    """Mirror everything printed during a run/flow into a .log beside its
    record — logging is the DEFAULT (Newton, 2026-09-02); the console is not
    a record."""

    def __init__(self, path: pathlib.Path):
        self.path = pathlib.Path(path)
        self._fh = None
        self._stdout = None

    def __enter__(self):
        self._fh = open(self.path, "a")
        self._stdout = sys.stdout
        sys.stdout = self
        self._fh.write(f"# {datetime.datetime.now().isoformat(timespec='seconds')} "
                       f"{' '.join(sys.argv)}\n")
        return self

    def write(self, text):
        self._stdout.write(text)
        self._fh.write(text)

    def flush(self):
        self._stdout.flush()
        self._fh.flush()

    def isatty(self):
        return False

    def __exit__(self, *exc):
        sys.stdout = self._stdout
        self._fh.close()
        return False


def ledger_dir(sdk: str) -> pathlib.Path:
    # The ledger is the HARDWARE record (C9 convention). An emulator run —
    # routine since S3 faked the program channel — is stamped by the handle's
    # provenance and filed under emulated/ so it can never pose as a real one.
    d = (REPO / "Resource" / "runs" / "tracks" / ("emulated" if sdk == "emulated" else ".")
         / datetime.date.today().isoformat()).resolve()
    d.mkdir(parents=True, exist_ok=True)
    return d


def cmd_run(args) -> int:
    from alix_dispatch import connect
    side = args.side
    t = by_name(manifest_for(args), args.track, side)
    lo, hi = t["band_5p5_5p6_s"]
    refuse_canfd_only_on_upload([t], getattr(args, "channel", None))
    rest_want = taught_rest_setting(t)
    legs = " and ".join(w for w in ("before", "after") if rest_want.get(w))
    if legs:
        print(f"   NOTE: {t['name']} takes a move to rest {legs} its stroke in `flow`; `run` does "
              f"NOT perform it — use `flow` for that")
    confirm(f"RUN slot {t['slot']} ({t['name']}, {t['points']} pts, band [{lo}, {hi}] s, "
            f"speed {args.speed} restored to {args.restore_speed})", side, args.ip or "default",
            assume_yes=args.yes)
    # S2 (F109): one command source per arm. If a driver serves this side,
    # take custody through ITS choreography; only run bare when none is up.
    custody = None
    if driver_present(side):
        if args.record_joints is True and not args.steal_push:
            raise SystemExit(
                "--record-joints: the driver owns the UDP push while it is up. "
                "Run without it (the driver's own capture covers state), or "
                "--steal-push if you accept starving it.")
        if args.record_joints is None and not args.steal_push:
            print("driver detected — it owns the UDP push: frames capture OFF for this run "
                  "(its own capture covers state); --steal-push to take it anyway")
            args.record_joints = False
        import arm_custody
        custody = arm_custody.ArmCustody(side)
        print(f"driver detected on the {side} side — taking custody "
              f"(deactivate JTC + hold_release) and verifying the handback")
        if not custody.enter(print):
            raise SystemExit("custody enter FAILED — arm left held; not running")
    h = open_or_die(side, args.ip)
    sdk = connect.sdk_provenance(h.robot)
    rec_dir = ledger_dir(sdk)
    stamp = datetime.datetime.now().strftime("%H%M%S")
    log_path = rec_dir / (f"{'canfd_slot' if (getattr(args, 'channel', None) == 'canfd') else 'run_slot'}"
                          f"{t['slot']}_{stamp}.log")
    try:
        with _Tee(log_path):
            rc, _out = run_one(h, t, args, rec_dir, sdk, stamp=stamp)
    finally:
        connect.close_arm(h)
        if custody is not None:
            if not custody.exit(print):
                print("** CUSTODY HANDBACK FAILED VERIFICATION — do not stream "
                      "goals to this arm until ~/status is inspected")
        # repo_rel, not relative_to: this is a FINALLY. A ledger directory
        # outside the repo (a redirected run, a test) made the reporting line
        # raise ValueError out of the finally and take a completed run's rc
        # with it (hardening review 2026-09-08).
        print(f"console log -> {repo_rel(log_path)}")
    return rc


def cmd_flow(args) -> int:
    """The whole side, task after task (module docstring, THE FLOW)."""
    from alix_dispatch import connect
    man = manifest_for(args)
    side = args.side
    rows = rows_for(man, side)
    if args.track:
        # THE ORDER GIVEN, REPEATS INCLUDED (Newton, 2026-09-22: one command for the operator's flow
        # sheet, which runs 109 twice and not in slot order). This used to keep slot order and drop
        # a repeat, so `flow 101 102 108 109 110 103 109 ...` silently ran something else.
        by_slot = {t["slot"]: t for t in rows}
        rows = [by_slot[by_name(man, tok, side)["slot"]] for tok in args.track]
    if args.start:
        first = by_name(man, args.start, side)["slot"]
        if args.track:
            slots = [t["slot"] for t in rows]
            rows = rows[slots.index(first):] if first in slots else []
        else:
            rows = [t for t in rows if t["slot"] >= first]
    if not args.track and (getattr(args, "channel", None) or "upload") != "canfd":
        rows = drop_canfd_only(rows, "flow on the upload channel")
    if not rows:
        raise SystemExit(f"nothing to run for the {side} arm")
    refuse_canfd_only_on_upload(rows, getattr(args, "channel", None))
    if not args.no_devices:
        lacking = [t["name"] for t in rows if not t.get("pre_stroke")]
        if lacking:
            raise SystemExit(f"rows without pre_stroke (emitted before 2026-09-02): {lacking} "
                             f"— re-emit, or --no-devices")
    plan_s = sum(t["track_s"] for t in rows)
    # THE REST LEGS ARE RESOLVED AND SHOWN BEFORE ANYTHING MOVES (review 2026-09-21): a pose that
    # cannot be read is a clean refusal here, not a traceback mid-flow, and the operator sees how
    # far each uncollision-checked leg swings, and at what speed, before typing the confirmation.
    rest_note = ""
    if args.rest:
        for t in rows:
            want = taught_rest_setting(t)
            if not (want.get("before") or want.get("after")):
                continue
            try:
                legs = taught_rest_legs(t)
            except Exception as exc:                                   # noqa: BLE001
                raise SystemExit(f"{t['name']} asks for a move to rest but the {side} rest pose "
                                 f"cannot be read ({exc}) — nothing has moved")
            for when, lg in legs.items():
                if when == "before" and want.get("at") == "start":
                    # the move before the stroke goes from WHEREVER the arm is to the drag start
                    # pose — not knowable before the arm is opened, so say so rather than print 0
                    print(f"   rest before: {t['name']} the arm's pose at run time -> the stroke's "
                          f"start (the drag start pose), joint-space movej, NOT collision-checked, "
                          f"speed {taught_rest_speed(args)}: measured at run time, skipped within "
                          f"{REST_AT_TOL_DEG:g} deg (from the side's move_to_rest it is ~30 deg, J6)")
                    continue
                print(f"   rest {when}: {t['name']} {lg['leg']}, joint-space movej, NOT collision-"
                      f"checked, speed {taught_rest_speed(args)}: excursion "
                      + " ".join(f"J{j + 1} {x:+.1f}" for j, x in enumerate(lg["excursion_deg"])
                                 if abs(x) >= 1.0))
            rest_note = f", rest legs at speed {taught_rest_speed(args)} (listed above)"
    # THE ACCELERATION DEMAND (Newton, 2026-09-23): a slot emitted past the controller's joint
    # acceleration ceiling (MAX DEV `+acc`) says so BEFORE the operator confirms — the arm will not
    # follow it exactly
    for t in rows:
        g = t.get("tcp_gate") or {}
        if g.get("beyond_ceiling"):
            print(f"   ** slot {t['slot']} {t['name']}: {g.get('warning') or 'demands acceleration beyond the controller ceiling'}")
    confirm(f"FLOW {len(rows)} task(s) on slots {[t['slot'] for t in rows]} "
            f"{'in the order given' if args.track else 'in schedule order'}, "
            f"~{plan_s:.0f} s of replay, speed {args.speed} restored to {args.restore_speed}, "
            f"{'devices from pre_stroke at MAX speed' if not args.no_devices else 'NO device commands'}, "
            f"{'retreat via the plan' if args.retreat else 'NO retreat'}{rest_note}, "
            f"{'keypress per task' if not args.no_pause else 'CONTINUOUS'}"
            f"{'' if args.repeat == 1 else ', repeat ' + (str(args.repeat) if args.repeat else 'until q')}",
            side, args.ip or "default", assume_yes=args.yes)
    if driver_present(side):
        raise SystemExit("flow is an OFF-DRIVER tool: the alix_hw driver owns the pole and the "
                         "hand while it is up. Stop it (or use the task executives).")
    h = open_or_die(side, args.ip)
    sdk = connect.sdk_provenance(h.robot)
    rec_dir = ledger_dir(sdk)
    t_flow = datetime.datetime.now()
    log_path = rec_dir / f"flow_{side}_{t_flow.strftime('%H%M%S')}.log"
    with _Tee(log_path):
        rc_all = _flow_body(h, sdk, rec_dir, rows, args, t_flow)
    print(f"console log -> {repo_rel(log_path)}")
    return rc_all


def _flow_body(h, sdk: str, rec_dir: pathlib.Path, rows: list, args, t_flow) -> int:
    from alix_dispatch import connect, peripherals as per
    side = args.side
    pole = hand = None
    if not args.no_devices:
        pole = per.Pole(h.robot, log=print)
        hand = per.HandModbus(h.robot, side, log=print)
        if hand.open(per.HAND_SPEED_SET_MAX) != 0:
            connect.close_arm(h)
            raise SystemExit("hand modbus mode unavailable — not running the flow")
        print(f"pole now: {pole.height_m()} m")
    state = dict(pole_m=None, hand=None, parked=False)
    # One cache for the whole flow: the controller's configured limits are read
    # once and reused, instead of twice per task (F120 — the snapshot cost
    # 0.3-2.0 s a call and there are two per run).
    limits_cache: dict = {}
    summary, rc_all, rep, stop = [], 0, 0, False
    prev_holder = [None]

    def park_at_rest(t, verdict=None):
        """Park the arm at its rest pose after the replay: the PLAN's own move_to_rest
        (`peripherals.rest_pose`) for a plan row; for a taught row, which has no plan, the
        pose `taught_rest` resolves — the side's move_to_rest from the cleaning plans, the
        stroke's own first line, or a pose stored in the row — at min(--rest-speed,
        --movej-speed).

        Newton, 2026-09-04: a replay should not leave the arm wherever the
        stroke ended. The pose is never invented, and the move runs under the capture after
        the settle so the frames carry it.
        """
        # the flag is only true once THIS park has put the arm there (a raise below must not
        # leave a stale True from an earlier move — review 2026-09-21)
        state["parked"] = False
        taught = not t.get("plan")
        speed = taught_rest_speed(args) if taught else int(args.rest_speed)
        out = dict(ok=True, stage=None, deg=None, ret=None, speed=speed, global_speed=None)
        if taught and verdict in PRE_MOTION_VERDICTS:
            # the stroke never moved: either nothing did, or the prepended move already left the
            # arm at rest. Before 2026-09-21 a taught row never parked, so a refusal moved nothing;
            # it must stay that way.
            print(f"   rest: skipped — the run was refused before the stroke moved ({verdict})")
            out["skipped"] = f"refused before the stroke moved ({verdict})"
            return out
        if taught and not taught_rest_setting(t).get("after"):
            # say WHY there is no park: an explicit opt-out, or a dry/experiment row that never asked
            why = ("opted out (rest.after false; `taught_tracks.py rest --after|--around` sets it)"
                   if t.get("rest") is not None else "the row does not ask for a move to rest")
            print(f"   rest (after): skipped — {why}")
            out["skipped"] = why
            return out
        try:
            rp = rest_for(t)
        except Exception as exc:                                       # noqa: BLE001
            print(f"   ** rest: cannot read the rest pose ({exc}) — no park move")
            out.update(ok=False, error=repr(exc))
            return out
        if rp is None:
            # Not a failure: a taught row has no plan and therefore no park
            # move (it ends where the recording ends). This used to report
            # `ok: False` on every taught step, which read as a failed park.
            print("   rest: no move_to_rest stage" +
                  (" (a taught row has no plan; `taught_tracks.py rest --around` adds one)"
                   if not t.get("plan") else "") +
                  " — no park move")
            out["skipped"] = "no move_to_rest stage"
            state["parked"] = False
            return out
        out["stage"], out["deg"] = rp["stage"], rp["deg"]
        # Same trap as the transit: `v` is a percent of the persistent global
        # ratio, so the park must set it too or it inherits the last value.
        from alix_dispatch import track as _tr
        print(f"   rest: movej to {rp['stage'] if taught else 'the plan' + chr(39) + 's ' + rp['stage']} at speed "
              f"{speed} of a 100 % global ratio")
        # M48: `log=` so the line is the caller's stream rather than a bare
        # print, and `verdict=` so the park's OWN restore return reaches the
        # record. There is no getter for the global ratio, so ret is the only
        # evidence there is that the arm was handed back at --restore-speed.
        gs = {}
        with _tr.global_speed_guard(h.robot, args.restore_speed, log=print, verdict=gs):
            _tr.set_global_speed(h.robot, 100)
            ret = int(h.robot.rm_movej([float(x) for x in rp["deg"]], speed, 0, 0, 1))
        out["ret"] = ret
        out["global_speed"] = gs
        if ret != 0:
            print(f"   ** rest movej REFUSED ret={ret}")
            out["ok"] = False
        state["parked"] = out["ok"]
        return out

    def before_run(t):
        """Retreat + devices, then a taught row's move to rest — all OUTSIDE the program and
        UNDER the capture."""
        prev = prev_holder[0]
        # `order`: what actually moved, in sequence, each step recorded once it SUCCEEDED (a refused
        # movej or an unsettled hand is not a move) — pole and hand before any arm move of this task
        pre = dict(ok=True, retreat=None, rest=None, hand_open=None, pole=None, hand=None, order=[])
        # 1. retreat along the previous plan's own first post-stroke move.
        # Redundant once the previous task parked at rest (2026-09-04): the arm
        # is already clear of the fixture, so retreating to the PREVIOUS task's
        # post-stroke pose would walk it back toward the surface it just left.
        if args.rest and prev is not None and state.get("parked"):
            print("   retreat: skipped — the previous task parked at rest")
        elif args.retreat and prev is not None and prev.get("retreat_deg"):
            print(f"   retreat: movej to {prev['name']}'s {prev.get('retreat_stage')} "
                  f"pose at speed {args.speed} (the plan's own way out; rm move, "
                  f"collision-UNchecked)")
            ret = h.robot.rm_movej([float(x) for x in prev["retreat_deg"]],
                                   int(args.speed), 0, 0, 1)
            pre["retreat"] = ret
            if ret != 0:
                print(f"** retreat movej REFUSED ret={ret}")
                pre["ok"] = False
                return pre
            pre["order"].append("retreat")
        # 2./3. the devices, from what the PLAN had them at
        if pole is not None:
            ps = t["pre_stroke"]
            # WHERE THE POLE ACTUALLY IS, not where we last told it to go.
            # `state["pole_m"]` held the commanded TARGET, so a move that
            # stopped short was cached as correct for the rest of the flow; and
            # the skip band was 3 mm inside a 5 mm arrival tolerance
            # (peripherals.LIFT_TOL_M), which makes 8 mm of standoff error
            # arithmetically reachable while every log reads OK. Read the arm
            # (~6 ms) and use ONE tolerance for both decisions.
            here = None
            try:
                here = pole.height_m()
            except Exception as exc:                                   # noqa: BLE001
                print(f"   pole height unreadable ({exc!r}) — falling back to the "
                      f"last commanded target")
                here = state["pole_m"]
            state["pole_m"] = here
            pole_change = (ps.get("pole_m") is not None and
                           (here is None or abs(here - ps["pole_m"]) > per.LIFT_TOL_M))
            if ps.get("pole_m") is not None and here is not None and not pole_change:
                print(f"   pole: already at {here * 1000:.0f} mm, the plan's "
                      f"{ps['pole_m'] * 1000:.0f} mm (within {per.LIFT_TOL_M * 1000:.0f} mm)")
            if pole_change:
                # The release pose the PLAN authored before its stroke, else the
                # SRDF's own `release` group state. It used to fall back to the
                # hard-coded HAND_OPEN [1000]*6, which no plan authored and
                # which the log then called "the plans' own choreography" — on
                # 6 of 12 right tasks, 3 of them pole-change tasks. Say which
                # source is in play instead of asserting the wrong one.
                rel, src = ps.get("hand_release_counts"), "the plan's own release stage"
                if rel is None:
                    _st, rel = hand_from_frame_release(h.side)
                    src = f"the SRDF `{_st}` state (this plan authors no release)"
                print(f"   hand: OPEN before the pole moves — from {src}")
                r = hand.command(rel)
                pre["hand_open"] = r
                if not r["ok"]:
                    print("** hand did not settle open")
                    pre["ok"] = False
                    return pre
                pre["order"].append("hand_open")
                # With --blocking-probe the program start becomes blocking, so
                # the pole must consume its OWN arrival rather than leave one
                # for that call to eat (F89: "a lift arrival landing inside a
                # later blocking movej"). Otherwise the probe's -4 would be
                # ambiguous between "the trajectory was stopped" and "a stale
                # lift arrival" — the exact reading the probe exists to make.
                # F9: a lift command landing while a planned arm move is in
                # flight truncates BOTH — the arm stops and latches joint
                # errors, the pole stops silently ~25 mm in. Every task after
                # the first has just parked, so the arm is provably still; the
                # FIRST task of a flow has no retreat and no park before it and
                # moves the pole from wherever the arm was left. Measured over
                # 68 runs the arm never moved during a pole travel (max
                # 0.05 deg/s), so this has not bitten — but it is the one place
                # the hazard is reachable, and the check costs 0.15 s.
                from alix_dispatch import track as _trk
                rest = _trk.wait_until_at_rest(h.robot, timeout_s=5.0)
                if not rest["at_rest"]:
                    print(f"** the arm is still moving ({rest['peak_deg_s']:.1f} deg/s) — "
                          f"refusing to move the pole into it (F9)")
                    pre["ok"] = False
                    pre["pole_refused_arm_moving"] = rest
                    return pre
                _bp = getattr(args, "_bp", None)
                r = pole.move(ps["pole_m"], speed_pct=100,
                              block=(_bp.for_consumer("pole") if _bp is not None
                                     else getattr(args, "blocking_probe", False)))
                pre["pole"] = r
                # what the pole ACTUALLY did, not what we asked for: a blocking
                # program start next to a non-blocking lift eats the lift's
                # arrival and returns a FALSE -4 (L50/F89).
                if _bp is not None:
                    _bp.observe("pole", r.get("block"))
                    pre["blocking_probe"] = _bp.check()
                if not r["ok"]:
                    print("** pole did not reach its height")
                    pre["ok"] = False
                    return pre
                pre["order"].append("pole")
                # the height the arm reports, not the one we asked for
                state["pole_m"] = r.get("final_m", ps["pole_m"])
                state["hand"] = None
            if ps.get("hand_counts") and state["hand"] != ps["hand_counts"]:
                print(f"   hand: {ps.get('hand_state')} (the state the stroke was planned with)")
                r = hand.command(ps["hand_counts"])
                pre["hand"] = r
                if not r["ok"]:
                    print("** hand did not settle")
                    pre["ok"] = False
                    return pre
                pre["order"].append("hand")
                state["hand"] = list(ps["hand_counts"])
        # 4. THE MOVE TO REST, AFTER THE POLE AND THE HAND (Newton, 2026-09-22: "verify that the
        # flow will move the pole and hand first before the arm movements, very important").
        # Until then it ran BEFORE the devices; now this task's arm moves only once its pole and
        # hand are set. The program's own bridge movej then goes from rest to the stroke's first
        # point.
        if taught_rest_setting(t).get("before"):
            if not args.rest:
                print("   rest (before): skipped — --no-rest")
                pre["rest"] = dict(skipped="--no-rest")
            else:
                from alix_dispatch import track as _tr
                try:
                    rb = taught_rest(t, "before")
                except Exception as exc:                               # noqa: BLE001
                    print(f"** rest (before): cannot read the rest pose ({exc}) — refusing")
                    pre["rest"] = dict(error=repr(exc))
                    pre["ok"] = False
                    return pre
                # SKIP ONLY ON THE MEASURED POSITION, never on a remembered flag: the arm can be
                # moved between tasks (pendant, drag-teach during the keypress pause).
                gap = _tr.start_gap_deg(h.robot, [int(round(x * 1000)) for x in rb["deg"]])
                if gap is not None and gap <= REST_AT_TOL_DEG:
                    print(f"   rest (before): skipped — the arm is at rest (measured {gap:.2f} deg)")
                    pre["rest"] = dict(skipped=f"at rest, measured gap {gap:.2f} deg")
                    state["parked"] = True
                else:
                    speed = taught_rest_speed(args)
                    print(f"   rest (before): movej to {rb['stage']} at speed {speed} "
                          f"of a 100 % global ratio (gap {gap if gap is None else round(gap, 1)} deg)")
                    gs = {}
                    with _tr.global_speed_guard(h.robot, args.restore_speed, log=print, verdict=gs):
                        ratio_ret = _tr.set_global_speed(h.robot, 100)
                        ret = int(h.robot.rm_movej([float(x) for x in rb["deg"]], speed, 0, 0, 1))
                    pre["rest"] = dict(stage=rb["stage"], deg=rb["deg"], ret=ret, speed=speed,
                                       ratio_ret=ratio_ret, gap_deg=gap, global_speed=gs)
                    if ret != 0:
                        print(f"** rest (before) movej REFUSED ret={ret}")
                        pre["ok"] = False
                        return pre
                    pre["order"].append("rest_before")
                    state["parked"] = True
        return pre

    try:
        while not stop:
            rep += 1
            for i, t in enumerate(rows):
                label = f"[{rep}:{i + 1}/{len(rows)}] slot {t['slot']} {t['name']}"
                if not args.no_pause:
                    ans = input(f"\n{label}: ENTER to proceed, q to stop > ").strip().lower()
                    if ans == "q":
                        print("stopped at the keypress")
                        stop = True
                        break
                else:
                    print(f"\n{label}")
                # the program: controller movej to the first point + the stroke —
                # retreat and devices run first, inside run_one's capture window
                rc, out = run_one(h, t, args, rec_dir, sdk,
                                  before_run=lambda t=t: before_run(t),
                                  after_run=((lambda verdict=None, t=t: park_at_rest(t, verdict))
                                             if args.rest else None),
                                  limits_cache=limits_cache)
                step = dict(slot=t["slot"], name=t["name"], rep=rep, rc=rc,
                            verdict=out.get("verdict"), tcp_gate=out.get("tcp_gate"),
                            replay_s=out.get("replay_s"),
                            transit_s=out.get("transit_s"), pre_run=out.get("pre_run"),
                            post_run=out.get("post_run"))
                summary.append(step)
                if rc != 0:
                    print(f"** {t['name']}: rc {rc} ({out.get('verdict')}) — stopping the flow")
                    rc_all, stop = rc, True
                    break
                prev_holder[0] = t
            else:
                if args.repeat != 0 and rep >= args.repeat:
                    stop = True
    except KeyboardInterrupt:
        print("** flow interrupted")
        rc_all = 130
    finally:
        if hand is not None:
            hand.close()
        connect.close_arm(h)
        rec = rec_dir / f"flow_{side}_{t_flow.strftime('%H%M%S')}.json"
        json.dump({"when": t_flow.isoformat(timespec="seconds"), "host": _socket.gethostname(),
                   "sdk": sdk, "arm_ip": h.ip, "side": side, "rc": rc_all,
                   "slots": [t["slot"] for t in rows], "speed": args.speed,
                   "restore_speed": args.restore_speed, "devices": not args.no_devices,
                   "retreat": bool(args.retreat), "repeat": args.repeat,
                   # How the flow was DRIVEN. Newton asked on 2026-09-03 whether a
                   # cut correlated with running without the per-task keypress, and
                   # the record could not answer it — none of these were stored.
                   "no_pause": bool(args.no_pause), "steal_push": bool(args.steal_push),
                   "blocking_probe": bool(getattr(args, "blocking_probe", False)),
                   "certificate_forced": bool(getattr(args, "force", False)),
                   "rest": bool(getattr(args, "rest", False)),
                   "rest_speed": getattr(args, "rest_speed", None),
                   "movej_speed": getattr(args, "movej_speed", None),
                   "start": args.start, "tracks": list(args.track or []),
                   "capture": args.record_joints is not False, "steps": summary},
                  open(rec, "w"), indent=1)
        print(f"flow record -> {repo_rel(rec)}")
    done = sum(1 for x in summary if x.get("rc") == 0)
    print(f"\nflow {'COMPLETE' if rc_all == 0 else 'STOPPED'}: {done} run(s) OK"
          f"{'' if rc_all == 0 else f', rc {rc_all}'}")
    return rc_all


def preflight_report(pf: dict, rows: list, other_rows: list, tool_frame_raw,
                     repo: pathlib.Path = REPO) -> dict:
    """The preflight verdict as a PURE function of what was read, so it can be
    tested without a controller (B15, H25, H35).

    Three things are asserted per slot, and every one of them is a refusal:

    * the bytes on disk still hash to what the manifest pinned (B15);
    * the controller's stored SIZE matches the .proj — and when there is no
      local .proj the manifest's pinned `proj_bytes` stands in, so the gate is
      no longer blind exactly where B15 bit;
    * the controller's stored `trajectory_name` is this row's file (H25). That
      is the real side-identity assert: `track.assert_side` compares the string
      that chose the IP with itself and can never fail, while slots 70..78 name
      DIFFERENT tasks on the two arms — so a swapped controller is only
      visible from the far end of the cable.

    Plus, once per side, the tool frame the controller is actually holding
    against the row's pinned expectation (H35).
    """
    out = dict(ok=True, slots={}, refusals=[], warnings=[], tool_frame=None,
               name_convention_seen=None)
    # A PREFLIGHT THAT CHECKED NOTHING IS NOT A PASS (hardening review
    # 2026-09-08). With no rows — an empty dry registry, a side with nothing
    # emitted — the loop below never ran and the report came back ok, which is
    # the same shape of answer as B15's ok=None.
    if not rows:
        out["ok"] = False
        out["refusals"].append("there are no emitted rows for this side to check — a "
                               "preflight that checked nothing is not a pass")
        return out
    by_name_row = {r["slot"]: r for r in rows}
    unchecked = [r for r in rows if r.get("slot") not in (pf.get("by_slot") or {})]
    for r in unchecked:
        out["ok"] = False
        out["refusals"].append(f"slot {r.get('slot')} ({r.get('name')}) was never looked up "
                               f"on the controller — it is in the rows but not in the "
                               f"program-list readback")
    # IS OUR NAMING MODEL EVEN TRUE OF THIS CONTROLLER? The expectation is the
    # basename of the .proj we send, which is what the emulated double stores
    # and what the vendor struct implies — but NO hardware record carries a
    # program list, so it has never been observed on an arm. If not one stored
    # name on this controller resembles any manifest row's file, the model is
    # wrong here rather than the slots being wrong, and a bringup must not die
    # on it: the name findings are then reported as WARNINGS and the stored-size
    # gate carries the check alone. A swapped controller cannot hide in that
    # branch — it would be holding OUR names, so the model would be confirmed
    # and the mismatch a hard refusal.
    seen_names = [str(n).strip().rstrip("\x00") for n in (pf.get("by_slot") or {}).values() if n]
    out["name_convention_seen"] = name_convention_confirmed(
        seen_names, list(rows) + list(other_rows or []))
    if seen_names and not out["name_convention_seen"]:
        out["warnings"].append(
            f"none of the controller's stored names {sorted(set(seen_names))[:4]} look like the "
            f"files we upload, so the c_char*32 `trajectory_name` convention this check assumes "
            f"is NOT what this controller does. Name findings are reported, not refused; the "
            f"stored-size comparison is the identity gate here. Record the real names.")
    for slot in sorted(pf.get("by_slot", {})):
        row = by_name_row.get(slot)
        name = pf["by_slot"][slot]
        e = dict(slot=slot, controller_name=name, reasons=[], warnings=[])
        if row is None:
            e["reasons"].append("no manifest row for this slot")
            out["slots"][slot] = e
            out["ok"] = False
            continue
        prov = check_row_provenance(row, repo)
        e["reasons"] += prov["reasons"]
        e["warnings"] += prov["warnings"]
        e["disk"] = prov["disk"]
        if name is None:
            e["reasons"].append("MISSING from the controller — run upload")
        else:
            nc = trajectory_name_check(name, row, other_side_rows=other_rows,
                                       stored_size=(pf.get("size_check") or {})
                                       .get(slot, {}).get("stored"))
            e["name_check"] = nc
            if out["name_convention_seen"]:
                e["reasons"] += nc["reasons"]
            else:
                e["warnings"] += [f"name (advisory, convention unconfirmed): {r}"
                                  for r in nc["reasons"]]
        sc = (pf.get("size_check") or {}).get(slot) or {}
        e["size_check"] = dict(sc)
        if sc.get("ok") is False:
            e["reasons"].append(f"stored {sc.get('stored')} B vs local {sc.get('local')} B "
                                f"(delta {sc.get('delta'):+d}) — the controller did not keep "
                                f"the whole file (F112)")
        elif sc.get("ok") is None and name is not None:
            # THE B15 BLIND SPOT. preflight_slots returns ok=None when there is
            # no local .proj, and its caller only counted `ok is False`. The
            # manifest's pinned proj_bytes is a second source for the same
            # comparison; with neither, this is a refusal, not a shrug.
            want = row.get("proj_bytes")
            got = sc.get("stored")
            if want and got is not None:
                delta = int(got) - int(want)
                e["size_check"] = dict(sc, local=int(want), delta=delta,
                                       ok=abs(delta) <= max(64, int(want) // 1000),
                                       source="manifest proj_bytes")
                if not e["size_check"]["ok"]:
                    e["reasons"].append(f"stored {got} B vs the manifest's pinned "
                                        f"{want} B (delta {delta:+d})")
            else:
                e["reasons"].append("no local .proj AND no pinned proj_bytes — the stored "
                                    "size cannot be compared with anything (this is the "
                                    "B15 blind spot); run `provenance --write` then upload")
        out["slots"][slot] = e
        if e["reasons"]:
            out["ok"] = False
            out["refusals"] += [f"slot {slot}: {r}" for r in e["reasons"]]
        out["warnings"] += [f"slot {slot}: {w}" for w in e["warnings"]]
    for extra in pf.get("extra") or []:
        out["warnings"].append(f"slot {extra} is in the reserved range but not in the "
                               f"manifest (stale? re-emit + upload)")
    if rows:
        tfc = tool_frame_check(tool_frame_raw, rows[0])
        out["tool_frame"] = tfc
        # Every row of a side pins the same observed frame today, but say so
        # rather than assume it: a row that disagrees with the side's first is
        # its own refusal.
        # RECORDED, NEVER REFUSED (Newton, 2026-09-15): a frame other than the
        # one the rows' own runs were made under is reported as a warning and
        # written into the preflight report; it was a refusal until then.
        for r in rows[1:]:
            if (r.get("tool_frame") or {}).get("name") != (rows[0].get("tool_frame") or {}).get("name"):
                out["warnings"].append(
                    f"{r['name']} pins tool frame "
                    f"{(r.get('tool_frame') or {}).get('name')!r} but "
                    f"{rows[0]['name']} pins "
                    f"{(rows[0].get('tool_frame') or {}).get('name')!r} — recorded, "
                    f"not a refusal (2026-09-15)")
        if not tfc["ok"]:
            out["warnings"] += [f"tool frame (recorded, not a refusal): {r}"
                                for r in tfc["reasons"]]
    return out


def snapshot_warnings(snap: dict) -> list:
    """What a bringup snapshot says that a session must read before it runs
    anything. WARNINGS, not refusals: the run-mode / error-code session gate is
    the executive's (H25), and a false refusal here would block a bringup on a
    getter shape we have not seen. Slot 78's cut was a joint fault (4107 +
    encoder + CAN) visible only in the snapshot — so it must at least be SAID.
    """
    out = []
    st = snap.get("arm_state")
    if isinstance(st, dict):
        if st.get("err_codes"):
            out.append(f"the controller has LATCHED error codes {st['err_codes']} — "
                       f"clear them before running")
        if st.get("arm_err"):
            out.append(f"arm_err {st['arm_err']}")
        if st.get("sys_err"):
            out.append(f"sys_err {st['sys_err']}")
    jf = snap.get("joint_err_flag")
    jf = jf[1] if isinstance(jf, (tuple, list)) and len(jf) == 2 else jf
    if isinstance(jf, dict):
        jf = jf.get("err_flag")
    if isinstance(jf, (list, tuple)) and any(int(v or 0) for v in jf):
        out.append(f"joint error flags {list(jf)}")
    return out


def cmd_preflight(args) -> int:
    """Bringup check, once per session: every EMITTED row of the side must be
    on ITS controller, hashed to its pin, named for its own task, and the arm
    must be holding the tool frame the rows were run under. Read-only."""
    from alix_dispatch import connect, track as tr
    man = manifest_for(args)
    # a CANFD-only row (slot > 100) has no controller slot to look for
    rows = [t for t in rows_for(man, args.side) if not canfd_only_slot(t.get("slot"))]
    other = [t for t in rows_for(man, "left" if args.side == "right" else "right")
             if not canfd_only_slot(t.get("slot"))]
    h = open_or_die(args.side, args.ip)
    try:
        pf = tr.preflight_slots(h.robot, rows, repo=REPO)
        snap = tr.controller_snapshot(h.robot)
        tf_raw = snap.get("tool_frame")
    finally:
        connect.close_arm(h)
    rep = preflight_report(pf, rows, other, tf_raw, repo=REPO)
    print(f"{args.side} arm: {pf['present']}/{pf['checked']} manifest slots present "
          f"(list ret={pf['ret']})")
    for slot in sorted(rep["slots"]):
        e = rep["slots"][slot]
        sc = e.get("size_check") or {}
        if sc.get("ok") is True:
            src = f" ({sc['source']})" if sc.get("source") else ""
            extra = f"   stored {sc['stored'] / 1024:7.1f}KB == local{src}  ok"
        elif sc.get("ok") is False:
            extra = (f"   stored {sc['stored'] / 1024:7.1f}KB vs "
                     f"{(sc.get('local') or 0) / 1024:7.1f}KB  delta {sc.get('delta'):+d} B"
                     f"  ** MISMATCH")
        else:
            extra = "   (size not comparable)"
        nm = e.get("controller_name")
        print(f"  slot {slot:>3}: {'MISSING — run upload' if nm is None else nm}{extra}")
        for w in e.get("warnings") or []:
            print(f"          !! {w}")
        for r in e.get("reasons") or []:
            print(f"          ** {r}")
    for line in snapshot_warnings(snap):
        print(f"  !! {line}")
    tfc = rep.get("tool_frame") or {}
    if tfc:
        print(f"\n  tool frame: controller holds {tool_frame_line(tfc.get('found'))}")
        print(f"              rows expect     {tool_frame_line(tfc.get('expected'))}")
        for r in tfc.get("reasons") or []:
            print(f"          ** {r}")
    if pf["extra"]:
        print(f"  reserved-range slots not in the manifest: {pf['extra']} "
              f"(stale? re-emit + upload)")
    if not rep["ok"]:
        print(f"\n  ** preflight REFUSES the {args.side} arm: {len(rep['refusals'])} "
              f"problem(s). A short slot ends its program early with no error at all "
              f"(F112); a slot holding another task's bytes is B15. Neither "
              f"announces itself at run time. (A controller holding another tool "
              f"frame than these rows' own runs were made under is reported above "
              f"as a warning and recorded, never refused — 2026-09-15.)")
        return 1
    print(f"\n  {args.side} arm PASSES preflight")
    return 0


def _run_args(p) -> None:
    p.add_argument("--speed", type=int, default=50,
                   help="movej-transit speed AND the global ratio while running (default 50)")
    p.add_argument("--restore-speed", type=int, default=50,
                   help="global ratio restored after the run, ALWAYS (default 50)")
    p.add_argument("--accept-out-of-band", action="store_true")
    p.add_argument("--start-tolerance-deg", type=float, default=2.0,
                   help="per-joint gap above which the controller's movej transit to the "
                        "first point is engaged (and announced). Default behaviour is to "
                        "ALLOW it (ruling 2026-09-01)")
    p.add_argument("--require-at-start", action="store_true",
                   help="strict flow: refuse instead — stream move_to_start over CANFD "
                        "first, then run")
    p.add_argument("--ip", default=None)
    p.add_argument("--movej-speed", type=int, default=80,
                   help="speed for the explicit movej that bridges to the track's FIRST "
                        "point (default 80). The controller's own implicit transit runs at "
                        "--speed, so this makes it independent; 0 restores the old behaviour "
                        "of letting the controller do it at --speed.")
    p.add_argument("--rest-speed", type=int, default=80,
                   help="speed for the movej that parks the arm at the plan's move_to_rest "
                        "pose after each replay (default 80). A TAUGHT row's rest moves (before "
                        "and after its stroke, `taught_tracks.py rest`) run at min(this, "
                        "--movej-speed): they are not collision-checked. --no-rest skips them.")
    p.add_argument("--no-rest", dest="rest", action="store_false",
                   help="no park at rest after a replay, and no move to rest before a taught "
                        "row's stroke (the arm goes straight to the stroke's first point)")
    p.set_defaults(rest=True)
    p.add_argument("--ignore-devices", action="store_true",
                   help="run a single task even though the pole is not at the height the "
                        "task was planned with. `run` never COMMANDS the devices — the "
                        "flow does — and since 2026-09-03 it refuses rather than replay a "
                        "stroke at the wrong standoff.")
    p.add_argument("--log-events", action="store_true",
                   help="register the SDK's arm-event callback and record the arrival / "
                        "program-finish stream around each run. Pairs with --blocking-probe: "
                        "its -4 means the SUCCESS message was not received, and this shows "
                        "whether RM_PROGRAM_RUN_FINISH_E was actually absent and what the "
                        "last arrival said. Opt-in: the callback is one per process and "
                        "cannot be exercised offline (the emulator fires no finish event).")
    p.add_argument("--blocking-probe", action="store_true",
                   help="start the program with the vendor's BLOCKING call in a worker "
                        "thread so its return code is observable: -4 means the trajectory "
                        "was stopped, 0 means the program finished (rm_interface.h:3270). "
                        "Called non-blocking (the default) the return is always 0 and says "
                        "nothing, which is why every run to 2026-09-03 is silent on this. "
                        "Opt-in: a second concurrent SDK caller on a live arm, never yet "
                        "run on hardware.")
    p.add_argument("--ignore-tool-frame", action="store_true",
                   help="NO EFFECT since 2026-09-15: the controller's tool frame is read "
                        "and RECORDED with every run (tool_frame_check in the run record) "
                        "and never refused — Newton: the frame must not block a run. Kept "
                        "so existing command lines still parse.")
    p.add_argument("--channel", choices=("upload", "canfd"), default="upload",
                   help="upload (default): play the file uploaded to the controller's slot. "
                        "canfd: stream the SAME lines from this host, one rm_movej_canfd per "
                        "line, with the same checks, guards, capture and verdict. Records "
                        "are named canfd_slot*, kept out of every upload-ledger reader.")
    p.add_argument("--canfd-period-ms", type=float, default=None,
                   help="CANFD: ms per line (default 5.0, the controller's own cycle; the old "
                        "5.5545 default was an upload whole-run AVERAGE, i.e. 5.0 ms plus 11 "
                        "percent of hold time, and CANFD has no holds). Every line must stay "
                        "within the --canfd-rate-gate ceiling at this period or the run is "
                        "refused. NOTE argparse %%-formats help text: write 'percent', not a "
                        "bare '%%' followed by a letter, or every --help on this verb crashes.")
    p.add_argument("--canfd-follow", choices=("high", "low"), default="high",
                   help="CANFD: follow mode (high needs a period <= 10 ms)")
    p.add_argument("--canfd-mode", type=int, choices=(0, 1, 2), default=0,
                   help="CANFD trajectory_mode, high follow only: 0 pass-through "
                        "(default), 1 curve fit, 2 filter")
    p.add_argument("--canfd-radio", type=int, default=0,
                   help="CANFD smoothing coefficient for modes 1 (0-100) and 2 (0-999)")
    p.add_argument("--canfd-rate-gate", choices=("per-joint", "flat"), default="per-joint",
                   help="CANFD: which rate ceiling every line is held to. 'per-joint' (DEFAULT "
                        "since 2026-09-21) holds each joint to its own rating, 180/180/225x5 — the "
                        "limit the arm actually enforces: J5 commanded 219.6 and J6 213.8 deg/s ran "
                        "with no error code on hardware, so the vendor's documented flat 180 "
                        "(JSON Protocol pp.46/48) is not enforced. 'flat' holds every joint to 180.")
    p.add_argument("--tool-frame", default=None, metavar="NAME",
                   help="select THIS controller tool frame for the run instead of the one "
                        "the row's own runs were supervised at (manifest tool_frame.name). "
                        "Proved by read-back, the pendant's frame put back afterwards, and "
                        "recorded as tool_frame_guard.wanted_from. A certificate's "
                        "calibration was measured at the row's frame, not at this one.")
    p.add_argument("--ignore-provenance", action="store_true",
                   help="run even though the track on disk no longer hashes to the "
                        "manifest's pin, OR the controller's stored slot is not the size "
                        "of that row's .proj. The band, the point count and every "
                        "collision figure describe the pinned bytes (B15). It does NOT "
                        "override a track that cannot be read at all. Recorded.")
    p.add_argument("--yes", action="store_true",
                   help="skip the typed confirmation (automation; side still hard-asserted)")
    p.add_argument("--steal-push", action="store_true",
                   help="DANGEROUS: take the realtime UDP push even though it is "
                        "enabled toward a consumer (starves a live driver in 250 ms). "
                        "Only when you are certain nothing consumes it.")
    p.add_argument("--line-speed", type=float, default=None,
                   help="TCP max LINEAR speed held during the replay, m/s (default 1.5). "
                        "The RM75's body spec is <=1.8 m/s; the API publishes no range.")
    p.add_argument("--angular-speed", type=float, default=None,
                   help="TCP max ANGULAR speed held during the replay, rad/s (default 5.0). "
                        "No published ceiling of any kind for this one.")
    p.add_argument("--cap-ratio", type=float, default=None,
                   help="each max ACCELERATION as a multiple of its own max speed "
                        "(default 4.0). The vendor requires >=3 — 'otherwise, abnormal "
                        "motion may occur' — and publishes no maximum.")
    p.add_argument("--no-caps", action="store_true",
                   help="do not touch the arm's Cartesian limits; replay under whatever "
                        "is already configured (the behaviour of every run before "
                        "2026-09-05)")
    p.add_argument("--udp-port", type=int, default=None,
                   help="local UDP port the capture binds and the controller pushes to "
                        "(default RM_UDP_PORT, else 8089). Use it when something else on "
                        "this host holds 8089 — see `push --show`.")
    p.add_argument("--record-joints", dest="record_joints", action="store_const", const=True,
                   default=None,
                   help="the DEFAULT: capture the controller's realtime UDP push for the whole "
                        "run (retreat + devices + program in a flow) — every section, "
                        "verbatim frames at 5 ms — to a _frames.ndjson beside the run "
                        "record; prior push config is read first and restored exactly. "
                        "Stated explicitly it REFUSES to run when the capture is impossible "
                        "(a driver owns the push); by default that case runs uncaptured "
                        "with a notice. A console .log is always written beside the record.")
    p.add_argument("--no-record-joints", dest="record_joints", action="store_const", const=False,
                   help="run UNLOGGED (no frames capture); the JSON record + .log remain")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    e = sub.add_parser("emit", help="emit one arm's cleaning tracks, slots 70.. on that arm (offline)")
    e.add_argument("--side", choices=SIDES, default="right")
    e.add_argument("--acc-scale", type=float, default=ACC_SCALE_DEFAULT,
                   help=f"channel acceleration policy for this emit (default {ACC_SCALE_DEFAULT})")
    e.add_argument("--region", action="append",
                   help="re-emit only these region key(s) INTO their existing slots")
    e.set_defaults(fn=cmd_emit)
    s = sub.add_parser("status", help="manifest table; --robot adds a controller's slot list")
    s.add_argument("--robot", action="store_true")
    s.add_argument("--side", choices=SIDES, default="right")
    s.add_argument("--ip", default=None)
    s.set_defaults(fn=cmd_status)
    u = sub.add_parser("upload", help="store one arm's tracks in its slots (only_save — no motion)")
    u.add_argument("track", nargs="*", help="slot numbers / region keys; default: all of the side")
    u.add_argument("--side", choices=SIDES, default="right")
    u.add_argument("--ip", default=None)
    u.add_argument("--yes", action="store_true",
                   help="skip the typed confirmation (automation; side still hard-asserted)")
    u.add_argument("--taught", action="store_true",
                   help="resolve the rows against Resource/tracks/taught/TAUGHT_TRACKS.json "
                        "(the taught lane), not the production manifest")
    u.add_argument("--experiment", action="store_true",
                   help="resolve the rows against Resource/tracks/experiments/"
                        "EXPERIMENT_TRACKS.json (slots 21-39, 61-69); refuses to overwrite a slot "
                        "holding a program that is not ours")
    u.add_argument("--overwrite-foreign", action="store_true",
                   help="with --experiment: overwrite slots holding programs that are not ours")
    u.set_defaults(fn=cmd_upload)
    p = sub.add_parser("preflight", help="controller slot list vs manifest (read-only bringup check)")
    p.add_argument("--side", choices=SIDES, required=True)
    p.add_argument("--ip", default=None)
    p.add_argument("--dry", action="store_true",
                   help="check the DRY registry's slots instead of the production manifest")
    p.add_argument("--taught", action="store_true",
                   help="check the taught lane's slots (TAUGHT_TRACKS.json) instead of "
                        "the production manifest")
    p.add_argument("--experiment", action="store_true",
                   help="check the experiment slots (EXPERIMENT_TRACKS.json)")
    p.set_defaults(fn=cmd_preflight)
    rc_ = sub.add_parser("ratecheck", help="every row against the ratings at the player's 5.0 ms ceiling, and "
                                           "what emit's rate fit would do (read-only)")
    rc_.add_argument("--side", choices=SIDES, default=None)
    rc_.add_argument("--taught", action="store_true")
    rc_.add_argument("--experiment", action="store_true")
    rc_.add_argument("--dry", action="store_true")
    rc_.set_defaults(fn=cmd_ratecheck)
    af = sub.add_parser("accflag", help="the ACCELERATION FLAG per row (Newton 2026-10-02 'Keep advisory, flag it'): "
                                        "20 Hz peak + %% of moving time above 600 / 900 deg/s^2 (offline; advisory)")
    af.add_argument("--side", choices=SIDES, default=None)
    af.add_argument("--write", action="store_true",
                    help="record the block in MANIFEST.json rows and the track sidecars (no track byte changes)")
    af.set_defaults(fn=cmd_accflag)
    ex = sub.add_parser("experiment",
                        help="install a plan-emitted track BYTE FOR BYTE as an experiment row "
                             "(slots 61-69), or list them")
    ex.add_argument("track", nargs="?", help="the emitted .txt (its .json/.proj beside it)")
    ex.add_argument("--plan", default=None,
                    help="the plan .json it was emitted from (omit for a RECORDED track: no staging)")
    ex.add_argument("--allow-over-rating", action="store_true",
                    help="install a track whose ticks exceed a joint rating (recorded in the row)")
    ex.add_argument("--ik-frame", default=None, help="contact frame recorded in the row")
    ex.add_argument("--side", choices=SIDES, default=None)
    ex.add_argument("--slot", type=int, default=None,
                    help=", ".join(f"{lo}-{hi}" for lo, hi in EXPERIMENT_SLOT_RANGES))
    ex.add_argument("--name", default=None, help="row name, e.g. commode_c_toplid_right_spline_0917")
    ex.add_argument("--tool-frame-from", default=None,
                    help="production row whose run records give the controller tool frame "
                         "(e.g. commode_c_toplid_right)")
    ex.add_argument("--replace", action="store_true")
    ex.add_argument("--list", action="store_true")
    ex.set_defaults(fn=cmd_experiment)
    pv = sub.add_parser("provenance",
                        help="pin every row to the bytes on disk + say what the run ledger "
                             "actually played (offline)")
    pv.add_argument("--side", choices=SIDES, default=None)
    pv.add_argument("--write", action="store_true",
                    help="write the pins (and the .proj files) into MANIFEST.json")
    pv.set_defaults(fn=cmd_provenance)
    dy = sub.add_parser("dry",
                        help="synthesise + upload a zero-exposure in-place track (ladder D1/D2)")
    dy.add_argument("--side", choices=SIDES, required=True)
    dy.add_argument("--slot", type=int, default=DRY_SLOT_DEFAULT,
                    help=f"controller slot, BELOW the reserved {SLOT_BASE}-{SLOT_MAX} range "
                         f"(default {DRY_SLOT_DEFAULT})")
    dy.add_argument("--points", type=int, default=1000,
                    help="track length in points (default 1000 = 5.55 s at the player clock)")
    dy.add_argument("--roll-deg", type=float, default=2.0,
                    help="peak deviation from the rest pose, degrees (default 2.0). NO point "
                         "of the track is further than this from rest, on any joint.")
    dy.add_argument("--cycles", type=int, default=2,
                    help="whole sine cycles over the track (default 2); whole cycles keep the "
                         "first and last point exactly at rest")
    dy.add_argument("--joint", type=int, default=DRY_JOINT_DEFAULT,
                    help=f"which joint dithers (default {DRY_JOINT_DEFAULT}, the wrist roll: "
                         f"the TCP does not translate)")
    dy.add_argument("--shape", choices=("sine", "still", "cruise"), default="sine",
                    help="sine: the zero-exposure dither (default). still / cruise: a HOLD PROBE, needs --probe")
    dy.add_argument("--probe", action="store_true",
                    help=f"allow a hold probe: J6/J7 only, <= {PROBE_MAX_RATE_DEG_S:g} deg/s at the 5.0 ms ceiling, "
                         f"<= {PROBE_MAX_AMPLITUDE_DEG:g} deg from rest, <= {PROBE_MAX_ACCEL_DEG_S2:g} deg/s^2")
    dy.add_argument("--speed-deg-s", type=float, default=None, help="cruise speed (probe)")
    dy.add_argument("--accel-deg-s2", type=float, default=600.0, help="reversal acceleration (probe)")
    dy.add_argument("--amplitude-deg", type=float, default=10.0, help="cruise amplitude from rest (probe)")
    dy.add_argument("--duration-s", type=float, default=20.0, help="probe length at one point per 5 ms")
    dy.add_argument("--rating-sweep", action="store_true",
                    help=f"a RATING SWEEP instead of a wrist probe: J{'/J'.join(str(j) for j in RATING_SWEEP_JOINTS)} "
                         f"only (rated {RATED_DEG_S[0]:g} deg/s = the CANFD limit, so the same bytes stream "
                         f"hold-free as a control), speed given by --speed-frac, <= "
                         f"{RATING_SWEEP_MAX_AMPLITUDE_DEG:g} deg amplitude. J1/J2 swing the whole arm "
                         f"through twice the amplitude: check the workspace before running one")
    dy.add_argument("--speed-frac", type=float, default=None,
                    help=f"rating sweep: commanded speed as a fraction of the joint's rating, "
                         f"<= {RATING_SWEEP_MAX_FRAC:g} (1.05 and 1.10 are the clamp conditions)")
    dy.add_argument("--no-upload", action="store_true",
                    help="synthesise and register only; send nothing")
    dy.add_argument("--overwrite-foreign", action="store_true",
                    help="overwrite a slot holding a program that is not ours (a pendant file); "
                         "without it the upload refuses and names what is there")
    dy.add_argument("--ip", default=None)
    dy.add_argument("--yes", action="store_true", help="skip the typed confirmation")
    dy.set_defaults(fn=cmd_dry)
    ph = sub.add_parser("push", help="show / repoint / disable the controller's realtime UDP push")
    ph.add_argument("--side", choices=SIDES, default="right")
    ph.add_argument("--ip", default=None)
    ph.add_argument("--udp-port", type=int, default=None)
    ph.add_argument("--clear", action="store_true",
                    help="disable the push (clears a stale enable left by a crashed session, "
                         "so later captures stop needing --steal-push)")
    ph.add_argument("--set", default=None, metavar="IP[:PORT]",
                    help="point the push at this consumer, e.g. a driver host you took it from")
    ph.add_argument("--yes", action="store_true", help="skip the typed confirmation")
    ph.set_defaults(fn=cmd_push)
    r = sub.add_parser("run", help="run one slot, watch to the end, verdict against its band")
    r.add_argument("track", help="slot number or region key")
    r.add_argument("--side", choices=SIDES, default="right")
    r.add_argument("--dry", action="store_true",
                   help="resolve the slot against DRY_TRACKS.json, not the production "
                        "manifest (ladder rungs D1/D2)")
    r.add_argument("--taught", action="store_true",
                   help="resolve the slot against Resource/tracks/taught/TAUGHT_TRACKS.json "
                        "(the taught lane), not the production manifest")
    r.add_argument("--experiment", action="store_true",
                   help="resolve the slot against EXPERIMENT_TRACKS.json (slots 61-69)")
    _run_args(r)
    r.set_defaults(fn=cmd_run)
    f = sub.add_parser("flow", help="one arm's tasks in schedule order: devices, movej, stroke, verdict")
    f.add_argument("track", nargs="*", help="subset (slot numbers / region keys); default: all")
    f.add_argument("--side", choices=SIDES, default="right")
    f.add_argument("--start", default=None, help="begin at this slot / region key")
    f.add_argument("--repeat", type=int, default=1, help="passes over the list; 0 = until q / Ctrl-C")
    f.add_argument("--no-pause", action="store_true", help="no keypress between tasks")
    f.add_argument("--no-retreat", dest="retreat", action="store_false",
                   help="skip the movej to the previous plan's first post-stroke pose")
    f.add_argument("--no-devices", action="store_true",
                   help="never command the pole or the hand (bench / mock only)")
    f.add_argument("--taught", action="store_true",
                   help="run the taught lane's rows (TAUGHT_TRACKS.json) instead of the "
                        "production manifest — pole/hand come from each recording's own "
                        "provenance (pre_stroke)")
    f.add_argument("--experiment", action="store_true",
                   help="run the experiment rows (EXPERIMENT_TRACKS.json, slots 61-69) — "
                        "pole/hand/retreat from each row's own plan")
    _run_args(f)
    f.set_defaults(fn=cmd_flow, retreat=True)
    args = ap.parse_args(argv)
    return args.fn(args)


if __name__ == "__main__":
    sys.exit(main())
