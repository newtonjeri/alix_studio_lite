"""Device staging for a track replay — the proven choreography, as a module
that REFUSES instead of exiting (B5, B6, H33, M41, M42, L49).

WHY THIS EXISTS. The pre-stroke choreography that ran 12/12 on hardware twice on
2026-09-04 lives inside `tools/cleaning_tracks.py` as a closure — `before_run`
(:1454-1558) plus `check_devices_as_found` (:892) and `park_at_rest` (:1423).
The dual-arm executive imports no `peripherals` at all, so both arms replay at
whatever pole height and hand state the previous task left; 3 of the 4
concurrent schedule entries need a pole change on BOTH sides, and one leg
boundary needs them to move 75 mm in OPPOSITE directions. Re-run of the leg-B
sweep in four pole configurations: 39.07 mm as verdicted, 8.49 / 8.58 / 8.58 mm
for the three mis-staged ones — every mis-staged configuration is contact class
(2026-09-08 DESIGN §3.3). The ledger has the precedent: `2026-09-03/
run_slot72_211316.json` replayed with the pole 165 mm from where its plan wanted
it.

THE ONE RULE THIS MODULE ADDS TO THE PROVEN SEQUENCE. Nothing here calls
``sys.exit``, raises ``SystemExit``, or lets a device refusal escape as an
exception. In a single-arm tool that was fine — a refusal ended the process. In
a dual-arm executive the same refusal ends ONE WORKER THREAD and leaves the
partner replaying against a stationary, mis-staged arm. So every function
returns the STAGING RESULT:

    {"ok": bool, "side": str, "refusal": str|None,
     "pole": {"target_m", "found_m", "verified_by": "frames"|"getter"|"no_op",
              "travel_s"},
     "hand": {"target", "read", "asserted"},
     "arm":  {"at_start", "gap_deg"},
     "half_staged": {} | {"pole": "...", "hand": "..."}}

Missing numbers are ``None``, never a guess. ``refusal`` is one of the
``REFUSE_*`` codes below so a caller can branch on it without parsing prose.

WHAT IS DELIBERATELY *NOT* HERE. The bridge to the track's first point stays in
``track.run_verified`` — this module only MEASURES the gap and reports
``arm.at_start``, so the proven run policy is untouched. And nothing here
switches controller modes as a side effect of a check or a stop.

HOW ``cleaning_tracks`` ADOPTS THIS WITHOUT CHANGING BEHAVIOUR. `_flow_body`
keeps its own ``pole``/``hand``/``state`` locals and replaces the BODY of
``before_run`` with::

    res = staging.stage_arm(h, t, pole=pole, hand=hand, log=print,
                            prev=prev_holder[0], prev_parked=state["parked"],
                            release_counts=hand_from_frame_release(h.side)[1],
                            release_source="the SRDF `release` state",
                            block_pole=getattr(args, "blocking_probe", False))
    state["pole_m"] = res["pole"]["found_m"]
    pre = {"ok": res["ok"], "retreat": ..., "pole": ..., "hand": ...}   # from res["steps"]

and ``park_at_rest``'s body with ``staging.park_arm(h, t, rest_speed=
args.rest_speed, restore_speed=args.restore_speed)``. The step order, the skip
conditions, the F9 interlock and the speed handling are the same; what changes
is that a refusal comes back as a value. `--no-devices` simply does not call
``stage_arm`` (calling it with both devices None is itself a refusal). That
rewiring is lane C's, in lane C's file; nothing here needs it to happen.

WHAT THE TWO GATES DEFAULT TO, AND WHY. ``require_hand_assert`` is ON: a task
that declares a hand state and has no modbus session is REFUSED, because
"fully open" is what an uncommanded run inherits and that is the 8.52 mm
configuration (H33). ``require_frames`` is OFF: `cleaning_tracks flow` starts
no recorder, so demanding a push channel there would refuse the flow that ran
12/12 on hardware. A dual executive that starts a recorder BEFORE staging
should pass ``require_frames=True`` and hand ``verify_from_frames=
recorder_frame_source(rec, side)``; without it a pole arrival can still be
signed off by the getter, which is the channel B6 is about.

THE HOLDING STATE. ``recover_arm`` is M42's "stop, park, report" for one arm,
so the two sides cannot drift apart in how they recover; the executive calls
it on both handles and does not retry or advance.
"""
from __future__ import annotations

import json
import pathlib
import time
from typing import Callable, Dict, List, Optional

from alix_dispatch import peripherals as per

# ── refusal codes ────────────────────────────────────────────────────────────
REFUSE_NO_DEVICES = "no_devices"                 # staging asked for with no pole/hand
REFUSE_HAND_MODBUS = "hand_modbus_unavailable"   # rm_set_modbus_mode refused
REFUSE_RETREAT = "retreat_refused"               # the plan's way out was refused
REFUSE_HAND_RELEASE = "hand_release_failed"      # could not open the hand pre-pole
REFUSE_ARM_MOVING = "arm_moving_refuse_pole"     # F9 interlock
REFUSE_POLE_SEND = "pole_command_refused"        # rm_set_lift_height ret != 0
REFUSE_POLE_ARRIVE = "pole_not_verified"         # never arrived / never witnessed
REFUSE_HAND_COMMAND = "hand_command_failed"      # ANGLE_SET did not settle
REFUSE_HAND_UNVERIFIED = "hand_unverified"       # commanded, but not read back (H33)
REFUSE_DEVICES_AS_FOUND = "devices_not_as_planned"   # the post-stage gate
REFUSE_NOT_AT_START = "arm_not_at_start"
REFUSE_NO_REST_POSE = "no_rest_pose"
REFUSE_REST_MOVE = "rest_move_refused"
REFUSE_POLE_UNSTAGED = "pole_not_staged"    # the task needs a pole; none supplied
REFUSE_HAND_UNSTAGED = "hand_not_staged"    # the task needs a hand; no session
REFUSE_POLE_AMBIGUOUS = "pole_height_ambiguous"  # the two channels disagree
#: `stop_devices` writes this when a device would not stop. It was the one
#: refusal in this module with no NAME and no export, so a caller that
#: recognises refusals by comparing against the exported vocabulary — which is
#: the only way to tell a retryable refusal from `pole_height_ambiguous`, the
#: one that means "operator, look at the pole" — could not match the single
#: refusal it must never retry (cross-lane contract review, 2026-09-09).
REFUSE_STOP_UNVERIFIED = "stop_unverified"

#: H33 — how close a read-back ANGLE_ACT must be to the commanded ANGLE_SET for
#: the hand to count as ASSERTED. `HAND_ARRIVE_COUNTS` (15) is the "arrived"
#: band inside `command()`; this is the wider *staging* band.
#:
#: MEASURED, not chosen. Over every hand command the run ledger accepted
#: (`ok: True`) in Resource/runs/tracks/2026-09-0*: 146 release commands, worst
#: residual 23 counts; 204 pre-stroke commands, median 17, p95 22 — and SIX at
#: 26-41 counts, every one of them on channel 6 and every one of them a stage
#: hardware went on to run (their own `arrived` flag was False, `settled` True).
#: So a flat 25-count band would refuse 3 % of the stages the proven flow has
#: already taken, and in a dual row one arm refusing is the schedule stopping.
#: Channel 6 is thumb ROTATION about its own axis, not a closure DOF, so it is
#: not what row 11's 35.5 mm clearance rests on; it gets a band that covers the
#: measured maximum with margin. The five closure channels keep the hand's own
#: 25-count deadband, which covers their measured worst (23) as it stands.
HAND_ASSERT_COUNTS = per.HAND_DEADBAND_COUNTS   # 25 — channels 1-5 (closure)
HAND_ASSERT_COUNTS_CH6 = 50                     # channel 6 — thumb rotation
HAND_ASSERT_BAND = [HAND_ASSERT_COUNTS] * 5 + [HAND_ASSERT_COUNTS_CH6]


def hand_asserted(read, target):
    """``(asserted, miss, band)`` for a read-back against a commanded pose.

    A single number cannot answer this: see `HAND_ASSERT_BAND`. Returns
    ``(False, None, band)`` when there is nothing to compare, because an
    unread hand is unasserted — never asserted-by-default (H33).
    """
    band = list(HAND_ASSERT_BAND)
    if not read or not target:
        return False, None, band
    # A None target is a HELD channel (exclude_thumb): nothing was commanded
    # there, so it cannot miss. 0, as alix_execution's staging reports it.
    miss = [0 if b is None else abs(int(a) - int(b)) for a, b in zip(read, target)]
    ok = all(m <= band[i] if i < len(band) else m <= HAND_ASSERT_COUNTS
             for i, m in enumerate(miss))
    return bool(ok), miss, band

#: What "the arm is at the track's first point" means, matching
#: `cleaning_tracks --start-tolerance-deg`.
START_TOLERANCE_DEG = 2.0


def _result(side: str) -> dict:
    """A STAGING RESULT with every contract key present and every number None."""
    return {
        "ok": False, "side": side, "refusal": None,
        "pole": {"target_m": None, "found_m": None, "verified_by": None,
                 "travel_s": None},
        "hand": {"target": None, "read": None, "asserted": False},
        "arm": {"at_start": None, "gap_deg": None},
        "half_staged": {},
        "steps": [], "lines": [],
    }


_MISSING = object()


def _guard(what: str, fn, *a, default=None, log: Callable[[str], None] = print,
           **kw):
    """Call *fn* and turn ANY exception into *default* plus a logged line.

    The one rule of this module is that a device problem comes back as a value.
    An SDK handle whose socket has died raises out of ``rm_*`` — inside an
    ``ArmWorker`` thread that kills the worker and leaves the partner replaying
    against a stationary arm, which is precisely the failure B5 is about. So
    every call that reaches the controller goes through here.
    """
    try:
        return fn(*a, **kw)
    except (Exception, SystemExit) as exc:                             # noqa: BLE001
        # SystemExit is caught too: a library that calls ``sys.exit`` inside a
        # worker thread ends that ONE thread silently and leaves the partner
        # replaying. KeyboardInterrupt is deliberately NOT caught — the
        # operator's Ctrl-C must still reach the executive, which answers it
        # with `stop_devices` on both controllers.
        log(f"   ** {what} raised {exc!r} — treated as a refusal")
        return default


def _refuse(out: dict, code: str, line: str, log: Callable[[str], None]) -> dict:
    out["ok"] = False
    out["refusal"] = code
    out["lines"].append(line)
    log(line)
    return out


# ── frame sources (B6) ───────────────────────────────────────────────────────
def recorder_frame_source(recorder, side: str) -> Callable[[], Optional[float]]:
    """A pole-height frame source backed by a live :class:`recording.RunRecorder`.

    The recorder already stores every 5 ms push frame as a row whose
    ``lift_height`` column is the physical pole height in mm (``recording.py``
    header: ``lift_height`` is the 22nd field, index 22 after ``t_mono`` plus
    seven each of five joint fields plus enables plus error codes). Returning
    the LAST row's height gives `Pole.move` the channel that does not lie.
    Returns None while no frame for that side has landed yet, which is exactly
    what `Pole.move` treats as "no frame source available".
    """
    try:
        idx = recorder.header().index("lift_height")
    except Exception:                                                  # noqa: BLE001
        idx = None

    def latest() -> Optional[float]:
        if idx is None:
            return None
        rows = recorder.rows(side)
        if not rows:
            return None
        return float(rows[-1][idx]) / per.LIFT_UNITS_PER_METRE

    return latest


def frames_from_series(series) -> Callable[[], Optional[float]]:
    """A replay frame source over a recorded ``[[t_s, height_mm], ...]`` series.

    This is how a hardware pole trace becomes an offline test: the source
    answers with the height the real pole was at, in order, one per call. The
    last value repeats forever so a caller that keeps polling after arrival sees
    a settled pole rather than a source that dies.
    """
    vals = [float(v) / 1000.0 for _, v in series] or [None]
    state = {"i": 0}

    def latest() -> Optional[float]:
        i = state["i"]
        state["i"] = min(i + 1, len(vals) - 1)
        return vals[i]

    return latest


# ── plan-derived facts ───────────────────────────────────────────────────────
_REST_CACHE: Dict[str, Optional[dict]] = {}


def _repo() -> pathlib.Path:
    from alix_dispatch.ladder import repo_root
    return repo_root()


def rest_for(task: dict, repo: Optional[pathlib.Path] = None) -> Optional[dict]:
    """The plan's own ``move_to_rest`` pose for *task*, cached per plan path.

    Read from the PLAN, not the manifest, so no re-emit is needed and the tracks
    on the controller stay exactly as uploaded (this is `cleaning_tracks.rest_for`
    lifted verbatim in behaviour). Rest is unique per side across all 24 plans,
    which is what makes the M42 holding state well defined.
    """
    key = task.get("plan")
    if not key:
        return None
    if key not in _REST_CACHE:
        try:
            root = pathlib.Path(repo) if repo else _repo()
            _REST_CACHE[key] = per.rest_pose(json.loads((root / key).read_text()))
        except Exception:                                              # noqa: BLE001
            _REST_CACHE[key] = None
    return _REST_CACHE[key]


# ── opening the devices ──────────────────────────────────────────────────────
def open_devices(h, *, log: Callable[[str], None] = print,
                 speed_set: int = per.HAND_SPEED_SET_MAX):
    """Open the pole and the hand on *h*. Returns ``(pole, hand, result)``.

    `cleaning_tracks._flow_body` raises ``SystemExit`` here. A dual executive
    cannot: this returns a refusal with ``pole`` still usable, because the pole
    needs no session and losing the hand must not also lose the pole stop.
    """
    out = _result(getattr(h, "side", "?"))
    pole = per.Pole(h.robot, log=log)
    hand = per.HandModbus(h.robot, getattr(h, "side", "right"), log=log)
    ret = _guard("hand modbus open", hand.open, int(speed_set), default=-1, log=log)
    if ret != 0:
        _refuse(out, REFUSE_HAND_MODBUS,
                f"** {out['side']}: hand modbus mode unavailable (ret={ret}) — the hand "
                f"cannot be commanded OR read, so nothing may be staged on this arm",
                log)
        return pole, None, out
    out["ok"] = True
    return pole, hand, out


# ── where the pole actually is ───────────────────────────────────────────────
def pole_here(pole, frame_source: Optional[Callable[[], Optional[float]]] = None):
    """``(height_m, source)`` — the pole's CURRENT height, frames preferred.

    B6: ``rm_get_lift_state`` led the physical pole by 73 mm six milliseconds
    after a command, so "is the pole already at the plan's height?" must not be
    decided on it when a push frame is available.
    """
    if frame_source is not None:
        try:
            fh = frame_source()
        except Exception:                                              # noqa: BLE001
            fh = None
        if fh is not None:
            return float(fh), "frames"
    try:
        return pole.height_m(), "getter"
    except Exception:                                                  # noqa: BLE001
        return None, "getter"


def pole_channels(pole, frame_source: Optional[Callable[[], Optional[float]]] = None):
    """``(height_m, source, getter_m, frames_m, agree)`` — BOTH channels.

    `pole_here` answers "where is the pole" and prefers the frames. That is the
    right preference DURING a move, where the getter leads. It is not enough
    BEFORE one: the decision "this command is a no-op, skip it" is taken on
    this reading, and a stale frame source taking it wrongly skips a move the
    plan needs — the B5 mis-stage, arrived at through the B6 fix. So when both
    channels answer, they must agree to within the pole's own 5 mm goal
    tolerance; when they do not, neither is believed and the caller refuses.

    Which one is stale is genuinely unresolvable from here: the getter echoes
    the last SETPOINT (so an aborted or stopped move leaves it reading a height
    the pole never reached — exactly the state `stop_devices` creates), and a
    recorder that has not rolled over to this task leaves the frames reading the
    previous one. ``agree`` is None when only one channel answered.
    """
    fm = None
    if frame_source is not None:
        fm = _guard("pole frame source", frame_source, default=None,
                    log=lambda *_: None)
        fm = None if fm is None else float(fm)
    gm = _guard("pole height getter", pole.height_m, default=None,
                log=lambda *_: None) if pole is not None else None
    agree = None if (fm is None or gm is None) else (abs(fm - gm) <= per.LIFT_TOL_M)
    height = fm if fm is not None else gm
    return height, ("frames" if fm is not None else "getter"), gm, fm, agree


# ── the staging call ─────────────────────────────────────────────────────────
def stage_arm(h, task: dict, *, pole=None, hand=None,
              log: Callable[[str], None] = print,
              verify_from_frames: Optional[Callable[[], Optional[float]]] = None,
              prev: Optional[dict] = None, prev_parked: bool = False,
              release_counts: Optional[List[int]] = None,
              release_source: Optional[str] = None,
              first_row_millideg=None,
              start_tolerance_deg: float = START_TOLERANCE_DEG,
              retreat_speed: int = 20, pole_speed_pct: int = 100,
              block_pole: bool = False, at_rest_timeout_s: float = 5.0,
              require_hand_assert: bool = True,
              require_frames: bool = False) -> dict:
    """Put ONE arm's devices into the state *task* was planned with.

    The sequence is `cleaning_tracks.before_run`'s, unchanged in order and in
    the reasons for each step:

      1. **retreat** along the PREVIOUS task's own first post-stroke arm move —
         skipped when the previous task parked at rest (the arm is already clear
         and retreating would walk it back toward the surface it just left).
      2. **hand open** to the plan's own release pose, but only when the pole is
         about to move: fingers on the tool while the base column translates is
         the thing this step exists to prevent.
      3. **F9 interlock** — ``track.wait_until_at_rest``: a lift command landing
         while a planned arm move is in flight truncates BOTH (hardware
         2026-08-07). Measured over 68 runs the arm never moved during a pole
         travel, but this is the one place the hazard is reachable.
      4. **pole** to the plan's height, arrival verified from *verify_from_frames*
         when it is supplied (B6).
      5. **hand** to the task's ``pre_stroke`` counts, and READ BACK (H33). Row
         11's 35.5 mm clearance rests on a hand seed nobody commands; the same
         arm poses read 42.03 mm grasped, 29.08 half-closed and 8.52 mm fully
         open, and fully open (993 counts) is what an uncommanded dual run
         inherits. A command result is NOT a verification: if ANGLE_ACT cannot
         be read back, ``hand.asserted`` is False and this refuses.
      6. **the gap** to the track's first point, MEASURED and reported. The
         bridge itself stays in ``track.run_verified``.

    *require_frames* gates the run on the push channel actually answering: with
    it set, a pole that only the getter says arrived is a REFUSAL. It is OFF by
    default so `cleaning_tracks flow`, which supplies no recorder, keeps the
    behaviour that ran 12/12; a dual executive that starts a recorder before
    staging should turn it on. *require_hand_assert* likewise: with it set (the
    default) a task that declares a hand state and has no modbus session is
    refused rather than staged blind.

    *release_counts* is the hand pose to open to before a pole move. Pass the
    plan's own release stage or the SRDF `release` state; when neither is given
    and the plan authors none, ``peripherals.HAND_OPEN`` is used and
    ``steps`` says so by name — the old code called that "the plans' own
    choreography" on 6 of 12 right tasks, which was false.
    """
    side = getattr(h, "side", task.get("side", "?"))
    out = _result(side)
    ps = task.get("pre_stroke") or {}
    out["pole"]["target_m"] = ps.get("pole_m")
    out["hand"]["target"] = list(ps["hand_counts"]) if ps.get("hand_counts") else None

    if pole is None and hand is None:
        return _refuse(out, REFUSE_NO_DEVICES,
                       f"** {side}: stage_arm called with neither a pole nor a hand — "
                       f"nothing can be staged, and pretending otherwise is how an arm "
                       f"replays at the previous task's height", log)
    # A task that declares a device state and a stager that cannot reach that
    # device is NOT a successful stage. Returning ok here is the same lie as
    # not staging at all: the caller reads ok, arms the barrier, and the arm
    # replays at whatever the previous task left (B5). Refuse by name.
    if pole is None and ps.get("pole_m") is not None:
        return _refuse(out, REFUSE_POLE_UNSTAGED,
                       f"** {side}: this task was planned with the pole at "
                       f"{float(ps['pole_m']) * 1000:.0f} mm and no pole object was "
                       f"supplied — the height cannot be set OR read, so this arm is "
                       f"not staged", log)
    if hand is None and ps.get("hand_counts") and require_hand_assert:
        return _refuse(out, REFUSE_HAND_UNSTAGED,
                       f"** {side}: this task was planned with the hand "
                       f"'{ps.get('hand_state')}' {list(ps['hand_counts'])} and there is "
                       f"no modbus session — the hand can be neither commanded nor read, "
                       f"and 'fully open' is what an uncommanded dual run inherits "
                       f"(H33)", log)

    # ---- 1. retreat ---------------------------------------------------------
    if prev is not None and prev_parked:
        out["steps"].append({"step": "retreat", "skipped": "previous task parked at rest"})
        log("   retreat: skipped — the previous task parked at rest")
    elif prev is not None and prev.get("retreat_deg"):
        log(f"   retreat: movej to {prev.get('name')}'s {prev.get('retreat_stage')} pose "
            f"at speed {retreat_speed} (the plan's own way out; rm move, "
            f"collision-UNchecked)")
        ret = int(_guard("retreat movej", h.robot.rm_movej,
                         [float(x) for x in prev["retreat_deg"]],
                         int(retreat_speed), 0, 0, 1, default=-1, log=log))
        out["steps"].append({"step": "retreat", "ret": ret,
                             "stage": prev.get("retreat_stage")})
        if ret != 0:
            return _refuse(out, REFUSE_RETREAT,
                           f"** {side}: retreat movej REFUSED ret={ret} — the arm is "
                           f"still in the previous fixture, so nothing may move", log)

    # ---- 2-4. the pole ------------------------------------------------------
    want = ps.get("pole_m")
    if pole is not None and want is not None:
        here, src, gm, fm, agree = pole_channels(pole, verify_from_frames)
        out["pole"]["found_m"] = here
        if agree is False:
            out["steps"].append({"step": "pole", "getter_m": gm, "frames_m": fm,
                                 "agree": False})
            return _refuse(out, REFUSE_POLE_AMBIGUOUS,
                           f"** {side}: the height getter reads {gm * 1000:.0f} mm and "
                           f"the push frames read {fm * 1000:.0f} mm BEFORE any command "
                           f"— the pole's position is unknown, so neither moving it nor "
                           f"calling it already-there is defensible", log)
        change = here is None or abs(here - float(want)) > per.LIFT_TOL_M
        if not change:
            out["pole"]["verified_by"] = "no_op"
            out["steps"].append({"step": "pole", "no_op": True, "source": src,
                                 "found_m": here})
            log(f"   pole: already at {here * 1000:.0f} mm, the plan's "
                f"{float(want) * 1000:.0f} mm (within {per.LIFT_TOL_M * 1000:.0f} mm, "
                f"read from the {src})")
        else:
            # 2. open the hand BEFORE the column moves
            if hand is None:
                # `_flow_body` refuses at startup when the modbus port will not
                # open, so a pole has NEVER moved on this robot without a hand
                # session behind it. Keep that invariant: fingers on the tool
                # while the base column translates is what step 2 prevents.
                return _refuse(out, REFUSE_HAND_RELEASE,
                               f"** {side}: the pole must move "
                               f"{abs((here or 0.0) - float(want)) * 1000:.0f} mm but "
                               f"there is no hand session to open the fingers first — "
                               f"refusing to translate the column", log)
            rel, rsrc = release_counts, release_source
            if rel is None:
                rel = ps.get("hand_release_counts")
                rsrc = "the plan's own release stage"
            if rel is None:
                rel = list(per.HAND_OPEN)
                rsrc = ("peripherals.HAND_OPEN — this plan authors no release "
                        "stage and the caller named none")
            log(f"   hand: OPEN before the pole moves — from {rsrc}")
            r = _guard("hand release command", hand.command, rel,
                       default={"ok": False, "partial": False, "write": None,
                                "target": list(rel), "after": None}, log=log)
            out["steps"].append({"step": "hand_open", "source": rsrc, "result": r})
            if not r["ok"]:
                if r.get("partial"):
                    out["half_staged"]["hand"] = _hand_half(r)
                return _refuse(out, REFUSE_HAND_RELEASE,
                               f"** {side}: the hand did not open before the pole "
                               f"move — refusing to translate the column with the "
                               f"fingers where they are", log)
            # 3. F9 interlock
            from alix_dispatch import track as _trk
            rest = _guard("wait_until_at_rest", _trk.wait_until_at_rest, h.robot,
                          timeout_s=float(at_rest_timeout_s),
                          default={"at_rest": False, "peak_deg_s": float("nan")},
                          log=log) or {"at_rest": False, "peak_deg_s": float("nan")}
            out["steps"].append({"step": "at_rest", "result": rest})
            if not rest.get("at_rest"):
                return _refuse(out, REFUSE_ARM_MOVING,
                               f"** {side}: the arm is still moving "
                               f"({rest.get('peak_deg_s', float('nan')):.1f} deg/s) — "
                               f"refusing to move the pole into it (F9)", log)
            # 4. the pole itself
            r = _guard("pole move", pole.move, float(want),
                       speed_pct=int(pole_speed_pct), block=block_pole,
                       frame_source=verify_from_frames,
                       default={"ok": False, "ret": None, "t_s": 0.0,
                                "verified_by": None, "frames": [], "final_m": None,
                                "frames_final_m": None}, log=log)
            out["steps"].append({"step": "pole", "result": r})
            out["pole"]["verified_by"] = r.get("verified_by")
            if r.get("frames_missing"):
                # The caller PROMISED a 5 ms push channel and it never yielded,
                # so this arrival was judged on the getter — the channel that
                # led the physical pole by 73 mm (B6). Say so in the steps
                # whatever happens; refuse only if the caller asked to be
                # gated on it, because the single-arm flow supplies no
                # recorder and must keep working exactly as it did.
                out["steps"].append({"step": "pole_frames_missing",
                                     "verified_by": r.get("verified_by")})
                log(f"   ** {side}: the pole arrival was verified on the GETTER — a "
                    f"frame source was supplied but never yielded a height")
                if require_frames and r.get("ok"):
                    out["pole"]["found_m"] = (r.get("frames_final_m")
                                              if r.get("frames") else r.get("final_m"))
                    out["half_staged"]["pole"] = (
                        f"commanded to {float(want) * 1000:.0f} mm and reported arrived "
                        f"on the getter only")
                    return _refuse(out, REFUSE_POLE_ARRIVE,
                                   f"** {side}: require_frames is set and the push "
                                   f"channel never answered — a getter arrival is not "
                                   f"a verification (B6)", log)
            out["pole"]["travel_s"] = round(float(r.get("t_s") or 0.0), 3)
            found = r.get("frames_final_m") if r.get("frames") else r.get("final_m")
            out["pole"]["found_m"] = found
            if not r["ok"]:
                out["half_staged"]["pole"] = (
                    f"commanded to {float(want) * 1000:.0f} mm, "
                    f"{'unverified' if found is None else f'{found * 1000:.0f} mm'} "
                    f"after {r.get('t_s', 0.0):.2f} s "
                    f"(verified_by={r.get('verified_by')}"
                    + (", the push channel and the height getter DISAGREED"
                       if r.get("channels_disagree") else "") + ")")
                code = REFUSE_POLE_SEND if r.get("ret") not in (0, None) else REFUSE_POLE_ARRIVE
                return _refuse(out, code,
                               f"** {side}: the pole did not reach its height — "
                               f"{out['half_staged']['pole']}", log)
    elif pole is not None:
        # This task declares no pole height. Record where the pole IS anyway —
        # the run record is the only place the executive can later answer "what
        # height did this replay actually happen at", and `state["pole_m"]`
        # carrying None is how a task inherits an unrecorded height.
        here, src, _gm, _fm, agree = pole_channels(pole, verify_from_frames)
        out["pole"]["found_m"] = here
        out["pole"]["verified_by"] = None if here is None else src
        out["steps"].append({"step": "pole", "skipped": "this task declares no pole "
                                                        "height", "found_m": here,
                             "source": src, "agree": agree})

    # ---- 5. the hand, commanded AND asserted (H33) --------------------------
    counts = ps.get("hand_counts")
    if hand is not None and counts:
        log(f"   hand: {ps.get('hand_state')} (the state the stroke was planned with)")
        r = _guard("hand command", hand.command, list(counts),
                   default={"ok": False, "partial": False, "write": None,
                            "target": list(counts), "after": None}, log=log)
        out["steps"].append({"step": "hand", "state": ps.get("hand_state"), "result": r})
        read = r.get("after")
        if read is None:
            read = _guard("hand ANGLE_ACT read", hand.read, default=None, log=log)
        out["hand"]["read"] = list(read) if read else None
        if not r["ok"]:
            if r.get("partial"):
                out["half_staged"]["hand"] = _hand_half(r)
            return _refuse(out, REFUSE_HAND_COMMAND,
                           f"** {side}: the hand did not settle at "
                           f"{ps.get('hand_state')} {list(counts)}", log)
        if read is None:
            out["hand"]["asserted"] = False
            if require_hand_assert:
                return _refuse(out, REFUSE_HAND_UNVERIFIED,
                               f"** {side}: the hand was COMMANDED to {list(counts)} but "
                               f"ANGLE_ACT could not be read back — a command result is "
                               f"not a verification (H33), and an unverified hand is what "
                               f"row 11's 35.5 mm rests on", log)
        else:
            asserted, miss, band = hand_asserted(read, counts)
            out["hand"]["asserted"] = asserted
            out["steps"].append({"step": "hand_assert", "miss": miss, "band": band,
                                 "asserted": asserted})
            if not asserted and require_hand_assert:
                worst = max(range(len(miss)), key=lambda i: miss[i] - band[i])
                return _refuse(out, REFUSE_HAND_UNVERIFIED,
                               f"** {side}: the hand read back {list(read)} against a "
                               f"commanded {list(counts)} — channel {worst + 1} is "
                               f"{miss[worst]} counts out, past its {band[worst]}"
                               f"-count staging band", log)
    elif counts:
        out["steps"].append({"step": "hand", "skipped": "no hand session"})
    else:
        # No hand state in the task at all. `asserted` stays False and says so:
        # an unasserted hand is a fact about this run, not a defect to hide.
        out["steps"].append({"step": "hand",
                             "skipped": "this task declares no pre_stroke hand_counts"})

    # ---- 6. the gap to the track's first point (measured, not moved) --------
    _measure_gap(h, out, first_row_millideg, start_tolerance_deg, log)
    out["ok"] = True
    return out


def _hand_half(r: dict) -> str:
    wr = r.get("write") or {}
    landed = [i + 1 for i, ok in enumerate(wr.get("landed") or []) if ok]
    return (f"ANGLE_SET {r.get('target')} landed on channel(s) {landed or 'none'} "
            f"of {per.HAND_DOF} (bulk ret {wr.get('bulk_ret')}, per-register "
            f"{wr.get('rets')}); readback {r.get('after')}")


def _measure_gap(h, out: dict, first_row_millideg, tol_deg: float,
                 log: Callable[[str], None]) -> None:
    if not first_row_millideg:
        return
    from alix_dispatch import track as _trk
    gap = _guard("start_gap_deg", _trk.start_gap_deg, h.robot, first_row_millideg,
                 default=None, log=log)
    out["arm"]["gap_deg"] = None if gap is None else round(float(gap), 3)
    out["arm"]["at_start"] = None if gap is None else bool(gap <= float(tol_deg))
    if gap is not None:
        log(f"   arm: {gap:.2f} deg from the track's first point "
            f"(tolerance {tol_deg:.1f} deg) — the bridge is run_verified's, not "
            f"staging's")


# ── the post-stage gate (check_devices_as_found, upgraded) ───────────────────
def verify_staged(h, task: dict, *, pole=None, hand=None,
                  log: Callable[[str], None] = print,
                  verify_from_frames: Optional[Callable[[], Optional[float]]] = None,
                  first_row_millideg=None,
                  start_tolerance_deg: float = START_TOLERANCE_DEG,
                  read_hand: bool = True, require_at_start: bool = False,
                  require_hand_assert: bool = False) -> dict:
    """What state the devices are ACTUALLY in, versus what *task* was planned
    with. Read-only and side-effect free — and specifically, it never switches
    controller modes: the hand is read only when an ALREADY-OPEN modbus session
    is handed in, which is why `hand_verified` is false on 41 of 41 hardware
    records and why this reports ``asserted: False`` rather than inventing one.
    """
    side = getattr(h, "side", task.get("side", "?"))
    out = _result(side)
    ps = task.get("pre_stroke") or {}
    want = ps.get("pole_m")
    counts = ps.get("hand_counts")
    out["pole"]["target_m"] = want
    out["hand"]["target"] = list(counts) if counts else None

    found, src, gm, fm, agree = (None, None, None, None, None)
    if pole is not None:
        found, src, gm, fm, agree = pole_channels(pole, verify_from_frames)
    out["pole"]["found_m"] = found
    # `verified_by` names the channel that ANSWERED. With no pole object nothing
    # answered, and labelling that "getter" would put a channel name next to a
    # None in the sidecar lane E writes.
    out["pole"]["verified_by"] = src if found is not None else None
    mismatch = False
    if agree is False:
        mismatch = True
        out["lines"].append(
            f"   ** devices AMBIGUOUS: the height getter reads {gm * 1000:.0f} mm and "
            f"the push frames read {fm * 1000:.0f} mm — the pole's position is not "
            f"known, which is not the same as being at the planned height")
    if want is None:
        out["lines"].append("   devices: this task declares no pole height to compare")
    elif found is None:
        # An unreadable device is NOT a passing check. `check_devices_as_found`
        # returned a clean result here and the console said nothing either way,
        # which is how 20 of 23 runs on 2026-09-02 replayed at the previous
        # task's height (F112).
        mismatch = True
        out["lines"].append(
            f"   ** devices UNREADABLE: this task was planned at "
            f"{float(want) * 1000:.0f} mm and the pole height cannot be read — "
            f"that is a refusal, not a pass")
    else:
        delta = abs(found - float(want))
        if delta > per.LIFT_TOL_M:
            mismatch = True
            out["lines"].append(
                f"   ** devices NOT SET: the pole is at {found * 1000:.0f} mm but this "
                f"task was planned at {float(want) * 1000:.0f} mm "
                f"({delta * 1000:.0f} mm off, read from the {src})")
        else:
            out["lines"].append(
                f"   devices: pole at {found * 1000:.0f} mm, matches the plan's "
                f"{float(want) * 1000:.0f} mm (read from the {src})")

    if hand is not None and read_hand and counts:
        read = _guard("hand ANGLE_ACT read", hand.read, default=None, log=log)
        out["hand"]["read"] = list(read) if read else None
        if read is None:
            out["lines"].append("   ** devices: ANGLE_ACT unreadable — the hand is "
                                "UNVERIFIED since staging, not verified-good")
            if require_hand_assert:
                mismatch = True
        else:
            asserted, miss, _band = hand_asserted(read, counts)
            out["hand"]["asserted"] = asserted
            if not asserted:
                mismatch = True
                out["lines"].append(
                    f"   ** devices NOT SET: the hand reads {list(read)} against the "
                    f"planned {list(counts)} (worst {max(miss)} counts)")
            else:
                out["lines"].append(
                    f"   devices: hand at {list(read)}, the planned "
                    f"'{ps.get('hand_state')}' within {max(miss)} counts")
    elif counts:
        out["lines"].append(
            f"   devices: hand should be '{ps.get('hand_state')}' {list(counts)} — NOT "
            f"verified (reading it needs the end-port Modbus mode)")
        # `hand_verified` is false on 41 of 41 hardware records for exactly this
        # reason, and the design asks for it recorded HONESTLY rather than for
        # P1's command result replayed as a P2 verification. So `asserted` stays
        # False and the gate is the caller's to set: OFF keeps the lifted
        # behaviour, ON is what a dual executive — which opens a session on both
        # sides anyway — should use.
        if require_hand_assert:
            mismatch = True

    _measure_gap(h, out, first_row_millideg, start_tolerance_deg, log)
    # `require_at_start` is OFF by default and must stay off until the streamed,
    # checked move_to_start leg exists: flipping it alone converts 100 % of rows
    # into `refused_not_at_start`, and in a dual row that is one arm refusing
    # while its partner is armed (DESIGN §3.2). It is here so the executive can
    # turn it on in ONE place once that leg lands.
    if require_at_start and out["arm"]["at_start"] is not True:
        gap = out["arm"]["gap_deg"]
        out["lines"].append(
            f"   ** {side}: the arm is "
            f"{'an unknown distance' if gap is None else f'{gap:.2f} deg'} from the "
            f"track's first point, past the {start_tolerance_deg:.1f} deg tolerance")
        for ln in out["lines"]:
            log(ln)
        out["refusal"] = REFUSE_NOT_AT_START
        out["ok"] = False
        return out
    for ln in out["lines"]:
        log(ln)
    if mismatch:
        out["refusal"] = REFUSE_DEVICES_AS_FOUND
        out["ok"] = False
    else:
        out["ok"] = True
    return out


# ── parking ──────────────────────────────────────────────────────────────────
def park_arm(h, task: dict, *, log: Callable[[str], None] = print,
             rest_speed: int = 20, restore_speed: int = 50,
             rest_deg: Optional[List[float]] = None,
             repo: Optional[pathlib.Path] = None) -> dict:
    """Park the arm at the PLAN's own ``move_to_rest`` pose after the replay.

    Newton, 2026-09-04: a replay should not leave the arm wherever the stroke
    ended. The pose is read from the plan (`peripherals.rest_pose`), never
    invented. Same trap as the transit: ``v`` is a percent of the PERSISTENT
    global ratio, so the park must set it too or it inherits the last value —
    hence the guard and the explicit 100 (`cleaning_tracks.park_at_rest`).
    """
    side = getattr(h, "side", task.get("side", "?"))
    out = _result(side)
    rp = {"stage": "caller", "deg": list(rest_deg)} if rest_deg else rest_for(task, repo)
    if rp is None:
        out["steps"].append({"step": "park", "skipped": "the plan has no move_to_rest"})
        return _refuse(out, REFUSE_NO_REST_POSE,
                       f"   rest: {side}: the plan has no move_to_rest stage — the arm "
                       f"stays where the stroke left it", log)
    from alix_dispatch import track as _trk
    log(f"   rest: movej to the plan's {rp['stage']} at speed {rest_speed} of a 100 % "
        f"global ratio")
    def _park():
        # the guard takes a `log` (M48) — two arms restoring the same percentage
        # to stdout is one arm's line appearing in the other's transcript
        with _trk.global_speed_guard(h.robot, int(restore_speed), log=log):
            _trk.set_global_speed(h.robot, 100)
            return int(h.robot.rm_movej([float(x) for x in rp["deg"]],
                                        int(rest_speed), 0, 0, 1))

    ret = int(_guard("rest movej", _park, default=-1, log=log))
    out["steps"].append({"step": "park", "stage": rp["stage"], "deg": rp["deg"],
                         "ret": ret, "speed": int(rest_speed)})
    if ret != 0:
        return _refuse(out, REFUSE_REST_MOVE,
                       f"** {side}: rest movej REFUSED ret={ret}", log)
    out["ok"] = True
    return out


# ── the stop (M41) ───────────────────────────────────────────────────────────
def stop_devices(h, *, pole=None, hand=None, log: Callable[[str], None] = print,
                 escalate: bool = True, hard: bool = False,
                 verify_from_frames: Optional[Callable[[], Optional[float]]] = None
                 ) -> dict:
    """Stop the arm, the pole AND the hand on one controller — M41.

    Today a watchdog trip calls ``rm_set_arm_slow_stop`` and nothing else, so a
    commanded pole keeps travelling for up to 2.6 s (measured, L49) and ends in
    an unrecorded mixed-pole state; on row 3 that is the 8.5 mm configuration.
    The C++ side has had the right sequence since ``rm_side_system.cpp:766-780``
    — ``arm_stop``, ``lift_stop``, ``hand_freeze``, and the promise is satisfied
    only AFTER all three have been ISSUED. The ORDER here is that order; the
    ARM VERB is not the same one, deliberately, and the difference matters:

    * ``hard=False`` (the default) issues ``rm_set_arm_slow_stop``, which is
      what `dual_arm_flow.py:226` issues today and what M41's own wording asks
      for. It decelerates ON the current path, which is the right thing while a
      hand is in contact with a fixture. It escalates to ``rm_set_arm_stop``
      only if it comes back non-zero, rather than reporting a stop that never
      happened.
    * ``hard=True`` issues ``rm_set_arm_stop`` first — the verb
      ``RmConnection::arm_stop`` (`rm_connection.cpp:309-316`) actually calls,
      which drops the firmware-latched target so healthy joints stop driving.
      That is what an E-STOP path wants. It cannot be exercised on hardware
      today, so it is a keyword rather than a default: whichever the caller
      picks, the verb that was issued is in the result.

    "Stopped" means every device was issued a stop AND returned 0. A device that
    could not be reached is reported, never assumed: ``stopped`` False with the
    per-device block saying which. Never raises.
    """
    side = getattr(h, "side", "?")
    out = _result(side)
    t0 = time.monotonic()
    order: List[str] = []
    stop = {"arm": None, "pole": None, "hand": None, "order": order,
            "stopped": False, "elapsed_s": None}
    out["stop"] = stop

    # 1. the arm — slow_stop decelerates ON the current path; escalate on a bad
    #    return rather than reporting a stop that never happened.
    arm = {"verb": "rm_set_arm_stop" if hard else "rm_set_arm_slow_stop",
           "hard": bool(hard), "ret": None, "escalated": None,
           "escalated_ret": None, "ok": False}
    from alix_dispatch import track as _trk
    if hard:
        arm["ret"] = int(_guard(f"{side} rm_set_arm_stop", h.robot.rm_set_arm_stop,
                                default=-1, log=log))
    else:
        # `log` is `_guard`'s own keyword, so slow_stop's has to be bound here
        # or the stop's console lines go to stdout instead of the run log.
        arm["ret"] = int(_guard(f"{side} slow stop",
                                lambda: _trk.slow_stop(h.robot, log=log),
                                default=-1, log=log))
    arm["ok"] = arm["ret"] == 0
    if not arm["ok"] and escalate and not hard:
        arm["escalated"] = "rm_set_arm_stop"
        arm["escalated_ret"] = int(_guard(f"{side} rm_set_arm_stop",
                                          h.robot.rm_set_arm_stop, default=-1,
                                          log=log))
        arm["ok"] = arm["escalated_ret"] == 0
    stop["arm"] = arm
    order.append("arm")

    # 2. the pole
    p = pole if pole is not None else per.Pole(h.robot, log=log)
    stop["pole"] = _guard(f"{side} pole stop", p.stop, log=log, default=None) or {
        "ok": False, "ret": None, "verb": "rm_set_lift_speed(0)", "height_m": None,
        "error": "the pole stop raised — the column may still be travelling"}
    order.append("pole")

    # 3. the hand, frozen at its MEASURED angle
    if hand is not None:
        stop["hand"] = _guard(f"{side} hand freeze", hand.freeze, log=log,
                              default=None) or {
            "ok": False, "issued": False, "ret": None, "counts": None, "write": None,
            "why": "the hand freeze raised"}
    else:
        stop["hand"] = {"ok": False, "issued": False, "ret": None, "counts": None,
                        "write": None,
                        "why": ("no hand session was handed to stop_devices — a stop "
                                "must not switch controller modes, so the hand is NOT "
                                "frozen")}
        log(f"   ** {side}: {stop['hand']['why']}")
    order.append("hand")

    stop["elapsed_s"] = round(time.monotonic() - t0, 3)
    stop["stopped"] = bool(arm["ok"] and stop["pole"]["ok"] and stop["hand"]["ok"])
    # WHERE THE POLE ENDED UP. The getter is the channel that lies (B6), so when
    # a push frame source is available the recorded stop height comes from it —
    # otherwise "the pole stopped at 150 mm" would be a setpoint echo written
    # into the abort record, which is the one record that has to be true.
    height, hsrc = pole_here(p, verify_from_frames)
    stop["pole"]["height_m"] = height
    stop["pole"]["height_source"] = hsrc
    out["pole"]["found_m"] = height
    out["pole"]["verified_by"] = "frames" if hsrc == "frames" else "getter"
    out["hand"]["read"] = stop["hand"].get("counts")
    out["ok"] = stop["stopped"]
    if not stop["stopped"]:
        failed = [k for k in ("arm", "pole", "hand") if not (stop[k] or {}).get("ok")]
        out["refusal"] = REFUSE_STOP_UNVERIFIED
        out["half_staged"] = {k: str((stop[k] or {}).get("why")
                                     or (stop[k] or {}).get("error")
                                     or f"ret={(stop[k] or {}).get('ret')}")
                              for k in failed}
        log(f"   ** {side}: STOP INCOMPLETE — {', '.join(failed)} not confirmed stopped "
            f"after {stop['elapsed_s']:.2f} s")
    else:
        log(f"   {side}: HALT issued to all devices in {stop['elapsed_s']:.2f} s "
            f"({arm['verb']}, rm_set_lift_speed(0), hand freeze at "
            f"{stop['hand'].get('counts')})")
    return out


# ── the half-staged holding state (M42) ──────────────────────────────────────
def recover_arm(h, task: dict, *, pole=None, hand=None,
                log: Callable[[str], None] = print,
                verify_from_frames: Optional[Callable[[], Optional[float]]] = None,
                park: bool = True, rest_speed: int = 20, restore_speed: int = 50,
                repo: Optional[pathlib.Path] = None) -> dict:
    """M42's defined holding state, for ONE arm. Never retries, never advances.

    The finding asks for it in words: "stop both arms, stop both poles, park
    both at ``move_to_rest``, do NOT retry, do NOT advance, write both poles
    and hands into a ``half_staged`` block". The *both* is the executive's —
    it calls this on each handle — and the per-arm half is here so the two
    sides cannot drift apart in how they recover. Rest is unique per side
    across all 24 plans, which is what makes the holding state well defined.

    The park is attempted only after the stop has been ISSUED to every device,
    and its refusal is carried rather than raised: an arm that will not park is
    a worse state than one that will, and the operator has to be told which.
    """
    side = getattr(h, "side", task.get("side", "?"))
    out = _result(side)
    stop = stop_devices(h, pole=pole, hand=hand, log=log,
                        verify_from_frames=verify_from_frames)
    out["stop"] = stop.get("stop")
    out["pole"] = dict(stop["pole"])
    out["hand"]["read"] = stop["hand"]["read"]
    ps = task.get("pre_stroke") or {}
    out["pole"]["target_m"] = ps.get("pole_m")
    out["hand"]["target"] = list(ps["hand_counts"]) if ps.get("hand_counts") else None
    out["half_staged"] = dict(stop.get("half_staged") or {})
    # What is actually where, in the block the record has to carry.
    out["half_staged"]["pole_found_m"] = out["pole"]["found_m"]
    out["half_staged"]["hand_read"] = out["hand"]["read"]
    out["park"] = None
    park_refused = False
    # THE PARK IS A `movej`, SO THE PROGRAM MUST BE VERIFIED OVER FIRST
    # (safety review, 2026-09-09). `stop_devices` ISSUES `rm_set_arm_slow_stop`
    # and reads its return code; gate G4 — does any arm verb abort a running
    # uploaded PROGRAM? — has never been measured, so `ret == 0` is not
    # evidence that the replay ended. Parking on top of a program that is still
    # playing is a second command source on one controller. The verified,
    # escalating ladder decides, and a park it will not clear is REFUSED and
    # named rather than issued.
    if park:
        from alix_dispatch import track as _trk2
        # `log` is `_guard`'s OWN keyword, so the ladder's console lines have to
        # be bound here or they go to stdout instead of the run log (M48).
        st = _guard(f"{side} verified stop",
                    lambda: _trk2.stop_program(h.robot, log=log),
                    default={"stopped": False, "rung": None,
                             "refusal": "the verified stop ladder could not be run"},
                    log=log)
        out["stop_verified"] = st
        if not (st or {}).get("stopped"):
            park = False
            park_refused = True
            out["half_staged"]["park"] = (
                "not parked: the program on this arm could not be VERIFIED stopped "
                f"(rung={(st or {}).get('rung')}), and a park movej beside a program "
                "that may still be playing is a second command source")
            log(f"   ** {side}: {out['half_staged']['park']}")
    if park:
        pk = park_arm(h, task, log=log, rest_speed=rest_speed,
                      restore_speed=restore_speed, repo=repo)
        out["park"] = pk
        out["steps"] += pk["steps"]
        if not pk["ok"]:
            out["half_staged"]["park"] = pk["refusal"]
    out["refusal"] = stop.get("refusal") or (
        "park_refused_program_not_stopped" if park_refused else
        None if (out["park"] is None or out["park"]["ok"]) else out["park"]["refusal"])
    # `ok` here means "the holding state was REACHED", never "the run may go on":
    # a recovered arm does not advance, and the caller must not read this as a
    # licence to retry the step that failed.
    out["ok"] = bool(stop["ok"] and not park_refused
                     and (out["park"] is None or out["park"]["ok"]))
    return out


__all__ = [
    "stage_arm", "verify_staged", "park_arm", "stop_devices", "open_devices",
    "recover_arm",
    "pole_here", "rest_for", "recorder_frame_source", "frames_from_series",
    "HAND_ASSERT_COUNTS", "HAND_ASSERT_COUNTS_CH6", "HAND_ASSERT_BAND",
    "hand_asserted", "START_TOLERANCE_DEG",
    "REFUSE_NO_DEVICES", "REFUSE_HAND_MODBUS", "REFUSE_RETREAT",
    "REFUSE_HAND_RELEASE", "REFUSE_ARM_MOVING", "REFUSE_POLE_SEND",
    "REFUSE_POLE_ARRIVE", "REFUSE_HAND_COMMAND", "REFUSE_HAND_UNVERIFIED",
    "REFUSE_DEVICES_AS_FOUND", "REFUSE_NOT_AT_START", "REFUSE_NO_REST_POSE",
    "REFUSE_REST_MOVE", "REFUSE_POLE_UNSTAGED", "REFUSE_HAND_UNSTAGED",
    "REFUSE_POLE_AMBIGUOUS", "REFUSE_STOP_UNVERIFIED", "pole_channels",
]
