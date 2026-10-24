"""Online-program (track upload) transport — the production port of the verbs
proven on hardware by exp01 lanes G/H (F98-F100, 20 recorded runs).

WHAT THIS TRANSPORT IS. ``rm_send_project`` stores a dense joint-point file in
one of the controller's 100 program slots; ``rm_set_program_id_run`` replays it
through the controller's own player at a NOMINAL ~5.55 ms tick — but the tick is
NOT precise: hardware measured 5.51-6.02 ms/pt, +-6% run to run on the SAME
track (F108b). Quote RATIOS, never absolute durations, and gate on completeness
+ fidelity rather than duration. The player enforces no rate limit and does not
smooth (measured: unity gain, no cutoff 2-85 Hz); the file's point SPACING is
the velocity profile, so the +-6% carries straight into wipe SPEED.

POLICY (Newton, 2026-08-31): this channel carries CLEANING STAGES ONLY
(execute_path); transits, mechanism, hand and pole stay on the ROS controllers.
Slots 70-100 are reserved for cleaning tracks, allocated in schedule order —
``Resource/tracks/<fixture>/MANIFEST.json`` is the authority.

HAZARDS THIS MODULE OWNS:
  * CRLF is mandatory — an LF-only upload was rejected with err_line=1
    (2026-08-28, V1.7.4). ``build_project_file`` normalises every terminator.
  * ``plan_speed`` / the run ``speed`` scale ONLY the movej transit to the first
    point AND set the controller's GLOBAL ratio, which SURVIVES POWER CYCLES
    (F98). There is no getter, so it cannot be read back: every run must
    RESTORE it explicitly — use :func:`global_speed_guard`.
  * Slots have no auto-reclaim and survive power cycles.

This module does NOT touch the UDP realtime push (one destination per
controller; reconfiguring it can kill a live driver — see FINDINGS F98).
Connections come from :mod:`alix_dispatch.connect` (one owner per controller).
"""
from __future__ import annotations

import contextlib
import json
import os
import pathlib
import threading
import time
import time as _time
from typing import Callable, Optional

from alix_dispatch import limits as lim
from alix_dispatch import runmode as _runmode

#: The player's nominal clock (F98: 29.167 s / 5251 intervals). NOT a precision
#: spec: hardware runs measured 5.51-6.02 ms/pt and the SAME track replayed
#: +-6% apart on repeat runs (F108, 2026-09-01) while the UDP push cadence held
#: 5.008-5.013 ms — the spread is the player's, not our timing, and the vendor's
#: trajectory-replay speed-variation fix is ALREADY installed (V1.7.4, not
#: V1.7.5 as this repo previously recorded). Judge RATIOS, never absolutes.
REPLAY_CLOCK_S = 0.0055545
#: Newton's original verification band (2026-08-31), +-1% around nominal. Kept
#: for manifest continuity and as an ADVISORY tightness marker; it is narrower
#: than the hardware's own spread, so it is no longer a pass/fail gate — see
#: CLOCK_ENVELOPE_MS and verify_replay_from_frames.
BAND_S_PER_PT = (0.0055, 0.0056)

EOL = b"\r\n"


def expected_band_s(points: int) -> tuple:
    """Pre-registered executed-duration band for an N-point track."""
    return ((points - 1) * BAND_S_PER_PT[0], (points - 1) * BAND_S_PER_PT[1])


def parse_track(path: pathlib.Path) -> list:
    """Read an NDJSON millidegree track -> list of 7-int rows. Refuses on any
    malformed line so a truncated file cannot be half-uploaded."""
    rows = []
    for i, ln in enumerate(path.read_bytes().replace(b"\r\n", b"\n").split(b"\n"), 1):
        if not ln.strip():
            continue
        try:
            row = json.loads(ln)["point"]
        except Exception as exc:                                       # noqa: BLE001
            raise ValueError(f"{path}:{i}: not a point line ({exc})") from exc
        if len(row) != 7 or any(not isinstance(v, int) for v in row):
            raise ValueError(f"{path}:{i}: expected 7 integer millidegrees, got {row!r}")
        rows.append(row)
    if len(rows) < 2:
        raise ValueError(f"{path}: {len(rows)} points — not a track")
    return rows


def build_project_file(track_txt: pathlib.Path, out: pathlib.Path, dof: int = 7) -> dict:
    """Write the uploadable project file: ``{"file":DOF}`` header + Folder line
    whose ``type`` is the POINT COUNT, then the track's own lines verbatim,
    every terminator CRLF (vendor C demo RMDemo_OnlineProgram main.c:81-112;
    LF-only was rejected err_line=1)."""
    rows = parse_track(track_txt)
    n = len(rows)
    raw = track_txt.read_bytes().replace(b"\r\n", b"\n").replace(b"\r", b"\n")
    lines = [ln for ln in raw.split(b"\n") if ln.strip()]
    header = (f'{{"file":{dof}}}'.encode() + EOL +
              f'{{"name":"Folder","num":1,"type":{n},"enabled":true,"parent_number":0}}'.encode() + EOL)
    out.write_bytes(header + EOL.join(lines) + EOL)
    return {"path": str(out), "points": n, "dof": dof, "bytes": out.stat().st_size,
            "expected_band_s": expected_band_s(n)}


def send_project(robot, project_path: pathlib.Path, save_id: int, *,
                 plan_speed: int = 0, only_save: bool = True,
                 project_type: int = 0) -> tuple:
    """Upload. Returns (ret, err_line, seconds). ret=0/err_line=-1 is success.
    only_save=True stores without running — the normal path; running is a
    separate, separately-confirmed step."""
    from Robotic_Arm.rm_robot_interface import rm_send_project_t
    sp = rm_send_project_t(str(project_path), int(plan_speed),
                           1 if only_save else 0, int(save_id), 0, 0, int(project_type))
    t0 = time.monotonic()
    ret, err_line = robot.rm_send_project(sp)
    return int(ret), int(err_line), time.monotonic() - t0


#: Programs per page of the list. The SDK parses each reply from a fixed buffer, and ONE page of 100
#: overran it once the left controller held ~70 programs: the JSON was cut off mid-entry, "Error
#: parsing JSON", ret -3 (2026-09-23, tools/rm_upload.py on the left arm). Ten is ~2 KB a page.
PROGRAM_LIST_PAGE = 10


def program_list(robot) -> tuple:
    """(ret, items). Firmware V1.7.4 keys the list 'trajectory_list' (earlier
    guesses 'tra_list'/'list' kept as fallbacks — exp01, 2026-08-28).

    Read a page of `PROGRAM_LIST_PAGE` at a time (page 1 is the first) until a
    page comes back short, or brings no slot not seen already.

    Each item is the vendor's ``rm_trajectory_data_t``: ``id``, ``size``,
    ``speed``, ``trajectory_name`` — NOT ``name``. This function read ``name``
    until 2026-09-02 (caught by writing the emulator double against the
    vendor struct), so every slot printed as None on hardware. Items are
    returned with a ``name`` alias filled from ``trajectory_name`` so both
    spellings work downstream."""
    out, seen = [], set()
    for page in range(1, 100 // PROGRAM_LIST_PAGE + 2):
        ret, d = robot.rm_get_program_trajectory_list(page, PROGRAM_LIST_PAGE, "")
        if int(ret) != 0:
            return int(ret), out
        items = d.get("trajectory_list") or d.get("tra_list") or d.get("list") or []
        fresh = 0
        for it in items:
            if not isinstance(it, dict):
                continue
            name = it.get("trajectory_name") or it.get("name")
            if (it.get("id") is None and not name) or it.get("id") in seen:
                continue
            seen.add(it.get("id"))
            fresh += 1
            out.append({**it, "name": name})
        if len(items) < PROGRAM_LIST_PAGE or not fresh:
            break
    return 0, out


def set_global_speed(robot, pct: int) -> int:
    if not 1 <= int(pct) <= 100:
        raise ValueError("global speed is 1..100")
    return int(robot.rm_set_plan_speed(int(pct)))


@contextlib.contextmanager
def global_speed_guard(robot, restore_pct: int,
                       log: Callable[[str], None] = print,
                       verdict: Optional[dict] = None):
    """ALWAYS restore the controller's global ratio after a run — there is no
    getter, the value persists across power cycles, and F98 measured a movej
    running 6x slower after a speed-10 program. Restores on ANY exit path.

    M48 (2026-09-08): this used to be the ONLY bare ``print(`` in this module,
    and it took no ``log``. Two arms restoring to the same percentage emit
    BYTE-IDENTICAL success lines, so on the dual path a missing one is
    indistinguishable from a duplicate — and the failure branch went to stderr,
    which the flow's ``_Tee`` does not capture and the dual path does not have.
    The line now goes through the caller's ``log`` (default ``print``, so the
    proven single-arm flow's console output is unchanged apart from the stream)
    and *verdict*, if a dict is passed, is filled in with
    ``{"global_speed_ret", "restore_pct", "restored"}``.

    THE RETURN CODE IS THE ONLY EVIDENCE. There is no getter for the global
    ratio, so a read-back is impossible; ``ret == 0`` is all the controller
    will ever say about it, and it therefore belongs in the run record.
    """
    if verdict is not None:
        verdict.clear()
        verdict.update({"global_speed_ret": None, "restore_pct": int(restore_pct),
                        "restored": None})
    try:
        yield
    finally:
        try:
            ret = set_global_speed(robot, restore_pct)
        except Exception as exc:                                       # noqa: BLE001
            ret = f"raised {exc!r}"
        ok = (ret == 0)
        if verdict is not None:
            verdict.update({"global_speed_ret": ret, "restored": bool(ok)})
        msg = f"global speed restored to {restore_pct} (ret={ret})"
        log(msg if ok else f"** {msg} — RESTORE FAILED, fix from the pendant")


def read_tool_frame(robot):
    """(name, pose_m, payload_kg, centroid_mm, raw_dict) the controller holds, or Nones.

    The RAW dict is carried too, unpicked: hardware spells the centroid as the separate
    `x`/`y`/`z` keys this reader does not look at, and `cleaning_tracks.normalise_tool_frame`
    needs the whole answer to produce a frame comparable with the one a snapshot records.
    """
    try:
        got = robot.rm_get_current_tool_frame()
        fr = got[1] if isinstance(got, tuple) else got
        if not isinstance(fr, dict):
            return (None, None, None, None, None)
        return (fr.get("name"), fr.get("pose"), fr.get("payload"),
                fr.get("centroid") or fr.get("center_of_mass"), fr)
    except Exception:                                                  # noqa: BLE001
        return (None, None, None, None, None)


@contextlib.contextmanager
def tool_frame_guard(robot, want_name: Optional[str],
                     log: Callable[[str], None] = print,
                     verdict: Optional[dict] = None):
    """SELECT the tool frame a certified file was shaped for, and ALWAYS put back
    whatever the pendant had — the same discipline `global_speed_guard` applies to
    the global ratio and `limits.guard` to the Cartesian envelope.

    WHY THIS EXISTS. This channel replays a JOINT-SPACE track, so the tool frame
    cannot change a single commanded number — but the frame's payload feeds gravity
    compensation and the collision stage, so a run is only comparable with runs
    made under the same frame, and the pendant's choice is whatever the last
    session left. Selecting it makes the frame a recorded, deliberate variable.

    CORRECTED 2026-09-17: this was first justified as "the frame decides whether
    a track runs" (slot 43 aborted twice at `L_glove_2` on 2026-09-16, with 7.7x
    the J2 current). That was a J2 FAULT WINDOW (17:38-18:21): every capture in it
    trips 4109 with J2 peaking at 19-22 M at BOTH frames, and slot 43 at
    `L_glove_2` completed normally on 2026-09-17 (J2 0.44 M vs 0.41 M at Arm_Tip).

    Newton offered exactly this branch when he ruled on 2026-09-15 — "we can either
    switch the frame implicitly during the run, or just collect data on the frame
    set, instead of blocking the run" — and collect-data was implemented first, which
    left the frame to whatever the pendant held. Selecting is not blocking: the
    ruling stands, nothing here refuses.

    A BARE SETTER IS NOT ENOUGH. On 2026-08-20 `rm_change_tool_frame("R_glove_2")`
    returned success and read back as `Arm_Tip`, pose all zeros — a silent 180 mm
    error on every waypoint (`backends.verify_tool_frame`). So the selection is
    PROVED by read-back, and a selection that cannot be proved is reported and the
    run goes on under the frame the arm actually holds, recorded.
    """
    as_found = read_tool_frame(robot)
    rec = {"as_found": as_found[0], "wanted": want_name, "selected": None,
           "took": None, "restored": None, "restore_ret": None,
           "as_found_payload_kg": as_found[2]}
    if verdict is not None:
        verdict.clear(); verdict.update(rec)
    need = bool(want_name) and as_found[0] != want_name
    if need and not as_found[0]:
        # THE AS-FOUND FRAME IS THE ONLY WAY BACK. `read_tool_frame` ignores the SDK return, so a
        # `-2` reply timeout reads as a nameless frame; selecting then left the run's frame and
        # payload on the controller with nothing to restore (2026-09-17 review). Run under what the
        # arm holds, recorded — never a refusal (Newton 2026-09-15).
        need = False
        rec["not_selected"] = ("the frame the controller holds could not be read, so a selection "
                               "could not be put back: the run goes on under it, unchanged")
        log(f"** tool frame: cannot read what the controller holds — {want_name!r} NOT selected; "
            f"the run goes on under the frame in place, recorded")
    if need:
        try:
            ret = int(robot.rm_change_tool_frame(want_name))
        except Exception as exc:                                       # noqa: BLE001
            ret = f"raised {exc!r}"
        got = read_tool_frame(robot)
        rec["selected"] = got[0]
        rec["took"] = bool(got[0] == want_name)
        rec["select_ret"] = ret
        # THE WHOLE FRAME, not just its name: `run_record_evidence` groups runs by the frame they ran
        # under, and a name alone cannot be compared with the payload/offsets a snapshot carries. Read
        # back, so this is what the controller HOLDS (2026-09-17 review, finding B1).
        rec["selected_frame"] = got[4]
        if rec["took"]:
            log(f"   tool frame: selected {want_name!r} for this run (was "
                f"{as_found[0]!r}, payload {as_found[2]} -> {got[2]} kg); restored after")
        else:
            log(f"** tool frame: {want_name!r} did NOT take (controller reports "
                f"{got[0]!r}, ret={ret}) — the run goes on under {got[0]!r}, recorded")
    elif want_name:
        rec["selected"] = as_found[0]; rec["took"] = True
        rec["selected_frame"] = as_found[4]
        log(f"   tool frame: already {want_name!r}, nothing to select")
    if verdict is not None:
        verdict.update(rec)
    try:
        yield rec
    finally:
        if need and as_found[0]:
            try:
                r = int(robot.rm_change_tool_frame(as_found[0]))
            except Exception as exc:                                   # noqa: BLE001
                r = f"raised {exc!r}"
            back = read_tool_frame(robot)
            ok = (back[0] == as_found[0])
            rec["restore_ret"] = r
            rec["restored"] = bool(ok)
            msg = f"tool frame restored to {as_found[0]!r} (ret={r})"
            log(f"   {msg}" if ok else
                f"** {msg} — RESTORE FAILED, controller now holds {back[0]!r}; fix from the pendant")
        if verdict is not None:
            verdict.update(rec)


def current_joints_deg(robot):
    """Current joint angles [deg] or None."""
    try:
        ret, st = robot.rm_get_current_arm_state()
        if ret == 0 and st and "joint" in st:
            return [float(x) for x in st["joint"]]
    except Exception:                                                  # noqa: BLE001
        pass
    return None


def start_gap_deg(robot, track_first_row_millideg) -> Optional[float]:
    """Worst per-joint gap between the arm NOW and the track's first point.

    RULING (Newton, 2026-09-01, amended same day): task transits run over CANFD
    high-follow with the MoveIt plan (collision-free, 180 deg/s cap); the rm
    planned primitives are not a silent fallback IN THE TASK FLOW. But for THIS
    situation — the arm away from a track's first point at run time — the
    controller's own movej (which rm_set_program_id_run performs implicitly) is
    the accepted DEFAULT: callers announce the gap and proceed, offering a
    strict mode that refuses and directs the user to stream move_to_start.
    """
    cur = current_joints_deg(robot)
    if cur is None:
        return None
    want = [v / 1000.0 for v in track_first_row_millideg]
    return max(abs(a - b) for a, b in zip(cur, want))


def assert_side(expected_side: str, handle_side: str) -> None:
    """Machine replacement for the CLI's typed-side confirmation (automation
    ruling 2026-09-01): the arm identity flows from the schedule row through
    the manifest to the connection handle, so a mismatch is a wiring bug, not
    an operator question. Hard refusal — never proceed on the wrong arm."""
    if expected_side not in ("left", "right"):
        raise ValueError(f"expected_side must be left/right, got {expected_side!r}")
    if handle_side != expected_side:
        raise ValueError(f"arm mismatch: task expects the {expected_side.upper()} arm "
                         f"but the connection is to the {handle_side.upper()} arm — refusing")


def preflight_slots(robot, rows, repo: Optional[pathlib.Path] = None) -> dict:
    """Bringup preflight (once per session, not per run): the controller's
    program list vs the manifest's EMITTED rows. Slots persist across power
    cycles with no auto-reclaim, so a run must never trust a stale slot.
    The list carries id/name only (no point count on V1.7.4), so this checks
    presence; content equality is established by the upload step itself.

    rows: manifest task rows (dicts with 'slot'/'name'/'status').
    Returns {'ret', 'checked', 'present', 'missing': [slot,...],
             'by_slot': {slot: controller_name_or_None}, 'extra': [ids...]}.
    """
    ret, items = program_list(robot)
    on_ctrl = {int(it["id"]): str(it.get("name") or "") for it in items if it.get("id") is not None}
    sizes = {int(it["id"]): it.get("size") for it in items if it.get("id") is not None}
    emitted = [r for r in rows if r.get("status") == "EMITTED" and r.get("slot") is not None]
    by_slot = {r["slot"]: on_ctrl.get(int(r["slot"])) for r in emitted}
    missing = [s for s, name in by_slot.items() if name is None]
    extra = sorted(i for i in on_ctrl if 70 <= i <= 100 and i not in by_slot)
    # ret=0/err_line=-1 at upload says the controller ACCEPTED the file, not
    # that it kept all of it, and a short slot would end its program early
    # with no error — the F112 signature. rm_trajectory_data_t.size is the
    # only vendor readback of what is actually stored, so compare it against
    # the .proj that was sent whenever that file is still on disk.
    size_check = {}
    for r in emitted:
        slot = int(r["slot"])
        proj = local_proj(r, repo)
        got = sizes.get(slot)
        if proj is None or got is None:
            size_check[slot] = {"local": None, "stored": got, "ok": None}
            continue
        delta = int(got) - proj
        size_check[slot] = {"local": proj, "stored": int(got), "delta": delta,
                            "ok": abs(delta) <= max(64, proj // 1000)}
    return {"ret": ret, "checked": len(emitted), "present": len(emitted) - len(missing),
            "missing": missing, "by_slot": by_slot, "extra": extra,
            "size_check": size_check,
            "size_mismatch": sorted(s for s, v in size_check.items() if v.get("ok") is False)}


def local_proj(row, repo: Optional[pathlib.Path] = None) -> Optional[int]:
    """Byte size of the .proj that `upload` wrote for this row, or None."""
    t = row.get("track")
    if not t:
        return None
    for base in (pathlib.Path(t), (repo / t) if repo else None):
        if base is None:
            continue
        f = base.with_suffix(".proj")
        if f.is_file():
            return f.stat().st_size
    return None


def run_verified(robot, *, save_id: int, band_s, first_row_millideg,
                 points: int, speed: int = 50, restore_pct: int = 50,
                 start_tolerance_deg: float = 2.0, require_at_start: bool = False,
                 log: Callable[[str], None] = print,
                 on_transition: Optional[Callable[[dict], None]] = None,
                 blocking_probe: Optional[bool] = False,
                 movej_speed: Optional[int] = None,
                 caps: Optional[dict] = None,
                 event_log: Optional["ArmEventLog"] = None,
                 replay_verdict_fn: Optional[Callable[[], Optional[dict]]] = None,
                 settle_after: bool = False,
                 monitor: Optional["MotionMonitor"] = None,
                 channel: str = "upload",
                 track_rows_millideg=None,
                 canfd: Optional[dict] = None,
                 canfd_watch: Optional["CanfdWatch"] = None,
                 ratio_held: bool = False) -> dict:
    """The full run policy as a LIBRARY call — no prompts, no SystemExit.

    *ratio_held* (CANFD only): the caller holds the global ratio at *speed*
    already and *restore_pct* equals it, so setting it and putting it back
    around the stream would change nothing; both calls are left out (ALIX
    Studio Lite's lid and seat runs, 2026-10-24: 0.17 s a move). The verdict's
    ``canfd_global_speed`` says ``held`` and ``restored.global_speed`` stays
    empty. Ignored on the upload channel, whose program speed is its own.

    *channel* "canfd" plays *track_rows_millideg* from the host with
    :func:`stream_track` (options in *canfd*: period_s, follow,
    trajectory_mode, radio) instead of starting slot *save_id*; every other
    step below is the same. Its differences, all forced by the transport: the
    lines are checked against the pass-through limits first
    (`refused_canfd_limits`); there is no implicit controller transit, so a
    start gap wider than one frame can close is bridged by the explicit movej
    or refused, and re-measured after the bridge; and no program means no
    probe and no finish event, so completeness is the stream sending every
    line (`not_early`) plus the caller's replay verdict.

    This is what the task executive uses (automation ruling 2026-09-01: the
    run structure is established up front, so there are no mid-flow human
    checks; every protection is a machine gate). Sequence:
      1. start-gap check vs the track's first point — unreadable state refuses;
         a gap above tolerance refuses under require_at_start, else the
         controller's implicit movej bridge is ANNOUNCED and allowed (ruling
         2026-09-01: collision-unchecked, so callers stream move_to_start
         first in the orchestrated flow and this branch should never fire);
      2. the run inside global_speed_guard (restore is mandatory — no getter);
      3. verdict against the pre-registered duration band.

    ``ok`` IS NO LONGER THE DURATION BAND (B1, 2026-09-08). Until today `ok`
    was ``band_lo <= replay <= band_hi``, a +-1 % window around the nominal
    5.5545 ms/pt clock. Counted over the committed ledger
    (``Resource/runs/tracks/``, 2026-09-08): of the **129** runs whose capture
    proves the arm covered its whole file (`complete == true`), **120 are
    OUTSIDE that band** by the record's own `in_advisory_band` — 93.0 %. The
    player varies +-6 % run to run on the SAME bytes (F108), so the band was
    never meetable. On a single arm the flow demoted it by hand
    (`cleaning_tracks.py:1218`); on the DUAL path the first arm to finish would
    read `ok == False`, call it a cut, and `slow_stop` BOTH controllers with
    the partner mid-wipe, pad on the fixture.

    `ok` is now the completeness question, judged on evidence that separates
    perfectly in the ledger:

      * **probe** — `blocking_probe.ret == 0`. Over the 125 probe runs the
        return is a perfect classifier: `(0, complete)` 118 times, `(-4,
        incomplete)` 7 times, no other combination. With the probe OFF the
        stand-in is "no EARLY departure": the replay did not end before
        `band_s[0]`, which is a one-sided test the +-6 % spread cannot trip
        (it can only make a run LONGER than nominal in the direction that
        matters here... a short run is short because it stopped).
      * **finish event** — `program_run_finish` seen on this handle's
        :class:`ArmEventLog`. Over the 94 records carrying `arm_events`:
        `(finish_seen, complete)` is `(True, True)` 90 times and
        `(False, False)` 4 times. Nothing else.
      * **replay verdict** — `replay_verdict(...)["outcome"] == "complete"`,
        supplied by *replay_verdict_fn* because it needs the UDP capture, which
        this call does not own.

    A component that was not measured is `None` and does not veto: `ok` is the
    AND of the components that exist, and `ok_components` in the result says
    which those were, so a record can never imply evidence it does not have.
    The band survives as `in_band` / `in_advisory_band` — reported, never `ok`.
    `verdict` keeps its existing spelling (`in_band` / `out_of_band`) because
    `cleaning_tracks.py:1207` and `task_executive_common.py` both branch on it.

    *blocking_probe* accepts `None` = AUTO (M43): the probe is turned OFF when
    a registered `event_log` is supplied, because the probe is a second
    concurrent SDK caller on the same arm and it induces `-2` reply timeouts on
    the state poller — **173** of them over **84 of 125** probe runs, each one
    blinding the fallback detector ~0.62 s. With the event log registered the
    finish event answers the same question with no second caller and no join.

    Returns {'ok': bool, 'verdict': str, 'gap_deg', 'duration_s',
             'implied_ms_per_pt', 'wrong_id_seen', 'transitions', 'run_ret',
             'in_band', 'in_advisory_band', 'ok_components', 'cut_reason',
             'restored', 'restore_failed', 'settle', 'poll_errors'}.

    `restored` carries the two guards' own verdicts (`caps` from `lim.guard`,
    `global_speed` from `global_speed_guard`) and `restore_failed` is the one
    key an executive gates on: True when either guard reported that it did NOT
    put the controller back, None when neither guard ran (every pre-run
    refusal returns before them), which is not "restored" either. It is kept
    OUT of `ok` on purpose — `ok` answers "did this stroke complete", and a
    complete stroke that leaked a 1.5 m/s envelope is a different emergency
    with a different response (H31).
    verdict: in_band | out_of_band | host_timed | refused_no_state |
             refused_not_at_start | did_not_start | no_end | wrong_id.
             in_band/out_of_band are both ADVISORY and `host_timed` is the
             CANFD spelling for "the player's band does not apply".
    ADVISORY; read `ok` for the pass/fail and `cut_reason` for why not.
    """
    lo, hi = float(band_s[0]), float(band_s[1])
    out = {"ok": False, "verdict": "", "gap_deg": None, "duration_s": None,
           "implied_ms_per_pt": None, "wrong_id_seen": False,
           "transitions": [], "run_ret": None,
           "in_band": None, "in_advisory_band": None,
           "ok_components": {"probe": None, "finish_event": None,
                             "replay": None, "not_early": None},
           "cut_reason": None, "restored": {}, "restore_failed": None,
           "settle": None,
           "poll_errors": None, "replay_span_resolved": None}
    # M43 AUTO: no probe when the event stream already answers the question.
    probe_on = (bool(blocking_probe) if blocking_probe is not None
                else not (event_log is not None
                          and bool(getattr(event_log, "registered", False))))
    out["probe_policy"] = ("auto" if blocking_probe is None else "explicit")
    def _detach_events():
        """Detach and summarise the event log. HARDENING (2026-09-08): the
        three PRE-RUN refusals below used to return without calling this, so a
        run that never started left its ctypes callback registered on the
        handle and its `arm_events` out of the record. The module keeps ONE
        strong reference to the trampoline (`_EVENT_CB_REF`), so the next
        `ArmEventLog.start()` on that handle silently replaced a sink nobody
        had detached."""
        if event_log is not None and callable(getattr(event_log, "stop", None)):
            try:
                d = event_log.stop()
            except Exception as exc:                                   # noqa: BLE001
                log(f"** detaching the event log raised {exc!r}")
                return None
            if isinstance(d, dict):
                out["arm_events"] = d
            return d
        return None

    def _refuse(verdict, reason):
        """Tell the monitor this arm's stroke is not happening. Every early
        exit below sets `ok` False in the RETURN VALUE, which a watchdog thread
        reading `monitor.should_stop` never sees."""
        if monitor is not None:
            monitor.on_run_refused(verdict, reason)

    is_canfd = (channel == "canfd")
    out["channel"] = "canfd" if is_canfd else "upload"
    copts = dict(canfd or {})
    c_period = float(copts.get("period_s") or REPLAY_CLOCK_S)
    if is_canfd:
        if not track_rows_millideg:
            out["verdict"] = "refused_canfd_limits"
            out["cut_reason"] = "the CANFD channel was given no lines to stream"
            log(f"** {out['cut_reason']}")
            _detach_events()
            _refuse(out["verdict"], out["cut_reason"])
            return out
        c_rate_gate = str(copts.get("rate_gate") or "per_joint")   # the enforced limit (2026-09-21)
        pf = canfd_preflight(track_rows_millideg, c_period, c_rate_gate)
        out["canfd_preflight"] = pf
        if not pf["ok"]:
            out["verdict"] = "refused_canfd_limits"
            out["cut_reason"] = "; ".join(pf["reasons"])
            log(f"** CANFD: the controller would not respond to this file — {out['cut_reason']}")
            _detach_events()
            _refuse(out["verdict"], out["cut_reason"])
            return out
        # The first line is one frame from wherever the arm is: that gap is
        # held to the same per-frame limit as every other line (80 % of it).
        start_tolerance_deg = min(float(start_tolerance_deg),
                                  0.8 * canfd_frame0_limit_deg(c_period, c_rate_gate))
        out["canfd_start_tolerance_deg"] = round(start_tolerance_deg, 4)
        jb = max(range(7), key=lambda k: pf["peak_rate_util"][k])
        log(f"   CANFD: {pf['lines']} lines at {c_period * 1e3:.4f} ms/line, {c_rate_gate} rate "
            f"gate, peak {pf['peak_rate_deg_s'][jb]:.1f} deg/s on J{jb + 1} = "
            f"{pf['peak_rate_util'][jb]:.3f} of its {pf['rate_limits_deg_s'][jb]:g}; start gap "
            f"must be <= {start_tolerance_deg:.3f} deg")
        if c_rate_gate != "flat":
            # LOUD, because this is the one switch that lets the arm be commanded past a
            # documented vendor limit. It is a probe, not a production setting.
            n180 = sum(1 for k in range(7)
                       if pf["peak_rate_deg_s"][k] > CANFD_MAX_RATE_DEG_S + 1e-6)
            log(f"   ** CANFD RATE GATE = {c_rate_gate}: the vendor's documented flat "
                f"{CANFD_MAX_RATE_DEG_S:g} deg/s is NOT being enforced. {n180} joint(s) peak above "
                f"it. The documented consequence is NON-RESPONSE (no error code, no clamp), so "
                f"watch commanded-vs-actual and error 4103, not a return value.")
    gap = start_gap_deg(robot, first_row_millideg)
    out["gap_deg"] = gap
    if gap is None:
        out["verdict"] = "refused_no_state"
        out["cut_reason"] = "the current joint state could not be read"
        log("could not read the current joint state — refusing to run")
        _detach_events()
        _refuse(out["verdict"], out["cut_reason"])
        return out
    if gap > start_tolerance_deg:
        if require_at_start:
            out["verdict"] = "refused_not_at_start"
            out["cut_reason"] = "the arm is not at the track's first point"
            log(f"arm is {gap:.2f} deg from the track's first point and strict mode is on: "
                f"stream the task's move_to_start leg first, then run")
            _detach_events()
            _refuse(out["verdict"], out["cut_reason"])
            return out
        if movej_speed:
            # Newton, 2026-09-04: the transit to the first point gets its OWN
            # speed. The controller's implicit movej runs at the PROGRAM's
            # speed, so the only way to set it independently is to make the
            # move ourselves and leave the program's own transit with nothing
            # to do. Same collision-unchecked class as the implicit one.
            #
            # It MUST set the global ratio first. `v` is a PERCENT of that
            # ratio, the ratio persists across power cycles and has no getter,
            # so a bare movej inherits whatever the last session left: the
            # first transit of the 2026-09-04 18:47 flow took 52 s for 73.7 deg
            # (1.4 deg/s, about 1 % of rated) for exactly that reason. Inside
            # the guard, so the ratio is restored on every exit path.
            with global_speed_guard(robot, restore_pct, log=log):
                sret = set_global_speed(robot, 100)
                log(f"** movej transit: bridging {gap:.2f} deg to the track's first point at "
                    f"speed {movej_speed} of a 100 % global ratio (set ret={sret}; explicit "
                    f"rm_movej, collision-UNchecked)")
                mret = int(robot.rm_movej([float(v) / 1000.0 for v in first_row_millideg],
                                          int(movej_speed), 0, 0, 1))
            out["movej_ret"] = mret
            out["movej_speed"] = int(movej_speed)
            out["movej_ratio_ret"] = int(sret)
            if mret != 0:
                out["verdict"] = "refused_movej"
                out["cut_reason"] = "the bridge movej to the first point was refused"
                log(f"** movej to the first point REFUSED ret={mret}")
                _detach_events()
                _refuse(out["verdict"], out["cut_reason"])
                return out
            if is_canfd:
                # The upload player closes whatever the bridge left; a stream's
                # first line cannot, so the arrival is measured, not assumed.
                gap2 = start_gap_deg(robot, first_row_millideg)
                out["gap_after_movej_deg"] = gap2
                if gap2 is None or gap2 > start_tolerance_deg:
                    out["verdict"] = "refused_not_at_start"
                    out["cut_reason"] = (f"after the bridge movej the arm is "
                                         f"{'unreadable' if gap2 is None else f'{gap2:.3f} deg'} "
                                         f"from the first line; one CANFD frame closes at most "
                                         f"{start_tolerance_deg:.3f} deg")
                    log(f"** {out['cut_reason']}")
                    _detach_events()
                    _refuse(out["verdict"], out["cut_reason"])
                    return out
        elif is_canfd:
            out["verdict"] = "refused_not_at_start"
            out["cut_reason"] = (f"the arm is {gap:.2f} deg from the first line and a CANFD "
                                 f"stream has no controller transit to close it — pass "
                                 f"--movej-speed N to bridge with an explicit movej")
            log(f"** {out['cut_reason']}")
            _detach_events()
            _refuse(out["verdict"], out["cut_reason"])
            return out
        else:
            log(f"** controller movej transit: bridging {gap:.2f} deg to the track's first "
                f"point at speed {speed} (rm move, collision-UNchecked — default per "
                f"ruling 2026-09-01)")
    try:
        # The Cartesian envelope wraps the PROGRAM only, not the transit: the
        # transit is a joint-space movej, and leaving the caps raised for a
        # moment longer than the stroke needs them is exactly the ratchet this
        # guard exists to prevent. It restores on every exit path, including
        # the KeyboardInterrupt below.
        caps_verdict, speed_verdict = {}, {}
        out["restored"] = {"caps": caps_verdict, "global_speed": speed_verdict}
        with lim.guard(robot, caps, log=log, verdict=caps_verdict) as caps_before:
            out["caps_before"] = caps_before
            out["caps_applied"] = dict(caps) if caps else None
            held = bool(ratio_held) and is_canfd and int(restore_pct) == int(speed)
            with (contextlib.nullcontext() if held else
                  global_speed_guard(robot, restore_pct, log=log, verdict=speed_verdict)):
                if is_canfd:
                    # PARITY: the upload run plays under the global ratio its
                    # `speed` sets (rm_set_program_id_run == the pendant slider),
                    # so the stream runs under the same one. Whether the ratio
                    # reaches a pass-through stream at all is unmeasured; this
                    # keeps it from being the difference.
                    out["canfd_global_speed"] = ({"pct": int(speed), "held": True} if held else
                                                 {"pct": int(speed), "ret": set_global_speed(robot, int(speed))})
                    stream_rec: dict = {}
                    out["canfd"] = stream_rec
                    # THE GATE IN FORCE BELONGS IN THE STREAM'S OWN BLOCK, not only in
                    # `canfd_preflight`. `stream_track` never sees it — it is a preflight policy,
                    # not a wire parameter — so it has to be stamped here or a reader of
                    # `canfd.rate_gate` gets None on a run that was gated `flat` (observed on the
                    # 2026-09-21 slot-51 control, where canfd_preflight said flat and this said
                    # None). Records before 2026-09-20 have the key nowhere at all, and absence
                    # there means "the field did not exist", never "flat".
                    stream_rec["rate_gate"] = c_rate_gate
                    s = stream_track(robot, track_rows_millideg, period_s=c_period,
                                     follow=bool(copts.get("follow", True)),
                                     trajectory_mode=int(copts.get("trajectory_mode", 0)),
                                     radio=int(copts.get("radio", 0)),
                                     watch=canfd_watch, result=stream_rec, log=log)
                    # STARTED = the controller took a line, not "it took the FIRST line": one
                    # transient failure on line 0 used to report `did_not_start` for a stream that
                    # then played the whole file (2026-09-17 review).
                    first_ok = (s["sent"] > 0 and s["send_errors"] < s["sent"])
                    res = {"started": first_ok, "ended": True,
                           "run_ret": (0 if s["first_bad_ret"] is None else s["first_bad_ret"]),
                           "duration_s": s.get("replay_s"), "transit_s": 0.0,
                           "replay_s": s.get("replay_s"), "transitions": [],
                           "wrong_id_seen": False, "blocking_probe": {},
                           "poll_errors": None}
                else:
                    res = run_program(robot, save_id, speed=speed, timeout_s=hi * 3 + 60,
                                      on_transition=on_transition,
                                      first_row_millideg=first_row_millideg,
                                      blocking_probe=probe_on, monitor=monitor)
    except KeyboardInterrupt:
        # The program keeps playing on the CONTROLLER if we just die — send a
        # best-effort on-trajectory stop before propagating (guard already
        # restored the global speed on the way out).
        # B12 (hardening, 2026-09-08): this was a bare `slow_stop`, whose
        # return code has never been shown to abort a running PROGRAM (0
        # hardware trials; the vendor's online-program group exposes no stop
        # verb at all). `stop_program` issues the SAME verb first, then
        # VERIFIES it against the run state and the joints and escalates when
        # it did not take — and it never raises, so it cannot mask the
        # KeyboardInterrupt it is unwinding.
        if is_canfd:
            # No program to stop: the stream stopped sending when the
            # interrupt landed, and a pass-through arm holds its last line.
            log("** interrupted during a CANFD stream — no further lines were sent; the arm "
                "holds the last one it accepted")
            out["interrupt_stop"] = {"channel": "canfd", "stopped_sending": True}
        else:
            log("** interrupted during a program run — running the verified stop ladder")
            out["interrupt_stop"] = stop_program(robot, log=log)
        # HARDENING (2026-09-08 review): this path re-raised WITHOUT detaching
        # the event log and without telling the monitor, so an interrupt left a
        # live ctypes callback registered on the handle (the module keeps one
        # strong reference, so the next `start()` silently replaced a sink
        # nobody had detached) and left a watchdog thread reading
        # `monitor.should_stop` believing the partner's stroke was healthy.
        # Neither may raise out of here: they would mask the KeyboardInterrupt.
        try:
            _detach_events()
            _refuse("interrupted", "the run was interrupted from the console")
        except Exception as exc:                                       # noqa: BLE001
            log(f"** unwinding the interrupt raised {exc!r}")
        raise
    out.update(run_ret=res["run_ret"], transitions=res["transitions"],
               wrong_id_seen=res["wrong_id_seen"], duration_s=res["duration_s"],
               transit_s=res.get("transit_s"), replay_s=res.get("replay_s"),
               blocking_probe=res.get("blocking_probe"),
               poll_errors=res.get("poll_errors"))
    # H31, as ONE key a caller can gate on (hardening, 2026-09-08 review). The
    # design row asks the executive to HARD STOP on a restore that did not take
    # — a leaked 1.5 m/s / 20 rad/s^2 envelope is global, shared with the Web
    # GUI and survives power cycles — and digging two levels into `restored`
    # for it is how such a check gets skipped. Derived HERE, the instant both
    # guards have unwound, so the `did_not_start` / `no_end` / `wrong_id`
    # exits below carry it too; it stays None only on the pre-run refusals,
    # which return before either guard is entered. None is "neither guard ran",
    # which is not "restored".
    _rv = [v.get("restored") for v in out["restored"].values()
           if isinstance(v, dict)]
    out["restore_failed"] = (any(v is False for v in _rv) if _rv else None)
    if settle_after:
        # H30: `run_state == 0` is NOT "at rest". 32 of the 126 ledger records
        # carrying a settle read `was_moving: true` (25.4 %), peak 11.94 deg/s,
        # waited up to 6.799 s. A dual executive that treats the program's end
        # as the end of motion plans its next bridge from a pose the arm has
        # already left, and barriers on an arm still travelling at the partner.
        out["settle"] = wait_until_at_rest(robot)
    # Detach and summarise the event log on EVERY exit path from here on — a
    # run that never started still consumed the sink, and its events are the
    # only thing that says what the controller thought it was doing.
    ev = _detach_events()
    bp = res.get("blocking_probe") or {}
    if bp.get("used"):
        log(f"blocking probe: rm_set_program_id_run(block=1) returned {bp.get('ret')} "
            f"after {bp.get('returned_s')} s  "
            f"(-4 = the trajectory was stopped, 0 = the program finished)")
    if not res["started"]:
        out["verdict"] = "did_not_start"
        out["cut_reason"] = "the controller never reported a running program"
        log(f"DID NOT START (run_ret={res['run_ret']})")
        _refuse(out["verdict"], out["cut_reason"])
        return out
    if not res["ended"]:
        out["verdict"] = "no_end"
        out["cut_reason"] = "started but never reported run_state 0"
        log("started but never reported run_state 0 — check the arm")
        _refuse(out["verdict"], out["cut_reason"])
        return out
    # The band is for the REPLAY alone (Newton's correction, 2026-09-01): the
    # program's own movej transit + dwell are excluded when the joint polling
    # resolved the departure; else the TOTAL is judged, with a warning.
    d = res.get("replay_s")
    replay_span_resolved = d is not None
    if d is None:
        d = res["duration_s"]
        log("** replay span unresolved (joint polling never latched the departure) — "
            "judging the TOTAL duration, movej included")
    else:
        log(f"transit {res['transit_s']:.2f} s (movej + dwell) + replay {d:.2f} s")
    out["implied_ms_per_pt"] = d / max(points - 1, 1) * 1e3
    if res["wrong_id_seen"]:
        out["verdict"] = "wrong_id"
        out["cut_reason"] = "a different program id was seen running"
        log("** a DIFFERENT program id was seen running — investigate before trusting this run")
        _refuse(out["verdict"], out["cut_reason"])
        return out
    # THE DURATION BAND IS THE PLAYER'S, AND IT DOES NOT APPLY TO A STREAM. `band_s` is derived from the
    # upload player's ms/pt; on CANFD the duration is the HOST's send period x lines, which this process
    # commanded rather than observed, so comparing it to the player's band tests nothing. Judged that way,
    # all 60 CANFD runs of 2026-09-18 read `out_of_band` while every stream had completed cleanly (0 send
    # errors, worst lateness 2.92 ms) — a label that said "wrong clock" about the one channel whose clock
    # is not in question. The lateness the streamer measures is the real screen; `clock_screen` says the
    # same in words. `in_band` stays None here, which is "not measured", not "failed".
    if is_canfd:
        in_band = None
        out["verdict"] = "host_timed"
    else:
        in_band = lo <= d <= hi
        out["verdict"] = "in_band" if in_band else "out_of_band"
        out["in_band"] = out["in_advisory_band"] = in_band
    # ---- B1: the completeness verdict, which is what `ok` now means --------
    comp = out["ok_components"]
    # HARDENING (2026-09-08): `not_early` is a test on the REPLAY. When the
    # departure never latched, `d` is the TOTAL — transit and dwell included —
    # so `d >= lo` is very nearly free and would have made an unmeasured run
    # score `ok` on nothing but the movej it took to get to the first point.
    # Unresolved is UNMEASURED (None), which vetoes nothing and claims nothing;
    # with the probe or the event log present neither is affected, and with
    # neither present `ok` is now False for "no evidence", which is correct.
    comp["not_early"] = bool(d >= lo) if replay_span_resolved else None
    if is_canfd:
        # The host commanded the duration, so "not early" is whether every line
        # went out: a stream the stop check or failed sends ended is short.
        cs = out.get("canfd") or {}
        comp["not_early"] = bool(cs.get("completed")) and not cs.get("aborted")
    out["replay_span_resolved"] = replay_span_resolved
    if bp.get("used"):
        r = bp.get("ret")
        # HARDENING (2026-09-08 review): a probe that was USED but came back
        # with something that is not a return code — `"still blocked at join"`
        # (the SDK call has NOT returned, so the program may STILL BE PLAYING)
        # or `"raised ..."` — used to record `None` here and then be replaced
        # by the `not_early` stand-in below, so the one case where nobody knows
        # whether the arm is moving could score `ok: True` off a duration. It
        # is a FAILED component, not an unmeasured one; `MotionMonitor.on_probe`
        # already trips rung 2 on exactly this value and the two must agree.
        comp["probe"] = (r == 0) if isinstance(r, int) else False
        if not isinstance(r, int):
            out["probe_uncoded"] = repr(r)
    if isinstance(ev, dict):
        usable = bool(ev.get("registered")) and not _is_emulated_handle(robot)
        # REGISTRATION IS A PROMISE, DELIVERY IS THE EVIDENCE. A stream that
        # registered and then delivered NOTHING says nothing about whether the
        # program finished — and AUTO turned the probe off on the strength of
        # that registration, so there is no second channel either. Measured
        # 2026-09-10: 2 of 7 concurrent releases had the left arm receive its
        # finish event and the right arm receive n_events 0, and the standing
        # claim "every hardware run delivered at least one event" died with it.
        #
        # Zero events is therefore UNMEASURED (None), not False. False asserts
        # the finish did not happen, which is real evidence of a cut and feeds
        # B15, the clock-envelope population and the ledger classifiers. Those
        # two runs were still correctly incomplete — `not_early` failed on both,
        # genuinely short — but on evidence they had, not evidence they lacked.
        n_ev = ev.get("n_events")
        delivered = n_ev is None or int(n_ev) > 0
        comp["finish_event"] = (bool(ev.get("finish_seen"))
                                if (usable and delivered and not is_canfd) else None)
        if is_canfd and ev.get("registered"):
            out["arm_events"]["not_evidence"] = (
                "CANFD stream: pass-through runs no program, so the controller fires no "
                "program_run_finish — the events are recorded, never read as a cut")
        if usable and not delivered:
            out["arm_events"]["not_evidence"] = (
                "the event stream registered and delivered NO events on this handle, "
                "so it cannot say whether the program finished. AUTO disabled the "
                "probe on the strength of that registration, so this run has no "
                "completeness channel at all — that is an absence of evidence, not "
                "evidence of a cut.")
        if not usable and ev.get("registered"):
            out["arm_events"]["not_evidence"] = (
                "emulated handle: the program double fires no program_run_finish")
    if replay_verdict_fn is not None:
        try:
            vd = replay_verdict_fn()
        except Exception as exc:                                       # noqa: BLE001
            vd = None
            log(f"** replay verdict raised {exc!r} — recorded as unmeasured")
        if isinstance(vd, dict):
            out["replay_outcome"] = vd.get("outcome")
            comp["replay"] = (vd.get("outcome") == "complete")
    if comp["probe"] is None and not bp.get("used") and not is_canfd:
        # No probe AT ALL: the stand-in is "it did not end early". A run cut
        # short is short; the +-6 % player spread cannot make a COMPLETE replay
        # finish before the -1 % band floor. A probe that RAN and gave no code
        # is handled above and never reaches this line — it is evidence of
        # failure, not an absence of evidence.
        comp["probe"] = comp["not_early"]
    elif is_canfd and not bp.get("used"):
        # NOT ON A CANFD STREAM. The probe blocks on `rm_set_program_id_run` returning, and a stream runs
        # no program — the same reason `finish_event` is None here. The stand-in would copy `not_early`,
        # which for a stream is already the stream's own send count and stop check, so `ok` would count
        # one fact twice and the record would read `probe: true` on a run that never probed anything
        # (every CANFD record written before 2026-09-17 says exactly that; review B5).
        out["blocking_probe"] = dict(bp, not_evidence=(
            "CANFD stream: no program is run, so a blocking probe cannot say whether the stream "
            "finished — `not_early` carries the stream's own evidence (every line sent, no abort)"))
    if monitor is not None:
        # rung 3, so a monitor-driven executive reaches the same verdict this
        # call does without re-deriving it (rung 2 was fed by the probe thread).
        monitor.on_finish(comp["finish_event"])
        if comp["replay"] is not None:
            # HARDENING (2026-09-08 review): this was
            # `out.get("replay_outcome") and {...}`, which passes None for a
            # verdict whose outcome is None or "" — the very case that sets
            # `comp["replay"] = False` and `ok = False`. The watchdog thread
            # reading `monitor.should_stop` then saw a clean stroke while the
            # return value said cut. Pass the dict whenever a verdict was
            # actually measured.
            monitor.on_replay_verdict({"outcome": out.get("replay_outcome")})
        if comp["probe"] is False and not bp.get("used"):
            # The `not_early` stand-in said the replay ended before the band
            # floor. Rung 2 never fires without a probe, so without this the
            # monitor is silent on a run the return value calls a cut.
            monitor.on_run_refused("early_replay",
                                   "the replay ended before the band's floor "
                                   "and no probe was run")
    # H31, as ONE key a caller can gate on (hardening, 2026-09-08 review). The
    # design row asks the executive to HARD STOP on a restore that did not take
    # — a leaked 1.5 m/s / 20 rad/s^2 envelope is global, shared with the Web
    # GUI and survives power cycles — and digging two levels into `restored`
    # for it is how such a check gets skipped. `None` means neither guard ran
    # (the pre-run refusals return before them), which is not "restored".
    measured = {k: v for k, v in comp.items() if v is not None}
    out["ok"] = bool(measured) and all(measured.values())
    if not out["ok"]:
        failed = sorted(k for k, v in measured.items() if not v)
        out["cut_reason"] = ("no evidence was measured (probe off, no event log, "
                             "no capture) — refusing to call this complete"
                             if not measured else "failed: " + ", ".join(failed))
        log(f"** NOT COMPLETE — {out['cut_reason']}")
    # `in_band is None` is NOT `False` here: on CANFD the band was never applied, and telling an operator
    # "OUT OF BAND" about a stream is the same wrong label the field itself used to carry.
    if in_band is None:
        cs = out.get("canfd") or {}
        late = cs.get("late_ms_max")
        log(f"stream {d:.2f} s at the host's {(cs.get('period_s') or 0) * 1e3:.3f} ms period — the "
            f"player's advisory band [{lo}, {hi}] s does not apply to a host-timed stream"
            + ("" if late is None else f"; worst send lateness {float(late):.2f} ms"))
    else:
        log(f"replay {d:.2f} s — advisory band [{lo}, {hi}] s: "
            + ("in band" if in_band else
               "OUT OF BAND (reported, not gated — the player varies +-6 %)"))
    if in_band is False:
        # The old line said "sessions have measured 5.57-5.90", which the
        # ledger outgrew: 181 complete runs span 5.465-6.654 ms/pt. An operator
        # was being told three different ranges for one run — this line, the
        # 5.4-6.2 envelope field, and the 5.5-5.6 advisory band.
        log(f"  implied clock {out['implied_ms_per_pt']:.3f} ms/pt on the COMMANDED point "
            f"count (nominal {REPLAY_CLOCK_S * 1e3:.4f}; the ledger's complete runs span "
            f"5.465-6.654, screen {CLOCK_ENVELOPE_MS}) — on an incomplete run this is the "
            f"clock times the completion fraction, not a clock; see `clock_ms_per_pt`")
    return out


class UdpRecorder:
    """Record EVERYTHING the controller's realtime UDP channel offers, during
    an off-driver track run (--record-joints).

    OFF-DRIVER ONLY: the push has ONE destination per controller; taking it
    while alix_hw streams would starve the driver (F98 hazard class). Here
    nothing owns it, and the previous configuration is read first
    (rm_get_realtime_push) and RESTORED EXACTLY on stop — enabled or not.

    The push is pointed at *host_ip* (the caller passes ArmHandle.host_ip —
    the address on the route to the arm; the controller silently accepts a
    wrong target and delivers nothing), cycle 1 (= 5 ms), and every
    custom_config section ON: joint_speed, lift_state, expand_state,
    hand_state, arm_current_status, aloha_state, plus_base, plus_state.
    force_coordinate stays -1 (unchanged): the RM75-6FB has no joint torque
    sensors. Frames are stored VERBATIM — one NDJSON line per datagram,
    {"t_mono": <monotonic s>, "frame": <payload as received>} — nothing is
    parsed away, so integer-quantised diagnostics etc. survive as sent.
    """

    def __init__(self, out_path, host_ip: str, port: Optional[int] = None,
                 log: Callable[[str], None] = print, allow_steal: bool = False,
                 monitor: Optional["MotionMonitor"] = None):
        self.out_path = pathlib.Path(out_path)
        self.host_ip = host_ip
        # RM_UDP_PORT is what `connect` reads for the same channel; honour it
        # here too, so one variable moves BOTH ends when 8089 is taken on this
        # host (2026-09-04 20:20: EADDRINUSE aborted a flow at task 1).
        self.port = int(port if port is not None else os.environ.get("RM_UDP_PORT", 8089))
        self.log = log
        self.allow_steal = allow_steal
        self.frames = 0
        #: B13 rung 0's ONLY production feed (hardening, 2026-09-08 review).
        #: `MotionMonitor` was reachable from `run_verified` for rungs 1-3 but
        #: nothing in the tree ever handed it a PUSH FRAME, so the ~15 ms rung
        #: — the entire reason B13 exists, against a probe that is 0.89-2.37 s
        #: late — could only ever be exercised by a test. The recorder is the
        #: one object that parses those frames, so it feeds them; see
        #: `attach_monitor`. Unset, nothing changes: no branch is taken and the
        #: proven single-arm flow behaves exactly as it did.
        self.monitor = monitor
        self._prev = None
        #: L51 evidence for the run record: what the restore actually did, and
        #: whether a stale SELF-enable had to be cleared on the way in.
        self.push_restored: Optional[dict] = None
        self.cleared_stale: Optional[dict] = None
        self._restored = False
        self._sock = None
        self._thread = None
        self._stop = False
        self._fh = None

    def start(self, robot):
        import socket
        import threading
        ret, prev = robot.rm_get_realtime_push()
        self._prev = dict(prev) if ret == 0 and isinstance(prev, dict) else None
        if self._prev is None:
            self.log(f"** could not read the current push config (ret={ret}) — "
                     f"it cannot be restored; recording anyway (off-driver)")
        elif self._prev.get("enable") and not self.allow_steal:
            owner = f"{self._prev.get('ip')}:{self._prev.get('port')}"
            if self._is_selfward(self._prev):
                # L51 (2026-09-08): the SELFWARD stale enable is not a consumer
                # to be stolen from — it is OUR OWN leftover. The controller is
                # already pushing at this host and this port, which no live
                # driver elsewhere can be doing, so the only thing it can be is
                # a capture that died without restoring. 31 of the 94 records
                # carrying `push_prev` found exactly this, and all 31 ran with
                # `steal_push: true` because that was the only way past it —
                # the stale enable was then restored VERBATIM on the way out
                # and demanded the flag again next time. It persisted for a
                # whole session. Clear it instead: disable the push, record
                # what was cleared, and carry on with no flag at all.
                self._clear_stale_self(robot, owner)
            else:
                # S2 HARD REFUSAL (F109): the push has ONE destination.
                # enable=true toward SOMEWHERE ELSE means a live consumer owns
                # it — a running driver on another host (starving it kills the
                # stream in 250 ms). Taking it silently is the F98 hazard; a
                # human decides, not this tool.
                who = ("THIS host on a DIFFERENT port — another capture here, or a "
                       "driver" if str(self._prev.get("ip")) == str(self.host_ip)
                       else "ANOTHER host — likely a live driver there")
                raise RuntimeError(
                    f"realtime push is ENABLED toward {owner} ({who}). Refusing to "
                    f"take it. If you are CERTAIN nothing consumes it, pass "
                    f"--steal-push / allow_steal=True.")
        elif self._prev.get("enable"):
            # Stealing: say WHOSE it was. The restore on stop puts the previous
            # config back, so one stale enable left by a crashed session makes
            # every later run need --steal-push forever, and nothing in the log
            # distinguished that from a live consumer (Newton, 2026-09-03: the
            # flag was needed on every run of the session).
            owner = f"{self._prev.get('ip')}:{self._prev.get('port')}"
            mine = self._prev.get("ip") == self.host_ip
            self.log(f"** --steal-push: the push was ENABLED toward {owner} and is being "
                     f"TAKEN. {'That is THIS host, so it is almost certainly a stale enable '
                     'from a session that never restored — harmless, but it will keep '
                     'demanding --steal-push until something clears it.' if mine else
                     'That is ANOTHER host: if a driver is running there, this starves it '
                     'in ~250 ms.'}")
        from Robotic_Arm.rm_robot_interface import (rm_realtime_push_config_t,
                                                    rm_udp_custom_config_t)
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.settimeout(0.25)
        try:
            self._sock.bind(("", self.port))
        except OSError as exc:
            # Local clash, and NOTHING on the controller has been touched yet:
            # say so, because the previous message ("[Errno 98] Address already
            # in use") read like a controller fault and left the operator with
            # no next step.
            self._sock.close()
            self._sock = None
            # A stranded config points the controller at OUR host and port, and
            # the SDK keeps a listener of its own for the push (its "bind
            # failed" is printed on every one of our runs, where WE hold the
            # port first). When it wins that race the port is gone before we
            # ask for it — and the fix is to clear the stale enable, not to
            # hunt a driver.
            selfward = isinstance(self._prev, dict) and self._is_selfward(self._prev)
            if self.cleared_stale is not None:
                # We already cleared it on the way in (L51) and the port is
                # STILL taken, so the owner is something else on this host.
                hint = (f"The stale self-enable was cleared, so the controller is no "
                        f"longer pushing here — something else on this host holds the "
                        f"port. Find it — `ss -lunp | grep {self.port}` or "
                        f"`sudo lsof -nP -iUDP:{self.port}`. ")
            else:
                hint = (f"The controller is ALREADY pushing to {self.host_ip}:{self.port} — a "
                    f"stale enable from a session that never restored — so the owner is "
                    f"most likely this process's own SDK listener. Clear it first: "
                    f"`tools/cleaning_tracks.py push --side <side> --clear`. "
                    if selfward else
                    f"Find the owner — `ss -lunp | grep {self.port}` or "
                    f"`sudo lsof -nP -iUDP:{self.port}` — and stop it (a running driver, "
                    f"or a session that never exited). ")
            touched = ("A stale self-enable was cleared on the way in; nothing else on "
                       "the controller was changed. "
                       if self.cleared_stale is not None else
                       "The controller was NOT touched. ")
            raise RuntimeError(
                f"UDP port {self.port} is already bound ON THIS HOST ({exc}). {touched}{hint}"
                f"Or record on another port with RM_UDP_PORT / --udp-port.") from exc
        custom = rm_udp_custom_config_t(joint_speed=1, lift_state=1, expand_state=1,
                                        arm_current_status=1, hand_state=1,
                                        aloha_state=1, plus_base=1, plus_state=1)
        cfg = rm_realtime_push_config_t(cycle=1, enable=True, port=self.port,
                                        force_coordinate=-1, ip=self.host_ip,
                                        custom_config=custom)
        ok, ret = self._apply_push(robot, cfg, dict(enable=True, ip=self.host_ip,
                                                    port=self.port), "capture")
        if not ok:
            self._sock.close()
            self._sock = None
            # We may have changed the controller's ONE push destination on the
            # way to failing. Put the previous owner back before raising —
            # otherwise a failed start strands the config exactly the way a
            # crashed session does (2026-09-04 20:18: a -2 left the push
            # pointing at us and the previous consumer starved).
            self._restore_prev(robot)
            raise RuntimeError(f"rm_set_realtime_push refused (ret={ret})")
        self._fh = open(self.out_path, "w")
        self._stop = False
        self._thread = threading.Thread(target=self._reader, daemon=True)
        self._thread.start()
        self.log(f"recording UDP push -> {self.out_path} (target {self.host_ip}:"
                 f"{self.port}, cycle 5 ms, all sections on)")

    def _is_selfward(self, cfg) -> bool:
        """Is this push config OUR OWN stale enable?

        The classifier was written for the EADDRINUSE hint (the SDK's own
        listener wins the port when the controller is already pushing at us);
        L51 lifts it into `start()` so the condition it identifies is FIXED
        rather than merely explained. `enable` toward this host AND this port
        cannot be a live driver anywhere else, so it is a capture that died
        without restoring.
        """
        try:
            return bool(cfg.get("enable")
                        and str(cfg.get("ip")) == str(self.host_ip)
                        and int(cfg.get("port") or 0) == self.port)
        except (AttributeError, TypeError, ValueError):
            return False

    def _clear_stale_self(self, robot, owner: str) -> None:
        """Disable a stale SELF-directed push, and remember that we did.

        The cleared config is NOT what gets restored on stop: `_prev` keeps the
        found configuration for the record (`push_prev` answers "who owned the
        push"), but `_restore_prev` puts back a DISABLED version of it, because
        re-arming a stale enable is exactly the loop this fixes.
        """
        from Robotic_Arm.rm_robot_interface import (rm_realtime_push_config_t,
                                                    rm_udp_custom_config_t)
        cfg = rm_realtime_push_config_t(
            cycle=int(self._prev.get("cycle", 1) or 1), enable=False,
            port=self.port, force_coordinate=-1, ip=str(self.host_ip),
            custom_config=rm_udp_custom_config_t())
        ok, ret = self._apply_push(robot, cfg, dict(enable=False), "clear stale self-enable")
        self.cleared_stale = {"owner": owner, "ok": bool(ok), "ret": ret}
        self.log(f"** the realtime push was ENABLED toward {owner} — that is THIS host and "
                 f"THIS port, so it is a stale enable from a capture that never restored, "
                 f"not a live consumer. Clearing it "
                 + ("(ok)." if ok else f"FAILED (ret={ret}) — clear it from the pendant or "
                                       f"with `push --side <side> --clear`.")
                 + " No --steal-push is needed for a self-enable.")
        if not ok:
            raise RuntimeError(
                f"realtime push is ENABLED toward {owner} (a stale self-enable) and it "
                f"could not be cleared (ret={ret}). Refusing to record over it.")

    #: How many times a push write is attempted before it is called a failure.
    #: The controller's command socket drops replies sporadically under load —
    #: on 2026-09-04 20:37 `rm_get_program_run_state` printed the SAME "revice
    #: error" twice mid-run and the polling loop simply went round again. The
    #: push write had no retry, so one dropped reply ended a 12-task flow at
    #: task 11, and another ended one at task 1.
    PUSH_ATTEMPTS = 3
    PUSH_RETRY_S = 0.25

    def _reads_back(self, robot, want: dict):
        """(matches, config) — what the controller actually holds right now."""
        try:
            rb, now = robot.rm_get_realtime_push()
        except Exception as exc:                                       # noqa: BLE001
            self.log(f"** push read-back raised {exc!r}")
            return False, {}
        now = now if isinstance(now, dict) else {}
        if rb != 0:
            return False, now
        if bool(now.get("enable")) != bool(want.get("enable")):
            return False, now
        if not want.get("enable"):
            return True, now             # disabled is disabled; ip/port are moot
        return (str(now.get("ip")) == str(want.get("ip"))
                and int(now.get("port") or 0) == int(want.get("port") or 0)), now

    def _apply_push(self, robot, cfg, want: dict, what: str):
        """Write the push config and VERIFY it, retrying a lost reply.

        A non-zero return is not a refusal: the vendor defines -2 as "data
        reception failed: a problem during communication, or the controller
        timed out without returning" (rm_interface.h:3456) and -3 as "the reply
        could not be parsed". On 2026-09-04 20:18 a -2 had in fact been
        APPLIED — the next run read our own target back off the controller. So
        the read-back decides, and a write that genuinely did not land is
        simply tried again. Returns (ok, last_ret).
        """
        ret = None
        for attempt in range(1, self.PUSH_ATTEMPTS + 1):
            ret = robot.rm_set_realtime_push(cfg)
            if ret == 0:
                return True, ret
            ok, now = self._reads_back(robot, want)
            if ok:
                self.log(f"** {what}: rm_set_realtime_push returned {ret}, but the "
                         f"controller reads back {now.get('ip')}:{now.get('port')} "
                         f"enable={now.get('enable')} — the reply was lost, the config "
                         f"took effect (rm_interface.h:3456).")
                return True, ret
            self.log(f"** {what}: rm_set_realtime_push returned {ret} and the controller "
                     f"still reads {now.get('ip')}:{now.get('port')} "
                     f"enable={now.get('enable')}"
                     + (f" — retrying ({attempt + 1}/{self.PUSH_ATTEMPTS})"
                        if attempt < self.PUSH_ATTEMPTS else ""))
            if attempt < self.PUSH_ATTEMPTS:
                time.sleep(self.PUSH_RETRY_S)
        return False, ret

    def _restore_prev(self, robot) -> None:
        """Put the push config we found back. Safe to call twice.

        THREE L51 CORRECTIONS (2026-09-08), all inside the rebuild:

        1. `plus_base` and `plus_state` are carried through. `start()` turns
           BOTH on; this rebuild left them out, so the vendor struct's default
           went back instead of what was found, and the two newest sections
           were silently dropped from whatever consumer came next.
        2. A stale SELF-enable that `start()` cleared is NOT re-armed. Putting
           the found config back "exactly" is right for a real consumer and
           wrong for our own leftover: it is what made one crashed capture
           demand `--steal-push` for a whole session.
        3. The verdict is kept on the recorder as `push_restored`
           ``{ok, ret, wanted, readback, cleared_stale}`` so the run record can
           carry it per side instead of only a console line.
        """
        from Robotic_Arm.rm_robot_interface import (rm_realtime_push_config_t,
                                                    rm_udp_custom_config_t)
        # _prev is NOT cleared: the run record reports it as `push_prev`, which
        # is how "who owned the push" is answerable after the fact. A flag
        # keeps the restore itself idempotent.
        p = self._prev
        if p is None or self._restored:
            return
        self._restored = True
        cc = p.get("custom_config") or {}
        get = (cc.get if isinstance(cc, dict) else lambda k, d=-1: -1)
        enable = bool(p.get("enable", False)) and self.cleared_stale is None
        cfg = rm_realtime_push_config_t(
            cycle=int(p.get("cycle", 1) or 1), enable=enable,
            port=int(p.get("port", 8089) or 8089),
            force_coordinate=int(p.get("force_coordinate", -1)),
            ip=str(p.get("ip", "") or "192.168.1.18"),
            custom_config=rm_udp_custom_config_t(
                joint_speed=get("joint_speed", -1), lift_state=get("lift_state", -1),
                expand_state=get("expand_state", -1), hand_state=get("hand_state", -1),
                arm_current_status=get("arm_current_status", -1),
                aloha_state=get("aloha_state", -1),
                plus_base=get("plus_base", -1), plus_state=get("plus_state", -1)))
        want = dict(enable=enable, ip=str(p.get("ip", "") or ""),
                    port=int(p.get("port", 8089) or 8089))
        ok, ret = self._apply_push(robot, cfg, want, "restore")
        _rb, now = self._reads_back(robot, want)
        self.push_restored = {"ok": bool(ok), "ret": ret, "wanted": want,
                              "readback": now if isinstance(now, dict) else {},
                              "cleared_stale": self.cleared_stale,
                              "re_armed": enable,
                              # The record has never stored WHICH target this
                              # capture used, so "was the enable we found our
                              # own?" is unanswerable after the fact: all 31
                              # stale enables in the ledger read the SAME
                              # host:8090 and nothing says whether that run's
                              # own port was 8090. Store it.
                              "our_target": {"ip": self.host_ip, "port": self.port},
                              "prev_was_selfward": (self._is_selfward(p)
                                                    if isinstance(p, dict) else None)}
        if ok and self.cleared_stale is not None:
            self.log("push left DISABLED: the enable found on the way in was a stale "
                     "self-enable and re-arming it is what kept demanding --steal-push")
        elif ok:
            self.log(f"push config restored to {p.get('ip')}:{p.get('port')} "
                     f"enable={p.get('enable')} (ret={ret})")
        else:
            self.log(f"** push restore FAILED (ret={ret}) — reconfigure from the pendant "
                     f"before the driver next runs")

    def _reader(self):
        while not self._stop:
            try:
                data, _addr = self._sock.recvfrom(65535)
            except OSError:
                continue
            t = time.monotonic()
            try:
                frame = json.loads(data)
            except Exception:                                          # noqa: BLE001
                frame = {"_raw_hex": data.hex()}      # never drop a datagram
            self._fh.write(json.dumps({"t_mono": t, "frame": frame},
                                      separators=(",", ":")) + "\n")
            self.frames += 1
            # AFTER the write, never before: the capture is the evidence and
            # must not be delayed or lost by a detector. `on_frame` never
            # raises (it counts what it cannot judge), so this cannot kill the
            # reader thread; it costs ~2.4 us against a 5 ms budget.
            m = self.monitor
            if m is not None:
                m.on_frame(t, frame)

    def attach_monitor(self, monitor: Optional["MotionMonitor"]) -> None:
        """Feed every push frame to *monitor* from here on (B13 rung 0).

        Settable after `start()` because the recorder is opened once per RUN
        and the monitor is built once per STROKE — the flow starts the capture
        before it knows which track it is about to play. Pass None to detach;
        an executive MUST detach at the end of a stroke, or the next stroke's
        frames land in a monitor whose `finished`/`tripped` latches belong to
        the last one.
        """
        self.monitor = monitor

    def stop(self, robot):
        # Idempotent: a failed run can reach stop() from both its own error
        # path and an outer `finally`, and the second call must not raise on a
        # closed file — or the real failure is lost behind a ValueError.
        self._stop = True
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        if self._sock is not None:
            self._sock.close()
            self._sock = None
        if self._fh is not None:
            self._fh.flush()
            self._fh.close()
            self._fh = None
        self._restore_prev(robot)
        if self.frames == 0:
            self.log("** 0 frames recorded — the push target is not this host, or the "
                     "channel never delivered (the controller accepts a wrong target "
                     "silently); verify host_ip before trusting an empty capture")
        self.log(f"recorded {self.frames} frames")


def read_capture(frames_path):
    """Parse a _frames.ndjson ONCE: (t_mono, arm_current_status, joint_position).

    The capture was being JSON-parsed twice per task on the critical path
    between tasks — once here for the status labels and once inside
    `verify_replay_from_frames` for the joint positions. Both consumers accept
    a pre-parsed capture now. The per-line try/except is kept exactly as both
    had it: one malformed datagram must skip that line and no more, or a single
    bad frame would change the result of both measurements at once.
    """
    import numpy as np
    ts, st, qs = [], [], []
    with open(frames_path) as fh:
        for line in fh:
            try:
                d = json.loads(line)
                t = d["t_mono"]
                f = d["frame"]
            except Exception:                                          # noqa: BLE001
                continue
            jp = None
            try:
                jp = f["joint_status"]["joint_position"]
            except Exception:                                          # noqa: BLE001
                jp = None
            ts.append(t)
            st.append(f.get("arm_current_status"))
            qs.append([v / 1000.0 for v in jp] if (jp and len(jp) == 7) else None)
    return ts, st, qs


def replay_span_from_status(frames_path, capture=None):
    """The EXACT replay window, from the controller's own `arm_current_status`.

    The push frames label what the arm is doing: the implicit transit reports
    `move_J`, the track replay reports `move_through_joint`. That is the
    controller stating its phase, so it beats inferring one from positions —
    measured 2026-09-01 against the position heuristic, the two disagree by
    -550..+60 ms, and the heuristic inflated the apparent clock spread to
    +-4.5% where the true figure is +-2.3% (F108c).

    Returns {'movej_s','dwell_s','replay_s'} or None when no
    `move_through_joint` segment is present (older captures, or a run that
    never started).
    """
    ts, st, _q = capture if capture is not None else read_capture(frames_path)
    if not ts:
        return None
    idx = [i for i, v in enumerate(st) if v == "move_through_joint"]
    if not idx:
        return None
    a, b = idx[0], idx[-1]
    mj = [i for i, v in enumerate(st) if v == "move_J" and i < a]
    t_mj_end = ts[mj[-1]] if mj else ts[0]
    return {"movej_s": t_mj_end - ts[0],
            "dwell_s": ts[a] - t_mj_end,
            "replay_s": ts[b] - ts[a]}


def replay_span_from_frames(frames_path, first_row_millideg, last_row_millideg,
                            tol_deg: float = 0.5):
    """Precise (5 ms) movej/dwell/replay split from a --record-joints capture:
    replay = DEPARTURE from the first point (last frame within tol before a
    sustained exit) -> ARRIVAL at the last point. Calibrated on the four
    2026-09-01 hardware runs (dwell 0.28-0.42 s; replay clocks 5.51-6.02
    ms/pt). Returns {'movej_s','dwell_s','replay_s'} or None when the span is
    not resolvable (the arm never reached the first point within tol)."""
    first = [v / 1000.0 for v in first_row_millideg]
    last = [v / 1000.0 for v in last_row_millideg]
    ts, gf, gl = [], [], []
    with open(frames_path) as fh:
        for line in fh:
            try:
                d = json.loads(line)
                jp = d["frame"]["joint_status"]["joint_position"]
            except Exception:                                          # noqa: BLE001
                continue
            if len(jp) != 7:
                continue
            q = [v / 1000.0 for v in jp]
            ts.append(d["t_mono"])
            gf.append(max(abs(a - b) for a, b in zip(q, first)))
            gl.append(max(abs(a - b) for a, b in zip(q, last)))
    n = len(ts)
    a0 = next((i for i in range(n) if gf[i] <= tol_deg), None)
    if a0 is None:
        return None
    dep = a0
    for i in range(a0, n - 3):
        if gf[i] <= tol_deg and all(g > tol_deg for g in gf[i + 1:i + 4]):
            dep = i
            break
    i1 = next((i for i in range(dep + 1, n) if gl[i] <= tol_deg), n - 1)
    return {"movej_s": ts[a0] - ts[0], "dwell_s": ts[dep] - ts[a0],
            "replay_s": ts[i1] - ts[dep]}


#: Executed-clock ENVELOPE for this hardware, ms/pt (F108, 2026-09-01).
#: Measured 5.51-6.02 across four runs, +-6% run-to-run ON THE SAME TRACK,
#: while the UDP push cadence held 5.008-5.013 ms (0.1%) — so the spread is the
#: PLAYER's, not our timing, and V1.7.4 already carries the vendor's
#: trajectory-replay speed-variation fix. The old [5.5, 5.6] band was +-1% and
#: could not be met; this envelope is what the controller actually does.
CLOCK_ENVELOPE_MS = (5.4, 6.2)

#: A TRACK THAT ENDS HOLDING STILL (2026-09-15). Commanded lines within
#: STILL_TAIL_TOL_DEG (max-norm) of the final line are the track's still tail;
#: `verify_replay_from_frames` credits the tail as visited when the ordered
#: matcher has reached its start AND the arm's last captured position is
#: within STILL_TAIL_END_TOL_DEG of that final line. Taught recordings keep
#: the operator's final pause (88 lines / 0.49 s on 42_taught_lid_sid_01_right,
#: 524 lines on the raw bowl-out recordings); identical commanded points give
#: the matcher no reason to advance, so without this a complete run read "cut".
STILL_TAIL_TOL_DEG = 0.05
STILL_TAIL_END_TOL_DEG = 0.5

#: THE SINGLE HOME FOR THE FLAT SCREEN. `tools/telemetry_id.py` and
#: `tools/track_report.py` each carried `(5.4, 6.2)` as their own literal, so
#: three copies had to be edited together and never were. They import this now.
#: The length-aware AFFINE band lives in `tools/dual_arm_verdicts._clock_band`
#: and is a different instrument for a different job: this one screens ONE run
#: on its own, that one is fitted per point count and is what a concurrency
#: window is derived from. They disagree — both of the 2026-09-10 runs that
#: fall outside `_clock_band` record `in_clock_envelope: True` here — because a
#: flat band cannot express the fixed ramp-in/ramp-out share of a short track.
#: Neither is wrong; a reader must be told which one is speaking.


def played_points(points, phase_last=None):
    """How many of a track's points the arm actually PLAYED.

    `phase_last` is the ordered coverage `verify_replay_from_frames` reports:
    the fraction of the file the arm's own trace reached. On a complete run it
    is 1.0 and this returns `points`. Returns None when coverage is unknown —
    an unmeasured count, never an assumed full one.
    """
    if phase_last is None or points is None:
        return None
    n = int(points)
    return max(1, int(round(float(phase_last) * (n - 1))) + 1)


def clock_ms_per_pt(replay_s, points, phase_last=None):
    """The PLAYER'S CLOCK, in ms per point, on the points that were played.

    WHY THE DENOMINATOR MATTERS (2026-09-10). `implied_ms_per_pt` divides the
    replay span by the COMMANDED point count, so on an incomplete run it is the
    product of the true clock and the completion fraction — not a clock at all.
    The two cut strokes of Newton's pass read 3.385 and 5.232 ms/pt that way;
    on the points they actually played they read 5.572 and 5.738, and over the
    ledger's 14 cuts the played-denominator clock spans 5.572-6.613 ms/pt
    (p50 5.737) against the complete-run distribution's 5.465-6.654. Which is
    the correct answer for a fixed-clock player that was interrupted: the clock
    did not change, the run stopped.

    The consequence of the old denominator was that `in_clock_envelope` became
    an accidental COMPLETENESS detector — False on exactly the incomplete runs
    of that pass and True on all 15 complete ones. That reads as validation and
    is an artefact; anyone calibrating the envelope on that field is fitting a
    mixture of clock and completeness. `dual_arm_verdicts` already excludes cut
    runs from its refit for this reason and says so; the run record did not.
    """
    n = played_points(points, phase_last)
    denom = (int(points) if n is None else n) - 1
    return float(replay_s) / max(denom, 1) * 1e3


#: THE PLAYER'S FIXED RAMP, in seconds, and the per-line rate band once it is removed.
#:
#: `replay_s` is not proportional to the point count: it carries a fixed ramp-in/ramp-out cost. Fitted
#: over 158 complete sha-matched upload runs spanning 587-12863 lines,
#:
#:     replay_s = 0.2322 s + 5.6737 ms x (N-1)     R2 0.99924
#:
#: and a median-pairwise-slope refit agrees (0.2372 s, 5.6779 ms), so the intercept is not an artefact of
#: the long files. Three independent fits now bracket it: 0.283036 s over 588-4281 points
#: (`dual_arm_verdicts.CLOCK_FIT`, 166 runs), 0.269 s (execution-channel OLS) and 0.2322 s here — this one
#: is used because it is the only one fitted across the full length span the screens are applied to.
#:
#: WHY THIS EXISTS (2026-09-17). `CLOCK_ENVELOPE_MS` is a FLAT band on `replay_s/(N-1)`, and the module
#: comment above it already said a flat band "cannot express the fixed ramp-in/ramp-out share of a short
#: track" — but the screen kept using it. The bias is measured: `replay_s/(N-1)` reads 0.249 ms (4.3 %)
#: higher on tracks under 1200 lines than on longer ones, and ALL 8 of the 158 runs the flat screen
#: rejects are short (8 of 35 short, 0 of 123 long). Removing the ramp collapses the short-vs-long gap to
#: 0.018 ms and leaves 4 rejects, 3 short and 1 long — genuine outliers instead of a length artefact.
#: This also ends the documented disagreement with `dual_arm_verdicts._clock_band`: both are now affine
#: in the point count, and this form extends past that band's measured 588-4281 span because a rate and a
#: constant extrapolate where a flat band does not.
PLAYER_RAMP_S = 0.2322
#: The slope of that same fit, in ms per line. It was only ever in the comment above while two callers
#: re-typed it; `expected_replay_s` is the one place it is applied now.
PLAYER_MS_PER_PT = 5.6737
#: p1/p99 of the de-ramped rate over those 158 runs, rounded outward to 0.05 ms.
CLOCK_RATE_ENVELOPE_MS = (5.35, 6.10)


def expected_replay_s(points) -> Optional[float]:
    """How long the upload player should take over *points* lines, by the fitted law."""
    if points is None or int(points) < 2:
        return None
    return PLAYER_RAMP_S + PLAYER_MS_PER_PT * 1e-3 * (int(points) - 1)


def played_fraction(points, *, replay_s=None, lines_sent=None) -> Optional[float]:
    """The share of the track the channel actually got through — the second completeness witness.

    EXACT on a stream, where the host counted the lines it sent; a ratio against the fitted player law
    on an upload, whose resolution `PLAYED_SHORT_FRAC` states. `lines_sent` wins when both are given,
    and None means no second witness exists, which `replay_verdict` treats as "coverage alone".
    """
    if points is None or int(points) < 2:
        return None
    if lines_sent is not None:
        return float(lines_sent) / float(points)
    exp = expected_replay_s(points)
    if replay_s is None or not exp:
        return None
    return float(replay_s) / exp


def deramped_ms_per_pt(replay_s, points, phase_last=None, ramp_s: float = PLAYER_RAMP_S):
    """The player's ASYMPTOTIC per-line clock: the ramp removed, so a short track reads like a long one.

    This is the quantity to compare between tracks of different lengths, and the one `clock_screen`
    judges. `clock_ms_per_pt` keeps its own meaning untouched for ledger continuity.
    """
    if replay_s is None or points is None:
        return None
    n = played_points(points, phase_last)
    denom = (int(points) if n is None else n) - 1
    if denom < 1:
        return None
    return (float(replay_s) - float(ramp_s)) / denom * 1e3


def in_clock_envelope(ms_per_pt):
    """RAW flat screen on ONE run's clock, kept for ledger continuity.

    LENGTH-BIASED BY CONSTRUCTION — see `PLAYER_RAMP_S`. It rejects short tracks for carrying the
    player's fixed ramp and nothing else; every reject in the ledger is a track under 1200 lines. Use
    `clock_screen`, which removes the ramp first. This is left in place because `in_clock_envelope` is a
    recorded field on 190 run records and its meaning there must not change under a reader.
    """
    if ms_per_pt is None:
        return None
    lo, hi = CLOCK_ENVELOPE_MS
    return bool(lo <= float(ms_per_pt) <= hi)


def clock_screen(replay_s, points, phase_last=None, *, channel: str = "upload",
                 period_s: Optional[float] = None) -> dict:
    """Is this run's clock inside the measured envelope? The ramp-corrected screen.

    Returns the raw and de-ramped rates, the band, and `ok` — with `ok=None` for "not applicable" rather
    than a guess. A CANFD stream is not judged against the player's envelope at all: its clock is the
    host's send period, which is commanded rather than measured, so the honest answer there is the
    lateness the streamer already reports.
    """
    out = dict(channel=channel, raw_ms_per_pt=None, ms_per_pt=None, ramp_s=None,
               band_ms=CLOCK_RATE_ENVELOPE_MS, ok=None, why=None)
    if str(channel) == "canfd":
        out["ramp_s"] = 0.0
        out["raw_ms_per_pt"] = out["ms_per_pt"] = (
            None if period_s is None else round(float(period_s) * 1e3, 4))
        out["why"] = ("host-timed stream: the clock is the commanded send period, not the player's, so "
                      "the player's envelope does not apply — read `canfd.late_ms_max` instead")
        return out
    raw = clock_ms_per_pt(replay_s, points, phase_last) if replay_s is not None else None
    de = deramped_ms_per_pt(replay_s, points, phase_last)
    out["raw_ms_per_pt"] = None if raw is None else round(float(raw), 4)
    out["ms_per_pt"] = None if de is None else round(float(de), 4)
    out["ramp_s"] = PLAYER_RAMP_S
    if de is None:
        out["why"] = "the replay span or the point count is unknown, so there is no clock to screen"
        return out
    lo, hi = CLOCK_RATE_ENVELOPE_MS
    out["ok"] = bool(lo <= de <= hi)
    if not out["ok"]:
        out["why"] = (f"{de:.3f} ms/pt with the {PLAYER_RAMP_S:g} s ramp removed is outside the measured "
                      f"[{lo}, {hi}] ms band (raw {raw:.3f} on {points} points)")
    return out


def push_error_codes(frames_path, t_start=None, t_end=None) -> dict:
    """Every error code the UDP push carried during a capture, counted.

    WHY (2026-09-15): the run record reads the controller's state before and
    after the program, and both were clean on right slot 42 while the frames
    in between carried 4105 ("Speed out of limit") on 1,139 of them, 2.3–8.0 s
    into an 8.2 s replay — the controller flags, keeps playing and clears it,
    and nothing in the record said so. This is the capture's own account.

    Returns `{code: {"frames": n, "first_s": t, "last_s": t}}` with times
    relative to the capture's first frame; arm codes as their number, joint
    codes as `"joint<i>:<code>"`. Empty when the push carried none.
    """
    out: dict = {}
    t0 = None
    with open(frames_path) as fh:
        for line in fh:
            try:
                d = json.loads(line)
                f = d["frame"]
                t = float(d["t_mono"])
            except Exception:                                          # noqa: BLE001
                continue
            if t0 is None:
                t0 = t
            if (t_start is not None and t < t_start) or (t_end is not None and t > t_end):
                continue
            seen = [str(c) for c in (f.get("err") or []) if c]
            jec = ((f.get("joint_status") or {}).get("joint_err_code")) or []
            seen += [f"joint{i + 1}:{c}" for i, c in enumerate(jec) if c]
            for c in seen:
                e = out.setdefault(c, {"frames": 0, "first_s": round(t - t0, 3), "last_s": None})
                e["frames"] += 1
                e["last_s"] = round(t - t0, 3)
    return out


def replay_verdict(vr: dict, settle: Optional[dict] = None, *,
                   played_fraction: Optional[float] = None,
                   end_speed_deg_s: Optional[float] = None) -> dict:
    """Turn a `verify_replay_from_frames` result into the run's outcome.

    Kept out of the CLI so it can be tested without hardware: the capture path
    cannot be exercised on the emulator (its push is a Python callback, not a
    socket), so this is the only place the F112 verdict rules are checked.

      complete  — the ordered progress covers the whole file.
      truncated — the capture stopped while the arm was still moving, so where
                  the run ended was never recorded. `complete` is None:
                  UNRESOLVED, never "cut".
      cut       — the arm came to rest short of the end AND the replay ran
                  short, or there is no second witness to say otherwise.

    *played_fraction* is that second witness: the replay's length over the
    length it was expected to have (`lines_sent / points` on a stream, the
    fitted player law on an upload). See `PLAYED_SHORT_FRAC` for what it can
    and cannot resolve. Omitting it leaves the verdict on coverage alone.
    """
    # ── WHAT THE MATCHER CAN AND CANNOT RESOLVE (2026-09-18) ───────────────────────────────────────
    # The rule was `phase_first <= 0.01 and phase_last >= 0.99`, both fractions of the WHOLE file. But
    # the projection only resolves position where the command moves more than `STILL_TAIL_TOL_DEG` per
    # line, and a single-joint sweep spends much of its length below that: 44 % of `sweep_j7_fast_raw`
    # (a 151-line near-still opening and a 24-line decelerating tail), 133 of the 7887 lines of
    # `sweep_j6_slow_raw`, all 1601 lines of the J7 hold probe's `still` shape. Those gates therefore
    # demanded coverage of stretches that carry no position, and every failure was one-directional:
    # the verdict claimed the arm STOPPED EARLY on runs where nothing of the kind was observed.
    #
    # So the resolvable span is the bar, and the two ends are judged by different instruments because
    # the two instruments fail in the same direction. F112: a global nearest match JUMPS AHEAD on a
    # self-similar path. At the START that makes `phase_first_global` an upper bound on where the run
    # began — safe to use, and resolution-independent, where the ordered `phase_first` cannot even lock
    # until the command moves (0.1743 on a `sweep_j4_fast_smooth5_canfd` run that finished 0.006 deg
    # from its last line with all 592 lines sent). At the END the same jumping-ahead would manufacture a
    # completion, so there the bar is the ordered `phase_last` and nothing else.
    #
    # The start test is weak by construction and is kept as a sanity check, not as evidence: the player
    # always starts a program at its first line and a stream sends 0..N-1 in order with `lines_sent` to
    # prove it, so neither channel can begin mid-file. Completeness rests on the end test.
    #
    # AN ABSENT SPAN IS NOT AN UNRESOLVABLE ONE. The span fields arrived on 2026-09-18; the 190 records
    # written before carry a `replay_check` without them, as does any caller passing a hand-built dict.
    # Present-and-None means the track genuinely has no resolvable line, which is unmeasurable. ABSENT
    # means this result predates the measure, and it is judged under the rule it was measured with.
    legacy = "first_resolvable" not in vr and "last_resolvable" not in vr
    fr, lr = vr.get("first_resolvable"), vr.get("last_resolvable")
    # TRUNCATION HAS TWO WITNESSES AND EITHER IS ENOUGH. `settle` is the run path's own post-run wait,
    # on records written since 2026-09-04; `end_speed_deg_s` is the capture's last frame, which covers
    # the rest. Reading `settle` alone scored `run_slot71_205442` — arm moving at 83.5 deg/s when its
    # capture stopped, correctly UNRESOLVED since the F112 repair — as a cut at 88.9 %.
    truncated = bool((settle and not settle.get("at_rest", True))
                     or (end_speed_deg_s is not None
                         and float(end_speed_deg_s) > TRUNCATED_DEG_S))
    if not legacy and (fr is None or lr is None):
        return {
            "complete": None,
            "capture_truncated": truncated,
            "played_fraction": (None if played_fraction is None
                                else round(float(played_fraction), 4)),
            "outcome": "unresolved_unmeasurable",
            "unmeasurable": ("no commanded line clears the matcher's resolution floor "
                             f"({STILL_TAIL_TOL_DEG:g} deg/line), so ordered coverage cannot judge "
                             "this track: read the duration, or a stream's own line count"),
            "tag": "** UNRESOLVED (coverage not measurable on this track)",
        }
    if legacy:
        start_bar, end_bar = 0.01, 0.99                    # exactly the pre-2026-09-18 rule
    else:
        # 2 lines of slack on the span's ends, never less than the historical 1 %. An UNKNOWN point
        # count falls back to 1 % — it must not widen the gate (a 2/1 fallback would pass everything).
        npts = vr.get("points")
        edge = max(0.01, 2.0 / (float(npts) - 1.0)) if (npts and float(npts) > 3) else 0.01
        # The start bar is the unresolvable PREFIX: every line before `first_resolvable` is within the
        # resolution floor of line 0, so landing anywhere inside it IS "started at the beginning". A
        # flat 0.01 read `sweep_j6_slow_raw` as a cut for a global-nearest at line 89 of a 133-line
        # prefix, on a run that finished 0.006 deg from its literal last line.
        start_bar, end_bar = max(edge, fr), lr - edge
    start = vr.get("phase_first_global", vr["phase_first"])
    complete = bool(start <= start_bar and vr["phase_last"] >= end_bar)
    # A CUT MUST BE SHORT ON BOTH INSTRUMENTS. Ordered coverage is the only progress measure we have,
    # and on a single-joint path it can lock onto the wrong cycle: 7 runs read 76-98 % coverage while
    # their replay ran 99-122 % of its expected length — the 6 `probe_left_j7_cruise_v45_a600` replays,
    # whose final pose has a twin 0.010 deg away at 6.6 % of the file, and one `sweep_j5_fast_raw`. The
    # player cannot both play every line and stop three quarters of the way through, so those two
    # readings cannot both be right, and calling them cuts records a stop nobody observed.
    ran_short = (played_fraction is not None and float(played_fraction) < PLAYED_SHORT_FRAC)
    ambiguous = bool(not complete and played_fraction is not None and not ran_short)
    return {
        "complete": None if ((truncated or ambiguous) and not complete) else complete,
        "capture_truncated": truncated,
        "played_fraction": (None if played_fraction is None else round(float(played_fraction), 4)),
        "outcome": ("complete" if complete else
                    "unresolved" if truncated else
                    "unresolved_ambiguous" if ambiguous else "cut"),
        "unmeasurable": (None if not ambiguous else
                         f"the replay ran {float(played_fraction) * 100:.0f} % of its expected length "
                         f"while ordered coverage reached {vr['phase_last'] * 100:.1f} % — the two "
                         f"instruments disagree, so where this run ended is not resolved"),
        "tag": ("COMPLETE" if complete else
                "** UNRESOLVED (capture truncated)" if truncated else
                "** UNRESOLVED (full-length replay, short coverage)" if ambiguous else
                "** INCOMPLETE"),
    }


# The SDK's event callback is ONE PER PROCESS and the ctypes trampoline must
# outlive every call into the library: garbage-collecting it crashes the SDK's
# receive thread (F91 class). Both live here, at module scope, registered once
# and never replaced.
_EVENT_CB_REF = None            # STRONG ref to the ctypes trampoline — never drop
#: handle_id -> the ArmEventLog collecting for that arm. A dict, not a slot:
#: the callback is process-global and cannot be unregistered, so two arms must
#: share it rather than overwrite each other.
_EVENT_SINKS: dict = {}
_EVENT_NAMES = {0: "none", 1: "trajectory_state", 2: "program_run_finish"}
_EVENT_DEVICES = {0: "joint", 1: "gripper", 2: "hand", 3: "lift", 4: "expand"}


class ArmEventLog:
    """Capture the controller's arrival / program-finish events around a run.

    Why this exists (F114): `rm_set_program_id_run(block=1)` returns -4 on
    exactly the runs that cut and 0 on exactly the runs that complete, and the
    vendor's -4 means "the run state stopped but the SUCCESS MESSAGE was not
    received". That is the absence of `RM_PROGRAM_RUN_FINISH_E`. This logs the
    event stream itself, so the next session can say whether the finish event
    is missing (the program was stopped) and what the last
    `RM_CURRENT_TRAJECTORY_STATE_E` before it said — its `trajectory_state`
    flag is the controller's own "did this trajectory reach its target", and
    its `device` field says WHICH device the arrival belonged to.

    Cannot be exercised offline: the emulator fires trajectory-state events for
    its own moves but never a program-finish, and its program double fires
    none. The emulator models the robot's motion, not its event stream (F89).
    """

    def __init__(self, handle_id=None, monitor=None):
        self.handle_id = handle_id
        #: LIVE notification target. The finish event is the completeness label
        #: (F123) and it arrives WITH `run_state 0`, so the monitor has to hear
        #: it from here — `run_verified` only tells it at end-of-run, which is
        #: far too late to stop rung 1 from stopping the partner mid-stroke.
        self.monitor = monitor
        self.events = []
        #: True once `start()` has a live ctypes trampoline. `run_verified`
        #: reads it to decide whether the probe is still needed (M43 AUTO) —
        #: a module global cannot answer that, because the callback is
        #: process-wide and another arm may have registered it.
        self.registered = False
        self._t0 = None

    def start(self, robot):
        """Register once per process; later calls ADD a sink.

        The callback is one per process and cannot be unregistered, so a second
        arm used to silently STEAL the stream from the first: `_EVENT_SINK` was
        a single slot and `start` overwrote it. For a dual-arm run both arms
        need their own log, so the sink is a registry keyed by `handle_id` and
        every event is delivered to the sink that owns it — falling back to
        broadcasting when the id is unknown, which is what a single-arm run has
        always effectively done.
        """
        global _EVENT_CB_REF
        self._t0 = time.monotonic()
        self.events = []
        _EVENT_SINKS[self.handle_id] = self
        if _EVENT_CB_REF is not None:
            self.registered = True
            return True
        try:
            from Robotic_Arm.rm_ctypes_wrap import rm_event_callback_ptr
        except Exception:                                              # noqa: BLE001
            self.registered = False
            return False

        def _on_event(data):
            # Route to the arm that owns this event. An unknown or missing id
            # broadcasts, which is what the single-slot version effectively did
            # and keeps every existing single-arm capture identical.
            hid = getattr(data, "handle_id", None)
            sinks = ([_EVENT_SINKS[hid]] if hid in _EVENT_SINKS
                     else list(_EVENT_SINKS.values()))
            for sink in sinks:
                try:
                    sink._record(data)
                except Exception:                                      # noqa: BLE001
                    pass                   # never raise into the SDK's thread

        try:
            _EVENT_CB_REF = rm_event_callback_ptr(_on_event)
            robot.rm_get_arm_event_call_back(_EVENT_CB_REF)
        except Exception:                                              # noqa: BLE001
            _EVENT_CB_REF = None
            self.registered = False
            return False
        self.registered = True
        return True

    def _record(self, d):
        et = int(getattr(d, "event_type", 0) or 0)
        dev = int(getattr(d, "device", -1) or 0)
        self.events.append({
            "t": round(time.monotonic() - (self._t0 or time.monotonic()), 3),
            "handle_id": getattr(d, "handle_id", None),
            "event": _EVENT_NAMES.get(et, str(et)),
            "trajectory_state": bool(getattr(d, "trajectory_state", False)),
            "device": _EVENT_DEVICES.get(dev, str(dev)),
            "trajectory_connect": getattr(d, "trajectory_connect", None),
            "program_id": getattr(d, "program_id", None),
        })
        if (self.monitor is not None
                and _EVENT_NAMES.get(et) == "program_run_finish"):
            try:
                self.monitor.note_finish_event()
            except Exception:                                          # noqa: BLE001
                pass                       # never raise into the SDK's thread

    def stop(self) -> dict:
        """Detach the sink (the registration itself is never undone) and
        summarise. `finish_seen` false on a run the controller reported as
        ended is the signature the -4 describes."""
        for k, v in list(_EVENT_SINKS.items()):
            if v is self:
                _EVENT_SINKS.pop(k, None)
        mine = [e for e in self.events
                if self.handle_id is None or e["handle_id"] in (None, self.handle_id)]
        fin = [e for e in mine if e["event"] == "program_run_finish"]
        arr = [e for e in mine if e["event"] == "trajectory_state"]
        return {
            "registered": _EVENT_CB_REF is not None,
            "events": mine[-40:],
            "n_events": len(mine),
            "finish_seen": bool(fin),
            "finish_program_id": fin[-1]["program_id"] if fin else None,
            "arrivals": len(arr),
            "failed_arrivals": [e for e in arr if not e["trajectory_state"]][-8:],
            "last_device": arr[-1]["device"] if arr else None,
        }


def _is_emulated_handle(robot) -> bool:
    """Is this handle the emulator (possibly behind the program surface)?

    Used only to keep an offline run HONEST. The emulator models the robot's
    motion, not its event stream (F89): its program double fires no
    `program_run_finish` at all, so a missing finish event offline says nothing
    about the run and must be recorded as UNMEASURED rather than as a cut. On
    hardware every one of the 94 records carrying `arm_events` delivered at
    least one event (min 1, max 3), and the four that missed the finish are
    exactly the four incomplete runs.
    """
    if (type(robot).__module__ or "").startswith("alix_emulator"):
        return True
    inner = getattr(robot, "_emulated_inner", None)
    return inner is not None and (type(inner).__module__ or "").startswith("alix_emulator")


def wait_until_at_rest(robot, *, poll_s: float = 0.025, still_deg_s: float = 1.6,
                      need: int = 6, timeout_s: float = 20.0,
                      still_deg: float = None) -> dict:
    """Block until the arm has actually stopped, and report what it was doing.

    `rm_get_program_run_state` returning 0 does NOT mean the arm is at rest: on
    2026-09-02 six of seven "cut" runs were still executing at 13-96 deg/s when
    the capture was stopped 0.6 s later, which truncated the evidence and made
    the runs unjudgeable (F112). Every capture must outlive the motion.

    THE GATE IS A SPEED, EXPLICITLY (2026-09-05). It used to be a per-poll
    POSITION step (`still_deg` 0.08 deg) which is only a speed once you divide
    by the poll interval — 0.08 / 0.05 s = 1.6 deg/s — so shortening the poll
    silently loosened the gate in proportion. Stating it as deg/s decouples the
    two, and the interval then halves (0.05 -> 0.025 s) with the SAME six
    consecutive observations under the SAME 1.6 deg/s: 0.15 s of settle per
    task instead of 0.30, ~1.8 s per 12-task flow, and no protection traded.
    Measured across the two complete flows of 2026-09-04: at_rest true on 24 of
    24, peak 0.051-1.213 deg/s, every one exiting at the floor — so the wait has
    never yet had to wait, and this shortens the floor rather than the gate.

    `still_deg` is accepted for compatibility and converted at the caller's own
    poll interval.

    Returns {'waited_s', 'was_moving', 'peak_deg_s', 'at_rest'}. `was_moving`
    true means the program reported its end while the arm was still moving —
    the run's own duration is then a lower bound, not the motion time.
    """
    if still_deg is not None:
        still_deg_s = float(still_deg) / max(poll_s, 1e-6)
    t0 = time.monotonic()
    prev, quiet, peak = None, 0, 0.0
    while time.monotonic() - t0 < timeout_s:
        try:
            st = robot.rm_get_current_arm_state()
        except Exception:                                              # noqa: BLE001
            break
        q = None
        if isinstance(st, (tuple, list)) and len(st) >= 2 and isinstance(st[1], dict):
            q = st[1].get("joint")
        elif isinstance(st, dict):
            q = st.get("joint")
        if q is None:
            break
        q = [float(x) for x in q]
        now = time.monotonic()
        if prev is not None:
            dt = max(now - prev[0], 1e-3)
            step = max(abs(a - b) for a, b in zip(q, prev[1]))
            peak = max(peak, step / dt)
            quiet = quiet + 1 if (step / dt) < still_deg_s else 0
            if quiet >= need:
                return {"waited_s": now - t0, "was_moving": (now - t0) > (need + 2) * poll_s,
                        "peak_deg_s": peak, "at_rest": True}
        prev = (now, q)
        time.sleep(poll_s)
    return {"waited_s": time.monotonic() - t0, "was_moving": True,
            "peak_deg_s": peak, "at_rest": False}


#: Above this, the arm was still moving at the last captured frame, so where the run ENDED was never
#: observed. 5.0 deg/s is the F112 threshold (`repair_run_records.STILL_DEG_S`), kept identical so the
#: two generations of records are judged the same way.
#:
#: IT IS THE CALLER'S MEASUREMENT, not the matcher's, and the reason is the window. `replay_verdict`
#: takes it as `end_speed_deg_s`, read from the SDK's `joint_speed` at the LAST FRAME OF THE CAPTURE.
#: Deriving it inside the matcher from the windowed positions was tried and is wrong: the window is
#: deliberately clamped at the moment the arm comes to REST (`rest_time_after`, so the analysis never
#: reaches into the park movej), so the 0.1 s before its end is the deceleration into that rest. On
#: five parked runs that read 8.8-10.5 deg/s against the SDK's 0.4-1.1 at the capture's real end — a
#: false "truncated" on every record whose window is clamped that way.
TRUNCATED_DEG_S = 5.0
#: Below this fraction of its expected length, a replay demonstrably RAN SHORT — the second, independent
#: witness on completeness, and the only one that does not go through the position matcher.
#:
#: Measured over the ledger (2026-09-18). For upload the expectation is the fitted player law
#: (`PLAYER_RAMP_S` + 5.6737 ms x (N-1)); for a stream it is `lines_sent / points`, which is exact. Over
#: 202 runs whose coverage is complete the ratio spans **0.854-1.294** — the player is not a metronome
#: and the law is a fit — so this instrument CANNOT resolve a shortfall of 10-15 %. It resolves a large
#: one cleanly: every genuine cut in the ledger reads 0.010, 0.073, 0.078, 0.419 or 0.664, none of them
#: near the bar.
#:
#: 0.85 is therefore "no shorter than the shortest run we have ever confirmed complete", and a run above
#: it whose coverage falls short is UNRESOLVED rather than cut — the two instruments disagree and neither
#: settles it. That is the F112 direction: not observing where a run ended is not evidence that it ended
#: early. The cost is real and stated: a true cut in the last 15 % of a file reads unresolved here, so it
#: is excluded from analysis instead of counted as a cut.
PLAYED_SHORT_FRAC = 0.85
def verify_replay_from_frames(frames_path, cmd_rows, t_start=None, t_end=None, capture=None, *,
                              clock_s: float = REPLAY_CLOCK_S):
    """What a replay must actually prove, measured from a --record-joints
    capture. Duration was only ever a PROXY for these (F108):

      * completeness — the arm visited the whole track, start to end
        (`phase_first`/`phase_last` as fractions of the point count);
      * fidelity — how far it ever strayed from the authored path
        (nearest-point joint distance, degrees: p50/p99/max);
      * cadence — the push interval, which validates the timing itself;
      * clock — executed ms/pt, now an OBSERVABLE against CLOCK_ENVELOPE_MS.

    *clock_s* is the period the lines were commanded at — the player's nominal clock for an upload
    run, the stream's own period for a CANFD one. It only places the matcher's search window.

    Nearest-point DISTANCE uses the whole track (no ordering assumption): a
    windowed matcher races forward on one bad match and reports nonsense
    (caught 2026-09-01 — it claimed 40 deg median on a clean run).

    PROGRESS is a different question and must not be read off the same match.
    `near.max()` is "the furthest-along point the arm was ever NEAR", and on a
    raster the stripes are geometrically close, so it jumps ahead: it reported
    89.5 % on a bottomlid run that ended, verifiably, at 49.4 % (F112). Progress
    therefore comes from two ordered measures instead:

      * `phase_end` — the commanded index nearest the arm's LAST captured
        position, with `phase_end_dev_deg` and `phase_end_margin_deg` (the best
        distance from anywhere outside +-3 % of the file). A small dev with a
        large margin makes it unambiguous, and it agrees with the lag-budget
        matcher on every 2026-09-01/02 run.
      * `phase_last` — the ordered matcher: nearest point inside a LAG BUDGET
        around the nominal-clock index, which neither races nor stalls.

    `phase_last_global` keeps the old number so historical records stay
    comparable. Completeness is judged on the ordered pair, never on the global.
    """
    import numpy as np
    cmd = np.asarray(cmd_rows, dtype=float) / 1000.0
    if capture is not None:
        cts, _cst, cqs = capture
        ts = [t for t, q in zip(cts, cqs) if q is not None]
        qs = [q for q in cqs if q is not None]
    else:
        ts, qs = [], []
        with open(frames_path) as fh:
            for line in fh:
                try:
                    d = json.loads(line)
                    jp = d["frame"]["joint_status"]["joint_position"]
                except Exception:                                      # noqa: BLE001
                    continue
                if len(jp) == 7:
                    ts.append(d["t_mono"])
                    qs.append([v / 1000.0 for v in jp])
    if len(ts) < 10:
        return None
    ts = np.asarray(ts)
    qs = np.asarray(qs)
    m = np.ones(len(ts), bool)
    if t_start is not None:
        m &= ts >= t_start
    if t_end is not None:
        m &= ts <= t_end
    tw, qw = ts[m], qs[m]
    if len(tw) < 10:
        return None
    dev = np.empty(len(qw))
    near = np.empty(len(qw), dtype=int)
    # The max-norm distance from every captured sample to every commanded point.
    # It stays a FULL search on purpose (F112: a windowed matcher reported
    # 40 deg median on a clean run), but it is accumulated one joint at a time
    # into a (b, N) buffer instead of materialising (b, N, 7) — that temporary
    # was 48 MB per chunk on a 4281-point track and made this call ~0.9 s, once
    # per task on the critical path between tasks.
    for lo in range(0, len(qw), 200):                 # chunked: (b, N) stays small
        blk = qw[lo:lo + 200]
        d = np.abs(blk[:, :1] - cmd[None, :, 0])
        for j in range(1, cmd.shape[1]):
            np.maximum(d, np.abs(blk[:, j:j + 1] - cmd[None, :, j]), out=d)
        dev[lo:lo + len(blk)] = d.min(axis=1)
        near[lo:lo + len(blk)] = d.argmin(axis=1)
    n = len(cmd) - 1
    cadence = np.diff(tw) * 1e3
    # --- ordered progress (see the docstring) --------------------------------
    d_end = np.abs(cmd - qw[-1]).max(axis=1)
    k_end = int(d_end.argmin())
    band = max(1, len(cmd) // 33)
    outside = np.ones(len(cmd), bool)
    outside[max(0, k_end - band):k_end + band] = False
    margin = float(d_end[outside].min()) if outside.any() else float("nan")
    lag_lo, lag_hi = 400, 150            # points behind / ahead of the nominal clock
    t_rel = tw - tw[0]
    # Vectorised, and EXACTLY equivalent to the per-frame loop it replaces —
    # same candidate window [nom-400, nom+150), same max-norm, same argmin
    # tie-break (the first index wins in both). The loop cost 1.10 s on a
    # 4281-point track here and ~3x that on the robot host, once per task on
    # the critical path between tasks; this is ~40x faster. Windows are taken
    # from a stride view and the out-of-range candidates masked, rather than
    # sliced per frame, so the candidate set is identical near both ends.
    # ANCHORED ON THE CHANNEL'S OWN PERIOD (2026-09-17). This was REPLAY_CLOCK_S for every channel,
    # so a CANFD stream at another period drifted out of the candidate window — 20 lines per second
    # at 5.0 ms, which reads a COMPLETE 39 s sweep as a cut at 93 %.
    nom = np.minimum(len(cmd) - 1, (t_rel / float(clock_s)).astype(int))
    lo_i = np.maximum(0, nom - lag_lo)
    hi_i = np.minimum(len(cmd), nom + lag_hi)
    width = lag_lo + lag_hi
    phase_seq = np.empty(len(qw), dtype=int)
    if len(cmd) >= width:
        starts = np.minimum(lo_i, len(cmd) - width)
        win = np.lib.stride_tricks.sliding_window_view(cmd, width, axis=0)
        offs = np.arange(width)
        for lo in range(0, len(qw), 256):                # bounded memory
            sl = slice(lo, min(lo + 256, len(qw)))
            idx = starts[sl][:, None] + offs[None, :]
            d = np.abs(win[starts[sl]] - qw[sl][:, :, None]).max(axis=1)
            d = np.where((idx >= lo_i[sl][:, None]) & (idx < hi_i[sl][:, None]), d, np.inf)
            phase_seq[sl] = starts[sl] + d.argmin(axis=1)
    else:
        for i in range(len(qw)):
            phase_seq[i] = lo_i[i] + int(
                np.abs(cmd[lo_i[i]:hi_i[i]] - qw[i]).max(axis=1).argmin())
    tail = max(3, len(phase_seq) // 20)
    last_idx = float(np.percentile(phase_seq[-tail:], 90))
    ordered_raw = last_idx / n
    # A TRACK THAT ENDS HOLDING STILL (2026-09-15). A taught recording keeps
    # the operator's final pause — 88 identical lines (0.49 s) on
    # 42_taught_lid_sid_01_right — and identical commanded points give the
    # ordered matcher no reason to advance: argmin keeps the FIRST of a tie,
    # so `phase_last` stalled at the start of the still tail and a run that
    # verifiably ended ON the final line (0.008 deg off it, at rest) read
    # "cut" at 0.935. The 2026-09-14 ledger's two false negatives on the
    # taught rows are the same mechanism. The still tail commands the pose
    # the arm is holding, so it counts as visited when the matcher has reached
    # its start AND the arm's last captured position IS the final commanded
    # point. A run cut short anywhere else fails the second test; a run cut
    # inside the tail is indistinguishable from complete at this level.
    still = np.abs(cmd - cmd[-1]).max(axis=1) <= STILL_TAIL_TOL_DEG
    moving = np.flatnonzero(~still)
    n_still = int(len(cmd) - (moving.max() + 1)) if moving.size else int(len(cmd))
    # THE STALL IS A RESOLUTION LIMIT, AT BOTH ENDS (2026-09-18). The rule above keyed on distance to
    # the FINAL pose, which catches an operator's closing pause (88 identical lines) but not a
    # DECELERATING tail: `sweep_j7_fast_raw` steps 0.005-0.046 deg over its last 24 lines, only 5 of
    # which sit within 0.05 deg of the final point, so the matcher stalled 12 lines early and all four
    # runs read `cut` at an identical 0.97468 — the giveaway that it is a property of the FILE, not the
    # run. The same blindness opens the file: its first 151 lines step under 0.15 deg, which is why
    # `phase_first` read 0.0101 against a 0.01 gate and 9 more runs failed by a hair.
    #
    # Below ~STILL_TAIL_TOL_DEG per line the ordered matcher cannot tell one line from the next
    # (argmin keeps the first of a tie), so coverage is only measurable over the RESOLVABLE span.
    # This reports that span and credits what lies outside it, which subsumes the final-pause rule:
    # 88 identical lines step 0, so they are unresolvable too.
    step = (np.abs(np.diff(cmd, axis=0)).max(axis=1) if len(cmd) > 1
            else np.zeros(0, dtype=float))
    res = np.flatnonzero(step >= STILL_TAIL_TOL_DEG)
    first_res = float(res.min()) if res.size else None
    last_res = float(res.max() + 1) if res.size else None
    tail_credited = False
    slack = max(2, len(cmd) // 100)
    if last_res is not None and last_res < len(cmd) - 1:
        if last_idx >= last_res - slack and float(d_end[-1]) <= STILL_TAIL_END_TOL_DEG:
            last_idx, tail_credited = float(n), True
    elif n_still > 1 and moving.size:
        if last_idx >= float(moving.max()) - slack and float(d_end[-1]) <= STILL_TAIL_END_TOL_DEG:
            last_idx, tail_credited = float(n), True
    ordered_last = last_idx / n
    return {
        "phase_first": float(np.percentile(phase_seq[:max(3, len(phase_seq) // 20)], 10)) / n,
        "phase_last": ordered_last,
        "phase_last_ordered_raw": ordered_raw,
        "still_tail_points": n_still,
        "still_tail_credited": tail_credited,
        # the span coverage can actually be measured over, as fractions of the point count; None
        # when NO line clears the resolution floor (an all-still track: the probe's `still` shape)
        "points": int(len(cmd)),
        "first_resolvable": (None if first_res is None else first_res / n),
        "last_resolvable": (None if last_res is None else last_res / n),
        "resolvable_lines": int(res.size),
        "phase_end": float(k_end) / n,
        "phase_end_dev_deg": float(d_end[k_end]),
        "phase_end_margin_deg": margin,
        "phase_first_global": float(near.min()) / n,
        "phase_last_global": float(near.max()) / n,
        "monotone_frac": float(np.mean(np.diff(near) >= 0)),
        "dev_p50_deg": float(np.median(dev)), "dev_p99_deg": float(np.percentile(dev, 99)),
        "dev_max_deg": float(dev.max()),
        "cadence_ms_median": float(np.median(cadence)),
        "cadence_ms_p99": float(np.percentile(cadence, 99)),
        "frames": int(len(tw)),
    }


#: How long the arm must hold still for the motion to count as OVER.
#:
#: Measured across all 53 parked captures in the ledger (2026-09-05): the
#: longest still stretch INSIDE a replay is **0.235 s** — the arm pausing at a
#: raster reversal — and the shortest still gap BEFORE the park movej is
#: **0.405 s**. 0.30 s sits between them with 0.065 s of margin below and
#: 0.105 s above. Re-measure before trusting it on a different track family.
REST_STILL_S = 0.30
REST_STILL_DEG = 0.02


def rest_time_after(times, positions, t_from: float,
                    still_s: float = REST_STILL_S,
                    eps_deg: float = REST_STILL_DEG) -> Optional[float]:
    """When the arm first holds STILL for `still_s`, at or after `t_from`.

    For bounding an analysis window at the end of the motion being measured,
    on a capture that outlives it. Since 2026-09-04 every replay is followed by
    a park movej under the same capture, so "the last frame" is no longer "the
    resting position of the run" — `run_one` stamps `park_t_mono` for exactly
    this, and this function is the fallback for records written before it.

    Returns None if the arm never settles for that long, which is the honest
    answer: the caller should then not claim to know where the run ended.
    """
    import numpy as np
    t = np.asarray(times, dtype=float)
    q = np.asarray(positions, dtype=float)
    if len(t) < 3 or len(t) != len(q):
        return None
    step = np.abs(np.diff(q, axis=0)).max(axis=1)
    tm = t[1:]
    start = None
    for i in range(len(tm)):
        if tm[i] < t_from:
            continue
        if step[i] < eps_deg:
            if start is None:
                start = tm[i]
            elif tm[i] - start >= still_s:
                return float(tm[i])
        else:
            start = None
    return None


def slow_stop(robot, log: Callable[[str], None] = print) -> int:
    """Best-effort on-trajectory stop (rm_set_arm_slow_stop): decelerates on the
    current path rather than freezing joints. Used when a program run must be
    aborted mid-replay; never raises."""
    try:
        ret = int(robot.rm_set_arm_slow_stop())
    except Exception as exc:                                           # noqa: BLE001
        log(f"** slow stop FAILED to send ({exc}) — stop the arm from the pendant")
        return -1
    log(f"slow stop sent (ret={ret})" if ret == 0
        else f"** slow stop returned {ret} — verify the arm from the pendant")
    return ret


# ---------------------------------------------------------------------------
# B13 — the three-rung cut detector
# ---------------------------------------------------------------------------
#: How many CONSECUTIVE push frames must disagree with `move_through_joint`
#: before rung 0 believes the phase label. Measured on the cut captures in the
#: ledger, the status change appears in the very next frame after the last
#: `move_through_joint` one (+4.8 to +5.2 ms; push cadence p50 4.9954 to
#: 5.0047 ms across those captures), so three frames is ~15 ms of confirmation
#: against one dropped datagram.
STATUS_CONFIRM_FRAMES = 3
#: How many consecutive STILL frames make "the arm stopped moving" true.
#: It must exceed the longest still stretch that occurs INSIDE a legitimate
#: replay — 0.235 s, the arm pausing at a raster reversal, measured across all
#: 53 parked captures (see REST_STILL_S). 60 frames x 5 ms = 0.30 s, the same
#: number REST_STILL_S carries and for the same reason.
MOTION_CEASE_FRAMES = 60
#: How long the arm is allowed to COAST after the controller stops labelling
#: the phase `move_through_joint`, before its position is judged.
#:
#: This is a real, measured lag, and it is why rung 0 cannot simply read the
#: status label. Over the 92 complete captures whose on-disk track still
#: matches the record's `track_sha256`, the arm is up to **6.29 deg** from the
#: last commanded point at the instant the label changes, and it coasts in over
#: the next fraction of a second: 3.41 deg at +0.25 s, 2.37 deg at +0.40 s. The
#: three cut captures in the same set are 7.75 / 55.56 / 88.84 deg away at the
#: label change and 7.86 / 51.54 / 89.96 deg at +0.25 s — so 0.25 s is where
#: the two populations separate by better than 2x.
STATUS_SETTLE_S = 0.25
#: Max distance from the last commanded point that still counts as ARRIVED,
#: judged after STATUS_SETTLE_S. Complete: <= 3.41 deg. Cut: >= 7.86 deg.
ARRIVAL_TOL_DEG = 5.0
#: A distance no arm can coast away in a settle window, so it needs no window:
#: rung 0 trips on it IMMEDIATELY. The largest coast measured on a complete run
#: is ~2.9 deg and the largest residual 6.29 deg; two of the three cut captures
#: are 55 and 89 deg out at the label change.
IMMEDIATE_CUT_DEG = 15.0
#: Ordered progress that counts as the end of the file. Over the same 92
#: complete captures, `phase_end` at +0.25 s is >= 0.99 on 92 of 92; the cuts
#: read 0.0172, 0.7805 and 0.9524.
COMPLETE_PHASE = 0.99
#: Fraction of `still_frames x the nominal 5 ms push period` that the stillness
#: must ALSO last in WALL-CLOCK seconds before `_ceased` believes it.
#:
#: MOTION_CEASE_FRAMES is a COUNT, and a count is only a duration while the
#: feed runs at the push cadence (measured p50 5.0010 ms, 5.008-5.013 ms
#: spread). A caller that decimates the stream — one frame in five off a
#: replayed capture, say — would otherwise reach 60 frames in 1.5 s of a
#: 0.235 s raster reversal and call a legitimate pause a freeze. 0.95 leaves
#: room for float accumulation and cadence jitter while still putting the floor
#: (0.285 s at the default 60) 21 % above the longest still stretch ever
#: measured INSIDE a replay (0.235 s; see REST_STILL_S).
MOTION_CEASE_CLOCK_FRACTION = 0.95
#: How much longer than STATUS_SETTLE_S rung 0 keeps trying to judge when the
#: frames in the window carried no `joint_position`. Two more settle windows;
#: after that the stroke is marked `unjudged` and SAID SO, rather than the rung
#: quietly retiring itself.
JUDGE_GRACE_S = 0.50


#: How long rung 1 holds an EARLY `run_state 0` before calling it a cut, giving
#: the controller's own program_run_finish event time to arrive. The event lands
#: with the transition, so this costs no meaningful latency — and without it a
#: legitimately fast program reads as a cut and stops the partner mid-stroke
#: (measured 2026-09-10, row 3 leg 1).
EARLY_TRIP_GRACE_S = 0.30


class MotionMonitor:
    """The cut detector as a FEEDABLE object: no sockets, no SDK, no threads.

    THE PROBLEM (B13). Today the only cut signal is the blocking probe's
    return, and it arrives **0.89 to 2.37 s after the cut** — the seven cut
    records in `Resource/runs/tracks/` measure +0.89, +0.94, +1.03, +1.53,
    +1.63, +2.31, +2.37 s past the poll that first read `run_state 0`, median
    1.53 s (recounted from the ledger, 2026-09-08). During that window a
    partner arm travels 0.29-1.71 m against 35.5-70.1 mm of certified margin —
    those two figures are the 2026-09-08 design audit's, not re-derived here.
    The signal that is ~1.3 s earlier is already being recorded and thrown away.

    THE RUNGS, weakest latency first:

    ==== ============================================== ===================
    rung signal                                          latency
    ==== ============================================== ===================
    0    the 5 ms push: `arm_current_status` leaves      ~15-20 ms when the
         `move_through_joint`, or joint motion ceases,   arm is >15 deg from
         and the arm is provably not at the end of its   the end, else
         file                                            ~0.25 s (see
                                                         STATUS_SETTLE_S)
    1    the state poll's own transition to `run_state   ~0.1 s
         0` EARLIER than `band_lo` after the departure
    2    `blocking_probe.ret == -4`                      +0.89..2.37 s
    3    no `program_run_finish`, then `replay_verdict`  end of stroke
    ==== ============================================== ===================

    Rung 2 is the ground-truth classifier and it is a PERFECT separator over
    the ledger: of the 125 runs that carried a probe, `(ret 0, complete)`
    occurs 118 times and `(ret -4, incomplete)` 7 times, with no other pairing.
    Rung 3's finish event is equally clean over the 94 records that carry one:
    `(finish_seen True, complete True)` 90 times, `(False, False)` 4 times.
    Rungs 0 and 1 exist only to get the SAME answer sooner.

    WHY RUNG 0 IS NOT JUST THE STATUS LABEL. The controller stops labelling the
    phase `move_through_joint` while the arm is still coasting toward the last
    point — up to 6.29 deg short of it, measured. Judging on the label alone
    would call every complete stroke a cut. So the label ARMS the rung and the
    POSITION decides it, either at once (>= IMMEDIATE_CUT_DEG away, which no
    coast can cover) or after STATUS_SETTLE_S.

    FEEDING IT. The caller owns every I/O thread; this object only judges.

      * :meth:`on_frame` — one parsed push frame, `(t_mono, frame_dict)`,
        exactly as `UdpRecorder` writes them. IN PRODUCTION that feed is
        `UdpRecorder(..., monitor=m)` / :meth:`UdpRecorder.attach_monitor`,
        which hands every datagram over AFTER writing it to the capture (added
        by the 2026-09-08 hardening review: rungs 1-3 were reachable from
        `run_verified` and rung 0 — the ~15 ms one, the entire reason this
        class exists — had no production feed at all). An executive MUST detach
        at the end of a stroke: `finished` and `tripped` latch, so the next
        stroke's frames would land in the last stroke's verdict.
        :meth:`on_record` takes the recorder's own `{"t_mono", "frame"}` line,
        so a test can replay a real capture straight off disk.
      * :meth:`on_transition` — a `run_program` transition dict (the `-2` poll
        errors included; they are counted, never tripped on).
      * :meth:`mark_replay_start` — the departure latch, in the transitions'
        own relative clock.
      * :meth:`on_probe`, :meth:`on_finish`, :meth:`on_replay_verdict`.

    WHAT IT CANNOT DO, said out loud. `summary()` carries
    `can_judge_arrival` — False when neither `cmd_rows` nor `last_row_millideg`
    was supplied, in which case rung 0 has no end to measure against and the
    object is not a cut detector at all — and `unjudged`, set when rung 0 armed
    its settle window and no frame in it carried a `joint_position` to judge on.
    Both exist so a silent monitor can never be read as a clean stroke.

    Supply `cmd_rows` (the whole millidegree track) for the ordered-progress
    test, which is what carries closed-loop rasters whose last point is
    geometrically next to their first; `last_row_millideg` alone still works
    and gives the distance test only.

    `tripped` is the FIRST stop condition (or None); `trips` is all of them,
    so a record can show that rung 0 saw at 15 ms what rung 2 confirmed at
    1.5 s. Nothing here stops an arm — the caller's stopper does that, and
    :func:`stop_program` is what it should call.

    NOT IMPLEMENTED ON PURPOSE: "three consecutive SDK `-2` returns means the
    arm is gone". 18 of the 125 probe runs have a run of three `-2`s during
    startup and ZERO have one after `run_state` first reads 1, so the rule
    would fire only on the runs where it must not. The `-2`s are counted into
    `poll_errors` and reported as BLINDNESS, which is what they are.
    """

    def __init__(self, *, side: str = "", points: Optional[int] = None,
                 band_lo_s: Optional[float] = None,
                 cmd_rows=None, last_row_millideg=None,
                 arrival_tol_deg: float = ARRIVAL_TOL_DEG,
                 immediate_deg: float = IMMEDIATE_CUT_DEG,
                 complete_phase: float = COMPLETE_PHASE,
                 status_settle_s: float = STATUS_SETTLE_S,
                 still_deg_s: float = 1.0,
                 status_frames: int = STATUS_CONFIRM_FRAMES,
                 still_frames: int = MOTION_CEASE_FRAMES,
                 still_min_s: Optional[float] = None,
                 judge_grace_s: float = JUDGE_GRACE_S,
                 log: Optional[Callable[[str], None]] = None):
        self.side = side
        self.cmd = ([[v / 1000.0 for v in r] for r in cmd_rows]
                    if cmd_rows is not None else None)
        self.points = points if points is not None else (
            len(self.cmd) if self.cmd else None)
        self.band_lo_s = None if band_lo_s is None else float(band_lo_s)
        #: a rung-1 earliness trip waiting on the finish event (see `_resolve_early`)
        self._early = None
        self._finish_event_seen = False
        if last_row_millideg is not None:
            self.last = [v / 1000.0 for v in last_row_millideg]
        elif self.cmd:
            self.last = list(self.cmd[-1])
        else:
            self.last = None
        self.arrival_tol_deg = float(arrival_tol_deg)
        self.immediate_deg = float(immediate_deg)
        self.complete_phase = float(complete_phase)
        self.status_settle_s = float(status_settle_s)
        self.still_deg_s = float(still_deg_s)
        self.status_frames = int(status_frames)
        self.still_frames = int(still_frames)
        self.still_min_s = (float(still_min_s) if still_min_s is not None else
                            self.still_frames * 0.005 * MOTION_CEASE_CLOCK_FRACTION)
        self.judge_grace_s = float(judge_grace_s)
        self.log = log
        self.trips: list = []
        self.tripped: Optional[dict] = None
        #: HARDENING (2026-09-08 review): rung 2 is fed from `run_program`'s
        #: PROBE WORKER THREAD (`monitor.on_probe` at track.py's
        #: `_blocking_start`) while rungs 0/1/3 are fed from the frame reader
        #: and the poll loop. `if self.tripped is None: self.tripped = t` is a
        #: check-then-set across those threads: two simultaneous trips could
        #: both pass the check and the FIRST trip — the whole latency claim of
        #: B13 — is whichever assignment landed last. Only `_trip` needs the
        #: lock; every other mutation has a single writer.
        self._lock = threading.RLock()
        self.frames = 0
        self.poll_errors = {"total": 0, "minus_two": 0, "max_consecutive": 0}
        self.t_first_frame: Optional[float] = None
        self.t_replay_first_frame: Optional[float] = None
        self.t_last_move_through: Optional[float] = None
        self.t_replay_start_rel: Optional[float] = None
        self.peak_deg_s = 0.0
        self.finished = False
        self._in_replay = False
        self._moved = False
        self._away = 0
        self._still = 0
        self._pending_since: Optional[float] = None
        self._still_since: Optional[float] = None
        self._freeze_judged = False
        self._prev = None
        self._consec_minus_two = 0
        self._state_running = False
        #: Set when rung 0 armed its settle window and never got a frame with
        #: joints to judge it on. Reported by `summary()`; it is BLINDNESS, not
        #: a verdict, and it is the honest answer to "why did nothing trip?"
        self.unjudged: Optional[dict] = None
        #: Feeds this object could not judge because judging them raised.
        #: Counted, never swallowed silently: a monitor that is BLIND must not
        #: read like a monitor that is QUIET.
        self.frame_errors = 0
        self._first_frame_error: Optional[str] = None
        # A monitor built with neither `cmd_rows` nor `last_row_millideg`
        # cannot answer "did the arm reach the end", which is the question
        # BOTH of rung 0's paths are decided on: it would sit silent through
        # every cut and look exactly like a clean stroke. Say so at once, and
        # report it in `summary()` so a record can never imply a rung that was
        # never armed.
        self.can_judge_arrival = bool(self.cmd or self.last is not None)
        if not self.can_judge_arrival and self.log:
            self.log(f"** MotionMonitor[{self.side}]: no cmd_rows and no "
                     f"last_row_millideg — rung 0 CANNOT judge arrival and will only "
                     f"report a freeze that is early against band_lo_s. This monitor "
                     f"is not a cut detector.")

    # ── judgement ────────────────────────────────────────────────────────
    @property
    def should_stop(self) -> bool:
        return self.tripped is not None

    def _trip(self, rung: int, reason: str, **detail) -> Optional[dict]:
        t = {"rung": int(rung), "reason": reason, "side": self.side,
             "frames": self.frames, **detail}
        with self._lock:
            self.trips.append(t)
            first = self.tripped is None
            if first:
                self.tripped = t
        if first and self.log:
            # Outside the lock: the caller's `log` is arbitrary code (a _Tee, a
            # ROS logger, a file write) and holding a lock a watchdog thread
            # polls across it is how a stop gets delayed.
            self.log(f"** MotionMonitor[{self.side}] rung {rung}: {reason} {detail}")
        return t

    def _note_feed_error(self, what: str, exc: BaseException) -> None:
        with self._lock:
            self.frame_errors += 1
            first = self.frame_errors == 1
            if first:
                self._first_frame_error = f"{what}: {exc!r}"
        if first and self.log:
            self.log(f"** MotionMonitor[{self.side}]: {what} could not be judged "
                     f"({exc!r}) — counted as BLINDNESS, not as a verdict")

    def progress(self, q) -> dict:
        """Where the arm is against the file: `{dist_last_deg, phase, dev_deg}`.

        `phase` is the ordered-progress question answered the way F112's
        `phase_end` answers it — the commanded index nearest the arm's CURRENT
        position — and it is None when no commanded track was supplied. On a
        closed raster it can read 0.0 at a perfectly complete end (the last
        point sits beside the first), which is exactly why the distance test
        exists alongside it and either one is enough to say ARRIVED.
        """
        if q is None:
            return {"dist_last_deg": None, "phase": None, "dev_deg": None}
        dist = (max(abs(a - b) for a, b in zip(q, self.last))
                if self.last is not None else None)
        phase = dev = None
        if self.cmd:
            best, k = None, 0
            for i, row in enumerate(self.cmd):
                d = max(abs(a - b) for a, b in zip(q, row))
                if best is None or d < best:
                    best, k = d, i
            phase = k / max(len(self.cmd) - 1, 1)
            dev = best
        return {"dist_last_deg": dist, "phase": phase, "dev_deg": dev}

    def _ceased(self, t: float) -> bool:
        """Has the arm been provably still long enough to call it stopped?

        BOTH a frame count and a wall-clock duration: the count is what the
        0.235 s / 60-frame measurement was made in, the clock is what keeps a
        decimated or stalled feed from reaching the count in far more time than
        the measurement covers (see MOTION_CEASE_CLOCK_FRACTION).
        """
        return bool(self._moved and self._still >= self.still_frames
                    and self._still_since is not None
                    and (t - self._still_since) >= self.still_min_s)

    def _arrived(self, q) -> Optional[bool]:
        """True / False / None(unknowable) — has the arm reached the end?"""
        return self._arrived_from(self.progress(q))

    def _arrived_from(self, pr: dict) -> Optional[bool]:
        if pr["dist_last_deg"] is None and pr["phase"] is None:
            return None
        if pr["dist_last_deg"] is not None and pr["dist_last_deg"] <= self.arrival_tol_deg:
            return True
        if pr["phase"] is not None and pr["phase"] >= self.complete_phase:
            return True
        return False

    # ── rung 0: the 200 Hz push ──────────────────────────────────────────
    def on_record(self, rec: dict):
        """One `{"t_mono", "frame"}` line as `UdpRecorder` writes it."""
        try:
            return self.on_frame(rec["t_mono"], rec["frame"])
        except (KeyError, TypeError):
            return None

    def on_frame(self, t: float, frame: dict):
        """One parsed push frame. NEVER raises (hardening, 2026-09-08 review).

        This is fed from whatever thread owns the UDP socket — in the flow, the
        `UdpRecorder` reader. A malformed datagram (a short `joint_position`, a
        string where a number belongs, a truncated section) used to propagate
        out of here and kill that thread, which stops the capture AND retires
        the detector, both silently and both in the safe-looking direction.
        Errors are counted into `frame_errors`, the first one is logged, and
        `summary()` reports them, so blindness is VISIBLE rather than absent.
        """
        try:
            return self._on_frame(t, frame)
        except Exception as exc:                                       # noqa: BLE001
            self._note_feed_error("a push frame", exc)
            return self.tripped

    def _on_frame(self, t: float, frame: dict):
        if not isinstance(frame, dict):
            return self.tripped
        self.frames += 1
        if self.t_first_frame is None:
            self.t_first_frame = t
        if self._early is not None:
            self._resolve_early()          # the push feed drives the grace window
        status = frame.get("arm_current_status")
        jp = None
        js = frame.get("joint_status")
        if isinstance(js, dict):
            jp = js.get("joint_position")
        q = [v / 1000.0 for v in jp] if (jp and len(jp) == 7) else None
        speed = None
        if q is not None and self._prev is not None:
            dt = max(t - self._prev[0], 1e-4)
            speed = max(abs(a - b) for a, b in zip(q, self._prev[1])) / dt
            self.peak_deg_s = max(self.peak_deg_s, speed)
        if q is not None:
            self._prev = (t, q)
        if self.finished or self.should_stop:
            return self.tripped
        if status == "move_through_joint":
            if not self._in_replay:
                self._in_replay = True
                self.t_replay_first_frame = t
            self.t_last_move_through = t
            self._away = 0
            self._pending_since = None            # a spurious label blip
        elif self._in_replay:
            self._away += 1
            if self._away == self.status_frames:
                pr = self.progress(q)                 # O(points): once, here
                arrived = self._arrived_from(pr)
                far = (pr["dist_last_deg"] is not None
                       and pr["dist_last_deg"] > self.immediate_deg
                       and arrived is not True)
                if far:
                    # No arm coasts 15 deg after the controller has stopped
                    # labelling the phase; judge now, at ~15 ms.
                    self._trip(0, "arm_current_status left move_through_joint far from "
                                  "the end of the track", t=t, status=status, **pr)
                elif arrived is True:
                    self.finished = True
                else:
                    # Let it coast in, then judge (STATUS_SETTLE_S).
                    self._pending_since = t
            elif self._pending_since is not None:
                ceased = self._ceased(t)
                if (t - self._pending_since) >= self.status_settle_s or ceased:
                    pr = self.progress(q)
                    arrived = self._arrived_from(pr)
                    if arrived is None:
                        # HARDENING (2026-09-08): this frame carried no usable
                        # joint_position, so nothing was judged. The window
                        # used to be CLOSED here regardless, which made a
                        # single jointless frame at exactly the settle instant
                        # retire rung 0 for the whole stroke — silently, and in
                        # the safe-looking direction. Keep the window open for
                        # `judge_grace_s` more, then give up LOUDLY into
                        # `unjudged`, which `summary()` reports.
                        if (t - self._pending_since) >= (self.status_settle_s
                                                         + self.judge_grace_s):
                            self._pending_since = None
                            self.unjudged = {
                                "t": t, "why": ("no joint_position in any frame of the "
                                                "settle window — rung 0 could not judge "
                                                "this stroke"),
                                "frames": self.frames}
                            if self.log:
                                self.log(f"** MotionMonitor[{self.side}] rung 0 is BLIND "
                                         f"on this stroke: {self.unjudged['why']}")
                    else:
                        self._pending_since = None
                        if arrived is False:
                            self._trip(0, "the arm came to rest short of the end of the "
                                          "track", t=t, status=status,
                                       settled_s=round(t - self.t_last_move_through, 4)
                                       if self.t_last_move_through else None, **pr)
                        else:
                            self.finished = True
        if self._in_replay and speed is not None:
            if speed > self.still_deg_s:
                self._moved = True
                self._still = 0
                self._still_since = None
            else:
                self._still += 1
                if self._still_since is None:
                    self._still_since = t
            if (self._ceased(t) and not self._freeze_judged
                    and not self.should_stop and not self.finished
                    and status == "move_through_joint"):
                # `_freeze_judged` latches: the progress search is O(points)
                # and this branch would otherwise re-run it on every frame of a
                # long still stretch. (It was `self._still == self.still_frames`
                # until 2026-09-08, which is the same guard only while the feed
                # runs at exactly the 5 ms push cadence — see `_ceased`.)
                self._freeze_judged = True
                # A FREEZE: the controller still claims to be replaying and the
                # arm is not moving. This is the only rung that closes that
                # case, and no other signal in the system ever will.
                pr = self.progress(q)
                arrived = self._arrived_from(pr)
                early = None
                if self.t_replay_first_frame is not None and self.band_lo_s is not None:
                    early = (t - self.t_replay_first_frame) < self.band_lo_s
                if arrived is False or (arrived is None and early is True):
                    self._trip(0, "joint motion ceased while the controller still "
                                  "reports move_through_joint", t=t, early=early,
                               still_s=(round(t - self._still_since, 4)
                                        if self._still_since is not None else None),
                               still_frames=self._still, **pr)
                elif arrived is True:
                    self.finished = True
        return self.tripped

    # ── rung 1: the state poll's own transitions ─────────────────────────
    def mark_replay_start(self, t_rel: float) -> None:
        """The departure latch, in `run_program`'s relative clock."""
        if self.t_replay_start_rel is None:
            self.t_replay_start_rel = float(t_rel)

    def on_transition(self, tr: dict):
        """A `run_program` transition. NEVER raises: this is called from inside
        the poll loop of a run in progress, and an exception here would abort
        the very stroke the monitor exists to watch."""
        try:
            return self._on_transition(tr)
        except Exception as exc:                                       # noqa: BLE001
            self._note_feed_error("a transition", exc)
            return self.tripped

    def _on_transition(self, tr: dict):
        if not isinstance(tr, dict):
            return self.tripped
        if tr.get("error_ret") is not None:
            self.poll_errors["total"] += 1
            if int(tr["error_ret"]) == -2:
                self.poll_errors["minus_two"] += 1
                self._consec_minus_two += 1
                self.poll_errors["max_consecutive"] = max(
                    self.poll_errors["max_consecutive"], self._consec_minus_two)
            else:
                self._consec_minus_two = 0
            return self.tripped                    # counted, NEVER tripped on
        self._consec_minus_two = 0
        rs = tr.get("run_state")
        if rs in (1, 2):
            self._state_running = True
            return self.tripped
        if rs == 0 and self._state_running and not self.should_stop:
            self._state_running = False
            if self.band_lo_s is None:
                return self.tripped
            # RUNG 0 HAS THE LAST WORD ON COMPLETENESS (2026-09-10 row 10 leg 0).
            # `finished` is set only where rung 0 read the arm AT its last
            # commanded point, from the 200 Hz frames — positive arrival
            # evidence, and independent of the event channel. A `run_state 0`
            # after that is the expected end of the program, and no duration
            # arithmetic may overrule it. On row 10 leg 0 the left monitor
            # carried `finished: true` and rung 1 tripped anyway, which stopped
            # the RIGHT arm 22.2 s into its own stroke.
            if self.finished:
                return self.tripped
            t = tr.get("t")
            if t is None:
                return self.tripped
            # Judge on EARLINESS. With the departure latched the span is the
            # replay's own; without it the raw t is a lower bound on the replay
            # (the transit only adds time), so `t < band_lo` is still one-sided
            # and safe.
            span = t - (self.t_replay_start_rel or 0.0)
            if span < self.band_lo_s:
                # DEFERRED, NOT TRIPPED. Earliness alone cannot tell a CUT from
                # a program that simply ran fast, and on 2026-09-10 row 3 leg 1
                # this rung got it wrong and cost the pair: `toplid_left`
                # finished in 9.741 s (1891 pts, 5.154 ms/pt) WITH its
                # program_run_finish event and `complete: true`, the floor was
                # 10.395 s, rung 1 tripped, the trip stops BOTH arms, and
                # `side_right` — 60 % through a 16.5 s stroke — was slow_stopped
                # into a real cut. Three complete runs that day sat under the
                # floor, all on the LEFT controller.
                #
                # The finish EVENT is the completeness label (F123) and it
                # arrives with run_state 0, so waiting a moment for it costs
                # almost no latency and removes the whole false-positive class.
                # If it does not arrive, the trip fires as before.
                self._early = {
                    "t": t, "span_s": round(span, 4), "band_lo_s": self.band_lo_s,
                    "departure_latched": self.t_replay_start_rel is not None,
                    "deadline": _time.monotonic() + EARLY_TRIP_GRACE_S}
                if self.log:
                    self.log(f"   MotionMonitor[{self.side}]: run_state 0 at {t:.3f} s is "
                             f"early against {self.band_lo_s:.3f} s — holding "
                             f"{EARLY_TRIP_GRACE_S:.2f} s for the finish event before "
                             f"calling it a cut")
            self._resolve_early()
        return self.tripped

    def note_finish_event(self) -> None:
        """The controller's program_run_finish arrived for this handle.

        Cancels a pending rung-1 trip: a program that reported its finish is not
        a cut, however fast it ran. Safe to call more than once, and safe to
        call for an arm with no pending trip.
        """
        with self._lock:
            self._finish_event_seen = True
            self._early = None

    def _resolve_early(self) -> Optional[dict]:
        """Fire a deferred rung-1 trip once its grace window has passed."""
        with self._lock:
            e = self._early
            # EITHER completeness channel cancels: the finish event (F123's
            # label) or rung 0's own arrival read, whichever lands first inside
            # the grace window. Row 10 leg 0 had the arrival and row 3 leg 1 had
            # the event, so neither channel alone would have saved both pairs.
            if e is None or self._finish_event_seen or self.finished:
                self._early = None
                return self.tripped
            if _time.monotonic() < e["deadline"]:
                return self.tripped
            self._early = None
        return self._trip(1, "the program reported run_state 0 earlier than the measured "
                             "floor and no program_run_finish event followed",
                          t=e["t"], span_s=e["span_s"], band_lo_s=e["band_lo_s"],
                          departure_latched=e["departure_latched"],
                          grace_s=EARLY_TRIP_GRACE_S)

    # ── rung 2: the blocking probe ───────────────────────────────────────
    def on_probe(self, ret, returned_s: Optional[float] = None):
        """-4 is the documented cut. Any OTHER non-zero return is also a stop
        condition, because it means this arm's program did not run to the end
        and nobody knows where it stopped: over the ledger's 125 probe runs the
        only values ever seen are 0 (118, all complete) and -4 (7, all cut), so
        anything else is unmapped and must not be read as success. A string
        (`"raised ..."`, `"still blocked at join"`) is the same case.

        Called from `run_program`'s PROBE WORKER THREAD, so it never raises:
        an exception there is invisible and would lose rung 2 entirely."""
        try:
            return self._on_probe(ret, returned_s)
        except Exception as exc:                                       # noqa: BLE001
            self._note_feed_error("the probe rung", exc)
            return self.tripped

    def _on_probe(self, ret, returned_s: Optional[float] = None):
        if ret == -4:
            self._trip(2, "blocking probe returned -4 (the trajectory was stopped "
                          "externally)", ret=ret, returned_s=returned_s)
        elif isinstance(ret, int) and ret != 0:
            self._trip(2, f"blocking probe returned {ret}, which is neither 0 (finished) "
                          f"nor -4 (stopped) — the program did not run to the end",
                       ret=ret, returned_s=returned_s)
        elif ret is not None and not isinstance(ret, int):
            self._trip(2, f"the blocking probe never returned a code ({ret!r}) — this "
                          f"arm's completion is UNKNOWN", ret=ret, returned_s=returned_s)
        return self.tripped

    # ── rung 3: the end-of-stroke evidence ───────────────────────────────
    def on_finish(self, finish_seen: Optional[bool]):
        if finish_seen:
            self.note_finish_event()
            return self.tripped
        if finish_seen is None:
            # No event log on this handle: the deferred trip has nothing to wait
            # for, so let it resolve on its own deadline rather than hang.
            return self._resolve_early()
        self._resolve_early()
        if finish_seen is False:
            self._trip(3, "no program_run_finish event on this handle",
                       finish_seen=False)
        return self.tripped

    def on_run_refused(self, verdict: str, reason: Optional[str] = None):
        """This arm's stroke is not going to happen (or did not finish).

        HARDENING (2026-09-08). `run_verified` has SIX exits that never reach
        rung 3 — `refused_no_state`, `refused_not_at_start`, `refused_movej`,
        `did_not_start`, `no_end`, `wrong_id` — and each one leaves `ok` False
        in the RETURN VALUE. An executive whose watchdog thread reads
        `monitor.should_stop` (which is the whole point of B13: the return
        value arrives 0.89-2.37 s too late) would have seen nothing at all on
        any of them, and let the partner run on against an arm that never
        moved or stopped somewhere unknown. They trip rung 3.
        """
        self._trip(3, f"the run was refused or never finished ({verdict})",
                   verdict=verdict, why=reason)
        return self.tripped

    def on_replay_verdict(self, vd: Optional[dict]):
        """A capture-derived verdict. `None` is UNMEASURED and says nothing.

        HARDENING (2026-09-08 review): a DICT whose outcome is None used to be
        silent as well, so `replay_verdict()` returning `{"outcome": None}`
        (the shape it takes when the capture is too short to judge — F112's
        seven UNRESOLVED records) trips nothing while `run_verified` scores the
        same run `ok: False`. A verdict that was measured and is not "complete"
        is a stop condition whatever its spelling; only the ABSENCE of a
        verdict is silence.
        """
        if isinstance(vd, dict) and vd.get("outcome") != "complete":
            self._trip(3, f"replay verdict is {vd.get('outcome')!r}, not complete",
                       outcome=vd.get("outcome"))
        return self.tripped

    # ── reporting ────────────────────────────────────────────────────────
    def summary(self) -> dict:
        return {"side": self.side, "frames": self.frames,
                "tripped": self.tripped, "trips": self.trips,
                "poll_errors": dict(self.poll_errors),
                "peak_deg_s": round(self.peak_deg_s, 4),
                "saw_replay": self._in_replay, "finished": self.finished,
                "replay_first_frame_t": self.t_replay_first_frame,
                "last_move_through_t": self.t_last_move_through,
                "can_judge_arrival": self.can_judge_arrival,
                "unjudged": self.unjudged,
                "frame_errors": self.frame_errors,
                "first_frame_error": self._first_frame_error,
                "rung": (self.tripped or {}).get("rung")}


def at_rest_barrier(robots: dict, *, log: Callable[[str], None] = print,
                    poll_s: float = 0.025, still_deg_s: float = 1.6,
                    need: int = 6, timeout_s: float = 20.0,
                    still_deg: Optional[float] = None) -> dict:
    """H30's both-arms-at-rest barrier: watch EVERY arm from the same instant.

    `rm_get_program_run_state() == 0` is not "at rest" — 32 of the 126 ledger
    records that carry a settle read `was_moving: true` (25.4 %), with peaks to
    11.94 deg/s and one wait of 6.799 s. A step loop that starts staging on
    that signal plans its next bridge from a pose the arm has already left.

    THE POLLING IS INTERLEAVED, not sequential (hardening, 2026-09-08). The
    first version settled one arm and only then began polling the next, which
    got the ok/still_moving answer right and the PER-SIDE RECORD wrong: the
    second arm's motion during the first arm's wait was never observed, so its
    `was_moving` read False and its `peak_deg_s` read ~0 no matter how fast it
    had been travelling — and `was_moving/peak_deg_s/waited_s` per side per row
    is exactly what H30 asks the record to carry. One loop, every arm sampled
    each time round, one shared clock.

    Returns ``{"ok", "sides": {side: {waited_s, was_moving, peak_deg_s,
    at_rest}}, "still_moving": [side, ...]}`` — the per-side dicts are the same
    shape :func:`wait_until_at_rest` returns, and the gate is the same speed.
    """
    if still_deg is not None:
        still_deg_s = float(still_deg) / max(poll_s, 1e-6)
    live = {s: r for s, r in sorted(robots.items()) if r is not None}
    sides = {}
    if not live:
        return {"ok": False, "sides": {}, "still_moving": []}
    state = {s: {"prev": None, "quiet": 0, "peak": 0.0, "polls": 0} for s in live}
    t0 = time.monotonic()
    while live and time.monotonic() - t0 < timeout_s:
        for side in list(live):
            robot = live[side]
            st_ = state[side]
            try:
                st = robot.rm_get_current_arm_state()
            except Exception:                                          # noqa: BLE001
                st = None
            q = None
            if isinstance(st, (tuple, list)) and len(st) >= 2 and isinstance(st[1], dict):
                q = st[1].get("joint")
            elif isinstance(st, dict):
                q = st.get("joint")
            if q is None:
                # Unreadable is never "at rest" — the same rule
                # `wait_until_at_rest` applies, and the same rule
                # `program_run_state` applies to `running`.
                sides[side] = {"waited_s": time.monotonic() - t0, "was_moving": True,
                               "peak_deg_s": st_["peak"], "at_rest": False,
                               "unreadable": True}
                del live[side]
                continue
            q = [float(x) for x in q]
            now = time.monotonic()
            st_["polls"] += 1
            if st_["prev"] is not None:
                dt = max(now - st_["prev"][0], 1e-3)
                rate = max(abs(a - b) for a, b in zip(q, st_["prev"][1])) / dt
                st_["peak"] = max(st_["peak"], rate)
                st_["quiet"] = st_["quiet"] + 1 if rate < still_deg_s else 0
                if st_["quiet"] >= need:
                    # `was_moving` is a POLL COUNT here, not `elapsed >
                    # (need + 2) * poll_s`. They mean the same thing —
                    # "settling took longer than the floor" — but the elapsed
                    # form silently counts the OTHER arms' getter time in a
                    # round-robin loop, so a second arm and a slow controller
                    # would stamp `was_moving` on an arm that never moved.
                    sides[side] = {"waited_s": now - t0,
                                   "was_moving": st_["polls"] > (need + 2),
                                   "polls": st_["polls"],
                                   "peak_deg_s": st_["peak"], "at_rest": True}
                    del live[side]
                    continue
            st_["prev"] = (now, q)
        if live:
            time.sleep(poll_s)
    for side in live:                       # timed out still moving
        sides[side] = {"waited_s": time.monotonic() - t0, "was_moving": True,
                       "peak_deg_s": state[side]["peak"], "at_rest": False}
    moving = sorted(s for s, r in sides.items() if not r.get("at_rest"))
    for side in sorted(sides):
        r = sides[side]
        if r.get("was_moving"):
            log(f"{side}: the program reported its end while the arm was still moving "
                f"(peak {r.get('peak_deg_s', 0):.2f} deg/s, waited "
                f"{r.get('waited_s', 0):.3f} s)")
    return {"ok": not moving and bool(sides), "sides": sides, "still_moving": moving}


# ---------------------------------------------------------------------------
# B12 / gate G4 — a VERIFIED, ESCALATING stop
# ---------------------------------------------------------------------------
#: The stop ladder, weakest first. `rm_set_arm_slow_stop` decelerates ON the
#: path (the only rung that does not throw the arm off its trajectory);
#: `rm_set_arm_pause` (rm_interface.h:1318) and `rm_set_arm_stop` (:1306) are
#: harder and leave the arm wherever they catch it.
#:
#: WHY THIS IS A LADDER AND NOT ONE CALL. Nobody has ever shown that ANY of
#: them aborts a running uploaded PROGRAM. The vendor's online-program group
#: (external/butterfli-realman-alix/.../rm_interface.h:3248-3354) exposes no
#: stop verb at all — the three above are scoped to TRAJECTORIES — and this
#: repo's own emulator documents the assumption as untested
#: (`emulated_program.py:50-55`, "gate G4 has not run"). 0 hardware trials
#: exist. So each rung is issued and then VERIFIED against the controller's own
#: program run state and against the joints actually stopping, and the result
#: says which rung worked; if none did, `stop_unverified` is True and the
#: caller must treat the partner arm as unprotected.
STOP_RUNGS = ("slow_stop", "pause", "stop")
_STOP_VERBS = {"slow_stop": "rm_set_arm_slow_stop",
               "pause": "rm_set_arm_pause",
               "stop": "rm_set_arm_stop"}


def program_run_state(robot) -> dict:
    """`{ret, run_state, id, running}` — never raises. `running` is None when
    the state could not be read, which is UNKNOWN and never "stopped"."""
    try:
        ret, st = robot.rm_get_program_run_state()
    except Exception as exc:                                           # noqa: BLE001
        return {"ret": -1, "run_state": None, "id": None, "running": None,
                "error": repr(exc)}
    if int(ret) != 0 or not isinstance(st, dict):
        return {"ret": int(ret), "run_state": None, "id": None, "running": None}
    rs = st.get("run_state")
    return {"ret": 0, "run_state": rs, "id": st.get("id"),
            "running": (rs in (1, 2)) if rs is not None else None}


def _joint_travel(robot, q_from, *, poll_s: float, until: float) -> tuple:
    """Poll joints until *until* (monotonic). Returns (last_q, travel_deg,
    peak_deg_s) — travel is the worst per-joint departure from `q_from`."""
    last, travel, peak, prev = q_from, 0.0, 0.0, None
    while time.monotonic() < until:
        q = current_joints_deg(robot)
        now = time.monotonic()
        if q is not None:
            last = q
            if q_from is not None:
                travel = max(travel, max(abs(a - b) for a, b in zip(q, q_from)))
            if prev is not None:
                dt = max(now - prev[0], 1e-3)
                peak = max(peak, max(abs(a - b) for a, b in zip(q, prev[1])) / dt)
            prev = (now, q)
        time.sleep(poll_s)
    return last, travel, peak


def stop_program(robot, *, log: Callable[[str], None] = print,
                 timeout_s: float = 1.5, poll_s: float = 0.02,
                 still_deg_s: float = 2.0, rungs=STOP_RUNGS) -> dict:
    """Stop a running program and PROVE it stopped, escalating until it does.

    For each rung in *rungs*: issue the verb (skipping one the SDK build does
    not expose), then for up to *timeout_s* watch (a) the controller's program
    run state and (b) the joints. The rung SUCCEEDED when the run state reads 0
    **and** the arm has gone still; it is UNVERIFIED when the run state could
    not be read at all. Only then does the next rung fire.

    Returns::

      {"stopped": bool, "rung": "slow_stop"|"pause"|"stop"|None,
       "stop_unverified": bool, "resumable": bool, "elapsed_s": float,
       "joint_travel_deg": float|None, "peak_deg_s": float|None,
       "joints_before": [...]|None, "joints_after": [...]|None,
       "state_before": {...}, "state_after": {...}, "arm_still": bool|None,
       "attempts": [{"rung","verb","present","ret","elapsed_s","state",
                     "travel_deg","peak_deg_s","worked","arm_still"}],
       "refusal": str|None}

    `arm_still` is the JOINT question, kept separate from the program question
    on purpose: `run_state == 0` is not "at rest" (H30 — 32 of 126 ledger runs
    were still moving at that poll), so an arm whose program has already ended
    but which is still travelling is NOT reported as stopped; the ladder runs
    on it anyway. `stopped` therefore means "the program ended AND the arm went
    still", which is the only pair a partner arm can be released on.

    `resumable` is True when the rung that worked was the PAUSE. The motion
    stopped — which is what a partner arm needs — but the vendor pairs pause
    with `rm_set_arm_continue`, so the program is SUSPENDED, not terminated,
    and the slot is not free. The ladder does not escalate past a rung that
    worked; it reports what that rung actually means.

    `joint_travel_deg` is the stop DISTANCE in joint space, which is the number
    the trip threshold W_trip has to be chosen against and which no record in
    the ledger contains (0 of 157 runs time a stop). It is measured here so the
    D1 rung can read it straight off the result.

    Never raises: it runs on the recovery path, which is already unwinding.
    """
    t0 = time.monotonic()
    q0 = current_joints_deg(robot)
    state_before = program_run_state(robot)
    out = {"stopped": False, "rung": None, "stop_unverified": False,
           "resumable": False,
           "elapsed_s": 0.0, "joint_travel_deg": None, "peak_deg_s": None,
           "joints_before": q0, "joints_after": q0,
           "state_before": state_before, "state_after": state_before,
           "attempts": [], "refusal": None, "arm_still": None}
    if state_before.get("running") is False:
        # H30 INSIDE B12 (hardening, 2026-09-08): `run_state == 0` is NOT "at
        # rest" — 32 of the 126 ledger records that carry a settle read
        # `was_moving: true`, peak 11.94 deg/s. This is exactly the instant a
        # dual executive issues the partner's stop, so returning
        # `stopped: True` on the state alone would certify a moving arm as
        # safe. Measure the joints before believing it; a MOVING arm falls
        # through into the ladder, which is what it needs.
        _q, _travel, _peak = _joint_travel(robot, q0, poll_s=poll_s,
                                           until=time.monotonic() + max(poll_s * 6, 0.12))
        out["joints_after"] = _q
        out["joint_travel_deg"] = round(_travel, 4)
        out["peak_deg_s"] = round(_peak, 4)
        out["arm_still"] = bool(_peak <= still_deg_s)
        if out["arm_still"]:
            out["stopped"] = True
            out["rung"] = "already_stopped"
            out["elapsed_s"] = round(time.monotonic() - t0, 4)
            log("stop_program: nothing was running and the arm is still")
            return out
        log(f"** stop_program: no program is running but the arm is STILL MOVING at "
            f"{_peak:.2f} deg/s — running the ladder anyway")
    # Seeded, not zeroed: the pre-ladder measurement above is part of this
    # stop's travel and must not be thrown away by the ladder's own maxima.
    travel_total = float(out["joint_travel_deg"] or 0.0)
    peak_total = float(out["peak_deg_s"] or 0.0)
    for rung in rungs:
        verb = _STOP_VERBS.get(rung)
        fn = getattr(robot, verb, None) if verb else None
        att = {"rung": rung, "verb": verb, "present": fn is not None,
               "ret": None, "elapsed_s": None, "state": None,
               "travel_deg": None, "peak_deg_s": None, "worked": False}
        ta = time.monotonic()
        if fn is None:
            # A build without the verb is not a failure to report as a refusal
            # by itself — the ladder simply moves on and says so.
            att["elapsed_s"] = 0.0
            out["attempts"].append(att)
            log(f"stop_program: {verb} is not on this SDK build — skipping to the next rung")
            continue
        try:
            att["ret"] = int(fn())
        except Exception as exc:                                       # noqa: BLE001
            att["ret"] = f"raised {exc!r}"
        qa = current_joints_deg(robot) or q0
        deadline = time.monotonic() + timeout_s
        worked = False
        while time.monotonic() < deadline:
            st = program_run_state(robot)
            att["state"] = st
            if st.get("running") is False:
                worked = True
                break
            time.sleep(poll_s)
        qb, travel, peak = _joint_travel(robot, qa, poll_s=poll_s,
                                         until=time.monotonic() + max(poll_s * 6, 0.12))
        att["travel_deg"] = round(travel, 4)
        att["peak_deg_s"] = round(peak, 4)
        att["elapsed_s"] = round(time.monotonic() - ta, 4)
        travel_total = max(travel_total, travel)
        peak_total = max(peak_total, peak)
        still = peak <= still_deg_s
        att["worked"] = bool(worked and still)
        att["arm_still"] = bool(still)
        out["arm_still"] = bool(still)
        out["attempts"].append(att)
        out["joints_after"] = qb
        if att["worked"]:
            out["stopped"] = True
            out["rung"] = rung
            log(f"stop_program: {verb} STOPPED the program (ret={att['ret']}, "
                f"{att['elapsed_s']:.3f} s, {att['travel_deg']:.2f} deg of travel)")
            if rung == "pause":
                # HARDENING (2026-09-08 review). A PAUSE is not an abort: the
                # vendor pairs it with `rm_set_arm_continue` (rm_interface.h
                # :1330), so the slot is still loaded and the arm is holding
                # mid-track. The motion has stopped, which is what a partner
                # arm needs, but the program has NOT been terminated and a
                # caller must not read `stopped: True` as "the slot is free".
                # We do not escalate past a rung that worked — that is a verb
                # Newton has not sanctioned issuing on a stopped arm, and G4
                # has never run — we SAY SO instead.
                out["resumable"] = True
                log("** stop_program: the rung that worked was the PAUSE — motion has "
                    "stopped but the program is only SUSPENDED (rm_set_arm_continue "
                    "would resume it). Do not treat the slot as free.")
            break
        if worked and not still:
            log(f"** stop_program: {verb} ended the program but the arm is still moving "
                f"at {peak:.2f} deg/s — escalating")
        elif (att.get("state") or {}).get("running") is None:
            out["stop_unverified"] = True
            log(f"** stop_program: {verb} sent (ret={att['ret']}) but the program run "
                f"state could NOT be read — the stop is UNVERIFIED; escalating")
        else:
            log(f"** stop_program: {verb} returned {att['ret']} and the program is STILL "
                f"RUNNING after {timeout_s:.2f} s — escalating")
    out["joint_travel_deg"] = round(travel_total, 4)
    out["peak_deg_s"] = round(peak_total, 4)
    out["elapsed_s"] = round(time.monotonic() - t0, 4)
    out["state_after"] = program_run_state(robot)
    if not out["stopped"]:
        out["stop_unverified"] = True
        tried = ", ".join(a["verb"] for a in out["attempts"] if a["present"]) or "nothing"
        still_running = (out["state_after"].get("running") is not False)
        what = ("the program is still running" if still_running else
                f"the program ended but the arm is still moving at {peak_total:.2f} deg/s")
        out["refusal"] = (f"{what} after {tried} — no software stop has been shown to work "
                          f"on this controller; use the pendant / e-stop")
        log("** " + out["refusal"])
    return out


def probe_stop_policy(robot, *, save_id: int, speed: int = 20,
                      dwell_s: float = 1.0, log: Callable[[str], None] = print,
                      timeout_s: float = 1.5, poll_s: float = 0.02,
                      settle_timeout_s: float = 20.0,
                      rungs=STOP_RUNGS) -> dict:
    """GATE G4, as a callable: does an arm-level stop abort a running PROGRAM?

    Newton can run this tomorrow with ZERO geometric exposure — point it at a
    slot holding a DRY, in-place track and it answers the question that every
    clearance verdict in the 2026-09-08 design is conditional on.

    It starts the slot non-blocking, lets the replay get *dwell_s* past its
    transit, then walks :func:`stop_program`'s ladder and reports which rung
    (if any) actually ended the program, how long it took, and how far the
    joints travelled during the stop.

    Returns ``{"answer": "aborts"|"ignores"|"unknown", "rung", "slot",
    "started", "stop": <stop_program result>, "dwell_s", "run_state_after",
    "settled", "ran_to_completion", "refusal", "why"}``. Every one of those
    keys is present on EVERY path — `ran_to_completion` used to exist only on
    the "ignores" branch, so a caller reading it after an "aborts" answer got a
    KeyError on the one path where the answer was good news.

    It REFUSES (and starts nothing) when a program is already running or the
    slot will not start, and every refusal path still leaves a recorded stop
    and a settle behind, so the arm is never handed back unlooked-at.
    """
    out = {"answer": "unknown", "rung": None, "slot": int(save_id), "started": False,
           "stop": None, "dwell_s": float(dwell_s), "run_state_after": None,
           "settled": None, "refusal": None, "why": "", "ran_to_completion": None}
    st = program_run_state(robot)
    if st.get("running") is not False:
        out["refusal"] = (f"a program is already running or the run state is unreadable "
                          f"({st}) — refusing to start the G4 probe on top of it")
        log("** " + out["refusal"])
        return out
    try:
        ret = int(robot.rm_set_program_id_run(int(save_id), int(speed), 0))
    except Exception as exc:                                           # noqa: BLE001
        out["refusal"] = f"rm_set_program_id_run raised {exc!r}"
        log("** " + out["refusal"])
        return out
    if ret != 0:
        out["refusal"] = f"slot {save_id} would not start (ret={ret})"
        log("** " + out["refusal"])
        return out
    # Wait for the controller to actually report the program running, then let
    # the replay get clear of the implicit movej transit.
    deadline = time.monotonic() + max(settle_timeout_s, dwell_s + 5.0)
    while time.monotonic() < deadline:
        if program_run_state(robot).get("running"):
            out["started"] = True
            break
        time.sleep(poll_s)
    if not out["started"]:
        out["refusal"] = "the program never reported run_state 1 — nothing to stop"
        log("** " + out["refusal"])
        # Recorded, not discarded: this recovery stop is the only evidence of
        # what state the arm was left in, and a caller that reads `refusal`
        # and nothing else would hand back an arm nobody had looked at.
        out["stop"] = stop_program(robot, log=log, timeout_s=timeout_s,
                                   poll_s=poll_s, rungs=rungs)
        out["run_state_after"] = out["stop"].get("state_after")
        out["settled"] = wait_until_at_rest(robot, timeout_s=settle_timeout_s)
        return out
    t_run = time.monotonic()
    while time.monotonic() - t_run < dwell_s:
        if not program_run_state(robot).get("running"):
            out["refusal"] = (f"the program ended within the {dwell_s:.2f} s dwell — use a "
                              f"longer dry track, there was nothing to stop")
            log("** " + out["refusal"])
            out["ran_to_completion"] = True
            out["run_state_after"] = program_run_state(robot)
            out["settled"] = wait_until_at_rest(robot, timeout_s=settle_timeout_s)
            return out
        time.sleep(poll_s)
    log(f"G4: {dwell_s:.2f} s into slot {save_id}'s replay — issuing the stop ladder")
    res = stop_program(robot, log=log, timeout_s=timeout_s, poll_s=poll_s, rungs=rungs)
    out["stop"] = res
    out["rung"] = res.get("rung")
    out["run_state_after"] = res.get("state_after")
    settle = wait_until_at_rest(robot, timeout_s=settle_timeout_s)
    out["settled"] = settle
    if res.get("stopped"):
        out["ran_to_completion"] = False
        out["answer"] = "aborts"
        out["why"] = (f"{_STOP_VERBS.get(res['rung'], res['rung'])} ended the program in "
                      f"{res['elapsed_s']:.3f} s with {res['joint_travel_deg']} deg of "
                      f"joint travel")
    elif res.get("state_after", {}).get("running") is None:
        out["answer"] = "unknown"
        out["why"] = "the program run state could not be read after the ladder"
    else:
        out["answer"] = "ignores"
        out["why"] = ("no arm-level stop ended the program — the partner arm is "
                      "UNPROTECTED and every concurrent clearance verdict is void")
        # The program is STILL PLAYING. On the dry in-place track this probe is
        # meant for that is harmless, but leaving it running and returning would
        # hand the operator a moving arm with no note of it, so wait it out and
        # say whether it ever ended.
        log("** the program is still playing — waiting for it to finish on its own "
            "(this is why G4 runs on a DRY track)")
        deadline = time.monotonic() + max(settle_timeout_s, 5.0)
        while time.monotonic() < deadline:
            if program_run_state(robot).get("running") is False:
                break
            time.sleep(poll_s)
        out["ran_to_completion"] = program_run_state(robot).get("running") is False
        out["run_state_after"] = program_run_state(robot)
        out["settled"] = wait_until_at_rest(robot, timeout_s=settle_timeout_s)
        if not out["ran_to_completion"]:
            log("** the program is STILL running and nothing here can stop it — "
                "use the pendant / e-stop")
    log(f"G4 answer: {out['answer'].upper()} — {out['why']}")
    return out


#: Getters whose answer is CONFIGURATION, not state: it cannot change while a
#: flow runs, so re-reading it twice per task buys nothing and costs SDK
#: round-trips. The snapshot took 0.3-2.0 s per call on 2026-09-04 and there
#: are two per run (F120). Everything NOT in here — error codes, joint error
#: flags, collision stage, tool frame, run mode, run state, and since
#: 2026-09-08 the four Cartesian caps — is re-read every time, because each of
#: those is exactly what a snapshot exists to catch.
#:
#: H31 (2026-09-08): the four Cartesian caps used to be in here, and they are
#: the ONE thing this flow deliberately changes mid-run — `lim.guard` raises
#: them to 1.5 / 6.0 / 5.0 / 20.0 for the replay and puts them back after. By
#: this cache's own stated criterion they are STATE, not configuration. Serving
#: them from the cache made the AFTER snapshot a copy of the BEFORE snapshot:
#: all 126 after-snapshots in the ledger read the same 0.800 as `caps_before`,
#: and 41 of 41 caps-raising runs recorded `limits_cached: true`, so a leaked
#: 1.5 m/s / 20 rad/s^2 envelope — global, GUI-shared, surviving power cycles —
#: was invisible in every record ever written. The cost of the fix is four
#: extra getters per run.
STATIC_SNAPSHOT_KEYS = ("joint_max_speed_deg_s", "joint_max_acc_deg_s2")
#: The four the guard mutates. Always read fresh, never cached.
CAP_SNAPSHOT_KEYS = ("arm_max_line_speed", "arm_max_line_acc",
                     "arm_max_angular_speed", "arm_max_angular_acc")


def controller_snapshot(robot, limits_cache: Optional[dict] = None) -> dict:
    """READ-ONLY picture of the controller around a program run (F111): the
    error list the push frames latch on first code only, joint error flags,
    the collision stage, the active tool frame (payload + centroid — the
    2026-08-08 false-collision root cause), and the controller's OWN joint
    speed / acceleration limits (what 4105 is judged against). Every getter
    is guarded: a snapshot never fails a run.

    *limits_cache*: a dict the caller keeps for the life of a flow. The six
    CONFIGURATION getters are read into it once and reused after that, and the
    snapshot records `limits_cached: True` so a reader knows the values were
    not re-read on this run. Pass None to read everything, every time.
    """
    out = {}

    def grab(key, fn):
        try:
            out[key] = fn()
        except Exception as exc:                                       # noqa: BLE001
            out[key] = f"unavailable: {exc!r}"

    def arm_state():
        ret, st = robot.rm_get_current_arm_state()
        if ret != 0 or not st:
            return {"ret": ret}
        err = st.get("err")

        def nonzero(seq):
            out = []
            for c in seq or []:
                try:
                    v = int(c)
                except (TypeError, ValueError):
                    continue
                if v:
                    out.append(v)
            return out

        if isinstance(err, dict):
            n = int(err.get("err_len") or 0)
            raw = list(err.get("err") or [])
            codes = nonzero(raw[:n] if n else raw)
        else:
            codes = nonzero(err) if isinstance(err, list) else err
        return {"ret": ret, "err_codes": codes, "arm_err": st.get("arm_err"),
                "sys_err": st.get("sys_err"), "joint": st.get("joint")}

    grab("arm_state", arm_state)
    grab("joint_err_flag", lambda: robot.rm_get_joint_err_flag())
    grab("collision_stage", lambda: robot.rm_get_collision_stage())
    grab("tool_frame", lambda: robot.rm_get_current_tool_frame())
    # M47: an arm left in SIMULATION is a phantom that executes every dispatch
    # virtually and still scores `complete`, and NO record in the 157-run
    # ledger carries a run-mode field. Do NOT infer SIM from the motor current
    # channel — F30 measured 2150 mA of holding torque in SIMULATION.
    grab("run_mode", lambda: _runmode.snapshot_run_mode(robot))
    if limits_cache is not None and all(k in limits_cache for k in STATIC_SNAPSHOT_KEYS):
        out.update({k: limits_cache[k] for k in STATIC_SNAPSHOT_KEYS})
        out["limits_cached"] = True
    else:
        grab("joint_max_speed_deg_s", lambda: robot.rm_get_joint_max_speed())
        grab("joint_max_acc_deg_s2", lambda: robot.rm_get_joint_max_acc())
        out["limits_cached"] = False
        if limits_cache is not None:
            limits_cache.update({k: out[k] for k in STATIC_SNAPSHOT_KEYS if k in out})
    # Newton, 2026-09-02: the GUI shows a speed-overrun error on the TCP as
    # well as in joint space — the Cartesian supervision has its own limits.
    # ALWAYS fresh (H31): these are the values the replay guard moves.
    grab("arm_max_line_speed", lambda: robot.rm_get_arm_max_line_speed())
    grab("arm_max_line_acc", lambda: robot.rm_get_arm_max_line_acc())
    grab("arm_max_angular_speed", lambda: robot.rm_get_arm_max_angular_speed())
    grab("arm_max_angular_acc", lambda: robot.rm_get_arm_max_angular_acc())
    out["caps_cached"] = False
    grab("program_run_state", lambda: robot.rm_get_program_run_state())
    return out


def snapshot_line(snap: dict) -> str:
    a = snap.get("arm_state") if isinstance(snap.get("arm_state"), dict) else {}
    tf = snap.get("tool_frame")
    tf = tf[1] if isinstance(tf, (tuple, list)) and len(tf) == 2 and isinstance(tf[1], dict) else tf
    tfs = (f"{tf.get('frame_name')} payload {tf.get('payload')} kg centroid "
           f"({tf.get('x')}, {tf.get('y')}, {tf.get('z')})" if isinstance(tf, dict) else str(tf)[:60])
    cs = snap.get("collision_stage")
    cs = cs[1] if isinstance(cs, (tuple, list)) and len(cs) == 2 else cs
    jf = snap.get("joint_err_flag")
    jf = jf.get("err_flag") if isinstance(jf, dict) else jf
    vs = snap.get("joint_max_speed_deg_s")
    vs = vs[1] if isinstance(vs, (tuple, list)) and len(vs) == 2 else vs
    ac = snap.get("joint_max_acc_deg_s2")
    ac = ac[1] if isinstance(ac, (tuple, list)) and len(ac) == 2 else ac
    def one(key):
        v = snap.get(key)
        return v[1] if isinstance(v, (tuple, list)) and len(v) == 2 else v
    rm = snap.get("run_mode")
    rm = rm.get("label") if isinstance(rm, dict) else str(rm)[:24]
    return (f"controller: run mode {rm} | err {a.get('err_codes')} joint_err {jf} "
            f"collision stage {cs} | "
            f"tool {tfs} | joint max speed {vs} acc {ac} | TCP max line "
            f"{one('arm_max_line_speed')} m/s acc {one('arm_max_line_acc')} | angular "
            f"{one('arm_max_angular_speed')} rad/s acc {one('arm_max_angular_acc')}")


def run_program(robot, save_id: int, *, speed: int = 50, timeout_s: float = 180.0,
                on_transition: Optional[Callable[[dict], None]] = None,
                poll_s: float = 0.1, first_row_millideg=None,
                blocking_probe: bool = False,
                monitor: Optional["MotionMonitor"] = None) -> dict:
    """Start slot ``save_id`` and watch rm_get_program_run_state to the end.

    Returns {'started','ended','duration_s','transitions','wrong_id_seen',
    'transit_s','replay_s'}. duration_s is wall-clock from the first running
    state to run_state 0 and INCLUDES the implicit movej transit to the first
    point plus a short dwell (~0.3-0.4 s measured, 2026-09-01 hardware).
    When *first_row_millideg* is given, the loop also polls the joints and
    latches the arm's DEPARTURE from the first point (last poll within 0.5 deg
    before a sustained exit): transit_s = start->departure, replay_s =
    departure->end — the number to judge against :func:`expected_band_s`
    (Newton's correction, 2026-09-01: the band is for the REPLAY alone).

    poll_s: state-poll period. The duration is quantised by it on BOTH edges,
    and at 0.25 s that error was as wide as a 2600-point track's whole band
    (0.26 s) — a nominal run could read OUT OF BAND (caught by the executive's
    fake-clock test, 2026-09-01). 0.1 s keeps the error ~1/3 of a short band.

    *monitor*: an optional :class:`MotionMonitor`. Every transition (including
    the `-2` poll errors) is fed to it as it happens, and the probe's return
    is fed to it the moment the worker thread lands, so the caller does not
    have to re-derive the cut from the returned record.

    ``poll_errors`` counts the polls that came back non-zero and how many of
    them were `-2`. M43 (2026-09-08): the blocking probe is a SECOND concurrent
    SDK caller on the same arm and the ledger shows **173** `-2` returns spread
    over **84 of 125** probe runs (and 0 of the 0 non-probe runs — the ledger
    has no non-probe run to compare against, so the association is suggestive,
    not causal). Each `-2` blinds the fallback detector for one SDK reply
    timeout. DO NOT trip on "three consecutive `-2`": counted over the same
    125 runs, **18** have a run of three during startup and **ZERO** have one
    after the first `run_state == 1`, so such a rule would fire only where it
    must not.
    """
    # `blocking_probe` is the ONLY documented way to tell "the controller
    # finished the program" from "something stopped the trajectory": with
    # block=1 the vendor returns -4, "run state has stopped but the successful
    # running message was not received — was the trajectory stopped externally"
    # (rm_interface.h:3270). Called with block=0, as every run to 2026-09-03 was,
    # the return is 0 whatever happens and carries no information (F112). The
    # start is issued from a worker thread so the state polling below is
    # unchanged; the handle is opened in the SDK's triple-thread mode, which is
    # what the vendor documents for concurrent calls. Opt-in, because it is a
    # second concurrent SDK caller on a live arm and has never run on hardware.
    probe = {"used": bool(blocking_probe), "ret": None, "returned_s": None}
    if blocking_probe:
        import threading
        started_at = time.monotonic()

        def _blocking_start():
            try:
                probe["ret"] = int(robot.rm_set_program_id_run(int(save_id), int(speed), 1))
            except Exception as exc:                                   # noqa: BLE001
                probe["ret"] = f"raised {exc!r}"
            probe["returned_s"] = time.monotonic() - started_at
            if monitor is not None:
                monitor.on_probe(probe["ret"], probe["returned_s"])
        th = threading.Thread(target=_blocking_start, daemon=True,
                              name=f"program-run-{save_id}")
        th.start()
        ret = 0
    else:
        th = None
        ret = robot.rm_set_program_id_run(int(save_id), int(speed), 0)
    if ret != 0:
        return {"started": False, "ended": False, "run_ret": int(ret),
                "duration_s": None, "transitions": [], "wrong_id_seen": False,
                "blocking_probe": probe,
                "poll_errors": {"total": 0, "minus_two": 0, "max_consecutive": 0}}
    t0 = time.monotonic()
    t_started = None
    transitions, last = [], None
    poll_errors = {"total": 0, "minus_two": 0, "max_consecutive": 0}
    consec = 0
    started = ended = wrong_id = False
    want = ([v / 1000.0 for v in first_row_millideg]
            if first_row_millideg is not None else None)
    t_at_first = None      # last poll within 0.5 deg of the first point
    t_depart = None        # latched once the arm leaves that ball for good
    while time.monotonic() - t0 < timeout_s:
        ret, st = robot.rm_get_program_run_state()
        now = time.monotonic()
        if ret != 0:
            tr_err = {"t": now - t0, "error_ret": int(ret)}
            transitions.append(tr_err)
            poll_errors["total"] += 1
            if int(ret) == -2:
                poll_errors["minus_two"] += 1
                consec += 1
                poll_errors["max_consecutive"] = max(poll_errors["max_consecutive"], consec)
            else:
                consec = 0
            if monitor is not None:
                monitor.on_transition(tr_err)
            time.sleep(poll_s)
            continue
        consec = 0
        cur = (st.get("run_state"), st.get("id"), st.get("plan_num"))
        if cur != last:
            tr = {"t": now - t0, "run_state": st.get("run_state"), "id": st.get("id"),
                  "plan_num": st.get("plan_num"), "plan_speed": st.get("plan_speed")}
            transitions.append(tr)
            if on_transition:
                on_transition(tr)
            if monitor is not None:
                monitor.on_transition(tr)
            last = cur
        rs = st.get("run_state")
        if rs in (1, 2):
            if not started:
                t_started, started = now, True
            if st.get("id") not in (save_id, 0, None):
                wrong_id = True
        if want is not None and started and t_depart is None:
            cur = current_joints_deg(robot)
            if cur is not None:
                gap = max(abs(a - b) for a, b in zip(cur, want))
                if gap <= 0.5:
                    t_at_first = time.monotonic()
                elif t_at_first is not None:
                    t_depart = t_at_first        # left the ball: replay begins
                    if monitor is not None:
                        monitor.mark_replay_start(t_depart - t0)
        if started and rs == 0:
            ended = True
            break
        time.sleep(poll_s)
    t_end = time.monotonic()
    dur = (t_end - t_started) if (started and ended and t_started) else None
    transit = replay = None
    if dur is not None and t_depart is not None:
        transit = t_depart - t_started
        replay = t_end - t_depart
    if th is not None:
        th.join(timeout=max(5.0, timeout_s - (time.monotonic() - t0)))
        if th.is_alive():
            probe["ret"] = "still blocked at join"
    return {"started": started, "ended": ended, "run_ret": 0, "duration_s": dur,
            "transit_s": transit, "replay_s": replay,
            "transitions": transitions, "wrong_id_seen": wrong_id,
            "blocking_probe": probe, "poll_errors": poll_errors}


# ═══════════════════════════════════════════ THE CANFD CHANNEL — the same file, host-timed
#
# WHY IT EXISTS (Newton, 2026-09-17): "a true comparison of the track upload
# channel and the canfd channel". The upload channel hands the controller a
# file and its own player sets the timing (5.5545 ms/pt nominal, +-6 % run to
# run, F108). This channel sends the SAME lines, one `rm_movej_canfd` per line,
# at a period the HOST keeps on an absolute deadline — so the only thing that
# differs between the two runs is who plays the file. Everything around the
# stream (start-gap check, movej bridge, tool-frame / caps / global-speed
# guards, the 5 ms capture, the replay verdict, the record) is the upload run's.
#
# A PASS-THROUGH STREAM FIRES NO EVENTS. There is no program, so no
# `program_run_finish`, no run state and no blocking probe; completeness is the
# stream sending every line plus the capture's replay verdict, as for the
# upload channel.

#: Vendor limits for `rm_movej_canfd` (Gen-3 V1.7.4). Both are STATED, verbatim
#: and page-pinned: `docs/Manuals/Realman/RealMan_Gen3_04_JSON_Protocol.pdf`
#: pp.46 and 48, in the WARNING block under the `movep_canfd` / `movej_canfd`
#: parameter tables (twice in 239 pages; Chinese original of the same chapter:
#: "帧与帧之间关节的角度最大不能超过 10°，并保证关节规划的速度不要超过 180°/s，
#: 否则关节会不响应"; live at /en/robot/json/motionConfig/ and, unrevised, in the
#: Gen-4 docs at /robot4th/json/motionConfig/). It governs `movep_canfd` too, so
#: it is a property of the CHANNEL, not of the joint-angle call.
#:
#: WHAT THE VENDOR DOES *NOT* SAY, and what we wrongly asserted here until
#: 2026-09-20: that a breaking line "cannot be executed". The documented
#: consequence is NON-RESPONSE — "otherwise the joint will not respond" — with
#: no error code, no return value and no clamp anywhere in the passage. 0x4000
#: latches on the 10 deg STEP rule (target-vs-motor), not on the rate. The code
#: the vendor does tie to over-speed in pass-through is 0x1007 / 4103 "Joint
#: overspeed" ("usually occurs during transparency and requires reducing the
#: planned trajectory speed"), which we have never once seen in 481 captures.
#:
#: AND THE 180 IS FLAT ONLY AS *WRITTEN*. The arm's own firmware reports
#: `libalgo_get_joint_vel_maxlim` = 180/180/225x5, and the controller's speed
#: supervisor demonstrably follows THAT, not a flat 180: across 481 captures,
#: 41 upload runs executed 180-228 deg/s on a 225-rated joint and stayed clean,
#: while 201 of the 209 that crossed a joint's OWN rating raised 4105 "speed out
#: of limit". Whether the pass-through path shares that supervisor is UNTESTED,
#: because this gate has never let a frame above 180 through — see
#: `CANFD_RATED_DEG_S` and the `rate_gate` argument, which exist to test it.
CANFD_MAX_RATE_DEG_S = 180.0
CANFD_MAX_STEP_DEG = 10.0
#: THE DEFAULT CANFD PERIOD IS THE CONTROLLER'S CYCLE, 5.0 ms (Newton, 2026-09-20).
#: It was `REPLAY_CLOCK_S` (5.5545 ms), which is not a clock: it is the upload
#: player's WHOLE-RUN AVERAGE, and 5.5545/5.000 = 1.1109 — an 11.09 % inflation
#: that lands inside the independently measured 10-13 % hold fraction. The player
#: advances at most one point per 5 ms controller cycle and holds stochastically;
#: the average is the cycle plus the holds. CANFD has NO holds (one line per
#: period on an absolute-deadline loop, measured lateness 0.13-3.18 ms,
#: `frames_late_over_period` 0), so on this channel the period IS the clock and
#: 5.5545 only streams 11 % slower than the controller can cycle.
#: Runs before 2026-09-20 used other periods — the 72 CANFD records carry their
#: own `canfd.period_s`, so nothing is lost; group by it, never assume it.
CANFD_DEFAULT_PERIOD_S = 0.005
#: The arm's OWN per-joint velocity limits, read offline from the vendor's
#: algorithm library (`libalgo_get_joint_vel_maxlim`, model 1 / dof 7 / ISF) and
#: identical to the RM75 Ontology Parameters table. The OPT-IN alternative gate:
#: `rate_gate="per_joint"` measures every joint against its own rating instead
#: of the flat 180. It is never the default — the flat figure is what the vendor
#: documents for this channel, and the asymmetry is brutal (being wrong upward
#: is a silent mid-stroke position loss on a position-controlled arm), so the
#: per-joint gate is for deliberate, instrumented probes only.
CANFD_RATED_DEG_S = (180.0, 180.0, 225.0, 225.0, 225.0, 225.0, 225.0)
#: The two gates, by name. `flat` is the vendor's documented channel rule.
CANFD_RATE_GATES = ("flat", "per_joint")
#: mode 1 (curve fit) buffers up to ~15 cycles (vendor); the tail is re-sent so
#: the arm is not left short of the last line.
CANFD_CURVE_FIT_LAG_CYCLES = 15
#: the last line is re-sent for this long in every mode, so the capture shows
#: the arm arriving under a held setpoint rather than a stream that just ended.
CANFD_HOLD_S = 0.10
#: Codes the push carries that the controller flags and keeps moving through.
#: 4105 "speed out of limit" is on complete upload runs; anything else STOPS
#: the stream (a stopped arm fed further lines is fed lines it cannot reach).
CANFD_WARN_ONLY_CODES = frozenset({"4105"})
#: THE STALL RULE, timing-free on purpose. Replaying it over the upload ledger
#: with a "command i sent at t0 + i*clock" model read 29-72 deg on COMPLETE
#: runs, because that player's clock drifts; a lag threshold cannot be set from
#: data this channel has never produced. What cannot happen on a following arm:
#: the commanded lines travel >= CANFD_STALL_CMD_DEG (per joint path length)
#: in CANFD_STALL_WINDOW_S while the arm travels <= CANFD_STALL_ARM_DEG. At the
#: fitted lags (tau_j 0.03-0.07 s) an arm following 20 deg/s moves ~3.6 deg in
#: that window; encoder noise over 50 frames is ~0.1 deg.
CANFD_STALL_WINDOW_S = 0.25
CANFD_STALL_CMD_DEG = 5.0
CANFD_STALL_ARM_DEG = 0.5
CANFD_CHECK_EVERY_S = 0.05
#: Sends that fail back to back mean the controller is not receiving. TWO, not the 0.10 s (18 frames)
#: this started as: the next line the controller DOES take is that many frames further along the file,
#: and 18 frames of a 0.9 deg/line track is a 16.2 deg jump — past the vendor's 10 deg/frame and far
#: past the 180 deg/s the preflight proved (2026-09-17 review).
CANFD_SEND_FAIL_FRAMES = 2
#: The stream's contract is one line per period. If the host is ever this many periods late the
#: controller is no longer being fed at the rate the preflight validated, so the stream stops rather
#: than firing the backlog off back to back (measured lateness to date: 0.13-3.18 ms, under 1 period).
CANFD_LATE_ABORT_PERIODS = 2.0
#: No push frame for this long and the stop check is blind — the stream stops instead of running on
#: unwatched (the push accepts a wrong target silently and delivers nothing).
CANFD_MAX_BLIND_S = 0.5


def canfd_rate_limits(rate_gate: str = "flat") -> tuple:
    """The seven per-joint rate ceilings the named gate enforces, deg/s.

    `flat` is the vendor's documented channel rule (180 on every joint);
    `per_joint` is the arm's own rating table. Anything else is a REFUSAL, not
    a silent fallback — a typo that widened the gate would reach the arm.
    """
    g = str(rate_gate)
    if g == "flat":
        return (CANFD_MAX_RATE_DEG_S,) * 7
    if g == "per_joint":
        return tuple(CANFD_RATED_DEG_S)
    raise ValueError(f"unknown CANFD rate gate {rate_gate!r}; expected one of {CANFD_RATE_GATES}")


def canfd_frame0_limit_deg(period_s: float, rate_gate: str = "flat") -> float:
    """The largest start gap the FIRST line may close in one frame.

    Held to the TIGHTEST joint's ceiling whatever the gate: the start gap is not
    attributed to one joint here, and the frame-0 trap (`--at-start-tol` above
    the per-frame limit makes frame 0 itself a violation) is what this guards.
    """
    return min(CANFD_MAX_STEP_DEG, min(canfd_rate_limits(rate_gate)) * float(period_s))


def canfd_preflight(rows_millideg, period_s: float, rate_gate: str = "flat") -> dict:
    """Every line against the vendor's pass-through limits at *period_s*.

    `rate_gate` picks the rate ceiling: `flat` (default — the vendor's
    documented 180 deg/s on every joint) or `per_joint` (the arm's own
    180/180/225x5 rating table, an OPT-IN probe gate; see `CANFD_RATED_DEG_S`).
    The STEP rule is the vendor's 10 deg either way — it has its own error code
    and no per-joint variant.

    Returns {'ok', 'period_s', 'lines', 'rate_gate', 'rate_limits_deg_s',
    'max_step_deg' (per joint), 'peak_rate_deg_s' (per joint),
    'peak_rate_util' (per joint, of the gate in force), 'lines_over_rate',
    'lines_over_step', 'first_over' (line index or None), 'reasons'}.
    """
    p = float(period_s)
    gate = canfd_rate_limits(rate_gate)
    n = len(rows_millideg)
    max_step = [0.0] * 7
    over_rate = over_step = 0
    first_over = None
    lim_rate = [g * p for g in gate]                     # per joint, so a gate change is local
    prev = rows_millideg[0] if n else None
    for i in range(1, n):
        cur = rows_millideg[i]
        bad_r = bad_s = False
        for j in range(7):
            s = abs(cur[j] - prev[j]) / 1000.0
            if s > max_step[j]:
                max_step[j] = s
            # "must not exceed": a step AT the limit is legal. The tolerance absorbs float noise
            # only — 1098 millideg at 6.1 ms is exactly 180 deg/s and read as 180.00000000000003.
            if s > CANFD_MAX_STEP_DEG + 1e-6:
                bad_s = True
            elif s > lim_rate[j] + 1e-6:
                bad_r = True
        if bad_r or bad_s:
            over_rate += int(bad_r)
            over_step += int(bad_s)
            if first_over is None:
                first_over = i
        prev = cur
    peak = [s / p for s in max_step]
    util = [peak[j] / gate[j] for j in range(7)]
    # THE SHORTEST PERIOD THESE LINES ALLOW, ROUNDED UP to 0.01 ms (2026-09-17: the advice "6.10 ms"
    # was the exact figure rounded DOWN, and two lines still read 180.0x deg/s at 6.1000 ms).
    import math as _math
    worst = max(max_step) if max_step else 0.0
    # the shortest period is set by the joint closest to ITS OWN ceiling, which under the
    # flat gate is just the largest step and under the per-joint gate need not be
    need_s = max((max_step[j] / gate[j] for j in range(7)), default=0.0)
    min_period_ms = (None if worst > CANFD_MAX_STEP_DEG + 1e-6 else
                     _math.ceil(need_s * 1e5 - 1e-6) / 100.0)
    reasons = []
    if n < 2:
        reasons.append(f"the track has {n} line(s); a stream needs at least 2")
    if over_step:
        reasons.append(f"{over_step} line(s) step more than {CANFD_MAX_STEP_DEG:g} deg in one frame")
    if over_rate:
        j = max(range(7), key=lambda k: util[k])         # the BINDING joint, not the fastest one
        reasons.append(f"{over_rate} line(s) exceed the {rate_gate} rate gate at "
                       f"{p * 1e3:.4f} ms/line (peak {peak[j]:.2f} deg/s on joint {j + 1}, "
                       f"{util[j]:.3f} of its {gate[j]:g} deg/s ceiling, first at line {first_over}); "
                       + (f"these lines need --canfd-period-ms >= {min_period_ms:.2f}, or a slower file"
                          if min_period_ms is not None else
                          f"no period can carry this file: a step over {CANFD_MAX_STEP_DEG:g} deg is "
                          f"refused whatever the period"))
    return {"ok": not reasons, "period_s": p, "lines": n,
            "rate_gate": str(rate_gate), "rate_limits_deg_s": list(gate),
            "max_step_deg": [round(s, 4) for s in max_step],
            "peak_rate_deg_s": [round(r, 1) for r in peak],
            "peak_rate_util": [round(u, 4) for u in util],
            "lines_over_rate": over_rate, "lines_over_step": over_step,
            "first_over": first_over, "min_period_ms": min_period_ms, "reasons": reasons}


class CanfdWatch:
    """The stream's stop check, fed by the capture (`UdpRecorder.attach_monitor`).

    `on_frame` runs on the recorder's thread and only stores; `check` runs on
    the streaming thread between sends. It trips on a STOPPING code in the push
    (anything outside CANFD_WARN_ONLY_CODES) or on the stall rule above, and
    everything it saw is in `summary()` whether it tripped or not.
    """

    def __init__(self, rows_millideg, period_s: float, *,
                 window_s: float = CANFD_STALL_WINDOW_S,
                 cmd_deg: float = CANFD_STALL_CMD_DEG,
                 arm_deg: float = CANFD_STALL_ARM_DEG):
        import collections
        self.period_s = float(period_s)
        self.window_s = float(window_s)
        self.cmd_deg = float(cmd_deg)
        self.arm_deg = float(arm_deg)
        self.w_lines = max(1, int(round(self.window_s / self.period_s)))
        # per-joint cumulative path length of the COMMAND, so the window's
        # travel is one subtraction per joint
        cum = [[0.0] * 7]
        for i in range(1, len(rows_millideg)):
            a, b, c = rows_millideg[i - 1], rows_millideg[i], cum[-1]
            cum.append([c[j] + abs(b[j] - a[j]) / 1000.0 for j in range(7)])
        self._cum = cum
        self._frames = collections.deque(maxlen=max(64, int(4 * self.window_s / 0.005)))
        self.n_frames = 0
        self.stop_codes: dict = {}
        self.warn_codes: dict = {}
        self.tripped: Optional[dict] = None
        self.checks = 0
        self.max_blind_s = 0.0
        self._last_frame_t = None
        self.t_start: Optional[float] = None
        self.codes_before_stream: dict = {}

    def start(self, t: float) -> None:
        """The stream's first send. Codes the push carried BEFORE it (a bridge
        movej, a latch from the last session) are kept apart and not tripped on:
        they are the state this run started in, recorded, not a stop it caused."""
        self.t_start = float(t)
        for book in (self.stop_codes, self.warn_codes):
            for c, tc in list(book.items()):
                if tc < self.t_start:
                    self.codes_before_stream[c] = tc
                    del book[c]

    def on_frame(self, t: float, frame: dict) -> None:
        try:
            self.n_frames += 1
            if self._last_frame_t is not None:
                self.max_blind_s = max(self.max_blind_s, t - self._last_frame_t)
            self._last_frame_t = t
            codes = [str(c) for c in (frame.get("err") or []) if c]
            js = frame.get("joint_status") or {}
            codes += [f"joint{i + 1}:{c}" for i, c in enumerate(js.get("joint_err_code") or []) if c]
            for c in codes:
                # A code seen BEFORE the first send is recorded apart (it is the state the run
                # started in, and `run_one` refuses to stream into a stop-class one) but it is NOT
                # muted afterwards: these are live values, not a latch — one capture carries
                # err [4109] and then [4105] — so a recurrence during the stream must trip
                # (2026-09-17 review).
                book = self.warn_codes if c in CANFD_WARN_ONLY_CODES else self.stop_codes
                book.setdefault(c, t)
            jp = js.get("joint_position")
            if jp and len(jp) == 7:
                self._frames.append((t, [v / 1000.0 for v in jp]))
        except Exception:                                              # noqa: BLE001
            pass                  # never kill the recorder thread; the capture is the evidence

    def check(self, now: float, i_sent: int) -> Optional[dict]:
        """A trip dict, or None. *i_sent* is the last line sent."""
        self.checks += 1
        if self.tripped is not None:
            return self.tripped
        if self.stop_codes:
            code, t = min(self.stop_codes.items(), key=lambda kv: kv[1])
            self.tripped = {"why": "stop_code", "code": code, "t_mono": t,
                            "line": i_sent,
                            "detail": f"the push carried {code}, which is not a warn-only code"}
            return self.tripped
        if i_sent < self.w_lines:
            return None
        a, b = self._cum[i_sent - self.w_lines], self._cum[i_sent]
        cmd = max(b[j] - a[j] for j in range(7))
        if cmd < self.cmd_deg:
            return None
        if self._last_frame_t is not None and now - self._last_frame_t > CANFD_MAX_BLIND_S:
            self.tripped = {"why": "blind", "t_mono": now, "line": i_sent,
                            "blind_s": round(now - self._last_frame_t, 3),
                            "detail": f"no push frame for {now - self._last_frame_t:.2f} s — the stop "
                                      f"check cannot see the arm, so the stream stops"}
            return self.tripped
        frames = [f for f in list(self._frames) if f[0] >= now - self.window_s]
        if len(frames) < max(4, int(0.5 * self.window_s / 0.005)):
            return None          # too few frames in THIS window to judge the stall rule
        arm = 0.0
        for j in range(7):
            s = 0.0
            for k in range(1, len(frames)):
                s += abs(frames[k][1][j] - frames[k - 1][1][j])
            arm = max(arm, s)
        if arm <= self.arm_deg:
            self.tripped = {"why": "stall", "t_mono": now, "line": i_sent,
                            "cmd_travel_deg": round(cmd, 3), "arm_travel_deg": round(arm, 3),
                            "window_s": self.window_s,
                            "detail": f"the lines travelled {cmd:.2f} deg in {self.window_s:g} s "
                                      f"and the arm {arm:.2f} deg — it is not following"}
        return self.tripped

    def summary(self) -> dict:
        return {"frames_seen": self.n_frames, "checks": self.checks,
                "tripped": self.tripped, "stop_codes": dict(self.stop_codes),
                "warn_codes": dict(self.warn_codes),
                "codes_before_stream": dict(self.codes_before_stream),
                "max_blind_s": round(self.max_blind_s, 4),
                "rule": {"window_s": self.window_s, "cmd_deg": self.cmd_deg,
                         "arm_deg": self.arm_deg,
                         "warn_only_codes": sorted(CANFD_WARN_ONLY_CODES)}}


def stream_track(robot, rows_millideg, *, period_s: float = REPLAY_CLOCK_S,
                 follow: bool = True, trajectory_mode: int = 0, radio: int = 0,
                 hold_s: float = CANFD_HOLD_S, watch: Optional[CanfdWatch] = None,
                 result: Optional[dict] = None, log: Callable[[str], None] = print,
                 clock: Callable[[], float] = time.monotonic,
                 sleep: Callable[[float], None] = time.sleep) -> dict:
    """Send every line as `rm_movej_canfd`, one per *period_s*, on absolute deadlines.

    `time.monotonic` is the capture's clock too, so `t_first_send`/`t_last_line`
    place the stream exactly inside the frames. *result* is filled as the
    stream goes, so an interrupt still leaves what was sent. The garbage
    collector is paused for the stream: a collection over a large heap is a
    multi-millisecond stall against a 5.55 ms period. Plain sleeps only — a
    spin would hold the GIL against the recorder thread stamping the frames.

    WHAT THE SEND TIMESTAMPS ARE, AND ARE NOT (2026-09-17 review, C4). Both are read BEFORE the
    `rm_movej_canfd` call for that line, so they are when the host was READY to send, not when the
    frame reached the controller: the SDK call itself, the socket write and the controller's own
    intake are all outside them. `late_ms_max` therefore measures the HOST's scheduling, and an
    analysis that aligns a capture to `t_first_send` inherits that offset as a constant — which is why
    the per-joint lag fits (26-33 ms) are read as a lag and not as an absolute transport delay. The
    offset is constant to within the loop's own jitter, so it cancels in every run-to-run comparison
    and in every lag DIFFERENCE, but "0.00 ms of alignment error" means the arithmetic, not the wire.
    """
    import gc
    import math
    r = result if result is not None else {}
    rows = [[v / 1000.0 for v in row] for row in rows_millideg]
    n = len(rows)
    p = float(period_s)
    tail = int(math.ceil(float(hold_s) / p)) + (CANFD_CURVE_FIT_LAG_CYCLES if int(trajectory_mode) else 0)
    check_every = max(1, int(round(CANFD_CHECK_EVERY_S / p)))
    fail_limit = CANFD_SEND_FAIL_FRAMES
    r.update(period_s=p, follow=bool(follow), trajectory_mode=int(trajectory_mode),
             radio=int(radio), lines=n, hold_frames=tail, sent=0, lines_sent=0,
             send_errors=0, first_bad_ret=None, first_bad_line=None,
             t_first_send=None, t_last_line=None, t_end=None, tripped=None,
             aborted=None, late_ms_max=0.0, late_ms_mean=0.0, frames_late_over_period=0,
             completed=False)
    late_sum = 0.0
    consecutive_fail = 0
    gc_was = gc.isenabled()
    gc.disable()
    t0 = clock()
    try:
        for k in range(n + tail):
            i = k if k < n else n - 1
            deadline = t0 + k * p
            now = clock()
            if deadline > now:
                sleep(deadline - now)
                now = clock()
            late = now - deadline
            if late > CANFD_LATE_ABORT_PERIODS * p:
                r["aborted"] = (f"the host was {late * 1e3:.1f} ms late at line {i} "
                                f"({late / p:.1f} periods): the controller is no longer being fed at "
                                f"the rate the preflight validated")
                break
            late_sum += late
            if late * 1e3 > r["late_ms_max"]:
                r["late_ms_max"] = late * 1e3
            if late > p:
                r["frames_late_over_period"] += 1
            try:
                ret = robot.rm_movej_canfd(rows[i], bool(follow), 0, int(trajectory_mode), int(radio))
            except Exception as exc:                                   # noqa: BLE001
                ret = f"raised {exc!r}"
            if r["t_first_send"] is None:
                r["t_first_send"] = now
                if watch is not None and callable(getattr(watch, "start", None)):
                    watch.start(now)
            if k == n - 1:
                r["t_last_line"] = now
            r["sent"] = k + 1
            r["lines_sent"] = min(k + 1, n)
            if ret not in (None, 0):
                r["send_errors"] += 1
                consecutive_fail += 1
                if r["first_bad_ret"] is None:
                    r["first_bad_ret"], r["first_bad_line"] = ret, i
                if consecutive_fail >= fail_limit:
                    r["aborted"] = (f"{consecutive_fail} sends in a row failed (last ret={ret!r}) — "
                                    f"the controller is not taking the stream")
                    break
            else:
                consecutive_fail = 0
            if watch is not None and k < n and k % check_every == 0:
                trip = watch.check(now, i)
                if trip is not None:
                    r["tripped"] = trip
                    r["aborted"] = f"stop check: {trip.get('detail')}"
                    break
        else:
            r["completed"] = True
    finally:
        r["t_end"] = clock()
        if gc_was:
            gc.enable()
        if r["sent"]:
            r["late_ms_mean"] = round(late_sum / r["sent"] * 1e3, 4)
        r["late_ms_max"] = round(r["late_ms_max"], 4)
        if r["t_first_send"] is not None:
            last = r["t_last_line"] if r["t_last_line"] is not None else r["t_end"]
            r["replay_s"] = last - r["t_first_send"]
            r["ms_per_line"] = (r["replay_s"] / max(r["lines_sent"] - 1, 1) * 1e3)
        if watch is not None:
            r["watch"] = watch.summary()
    if r["aborted"]:
        log(f"** CANFD stream STOPPED at line {r['lines_sent']} of {n}: {r['aborted']} — the arm "
            f"holds the last line it accepted")
    return r
