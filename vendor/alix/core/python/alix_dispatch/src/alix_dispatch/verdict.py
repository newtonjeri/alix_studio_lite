"""Did the run actually complete? — C9's mandatory check, and its vocabulary.

C9 is precise about what each rung of the ladder may CLAIM:

* the **emulator** rung means "the program ran", never that the joints were
  within limits — it emits no joint-speed telemetry at all, and traced 236 % of
  the commanded path on the reference geometry, so it cannot validate arc
  geometry either;
* **SIM** shares that blindness — every SIM run on the 2026-08-19 ramp reported
  0 % on all seven joints — so SIM **cannot clear** a rung on joint load, only
  fail one. But SIM is exact on path completion, and it reproduced the 1.00 m/s
  failure offline at the same waypoint with no hardware;
* therefore a SIM truncation **BLOCKS** REAL, and the distance and
  final-waypoint checks MUST run in SIM.

The thresholds below are the one thing here with no contract backing. They are
upstream's measured numbers, cited, and every record says so — see
`THRESHOLD_SOURCE` and `contract_backed`. They are NOT invented here (C16.4),
and they are NOT the contract's (the contract states no tolerance).
"""

from __future__ import annotations

import hashlib
import json
import math
from typing import Dict, List, Optional, Sequence, Tuple

#: A completed rung lands within ~1 mm of its last waypoint; the 2026-08-19
#: v=1.00 failure ended ~350 mm out. 20 mm sits far from both.
FINAL_WAYPOINT_TOL_M = 0.020
#: SIM traces ~101 % of the commanded path and REAL ~104 %; the failure traced
#: 53 % (SIM) / 57 % (REAL). 0.90 sits far from both.
TRACED_FLOOR = 0.90
THRESHOLD_SOURCE = ("RMDemo_DualArmConcept/src/run_speed_ramp.py:175-178, "
                    "measured 2026-08-19; the contract states no tolerance")

#: What each rung is permitted to say about itself.
RUNG_CLAIMS = {
    "emulator": "the program ran; NOT that joints were within limits, and "
                "arc geometry is not validated here",
    "sim": "the path completed; joints NOT evaluated (SIM's joint channel "
           "reads 0 % on all seven)",
    "real": "the path completed; joints evaluated only if telemetry carried "
            "them",
}


def _xyz(pose) -> List[float]:
    if isinstance(pose, dict):
        return [float(pose["x"]), float(pose["y"]), float(pose["z"])]
    return [float(pose[0]), float(pose[1]), float(pose[2])]


def commanded_path(program, side: Optional[str] = None,
                   stage: Optional[str] = None) -> dict:
    """The commanded geometry, read from the very calls that were dispatched.

    The program IS the command, so it is the only admissible reference — a
    reference re-derived from the config could differ from what was sent and
    would check the wrong thing (C16.4).

    *stage* SCOPES the path, and it must be set to whatever stage the recording
    is windowed to. The two are one concept: a whole-program reference checked
    against a chain-windowed stream compares the last CLEANING waypoint with
    the pose the arm was at when the window closed, and every shipped config
    ends `execute_path -> move_to_pre_start -> forbid_collision -> move_to_rest`
    — so unscoped, `poses[-1]` is the RETURN pose and the check is nonsense in
    both directions.

    `movej` carries joints, not a pose, so it contributes no Cartesian
    waypoint. It is REPORTED in `movej_stages` rather than dropped quietly.
    """
    poses: List[List[float]] = []
    names: List[Optional[str]] = []
    arc_vias: List[Optional[List[float]]] = []
    movej_stages: List[str] = []
    tool_frame = None
    pole_m = None
    chain_stage = None
    collapsed = 0
    arcs = 0

    for call in program.calls:
        if side is not None and getattr(call, "arm", None) not in (None, side):
            continue
        # The tool frame is a SESSION property, not a stage one — it is
        # selected before the first Cartesian move and still applies inside a
        # scoped window, so read it whatever stage we are scoped to.
        d = call.detail or {}
        if stage is not None and call.stage != stage and call.kind != "set_tool_frame":
            continue
        if call.kind == "set_tool_frame":
            tool_frame = d.get("tool_frame")
        elif call.kind == "movej":
            movej_stages.append(call.stage)
        elif call.kind in ("movel", "movej_p"):
            if d.get("pose"):
                if poses:
                    arc_vias.append(None)
                poses.append(_xyz(d["pose"]))
                names.append(d.get("pose_name") or call.stage)
        elif call.kind == "movel_chain":
            chain_stage = call.stage
            pole_m = d.get("pole_m", pole_m)
            collapsed += int(d.get("collapsed_points") or 0)
            arcs += int(d.get("arc_moves") or 0)
            for move in d.get("moves", []):
                if poses:
                    # The via belongs to the SEGMENT arriving at this move.
                    arc_vias.append(_xyz(move["via"]) if move.get("via") else None)
                poses.append(_xyz(move["pose"]))
                names.append(move.get("point"))
    return {
        "poses": poses,
        "arc_vias": arc_vias,
        "waypoint_names": names,
        "tool_frame": tool_frame,
        "pole_m": pole_m,
        "chain_stage": chain_stage,
        "num_waypoints": len(poses),
        "segments": max(0, len(poses) - 1),
        "movej_stages": movej_stages,
        "collapsed_points": collapsed,
        "arc_moves": arcs,
    }


def commanded_length_m(commanded: dict) -> float:
    """Commanded path length. An arc segment contributes its CHORD PAIR.

    That is upstream's convention and the number the contract's reference
    geometry was measured with. Integrating the true arc instead would move
    alix's figure off the contract's for the same path — a divergence with no
    benefit, since both sides only ever compare like with like.
    """
    poses, vias = commanded["poses"], commanded["arc_vias"]
    total = 0.0
    for i in range(len(poses) - 1):
        via = vias[i] if i < len(vias) else None
        if via is not None:
            total += math.dist(poses[i], via) + math.dist(via, poses[i + 1])
        else:
            total += math.dist(poses[i], poses[i + 1])
    return total


def program_fingerprint(program, *, blend_pct: Optional[int] = None,
                        tool_frames: Optional[dict] = None,
                        pole_m: Optional[dict] = None) -> str:
    """Identity of the COMMANDED MOTION, mode-invariant by construction.

    A SIM pass licenses REAL only for the identical commanded motion, so the
    ladder is keyed on this. It must exclude mode, addresses, timestamps and
    the SIM skip disposition — a fingerprint that varied with mode would match
    nothing on the REAL rung and the gate would pass vacuously.

    It covers `movej` and `movej_p` as well as the chain: a fingerprint over
    only the chain's moves would let a transit divergence through its own gate.
    """
    items: List[object] = []
    for call in program.calls:
        d = call.detail or {}
        if call.kind == "movej":
            items.append(["movej", call.arm, call.stage,
                          [round(float(v), 6) for v in d.get("joints_deg", [])],
                          d.get("r"), d.get("connect")])
        elif call.kind in ("movel", "movej_p"):
            items.append([call.kind, call.arm, call.stage,
                          [round(v, 6) for v in _xyz(d["pose"])] if d.get("pose") else None,
                          d.get("r"), d.get("connect")])
        elif call.kind == "movel_chain":
            items.append(["chain", call.arm, call.stage,
                          [[m.get("point"),
                            [round(v, 6) for v in _xyz(m["pose"])],
                            ([round(v, 6) for v in _xyz(m["via"])]
                             if m.get("via") else None),
                            m.get("v_pct"), m.get("r"), m.get("connect")]
                           for m in d.get("moves", [])]])
        elif call.kind == "lift":
            items.append(["lift", call.arm, call.stage,
                          round(float(d.get("height_m") or 0.0), 6)])
    blob = json.dumps({"task": getattr(program, "task", None), "items": items,
                       "blend_pct": blend_pct,
                       "tool_frames": tool_frames or {},
                       "pole_m": pole_m or {}},
                      sort_keys=True, default=str)
    return hashlib.sha256(blob.encode()).hexdigest()


def _traced_m(rows: Sequence[Sequence[float]], ix: int, iy: int, iz: int) -> float:
    total = 0.0
    for i in range(1, len(rows)):
        total += math.dist((rows[i][ix], rows[i][iy], rows[i][iz]),
                           (rows[i - 1][ix], rows[i - 1][iy], rows[i - 1][iz]))
    return total


def check_completion(commanded: dict, rows: Sequence[Sequence[float]],
                     header: Sequence[str],
                     window: Optional[Tuple[float, float]] = None
                     ) -> Tuple[bool, str, dict]:
    """Did the arm reach the last commanded waypoint, having travelled the path?

    FINAL WAYPOINT FIRST. The summed traced distance runs 7-10 % long on this
    stream from sample noise rather than travel, so a genuine shortfall can
    hide inside that margin; the arrival test cannot.
    """
    try:
        ix, iy, iz = (header.index("tcp_x"), header.index("tcp_y"),
                      header.index("tcp_z"))
    except ValueError:
        return False, "stream has no tcp_x/y/z columns", {}

    whole = list(rows)
    windowed = ([r for r in whole if window[0] <= r[0] <= window[1]]
                if window else whole)
    detail: Dict[str, object] = {
        "window": list(window) if window else None,
        "samples_windowed": len(windowed), "samples_whole": len(whole),
        "traced_whole_stream_m": round(_traced_m(whole, ix, iy, iz), 4),
        "threshold_source": THRESHOLD_SOURCE,
        "contract_backed": False,
    }
    if not windowed:
        detail["reason"] = "no samples in the window"
        return False, "NO SAMPLES — the check did not run, so it did not pass", detail

    want = commanded_length_m(commanded)
    got = _traced_m(windowed, ix, iy, iz)
    detail["traced_windowed_m"] = round(got, 4)
    detail["commanded_m"] = round(want, 4)
    detail["traced_ratio"] = round(got / want, 4) if want > 0 else None

    poses = commanded["poses"]
    if not poses:
        detail["reason"] = "the program commands no Cartesian waypoint"
        return False, "NO COMMANDED PATH — nothing to check against", detail
    end = [windowed[-1][ix], windowed[-1][iy], windowed[-1][iz]]
    miss = math.dist(end, poses[-1])
    detail["final_waypoint_miss_m"] = round(miss, 4)
    detail["final_waypoint_tol_m"] = FINAL_WAYPOINT_TOL_M
    detail["traced_floor"] = TRACED_FLOOR

    if miss > FINAL_WAYPOINT_TOL_M:
        return False, (f"TRUNCATED — ended {miss * 1000:.0f} mm from the last "
                       f"commanded waypoint (tol {FINAL_WAYPOINT_TOL_M * 1000:.0f} mm)"), detail
    if want > 0 and got < TRACED_FLOOR * want:
        return False, (f"TRUNCATED — traced {got:.3f} m of a commanded "
                       f"{want:.3f} m ({got / want:.0%}, floor "
                       f"{TRACED_FLOOR:.0%})"), detail
    return True, (f"COMPLETE — arrived within {miss * 1000:.0f} mm, traced "
                  f"{got:.3f} m of {want:.3f} m"), detail


def verdict(rung: str, commanded: dict, rows: Sequence[Sequence[float]],
            header: Sequence[str], window: Optional[Tuple[float, float]],
            *, sim: bool) -> Tuple[bool, str, dict]:
    """The rung's verdict, in the vocabulary that rung is entitled to use."""
    ok, text, detail = check_completion(commanded, rows, header, window)
    detail["rung"] = rung
    detail["claims"] = RUNG_CLAIMS.get(rung, "unknown rung")
    # C9: neither the emulator nor SIM carries a usable joint-speed channel, so
    # neither may be reported as having cleared the joint criteria. Say so in
    # the record rather than leaving a reader to infer it from an absence.
    detail["joint_criteria"] = (
        "NOT APPLIED (SIM: the joint-speed channel reads 0 % on all seven)"
        if rung in ("sim", "emulator") or sim else
        "evaluated only where the stream carried joint speeds")
    return ok, text, detail
