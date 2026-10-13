"""THE TCP EXECUTION GATE — how far the arm's pad may depart from the RAW taught path, per slot.

Newton, 2026-09-22: the lid and seat tasks (LID SEAT OPEN, SEAT ONLY CLOSE) must track the raw .txt
path "very high" — under 5 mm IN EXECUTION, speed may be sacrificed; the bowl inside and inner back
tasks under 10 mm. The budget is a column of the fixture's description.txt (`MAX DEV`), and it is
held in two places:

  BEFORE the run (tools/taught_pipeline.py): a budgeted row is emitted at the FASTEST rung of a speed
    ladder whose PREDICTED executed departure stays within the budget —
        predicted = (the file's own departure from the raw recording, both directions, exact)
                  + (the arm's departure from its command, bounded by ARM_ENVELOPE)
    a sum of two maxima, so an upper bound (slot 104 measured 10.41 against 4.74 + 7.02).
  AFTER the run (every run through cleaning_tracks.run_one — `run` and `flow` — and plot_runs.py):
    the executed pad path, from the recorded UDP push, is
    measured against the raw recording both ways; PASS when the larger maximum is within the budget.

The comparison is the pad (`<L|R>_glove_frame_2`) in the arm base frame, nearest-point both ways
against a 0.25 mm densified polyline: executed -> recording (does the arm stay on the taught path)
and recording -> executed (is every taught point reached). The same numbers plot_runs.py prints as
`exe->taught` / `taught->exe`.

ARM_ENVELOPE — the arm's own departure from a CANFD command, lag-free, as a function of the fastest
joint's speed v (deg/s). Fitted 2026-09-22 on 18 CANFD runs of smooth files — 12 on 2026-09-21
(slots 52 and 53 three runs each at 15:19-15:21 and 15:55-15:57; 56, 58 and 59 two each) and the 6
of slots 101-104 on 2026-09-22 before 16:02 — 1005 half-second windows. Left out: the three
2026-09-21 records of 52/53 that are not CANFD stream records, and 52's 5.5 ms run (15:24). The
window's maximum departure <= min(0.5 + 0.057 v, 7.5) mm on 99.5 % of windows, the worst excess
0.22 mm, and no window above 7.34 mm at any speed. The error does not scale away with speed (a
1.5-2.5 mm floor remains at 25-50 deg/s), which is why a 5 mm budget needs the arm slow AND the file
close to the recording. Re-run the fit: Resource/reports/2026-09-22_tcp_gate_envelope/
fit_arm_envelope.py (it reproduces every figure here).
Rough files (raw drag ripple, uncapped acceleration: slots 51, 54, 55) departed 24-65 mm and are
outside this envelope — the pipeline never emits one.
"""
from __future__ import annotations

import json
import pathlib
import sys

import numpy as np
from scipy.spatial import cKDTree

REPO = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "tools"))
import cut_dossier as cd                                                # noqa: E402
import taught_tracks as tt                                              # noqa: E402

DERIVED = REPO / "Resource/tracks/taught/derived"

#: the arm's own departure from its command on CANFD, mm, against the fastest joint's speed (deg/s)
ARM_ENVELOPE = dict(floor_mm=0.5, mm_per_deg_s=0.057, cap_mm=7.5,
                    fitted_on="18 CANFD runs of smooth files, slots 52/53/56/58/59 (2026-09-21) and "
                              "101-104 (2026-09-22): 1005 half-second windows, 99.5 % covered, "
                              "worst excess 0.22 mm, no window above 7.34 mm")

#: what the gate compares, in words, for the row and the run record
AGAINST = ("the executed pad path (the recorded UDP push) against the RAW taught recording, "
           "nearest point both ways, max")

#: executed frames kept after the last line, as plot_runs.py (TAIL_S)
TAIL_S = 0.3


def arm_envelope_mm(v_max_deg_s: float) -> float:
    e = ARM_ENVELOPE
    return float(min(e["floor_mm"] + e["mm_per_deg_s"] * float(v_max_deg_s), e["cap_mm"]))


def fastest_joint_deg_s(q_deg: np.ndarray, period_s: float) -> float:
    """The fastest any joint moves between two lines of the file, deg/s."""
    return float(np.max(np.abs(np.diff(q_deg, axis=0))) / period_s) if len(q_deg) > 1 else 0.0


def predict(file_vs_raw_mm: float, q_deg: np.ndarray, period_s: float) -> dict:
    v = fastest_joint_deg_s(q_deg, period_s)
    arm = arm_envelope_mm(v)
    return dict(file_vs_raw_mm=round(float(file_vs_raw_mm), 2), v_max_deg_s=round(v, 1),
                arm_envelope_mm=round(arm, 2), predicted_mm=round(float(file_vs_raw_mm) + arm, 2))


def frame_of(side: str) -> str:
    return f"{tt.side_char(side)}_glove_frame_2"


def pad_mm(q_deg: np.ndarray, side: str, frame: str | None = None) -> np.ndarray:
    return cd.fk_tool(np.asarray(q_deg, float), side, frame or frame_of(side))[0] * 1000.0


def densify(P: np.ndarray, step: float = 0.25) -> np.ndarray:
    s = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(P, axis=0), axis=1))])
    keep = np.concatenate([[True], np.diff(s) > 1e-9])
    su = np.arange(0.0, s[-1], step) if s[-1] > 0 else np.array([0.0])
    return np.column_stack([np.interp(su, s[keep], P[keep, k]) for k in range(3)])


def budget_of(row: dict) -> float | None:
    g = (row or {}).get("tcp_gate") or {}
    return float(g["max_mm"]) if g.get("max_mm") is not None else None


def _sha(p: pathlib.Path) -> str:
    return tt.sha256_of(p)


def raw_recording(row: dict):
    """(path, how) of the raw recording this row's track was derived from, else (None, why). The
    chain: the row's source (certify's stored copy of the derived file) -> derived/<stem>.json ->
    its `origin` {path, sha256}; the recording must still have that sha."""
    src = (row.get("source") or {}).get("path")
    stems = []
    if src:
        stems.append(pathlib.Path(str(src)).stem)
    if row.get("track"):
        stems.append(pathlib.Path(str(row["track"])).stem)
    for stem in stems:
        dj = DERIVED / f"{stem}.json"
        if not dj.is_file():
            continue
        try:
            o = json.loads(dj.read_text()).get("origin") or {}
        except Exception:                                              # noqa: BLE001
            continue
        if not o.get("path"):
            continue
        p = REPO / o["path"]
        if not p.is_file():
            return None, f"{o['path']} (the origin of derived/{dj.name}) is missing"
        if o.get("sha256") and _sha(p) != o["sha256"]:
            return None, f"{o['path']} no longer has the sha derived/{dj.name} names"
        return p, f"derived/{dj.name} origin"
    return None, "no derived record names this row's recording"


def executed_joints(frames: pathlib.Path, t0: float, t1: float, tail_s: float = TAIL_S):
    """(inside, with_tail, times): executed joint arrays (deg) and the frame timestamps from a
    `_frames.ndjson`, on the stream's own clock. `inside` is t0..t1 (the stream), `with_tail` runs
    tail_s past the last line, `times` are the timestamps of `with_tail` (the coverage rule)."""
    T, Q = [], []
    with open(frames) as fh:
        for line in fh:
            try:
                fr = json.loads(line)
            except Exception:                                          # noqa: BLE001
                continue
            t = fr.get("t_mono") or fr.get("t")
            jp = ((fr.get("frame") or {}).get("joint_status") or {}).get("joint_position")
            if t is None or not jp or len(jp) < 7 or not (t0 <= float(t) <= t1 + tail_s):
                continue
            T.append(float(t))
            Q.append([x / 1000.0 for x in jp[:7]])
    if not T:
        return np.zeros((0, 7)), np.zeros((0, 7)), np.zeros(0)
    T, Q = np.array(T), np.array(Q)
    order = np.argsort(T, kind="stable")
    T, Q = T[order], Q[order]
    return Q[T <= t1], Q, T


def measure_run(frames: pathlib.Path, t0: float, t1: float, side: str, raw: pathlib.Path,
                frame: str | None = None, period_s: float = 0.005) -> dict:
    """The executed pad path against the raw recording, both ways, mm (max and p95). A capture that
    does not cover the stream is NOT measured: the recording -> executed direction would count every
    taught point after the capture stopped as a departure (plot_runs.py's own coverage rule)."""
    q_in, q_all, T = executed_joints(frames, t0, t1)
    if len(q_in) < 2:
        return dict(ok=False, why="no joint_position frames inside the stream window")
    if T[0] > t0 + period_s or T[-1] < t1 - 1e-6:
        return dict(ok=False, why=f"a partial capture: the frames cover {T[0] - t0:.2f}..{T[-1] - t0:.2f} s "
                                  f"of the {t1 - t0:.2f} s stream")
    P_raw = pad_mm(tt.read_q_deg(raw), side, frame)
    P_in, P_all = pad_mm(q_in, side, frame), pad_mm(q_all, side, frame)
    fwd = cKDTree(densify(P_raw)).query(P_in)[0]                       # executed -> recording
    back = cKDTree(densify(P_all)).query(P_raw)[0]                     # recording -> executed
    return dict(ok=True, exe_to_raw_max_mm=round(float(fwd.max()), 2),
                exe_to_raw_p95_mm=round(float(np.percentile(fwd, 95)), 2),
                raw_to_exe_max_mm=round(float(back.max()), 2),
                raw_to_exe_p95_mm=round(float(np.percentile(back, 95)), 2),
                measured_mm=round(float(max(fwd.max(), back.max())), 2), frames=int(len(q_all)))


def verdict(budget_mm: float, measured: dict) -> dict:
    if not measured.get("ok"):
        return dict(max_mm=budget_mm, verdict="NOT MEASURED", why=measured.get("why"))
    return dict(max_mm=budget_mm, verdict="PASS" if measured["measured_mm"] <= budget_mm else "FAIL",
                **{k: v for k, v in measured.items() if k != "ok"})


def gate_run(row: dict, record: dict, frames: pathlib.Path) -> dict | None:
    """The gate for one CANFD run of *row* (None when the row has no budget). Used by `flow` right
    after the run, so the verdict is in the run record and on the console."""
    b = budget_of(row)
    if b is None:
        return None
    cs = record.get("canfd") or {}
    if cs.get("t_first_send") is None:
        return dict(max_mm=b, verdict="NOT MEASURED", why="no CANFD stream in this run")
    raw, how = raw_recording(row)
    if raw is None:
        return dict(max_mm=b, verdict="NOT MEASURED", why=how)
    t0 = float(cs["t_first_send"])
    t1 = float(cs.get("t_last_line") or cs.get("t_end") or t0)
    m = measure_run(frames, t0, t1, row["side"], raw, period_s=float(cs.get("period_s") or 0.005))
    if bool(cs.get("aborted")) or cs.get("completed") is False:
        # A STREAM THAT STOPPED never commanded the rest of the taught path, so recording -> executed
        # is no accuracy measure. Only the executed part can fail it: executed -> recording over the
        # budget is a real departure; otherwise the run says nothing about accuracy.
        why = (f"the stream stopped at line {cs.get('lines_sent')} of {cs.get('lines')}: "
               f"{cs.get('aborted') or 'not every line was sent'}")
        if m.get("ok") and m["exe_to_raw_max_mm"] > b:
            out = verdict(b, dict(m, measured_mm=m["exe_to_raw_max_mm"]))
            out["why"] = why + " — judged executed -> raw only"
        else:
            out = dict(max_mm=b, verdict="NOT MEASURED", why=why)
    else:
        out = verdict(b, m)
    try:
        raw_rel = str(raw.resolve().relative_to(REPO))
    except ValueError:
        raw_rel = str(raw)
    out.update(against=AGAINST, raw=raw_rel, raw_how=how)
    return out
