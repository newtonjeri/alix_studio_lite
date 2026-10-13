#!/usr/bin/env python3
"""Why did the controller end a track program early?  One dossier per run.

F111 left the cut unexplained: several strokes stop at 50-97 % of their file
with err [], no joint error flag, no collision, and a per-track reproducible
stop position.  This tool puts every candidate gate on the same table for the
run's OWN commanded file, measured at the stop index and in the 200 points the
arm never reached:

  * position margin per joint (URDF limits; J4's +-135 aborts right_test, F98)
  * commanded joint speed / acceleration / jerk vs the controller's own
    readback (180/180/225x5 deg/s, 600 deg/s^2)
  * TCP linear + angular speed and acceleration at the ik_frame
  * executed-vs-commanded following error from the capture
  * where each of those peaks over the WHOLE file, so a stop that coincides
    with a peak is separable from one that does not

Reads only the ledger + the tracks (git history included).  No hardware.

    python3 tools/cut_dossier.py                    # every run with a capture
    python3 tools/cut_dossier.py --slots 72 73      # one task
    python3 tools/cut_dossier.py --json out.json    # machine-readable
"""
from __future__ import annotations

import argparse
import json
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import track_report as tr                                              # noqa: E402
import track_report_pdf as trp                                         # noqa: E402

REPO = tr.REPO
CLOCK = tr.CLOCK_S
RATED_V = np.array([180, 180, 225, 225, 225, 225, 225], float)          # deg/s
RATED_A = np.full(7, 600.0)                                             # deg/s^2
LOOKAHEAD = 200          # points past the stop the controller could have seen
WINDOW = 40              # points either side of the stop for local peaks


SIDE_PREFIX = {"right": "R", "left": "L"}


def urdf_position_limits(side: str = "right") -> np.ndarray:
    """(7,2) deg. From the expanded description — the only place they live."""
    import xml.etree.ElementTree as ET
    root = ET.parse(_urdf()).getroot()
    lim = {}
    for j in root.findall("joint"):
        n, e = j.get("name"), j.find("limit")
        if n and e is not None:
            lim[n] = (float(e.get("lower")), float(e.get("upper")))
    out = []
    for i in range(7):
        n = f"{SIDE_PREFIX[side]}_joint{i + 1}"
        if n not in lim:
            raise SystemExit(f"no limit for {n} in the expanded URDF")
        out.append([np.degrees(lim[n][0]), np.degrees(lim[n][1])])
    return np.asarray(out, float)


def _urdf() -> pathlib.Path:
    import importlib
    pm = importlib.import_module("plan_min_time")
    return pm.URDF


_FK_CACHE = {}


_MANIFEST_BY_NAME = None


def manifest_rows() -> dict:
    """The commode_c manifest keyed BY REGION NAME."""
    global _MANIFEST_BY_NAME
    if _MANIFEST_BY_NAME is None:
        tasks = json.loads(
            (REPO / "Resource/tracks/commode_c/MANIFEST.json").read_text())["tasks"]
        _MANIFEST_BY_NAME = {t["name"]: t for t in tasks if t.get("name")}
    return _MANIFEST_BY_NAME


def manifest_row(rec: dict):
    """The manifest row a RUN RECORD belongs to — by NAME, never by slot.

    **A slot is not a task.** Slots are allocated "+1 per task in schedule
    order", so inserting a task renumbers every slot after it: the three
    hinge-area regions joined mid-schedule on 2026-09-04 at 15:59 and shifted
    everything from 72 up. A record written before that carries the OLD slot,
    and reading today's manifest by slot hands back a different region — with a
    different `ik_frame`, so its whole Cartesian reconstruction is rigidly
    displaced (41.6 mm frame_1<->2, 58.1 mm frame_2<->4, 62.1 mm frame_2<->3,
    which is 10-15 % of a raster's extent). Measured over the ledger: 35 of 116
    records resolved to the wrong region and 21 to the wrong frame; all 116
    resolve correctly by name.

    The slot fallback exists only for a record with no name, and only when it
    is unambiguous — a slot that names two regions returns None rather than
    guessing.
    """
    name = (rec or {}).get("name")
    rows = manifest_rows()
    if name and name in rows:
        return rows[name]
    slot = (rec or {}).get("slot")
    if slot is None:
        return None
    hits = [t for t in rows.values() if t.get("slot") == slot]
    return hits[0] if len(hits) == 1 else None


def fk_tool(q_deg: np.ndarray, side: str, frame: str):
    """TCP pose per row.  Returns (N,3) position and (N,3,3) rotation, or None."""
    try:
        sys.path.insert(0, str(REPO / "core/python/alix_workspace/src"))
        from alix_workspace.urdf_kinematics import UrdfModel
    except Exception:                                                  # noqa: BLE001
        return None, None
    key = (side, frame)
    if key not in _FK_CACHE:
        model = UrdfModel.from_file(str(_urdf()))
        base = f"{SIDE_PREFIX[side]}_base_link"
        _FK_CACHE[key] = (model, model.chain(base, frame))
    model, chain = _FK_CACHE[key]
    if len(q_deg) == 0:
        return np.empty((0, 3)), np.empty((0, 3, 3))
    # batched: the same products as model.fk row by row, bit-identical, ~17x faster
    T = model.fk_batch(chain, np.radians(np.asarray(q_deg, dtype=float)))
    return T[:, :3, 3].copy(), T[:, :3, :3].copy()


def derivs(q: np.ndarray, clock: float = CLOCK):
    v = np.diff(q, axis=0) / clock
    a = np.diff(v, axis=0) / clock
    j = np.diff(a, axis=0) / clock
    return v, a, j


def ang_rates(R, clock: float = CLOCK):
    """Angular speed (deg/s) and acceleration (deg/s^2) of a rotation sequence."""
    if R is None:
        return None, None
    w = []
    for k in range(len(R) - 1):
        dR = R[k].T @ R[k + 1]
        c = np.clip((np.trace(dR) - 1) / 2, -1, 1)
        w.append(np.degrees(np.arccos(c)) / clock)
    w = np.asarray(w)
    return w, np.diff(w) / clock


def dossier(rec_path: pathlib.Path, cache: pathlib.Path, limits: np.ndarray) -> dict | None:
    a = trp.analyse(rec_path, cache)
    T = a.get("track")
    if T is None or a.get("end_phase") is None:
        return None
    n = len(T)
    k = int(round(a["end_phase"] * (n - 1)))
    k = max(1, min(n - 1, k))
    v, acc, jrk = derivs(T)
    lo_m = T - limits[:, 0]
    hi_m = limits[:, 1] - T
    margin = np.minimum(lo_m, hi_m)                                    # (N,7) deg

    def at(arr, idx, default=None):
        idx = min(max(idx, 0), len(arr) - 1)
        return arr[idx] if len(arr) else default

    ahead = slice(k, min(n - 1, k + LOOKAHEAD))
    near = slice(max(0, k - WINDOW), min(n - 1, k + WINDOW))
    rec = a["rec"]
    d = dict(
        run=rec_path.name, when=a["when"], slot=a["slot"], name=a["name"],
        acc_scale=rec.get("acc_scale"), points=n, outcome=a["outcome"],
        cover=a.get("cover"), end_phase=a.get("end_phase"), stop_idx=k,
        replay_s=a.get("replay_s"), pace_ms=a.get("pace_ms"),
        err_codes=(rec.get("controller_after", {}).get("arm_state", {}) or {}).get("err_codes"),
        q_stop=[round(float(x), 2) for x in T[k]],
        margin_stop=[round(float(x), 2) for x in margin[k]],
        margin_min_file=[round(float(x), 2) for x in margin.min(0)],
        margin_min_idx=[int(x) for x in margin.argmin(0)],
        margin_min_ahead=[round(float(x), 2) for x in margin[ahead].min(0)] if ahead.stop > ahead.start else None,
        v_stop=[round(float(x), 1) for x in at(v, k, np.zeros(7))],
        v_max_near=[round(float(x), 1) for x in np.abs(v[near]).max(0)] if near.stop > near.start else None,
        v_max_ahead=[round(float(x), 1) for x in np.abs(v[ahead]).max(0)] if ahead.stop > ahead.start else None,
        v_max_file=[round(float(x), 1) for x in np.abs(v).max(0)],
        a_max_near=[round(float(x), 0) for x in np.abs(acc[near]).max(0)] if near.stop > near.start else None,
        a_max_ahead=[round(float(x), 0) for x in np.abs(acc[ahead]).max(0)] if ahead.stop > ahead.start else None,
        a_max_file=[round(float(x), 0) for x in np.abs(acc).max(0)],
        v_util_ahead=None, a_util_ahead=None,
    )
    if d["v_max_ahead"]:
        d["v_util_ahead"] = round(float(np.max(np.asarray(d["v_max_ahead"]) / RATED_V)), 3)
        d["a_util_ahead"] = round(float(np.max(np.asarray(d["a_max_ahead"]) / RATED_A)), 3)
    d["v_util_file"] = round(float(np.max(np.abs(v).max(0) / RATED_V)), 3)
    d["a_util_file"] = round(float(np.max(np.abs(acc).max(0) / RATED_A)), 3)
    # where the file's own worst joint-speed and worst margin sit, as a phase %
    d["v_peak_phase"] = round(float(np.abs(v / RATED_V).max(1).argmax()) / (n - 1) * 100, 1)
    d["a_peak_phase"] = round(float(np.abs(acc / RATED_A).max(1).argmax()) / (n - 1) * 100, 1)
    d["margin_min_phase"] = round(float(margin.min(1).argmin()) / (n - 1) * 100, 1)
    d["margin_min_joint"] = int(margin.min(0).argmin()) + 1
    d["margin_min_deg"] = round(float(margin.min()), 2)
    # executed side
    if a.get("dev") is not None and len(a["dev"]):
        d["dev_p99"] = round(float(np.percentile(a["dev"], 99)), 2)
        d["dev_at_stop"] = round(float(np.max(a["dev"][-5:])), 2)
    if a.get("jmax") is not None:
        d["exec_jmax"] = [round(float(x), 1) for x in a["jmax"]]
        d["exec_v_util"] = round(float(np.max(a["jmax"] / RATED_V)), 3)
    # TCP
    lin = lacc = w = wacc = None
    row = manifest_row(rec)
    frame = row.get("ik_frame") if row else None
    d["ik_frame"] = frame
    if frame:
        P, R = fk_tool(T, rec.get("side", "right"), frame)
        if P is not None:
            lin = np.linalg.norm(np.diff(P, axis=0), axis=1) / CLOCK
            lacc = np.diff(lin) / CLOCK
            w, wacc = ang_rates(R)
            d["tcp_lin_max_file"] = round(float(lin.max()), 3)
            d["tcp_lin_at_stop"] = round(float(at(lin, k, 0.0)), 3)
            d["tcp_lin_max_ahead"] = round(float(lin[ahead].max()), 3) if ahead.stop > ahead.start else None
            d["tcp_lacc_max_file"] = round(float(np.abs(lacc).max()), 2)
            d["tcp_lacc_max_ahead"] = round(float(np.abs(lacc[ahead]).max()), 2) if ahead.stop > ahead.start + 1 else None
            if w is not None:
                d["tcp_w_max_file"] = round(float(np.degrees(np.radians(w)).max()), 1)
                d["tcp_w_at_stop"] = round(float(at(w, k, 0.0)), 1)
                d["tcp_w_max_ahead"] = round(float(w[ahead].max()), 1) if ahead.stop > ahead.start else None
                d["tcp_wacc_max_ahead"] = round(float(np.abs(wacc[ahead]).max()), 1) if ahead.stop > ahead.start + 1 else None
    # ---- within-track control -------------------------------------------
    # The arm traversed most of THIS file successfully, so the sharpest test is
    # not "cut vs complete track" but "is the stop neighbourhood special inside
    # its own file?".  For each candidate gate: the percentile the stop window
    # sits at in that file's own distribution, and how far the file's worst
    # point is from the stop.  A gate that fires at the stop should rank ~100.
    prof = {}
    prof["v_util"] = np.abs(v / RATED_V).max(1)
    prof["a_util"] = np.abs(acc / RATED_A).max(1)
    prof["jerk"] = np.abs(jrk).max(1)
    prof["neg_margin"] = -margin.min(1)
    if d.get("tcp_lin_max_file") is not None:
        prof["tcp_lin"] = lin
        prof["tcp_lacc"] = np.abs(lacc)
        if w is not None:
            prof["tcp_w"] = w
            prof["tcp_wacc"] = np.abs(wacc)
    ranks = {}
    for key, arr in prof.items():
        m = len(arr)
        kk = min(max(int(k * m / n), 0), m - 1)
        lo, hi = max(0, kk - WINDOW), min(m, kk + WINDOW)
        local = float(arr[lo:hi].max()) if hi > lo else float(arr[kk])
        ranks[key] = dict(
            local=round(local, 3),
            pct=round(float((arr < local).mean() * 100), 1),
            file_max=round(float(arr.max()), 3),
            argmax_phase=round(float(arr.argmax()) / (m - 1) * 100, 1),
            ratio=round(local / float(arr.max()), 3) if arr.max() else None,
        )
    d["ranks"] = ranks
    d["stop_phase"] = round(k / (n - 1) * 100, 1)
    return d


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--slots", nargs="*", type=int)
    p.add_argument("--day", nargs="*")
    p.add_argument("--json")
    p.add_argument("--cache", default=str(REPO / ".track_cache"))
    a = p.parse_args(argv)
    recs = sorted((REPO / "Resource/runs/tracks").glob("*/run_slot*.json"))
    recs = [r for r in recs if r.with_name(r.stem + "_frames.ndjson").is_file()]
    if a.slots:
        recs = [r for r in recs if json.loads(r.read_text()).get("slot") in a.slots]
    if a.day:
        recs = [r for r in recs if r.parent.name in a.day]
    limits = urdf_position_limits("right")
    print("URDF position limits (deg): " +
          " ".join(f"J{i+1} [{limits[i,0]:.0f},{limits[i,1]:.0f}]" for i in range(7)))
    out = []
    for r in recs:
        try:
            d = dossier(r, pathlib.Path(a.cache), limits)
        except Exception as e:                                          # noqa: BLE001
            print(f"  {r.name}: {type(e).__name__}: {e}")
            continue
        if d:
            out.append(d)
    hdr = (f"{'when':<17}{'slot':<5}{'task':<20}{'acc':<5}{'out':<9}{'stop%':<7}"
           f"{'minMargin(J)':<15}{'@phase':<8}{'vUtil>':<8}{'aUtil>':<8}{'lin>':<7}{'w>':<7}")
    print("\n" + hdr)
    print("-" * len(hdr))
    for d in out:
        mm = f"{d['margin_min_deg']:.1f} (J{d['margin_min_joint']})"
        print(f"{d['when'][5:16]:<17}{d['slot']:<5}{d['name'][:19]:<20}"
              f"{str(d['acc_scale']):<5}{d['outcome']:<9}"
              f"{(d['end_phase'] or 0) * 100:<7.1f}{mm:<15}{d['margin_min_phase']:<8.1f}"
              f"{str(d.get('v_util_ahead')):<8}{str(d.get('a_util_ahead')):<8}"
              f"{str(d.get('tcp_lin_max_ahead')):<7}{str(d.get('tcp_w_max_ahead')):<7}")
    print("\nstop% = fraction of the commanded file the arm actually traversed.  "
          "minMargin = closest any joint gets to a POSITION limit anywhere in the file, "
          "with the phase it happens at.  vUtil>/aUtil>/lin>/w> are the worst values in "
          f"the {LOOKAHEAD} commanded points AFTER the stop — what the controller would "
          "have had to execute next.")
    if a.json:
        pathlib.Path(a.json).write_text(json.dumps(out, indent=1))
        print(f"\nwrote {a.json}")
    return out


if __name__ == "__main__":
    main()
