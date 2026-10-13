"""Motion geometry of a taught path — the part a cleaning-task author reads.

Given the smoothed TCP path (cartesian.path_metrics arrays) this answers:

    plane     which plane the hand worked in (PCA of the moving samples), how
              flat the motion was (out-of-plane RMS / p95) and the in-plane
              extent it covered;
    strokes   maximal runs of moving samples with curvature below
              `kappa_stroke` (radius above 1/kappa_stroke, 0.2 m by default),
              i.e. the wiping passes between turns: length, chord,
              straightness = chord/arc, bow (max deviation from the chord),
              mean/peak speed, in-plane direction;
    turns     what lies between strokes: duration, arc length, speed carried;
    pitch     perpendicular distance between consecutive (anti)parallel
              strokes — the raster spacing the hand chose;
    direction the dominant stroke orientation (length-weighted, mod 180 deg).

Everything is period-free except speeds and durations. Coordinates: `u, v`
are in-plane (metres, along the first two principal axes), `w` is along the
plane normal. Nothing here decides what a "good" geometry is; it measures
the one the hand produced so a generator can be compared to it.
"""
from __future__ import annotations

from typing import Optional

import numpy as np


def plane_fit(pos: np.ndarray, mask: Optional[np.ndarray] = None) -> dict:
    """Least-squares plane through the (masked) samples by SVD; extent and
    out-of-plane statistics are those of the fitted samples. The normal's
    sign is fixed so its largest component is positive (deterministic)."""
    P = np.asarray(pos, dtype=float)
    if mask is not None and mask.sum() >= 3:
        P = P[mask]
    c = P.mean(axis=0)
    _, S, Vt = np.linalg.svd(P - c, full_matrices=False)
    e1, e2, n = Vt[0], Vt[1], Vt[2]
    if n[int(np.argmax(np.abs(n)))] < 0:
        n = -n; e2 = -e2                        # keep (e1, e2, n) right-handed
    var = S ** 2 / max(P.shape[0] - 1, 1)
    d = P - c
    u, v, w = d @ e1, d @ e2, d @ n
    return {"centroid_m": c.tolist(), "e1": e1.tolist(), "e2": e2.tolist(), "normal": n.tolist(),
            "explained_variance": (var / max(var.sum(), 1e-300)).tolist(),
            "out_of_plane_rms_m": float(np.sqrt((w ** 2).mean())),
            "out_of_plane_p95_m": float(np.percentile(np.abs(w), 95)),
            "out_of_plane_max_m": float(np.abs(w).max()),
            "extent_u_m": float(u.max() - u.min()), "extent_v_m": float(v.max() - v.min()),
            "bbox_area_m2": float((u.max() - u.min()) * (v.max() - v.min())),
            "planarity": float(np.sqrt((w ** 2).mean()) / max(u.max() - u.min(), v.max() - v.min(), 1e-9)),
            "normal_tilt_from_base_z_deg": float(np.degrees(np.arccos(np.clip(abs(n[2]), 0, 1)))),
            "_frame": (c, e1, e2, n)}


def in_plane(pos: np.ndarray, plane: dict) -> np.ndarray:
    c, e1, e2, n = plane["_frame"]
    d = np.asarray(pos, dtype=float) - c
    return np.stack([d @ e1, d @ e2, d @ n], axis=1)


def _runs(mask: np.ndarray):
    """(start, end) index pairs of True runs, end exclusive."""
    m = np.asarray(mask, dtype=bool)
    if m.size == 0:
        return []
    edges = np.diff(m.astype(int))
    starts = list(np.where(edges == 1)[0] + 1); ends = list(np.where(edges == -1)[0] + 1)
    if m[0]:
        starts = [0] + starts
    if m[-1]:
        ends = ends + [m.size]
    return [(int(a), int(b)) for a, b in zip(starts, ends) if b > a]


def _angle_mod180(deg: float) -> float:
    return float(((deg + 90.0) % 180.0) - 90.0)


def strokes(metrics: dict, uvw: np.ndarray, pos3: np.ndarray, *, kappa_stroke_1_per_m: float = 5.0,
            min_length_m: float = 0.02, min_duration_s: float = 0.15, straight_min: float = 0.9,
            parallel_tol_deg: float = 20.0) -> dict:
    """Segment the moving path into strokes and turns; measure pitch.
    `uvw` are in-plane coordinates (direction, pitch), `pos3` the 3-D
    positions (chord, bow, straightness)."""
    A = metrics["_arrays"]; fs = metrics["fs_hz"]; dt = 1.0 / fs
    kap = np.nan_to_num(A["kappa"], nan=np.inf)
    low = A["moving"] & (kap < kappa_stroke_1_per_m)
    s = A["s"]; speed = A["speed"]; t = A["t"]
    out_strokes = []
    for a, b in _runs(low):
        arc = float(s[b - 1] - s[a]); dur = (b - 1 - a) * dt
        if arc < min_length_m or dur < min_duration_s:
            continue
        # chord and bow in 3-D (a stroke that runs out of the fitted plane is
        # still straight); direction and pitch in the plane
        q0, q1 = pos3[a], pos3[b - 1]
        cvec3 = q1 - q0; chord = float(np.linalg.norm(cvec3))
        if chord > 1e-9:
            d3 = cvec3 / chord
            rel = pos3[a:b] - q0
            bow = float(np.linalg.norm(rel - np.outer(rel @ d3, d3), axis=1).max())
        else:
            bow = float("nan")
        p0, p1 = uvw[a, :2], uvw[b - 1, :2]
        chord_vec = p1 - p0
        ang = float(np.degrees(np.arctan2(chord_vec[1], chord_vec[0]))) if np.linalg.norm(chord_vec) > 1e-9 else float("nan")
        out_strokes.append({
            "i0": a, "i1": b - 1, "t0_s": float(t[a]), "t1_s": float(t[b - 1]), "duration_s": float(dur),
            "arc_length_m": arc, "chord_m": chord, "straightness": float(chord / arc) if arc > 0 else None,
            "bow_m": bow, "direction_deg": ang, "orientation_deg": _angle_mod180(ang) if np.isfinite(ang) else None,
            "mean_speed_mps": float(arc / dur) if dur > 0 else None,
            "peak_speed_mps": float(speed[a:b].max()),
            "start_uv_m": [float(p0[0]), float(p0[1])], "end_uv_m": [float(p1[0]), float(p1[1])],
            "mid_uv_m": [float(x) for x in uvw[a:b, :2].mean(axis=0)],
            "mean_w_m": float(uvw[a:b, 2].mean()),
        })
    # turns: the moving samples between consecutive strokes
    turns = []
    for k in range(len(out_strokes) - 1):
        a = out_strokes[k]["i1"] + 1; b = out_strokes[k + 1]["i0"]
        if b <= a:
            continue
        mv = A["moving"][a:b]
        turns.append({"t0_s": float(t[a]), "t1_s": float(t[b - 1]), "duration_s": float((b - a) * dt),
                      "arc_length_m": float(s[b - 1] - s[a]), "min_speed_mps": float(speed[a:b].min()),
                      "stopped": bool((~mv).any()),
                      "heading_change_deg": _heading_change(out_strokes[k], out_strokes[k + 1])})
    # pitch between consecutive near-(anti)parallel straight strokes
    pitches = []
    for k in range(len(out_strokes) - 1):
        s0, s1 = out_strokes[k], out_strokes[k + 1]
        if (s0["straightness"] or 0) < straight_min or (s1["straightness"] or 0) < straight_min:
            continue
        if s0["orientation_deg"] is None or s1["orientation_deg"] is None:
            continue
        if abs(_angle_mod180(s0["orientation_deg"] - s1["orientation_deg"])) > parallel_tol_deg:
            continue
        p0 = np.array(s0["start_uv_m"]); d = np.array(s0["end_uv_m"]) - p0
        if np.linalg.norm(d) < 1e-9:
            continue
        nrm = np.array([-d[1], d[0]]) / np.linalg.norm(d)
        pitches.append(float(abs((np.array(s1["mid_uv_m"]) - p0) @ nrm)))
    lengths = [x["arc_length_m"] for x in out_strokes]
    straight = [x["straightness"] for x in out_strokes if x["straightness"] is not None]
    speeds = [x["mean_speed_mps"] for x in out_strokes if x["mean_speed_mps"] is not None]
    bows = [x["bow_m"] for x in out_strokes if np.isfinite(x["bow_m"])]
    # dominant orientation: length-weighted circular mean on the doubled angle
    ori = np.array([x["orientation_deg"] for x in out_strokes if x["orientation_deg"] is not None])
    wts = np.array([x["arc_length_m"] for x in out_strokes if x["orientation_deg"] is not None])
    if ori.size:
        z = (wts * np.exp(1j * np.radians(2 * ori))).sum() / max(wts.sum(), 1e-12)
        dominant = _angle_mod180(np.degrees(np.angle(z)) / 2.0); alignment = float(abs(z))
    else:
        dominant, alignment = None, None
    return {
        "kappa_stroke_1_per_m": kappa_stroke_1_per_m, "min_length_m": min_length_m,
        "count": len(out_strokes),
        "length_m": _stats(lengths), "straightness": _stats(straight), "bow_m": _stats(bows),
        "mean_speed_mps": _stats(speeds),
        "total_stroke_length_m": float(sum(lengths)),
        "stroke_fraction_of_path": float(sum(lengths) / max(metrics["path_length_m"], 1e-12)),
        "dominant_orientation_deg": dominant, "orientation_alignment": alignment,
        "pitch_m": _stats(pitches), "pitch_count": len(pitches),
        "turns": {"count": len(turns), "duration_s": _stats([x["duration_s"] for x in turns]),
                  "stopped_fraction": float(np.mean([x["stopped"] for x in turns])) if turns else None,
                  "list": turns},
        "list": out_strokes,
    }


def _heading_change(s0: dict, s1: dict) -> Optional[float]:
    if s0["direction_deg"] is None or s1["direction_deg"] is None:
        return None
    d = (s1["direction_deg"] - s0["direction_deg"] + 180.0) % 360.0 - 180.0
    return float(abs(d))


def _stats(x) -> dict:
    x = np.asarray([v for v in x if v is not None and np.isfinite(v)], dtype=float)
    if x.size == 0:
        return {"n": 0, "median": None, "mean": None, "min": None, "max": None, "p90": None}
    return {"n": int(x.size), "median": float(np.median(x)), "mean": float(x.mean()),
            "min": float(x.min()), "max": float(x.max()), "p90": float(np.percentile(x, 90))}


TRANSIT_FACTOR = 2.0     # first/last stroke longer than this x the median is a transit move


def transit_strokes(stroke_list: list, factor: float = TRANSIT_FACTOR) -> list:
    """Indices of the first/last stroke when they are transit moves (an
    approach from the start pose or a return to it): much longer than the
    median stroke. They are real motion but not the wiping pattern, and they
    would dominate a plane fit."""
    if len(stroke_list) < 3:
        return []
    med = float(np.median([x["arc_length_m"] for x in stroke_list]))
    out = []
    if stroke_list[0]["arc_length_m"] > factor * med:
        out.append(0)
    if stroke_list[-1]["arc_length_m"] > factor * med:
        out.append(len(stroke_list) - 1)
    return out


def summarize(metrics: dict, pos: np.ndarray, **kw) -> dict:
    """plane + strokes + pitch for one analysed path.

    Two passes: strokes are segmented from curvature and speed alone (plane-
    free); the work plane is then fitted on the samples of the non-transit
    strokes (falling back to all moving samples when there are none), and the
    in-plane quantities (direction, pitch) are read in that plane. Keeps the
    in-plane coordinates under `_uvw` for plots (stripped from YAML)."""
    A = metrics["_arrays"]
    p = np.asarray(pos, dtype=float)
    moving_mask = A["moving"] if A["moving"].sum() >= 3 else None
    plane0 = plane_fit(p, moving_mask)
    st0 = strokes(metrics, in_plane(p, plane0), p, **kw)
    transit = transit_strokes(st0["list"])
    # the work region: every moving sample except the transit strokes and the
    # approach/return motion beyond them (turns between strokes stay in)
    core = A["moving"].copy()
    lst = st0["list"]
    if 0 in transit:
        core[:lst[1]["i0"]] = False
    if lst and (len(lst) - 1) in transit:
        core[lst[-2]["i1"] + 1:] = False
    plane = plane_fit(p, core) if core.sum() >= 3 else plane0
    uvw = in_plane(p, plane)
    st = strokes(metrics, uvw, p, **kw)
    st["transit_strokes"] = [k + 1 for k in transit_strokes(st["list"])]
    plane_out = {k: v for k, v in plane.items() if not k.startswith("_")}
    plane_out["fitted_on"] = (f"moving samples between the transit strokes ({int(core.sum())} samples)" if transit and core.sum() >= 3
                              else f"all moving samples ({int(core.sum())})")
    w_all = (p - np.asarray(plane["centroid_m"])) @ np.asarray(plane["normal"])
    plane_out["out_of_plane_rms_all_moving_m"] = float(np.sqrt((w_all[A["moving"]] ** 2).mean())) if A["moving"].any() else None
    return {"plane": plane_out, "strokes": st, "_uvw": uvw}
