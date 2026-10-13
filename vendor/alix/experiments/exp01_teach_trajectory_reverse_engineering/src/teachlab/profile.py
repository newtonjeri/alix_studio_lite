"""The constraint profile — what a Cartesian path generator may consume.

Schema `exp01.constraint_profile/1`, deliberately small:

    schema, generated_by, source{file, sha256, samples, period_s,
        period_status, side, units, columns, signs}
    smoothing{method, ...}                      exactly what produced the reference
    joint{qdot_max_deg_s, peak_vel_deg_s, peak_acc_deg_s2, peak_jerk_deg_s3,
        utilization, utilization_basis, binding_joint, position_f99_hz,
        spectrum_window}
    cartesian{frame, base_link, path_length_m, mean_speed_full_mps,
        mean_speed_active_span_mps, speed{...}, speed_cv_moving, kappa{...},
        kappa_at_speed{...}, a_lat{...}, a_lat_at_speed{...}, a_tan{...},
        omega{...}, power_law{beta, c, r2, n, null_control{...}},
        corners{count, min_blend_radius_m, ..., turnarounds{count}, list[...]}}
    bounds_for_generator{v_max_mps, a_tan_max_mps2, a_lat_max_mps2,
        kappa_max_1_per_m, min_blend_radius_m, omega_max_deg_s, basis,
        stroke_pitch_m, stroke_length_median_m, work_plane_normal, out_of_plane_p95_m}
    geometry{plane{...}, strokes{...}, turns{...}, stroke_list[...], turn_list[...]}
        (teachlab.geometry — period-free; present when the CLI ran it)

`bounds_for_generator` is the coarse-scale envelope read AT SPEED (samples
>= v_fast, default 0.10 m/s) for curvature, radius and lateral acceleration —
the slow band near stroke ends is FK/derivative wobble against a stop, not a
measured fillet — and p99 of moving samples for speed, tangential
acceleration and orientation rate, with the observed max beside each.
Consumers should read `source.period_status`: while it is `assumed`, every
speed/acceleration scales with the true period (v ~ 1/T, a ~ 1/T^2,
jerk ~ 1/T^3) and only curvature, radii and path length are period-free.
The power-law beta is NOT a bound: any smooth path gives ~0.3 (see
cartesian.power_law_null); it is reported with its null band only.
"""
from __future__ import annotations

import datetime as _dt
import pathlib
from typing import Any, Optional

import yaml

from teachlab import __version__

SCHEMA = "exp01.constraint_profile/1"


def _strip(d: Any) -> Any:
    """Drop private array entries and convert numpy scalars for YAML."""
    import numpy as np
    if isinstance(d, dict):
        return {k: _strip(v) for k, v in d.items() if not str(k).startswith("_")}
    if isinstance(d, (list, tuple)):
        return [_strip(v) for v in d]
    if isinstance(d, (np.floating,)):
        return float(d)
    if isinstance(d, (np.integer,)):
        return int(d)
    if isinstance(d, np.ndarray):
        return d.tolist()
    return d


def build_profile(tf, smoothing_meta: dict, joint: dict, cart: dict,
                  geometry: Optional[dict] = None, interpretation: Optional[dict] = None) -> dict:
    """`geometry` (teachlab.geometry.summarize) and `interpretation`
    (teachfile.interpretation_for) are optional, additive sections."""
    u = joint["utilization"]
    sp = cart["speed_moving"]; kap = cart["kappa_1_per_m"]; kaps = cart["kappa_at_speed_1_per_m"]
    al = cart["a_lat_mps2"]; als = cart["a_lat_at_speed_mps2"]
    at = cart["a_tan_mps2"]; om = cart["omega_deg_s"]; co = cart["corners"]
    period_free = ["path_length_m", "kappa", "corners.radius_m", "bbox_m"]
    prof = {
        "schema": SCHEMA,
        "generated_by": f"exp01 teachlab {__version__} on {_dt.date.today().isoformat()}",
        "source": {
            "file": tf.path.name, "sha256": tf.sha256, "samples": tf.n,
            "period_s": tf.period_s, "period_status": tf.period_status,
            "side": tf.side, "units": tf.units, "columns": list(tf.columns),
            "signs": list(tf.signs), "duration_s": tf.duration_s,
            "problems": list(tf.problems),
            "period_scaling": ("while period_status is 'assumed': speed ~ 1/T, acceleration ~ 1/T^2, jerk ~ 1/T^3 "
                               "with the true record period T; path length, curvature, radii, plane, strokes and "
                               "pitch are period-free"),
            "units_source": getattr(tf, "units_source", "given"),
            "in_manifest": None if interpretation is None else bool(interpretation.get("in_manifest")),
            "provenance": {} if interpretation is None else dict(interpretation.get("provenance") or {}),
        },
        "smoothing": dict(smoothing_meta),
        "joint": {
            "qdot_max_deg_s": u["qdot_max_deg_s"],
            "peak_vel_deg_s": joint["smoothed"]["peak_vel_deg_s"],
            "peak_acc_deg_s2": joint["smoothed"]["peak_acc_deg_s2"],
            "peak_jerk_deg_s3": joint["smoothed"]["peak_jerk_deg_s3"],
            "utilization": u["utilization"], "max_utilization": u["max_utilization"],
            "utilization_basis": joint.get("utilization_basis"),
            "binding_joint": u["binding_joint"],
            "position_f99_hz": joint["position_f99"]["per_joint_hz"],
            "position_f99_pooled_hz": joint["position_f99"]["pooled_hz"],
            "spectrum_window": joint.get("spectrum_window"),
            "raw_peak_jerk_deg_s3_for_reference": joint["raw"]["peak_jerk_deg_s3"],
        },
        "cartesian": {
            "frame": cart["frame"], "base_link": cart["base_link"],
            "path_length_m": cart["path_length_m"],
            "duration_s": cart["duration_s"],
            "mean_speed_full_mps": cart["mean_speed_full_mps"],
            "mean_speed_active_span_mps": cart["mean_speed_active_span_mps"],
            "active_span_s": cart["active_span_s"],
            "moving_fraction": cart["moving_fraction"], "fast_fraction": cart["fast_fraction"],
            "v_moving_threshold_mps": cart["v_moving_threshold_mps"],
            "v_fast_threshold_mps": cart["v_fast_threshold_mps"],
            "speed_mps": sp, "speed_cv_moving": cart["speed_cv_moving"],
            "kappa_1_per_m": kap, "kappa_at_speed_1_per_m": kaps,
            "a_lat_mps2": al, "a_lat_at_speed_mps2": als,
            "a_tan_mps2": at, "omega_deg_s": om,
            "power_law": cart["power_law"],
            "corners": co,
            "bbox_m": cart["bbox_m"],
        },
        "bounds_for_generator": {
            "basis": ("speed/a_tan/omega: p99 of moving samples (> v_moving); kappa/a_lat/blend radius: "
                      "AT SPEED (>= v_fast) — the slow band is unresolved against the stop; observed max "
                      "alongside; period-free items: " + ", ".join(period_free)),
            "v_max_mps": sp["p99"], "v_max_observed_mps": sp["max"],
            "a_tan_max_mps2": at["p99"], "a_tan_max_observed_mps2": at["max"],
            "a_lat_max_mps2": als["p99"], "a_lat_max_observed_mps2": als["max"],
            "kappa_max_1_per_m": kaps["p99"], "kappa_max_observed_1_per_m": kaps["max"],
            "min_blend_radius_m": co["min_blend_radius_at_speed_m"],
            "median_blend_radius_m": co["median_blend_radius_at_speed_m"],
            "corner_speed_class_mps": co["v_fast_mps"],
            "corners_at_speed": co["count_at_speed"],
            "omega_max_deg_s": om["p99"], "omega_max_observed_deg_s": om["max"],
        },
    }
    if geometry is not None:
        g = _strip(geometry)
        st = g["strokes"]
        prof["geometry"] = {"plane": g["plane"],
                            "strokes": {k: v for k, v in st.items() if k not in ("list", "turns")},
                            "turns": {k: v for k, v in st["turns"].items() if k != "list"},
                            "stroke_list": st["list"], "turn_list": st["turns"]["list"]}
        prof["bounds_for_generator"].update({
            "stroke_pitch_m": st["pitch_m"]["median"], "stroke_length_median_m": st["length_m"]["median"],
            "work_plane_normal": g["plane"]["normal"], "out_of_plane_p95_m": g["plane"]["out_of_plane_p95_m"]})
    return _strip(prof)


def write_yaml(path, profile: dict) -> pathlib.Path:
    path = pathlib.Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w") as fh:
        yaml.safe_dump(profile, fh, sort_keys=False, default_flow_style=False, width=100)
    return path


def read_yaml(path) -> dict:
    with open(path) as fh:
        return yaml.safe_load(fh)
