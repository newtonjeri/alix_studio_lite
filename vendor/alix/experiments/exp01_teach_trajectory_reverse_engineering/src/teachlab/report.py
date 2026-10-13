"""Plots, the markdown summary and a self-contained HTML page for one analysed
teach file. The HTML embeds the PNGs (base64) so a single file can be opened,
sent or attached; nothing is fetched from anywhere."""
from __future__ import annotations

import base64
import html
import pathlib
import re
from typing import Iterable, Optional

import numpy as np

PLOT_CAPTIONS = {
    "joints_position.png": "Joint positions, raw (grey) vs the zero-phase smoothed reference (blue) every constraint is read from.",
    "spectra.png": "Position and velocity spectra (Hann). Where the energy stops is what a generator has to reproduce; the grey raw velocity floor is the 1×/2× poll beat.",
    "joints_derivatives.png": "Peak-over-joints velocity / acceleration / jerk, raw vs smoothed — the raw jerk is the beat artefact, the smoothed curve is the motion.",
    "tcp_speed_curvature.png": "TCP speed, curvature (red ticks = resolved corners) and lateral vs tangential acceleration over time.",
    "speed_curvature.png": "Speed against curvature on moving samples with the power-law fit; β near 1/3 is what any smooth path gives (null control in the table).",
    "path3d.png": "TCP path in the arm base frame coloured by speed; red = resolved corners.",
    "geometry.png": "Motion geometry in the fitted work plane: strokes numbered, corners (red ×) and turnarounds (orange +); per-stroke length and speed; corner radius vs speed with iso-lateral-acceleration curves; out-of-plane excursion.",
    "distributions.png": "Distributions a generator has to match: TCP speed, lateral acceleration at speed, orientation rate, and per-joint velocity utilization against the xacro ceilings.",
}


def _plt():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    return plt


def write_plots(outdir: pathlib.Path, tf, q_smooth_deg: np.ndarray, joint: dict, cart: dict,
                geom: Optional[dict] = None) -> list:
    plt = _plt()
    outdir = pathlib.Path(outdir); outdir.mkdir(parents=True, exist_ok=True)
    written = []
    t = tf.t
    J = q_smooth_deg.shape[1]

    # 1 positions raw vs smoothed
    fig, axes = plt.subplots(J, 1, figsize=(11, 2.0 * J), sharex=True)
    for j in range(J):
        axes[j].plot(t, tf.q_deg[:, j], lw=0.5, color="0.6", label="raw")
        axes[j].plot(t, q_smooth_deg[:, j], lw=1.0, color="C0", label="smoothed")
        axes[j].set_ylabel(f"j{j+1} deg")
    axes[0].legend(loc="upper right"); axes[-1].set_xlabel("t [s]")
    fig.suptitle(f"{tf.path.name} — joint positions (period {tf.period_s*1e3:.0f} ms, {tf.period_status})")
    fig.tight_layout(); p = outdir / "joints_position.png"; fig.savefig(p, dpi=110); plt.close(fig); written.append(p)

    # 2 spectra
    S = joint["_spectra"]; f = S["freqs"]
    fig, axes = plt.subplots(2, 1, figsize=(11, 7))
    for j in range(J):
        axes[0].loglog(f[1:], S["pos_raw"][1:, j], lw=0.7, label=f"j{j+1}")
    axes[0].axvline(joint["position_f99"]["pooled_hz"], color="k", ls="--", lw=0.8,
                    label=f"f99 pooled {joint['position_f99']['pooled_hz']:.2f} Hz")
    axes[0].set_title("position spectrum (raw)"); axes[0].set_ylabel("|X|^2"); axes[0].legend(ncol=4, fontsize=8)
    for j in range(J):
        axes[1].loglog(f[1:], S["vel_raw"][1:, j], lw=0.5, color="0.7")
        axes[1].loglog(f[1:], S["vel_smooth"][1:, j], lw=0.8, label=f"j{j+1} smoothed")
    axes[1].set_title("velocity spectrum (grey raw, colour smoothed)"); axes[1].set_xlabel("Hz"); axes[1].legend(ncol=4, fontsize=8)
    fig.tight_layout(); p = outdir / "spectra.png"; fig.savefig(p, dpi=110); plt.close(fig); written.append(p)

    # 3 derivatives
    dr, ds = joint["_derivatives_raw"], joint["_derivatives_smoothed"]
    fig, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
    for name, ax, unit in (("vel", axes[0], "deg/s"), ("acc", axes[1], "deg/s^2"), ("jerk", axes[2], "deg/s^3")):
        ax.plot(t, np.abs(dr[name]).max(axis=1), lw=0.4, color="0.7", label="raw (max over joints)")
        ax.plot(t, np.abs(ds[name]).max(axis=1), lw=0.9, color="C1", label="smoothed")
        ax.set_ylabel(unit); ax.set_yscale("log")
    axes[0].legend(); axes[-1].set_xlabel("t [s]")
    fig.suptitle("joint derivatives — raw vs smoothed"); fig.tight_layout()
    p = outdir / "joints_derivatives.png"; fig.savefig(p, dpi=110); plt.close(fig); written.append(p)

    # 4 TCP speed, curvature, a_lat
    A = cart["_arrays"]
    fig, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
    axes[0].plot(A["t"], A["speed"], lw=0.9); axes[0].set_ylabel("TCP speed m/s")
    axes[0].axhline(cart["v_moving_threshold_mps"], color="k", ls=":", lw=0.7)
    axes[1].semilogy(A["t"], A["kappa"], lw=0.7); axes[1].set_ylabel("kappa 1/m")
    for c in cart["corners"]["corners"]:
        axes[1].axvline(c["t_s"], color="C3", lw=0.5, alpha=0.5)
    axes[2].plot(A["t"], A["a_lat"], lw=0.7, label="a_lat = v^2 kappa"); axes[2].plot(A["t"], np.abs(A["a_tan"]), lw=0.6, label="|a_tan|")
    axes[2].set_ylabel("m/s^2"); axes[2].legend(); axes[-1].set_xlabel("t [s]")
    fig.suptitle(f"TCP ({cart['frame']}) — smoothed"); fig.tight_layout()
    p = outdir / "tcp_speed_curvature.png"; fig.savefig(p, dpi=110); plt.close(fig); written.append(p)

    # 5 speed vs curvature + power law
    pl = cart["power_law"]
    m = A["moving"] & np.isfinite(A["kappa"]) & (A["kappa"] > 0)
    fig, ax = plt.subplots(figsize=(7, 6))
    ax.loglog(A["kappa"][m], A["speed"][m], ".", ms=2, alpha=0.4, label="moving samples")
    if pl.get("beta") is not None:
        kk = np.logspace(np.log10(pl["kappa_range_1_per_m"][0]), np.log10(pl["kappa_range_1_per_m"][1]), 50)
        ax.loglog(kk, pl["c"] * kk ** (-pl["beta"]), "r-", label=f"fit beta={pl['beta']:.2f} (r2={pl['r2']:.2f})")
        ax.loglog(kk, pl["c"] * kk ** (-1 / 3), "k--", lw=0.8, label="beta=1/3 reference")
    ax.set_xlabel("curvature 1/m"); ax.set_ylabel("speed m/s"); ax.legend(); ax.set_title("speed–curvature")
    fig.tight_layout(); p = outdir / "speed_curvature.png"; fig.savefig(p, dpi=110); plt.close(fig); written.append(p)

    # 6 3D path
    pos = cart["_pos"]
    fig = plt.figure(figsize=(8, 7)); ax = fig.add_subplot(111, projection="3d")
    sc = ax.scatter(pos[:, 0], pos[:, 1], pos[:, 2], c=A["speed"], s=2, cmap="viridis")
    for c in cart["corners"]["corners"]:
        i = int(round(c["t_s"] * tf.fs)); i = min(i, pos.shape[0] - 1)
        ax.scatter(pos[i, 0], pos[i, 1], pos[i, 2], color="r", s=12)
    fig.colorbar(sc, label="speed m/s"); ax.set_xlabel("x"); ax.set_ylabel("y"); ax.set_zlabel("z")
    ax.set_title(f"{cart['frame']} path in {cart['base_link']} (red: corners)")
    fig.tight_layout(); p = outdir / "path3d.png"; fig.savefig(p, dpi=110); plt.close(fig); written.append(p)

    # 7 motion geometry in the work plane
    if geom is not None:
        written.append(_geometry_plot(plt, outdir, tf, cart, geom))
    # 8 distributions
    written.append(_distributions_plot(plt, outdir, tf, joint, cart))
    return written


def _geometry_plot(plt, outdir, tf, cart, geom) -> pathlib.Path:
    A = cart["_arrays"]; uvw = geom["_uvw"]; pl = geom["plane"]; st = geom["strokes"]
    fig = plt.figure(figsize=(15, 9))
    gs = fig.add_gridspec(3, 2, width_ratios=[1.6, 1.0])
    ax = fig.add_subplot(gs[:, 0])
    sc = ax.scatter(uvw[:, 0], uvw[:, 1], c=A["speed"], s=3, cmap="viridis", zorder=1)
    for k, s_ in enumerate(st["list"]):
        seg = uvw[s_["i0"]:s_["i1"] + 1]
        ax.plot(seg[:, 0], seg[:, 1], lw=2.0, color="C1", alpha=0.35, zorder=2)
        ax.annotate(str(k + 1), s_["mid_uv_m"], fontsize=8, color="C1", ha="center", va="center",
                    bbox=dict(boxstyle="circle,pad=0.15", fc="white", ec="C1", lw=0.6), zorder=4)
    fs = tf.fs
    for c in cart["corners"]["corners"]:
        i = min(int(round(c["t_s"] * fs)), uvw.shape[0] - 1)
        ax.plot(uvw[i, 0], uvw[i, 1], "x", color="red", ms=7, mew=1.5, zorder=3)
    for c in cart["corners"]["turnarounds"]["list"]:
        i = min(int(round(c["t_s"] * fs)), uvw.shape[0] - 1)
        ax.plot(uvw[i, 0], uvw[i, 1], "+", color="orange", ms=7, mew=1.5, zorder=3)
    ax.plot(uvw[0, 0], uvw[0, 1], "o", color="k", ms=6, zorder=5); ax.annotate("start", (uvw[0, 0], uvw[0, 1]), fontsize=8)
    ax.set_aspect("equal"); ax.set_xlabel("u [m] (1st principal axis)"); ax.set_ylabel("v [m] (2nd principal axis)")
    fig.colorbar(sc, ax=ax, label="TCP speed m/s", shrink=0.6)
    ax.set_title(f"work plane: normal tilt {pl['normal_tilt_from_base_z_deg']:.0f}° from base z, "
                 f"out-of-plane rms {pl['out_of_plane_rms_m']*1e3:.1f} mm, extent {pl['extent_u_m']*1e3:.0f}×{pl['extent_v_m']*1e3:.0f} mm\n"
                 f"{st['count']} strokes (κ < {st['kappa_stroke_1_per_m']:.0f} 1/m), median length "
                 f"{(st['length_m']['median'] or 0)*1e3:.0f} mm, pitch median "
                 f"{((st['pitch_m']['median'] or float('nan'))*1e3):.0f} mm (n={st['pitch_count']}), "
                 f"dominant orientation {st['dominant_orientation_deg'] if st['dominant_orientation_deg'] is None else round(st['dominant_orientation_deg'])}°",
                 fontsize=9)
    # per-stroke length and mean speed
    ax1 = fig.add_subplot(gs[0, 1])
    if st["list"]:
        idx = np.arange(1, len(st["list"]) + 1)
        ax1.bar(idx, [s_["arc_length_m"] * 1e3 for s_ in st["list"]], color="C1", alpha=0.6, label="length mm")
        ax1.set_ylabel("stroke length mm"); ax1.set_xlabel("stroke #")
        ax1b = ax1.twinx()
        ax1b.plot(idx, [s_["mean_speed_mps"] for s_ in st["list"]], "k.-", lw=0.8, label="mean speed")
        ax1b.set_ylabel("mean speed m/s")
        ax1.set_title("per stroke: length (bars), mean speed (line), straightness in colour", fontsize=9)
        for i, s_ in zip(idx, st["list"]):
            ax1.get_children()[i - 1].set_alpha(0.3 + 0.7 * float(s_["straightness"] or 0))
    # corner radius vs speed with iso-a_lat
    ax2 = fig.add_subplot(gs[1, 1])
    co = cart["corners"]
    if co["corners"]:
        r = np.array([c["radius_m"] for c in co["corners"]]); v = np.array([c["speed_mps"] for c in co["corners"]])
        fast = v >= co["v_fast_mps"]
        ax2.plot(r[~fast] * 1e3, v[~fast], "o", color="0.6", ms=4, label="corner below v_fast")
        ax2.plot(r[fast] * 1e3, v[fast], "o", color="red", ms=5, label=f"corner at speed (≥ {co['v_fast_mps']} m/s)")
        rr = np.linspace(max(r.min() * 0.5, 1e-3), r.max() * 1.5, 100)
        for a_lat in (0.25, 0.5, 1.0, 2.0):
            ax2.plot(rr * 1e3, np.sqrt(a_lat * rr), "--", lw=0.7, label=f"v = √(a_lat·r), a_lat = {a_lat} m/s²")
        ax2.set_xscale("log"); ax2.set_xlabel("blend radius mm"); ax2.set_ylabel("speed m/s"); ax2.legend(fontsize=7)
    ax2.set_title("corner speed vs radius", fontsize=9)
    # out-of-plane excursion
    ax3 = fig.add_subplot(gs[2, 1])
    ax3.plot(A["t"], uvw[:, 2] * 1e3, lw=0.7); ax3.set_xlabel("t [s]"); ax3.set_ylabel("w (out of plane) mm")
    ax3.set_title(f"out-of-plane: p95 {pl['out_of_plane_p95_m']*1e3:.1f} mm, max {pl['out_of_plane_max_m']*1e3:.1f} mm", fontsize=9)
    fig.suptitle(f"{tf.path.name} — motion geometry ({cart['frame']} in {cart['base_link']})")
    fig.tight_layout(); p = outdir / "geometry.png"; fig.savefig(p, dpi=110); plt.close(fig)
    return p


def _distributions_plot(plt, outdir, tf, joint, cart) -> pathlib.Path:
    A = cart["_arrays"]; u = joint["utilization"]
    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    sp = A["speed"][A["moving"]]
    axes[0, 0].hist(sp, bins=60, color="C0", alpha=0.8)
    for q, c in ((50, "k"), (99, "r")):
        axes[0, 0].axvline(np.percentile(sp, q) if sp.size else 0, color=c, ls="--", lw=0.8, label=f"p{q}")
    axes[0, 0].set_xlabel("TCP speed m/s (moving)"); axes[0, 0].legend(); axes[0, 0].set_title("speed", fontsize=9)
    al = A["a_lat"][A["fast"] & np.isfinite(A["a_lat"])]
    axes[0, 1].hist(al, bins=60, color="C2", alpha=0.8)
    if al.size:
        axes[0, 1].axvline(np.percentile(al, 99), color="r", ls="--", lw=0.8, label="p99")
    axes[0, 1].set_xlabel("lateral acceleration m/s² (at speed)"); axes[0, 1].legend(); axes[0, 1].set_title("a_lat", fontsize=9)
    om = A["omega"][A["moving"]]
    axes[1, 0].hist(om, bins=60, color="C4", alpha=0.8)
    if om.size:
        axes[1, 0].axvline(np.percentile(om, 99), color="r", ls="--", lw=0.8, label="p99")
    axes[1, 0].set_xlabel("orientation rate deg/s (moving)"); axes[1, 0].legend(); axes[1, 0].set_title("omega", fontsize=9)
    jj = np.arange(1, len(u["utilization"]) + 1)
    axes[1, 1].bar(jj, u["utilization"], color=["C3" if x > 0.7 else "C0" for x in u["utilization"]])
    axes[1, 1].axhline(0.7, color="r", ls="--", lw=0.8, label="0.7 trigger"); axes[1, 1].set_ylim(0, 1.05)
    axes[1, 1].set_xlabel("joint"); axes[1, 1].set_ylabel("peak |qdot| / ceiling"); axes[1, 1].legend()
    axes[1, 1].set_title(f"utilization (smoothed basis; max {u['max_utilization']:.2f} on j{u['binding_joint']})", fontsize=9)
    fig.suptitle(f"{tf.path.name} — distributions (period {tf.period_s*1e3:.0f} ms, {tf.period_status})")
    fig.tight_layout(); p = outdir / "distributions.png"; fig.savefig(p, dpi=110); plt.close(fig)
    return p


def _fmt(x, nd=3):
    if x is None:
        return "—"
    if isinstance(x, float):
        return f"{x:.{nd}f}"
    return str(x)


def _mm(x):
    return "—" if x is None else f"{x*1e3:.0f}"


def _pct(x):
    return "—" if x is None else f"{x:.0%}"


def write_markdown(outdir: pathlib.Path, tf, smoothing_meta: dict, joint: dict, cart: dict,
                   beat: Optional[dict] = None, residual: Optional[dict] = None,
                   geom: Optional[dict] = None, interpretation: Optional[dict] = None) -> pathlib.Path:
    u = joint["utilization"]; sp = cart["speed_moving"]; pl = cart["power_law"]; co = cart["corners"]
    nc = pl.get("null_control", {})
    lines = [f"# {tf.path.name} — analysis", "",
             f"- {tf.describe()}",
             f"- smoothing: {smoothing_meta}",
             f"- derivative scheme: {joint['scheme']}; spectra: {joint.get('spectrum_window')} window; "
             f"utilization basis: {joint.get('utilization_basis')}"]
    if interpretation is not None:
        src = "MANIFEST.yaml entry" if interpretation.get("in_manifest") else "defaults (NOT in MANIFEST.yaml — add an entry)"
        lines.append(f"- interpretation from {src}: units {tf.units} ({tf.units_source}), side {tf.side}, "
                     f"frame {cart['frame']}, period {tf.period_s*1e3:.1f} ms ({tf.period_status})")
        prov = interpretation.get("provenance") or {}
        if prov:
            lines.append("- provenance: " + "; ".join(f"{k}: {v}" for k, v in prov.items()))
    if tf.problems:
        lines += ["", "**Problems while reading:**"] + [f"- {p}" for p in tf.problems]
    lines.append("")
    if tf.period_status == "assumed":
        lines += ["> **Period is ASSUMED.** Speeds scale 1/T, accelerations 1/T², jerk 1/T³ with the true "
                  "record period; path length, curvature, radii, planes, strokes and pitch do not.", ""]
    if beat:
        lines += ["## Record-rate evidence (beat test)",
                  f"- spectral line in |dq|: {_fmt(beat.get('line_hz'), 2)} Hz "
                  f"(prominence {_fmt(beat.get('line_prominence'), 1)}x median)",
                  f"- step ratio modes: {beat.get('ratio_modes')}  1x-frac {_fmt(beat.get('frac_1x'))}  "
                  f"2x-frac {_fmt(beat.get('frac_2x'))}  → source/poll = {_fmt(beat.get('source_over_poll'), 3)} "
                  f"(beat {'DETECTED' if beat.get('beat_detected') else 'not detected'})", ""]
    if residual:
        lines += ["## Smoothing residual (raw − smoothed)",
                  f"- RMS per joint deg: {[round(v, 4) for v in residual['rms_deg']]}; overall {residual['rms_overall_deg']:.4f}", ""]
    lines += ["## Joint space", "",
              "| joint | range deg | peak vel (sm) | peak acc (sm) | peak jerk (sm) | raw peak jerk | util | f99 pos Hz | reversals |",
              "|---|---|---|---|---|---|---|---|---|"]
    for j in range(len(u["utilization"])):
        r = joint["reversals"]["per_joint"][j]
        lines.append(f"| {j+1} | {joint['range_deg'][j][0]:.1f}..{joint['range_deg'][j][1]:.1f} | "
                     f"{joint['smoothed']['peak_vel_deg_s'][j]:.1f} | {joint['smoothed']['peak_acc_deg_s2'][j]:.0f} | "
                     f"{joint['smoothed']['peak_jerk_deg_s3'][j]:.0f} | {joint['raw']['peak_jerk_deg_s3'][j]:.0f} | "
                     f"{u['utilization'][j]:.2f} | {joint['position_f99']['per_joint_hz'][j]:.2f} | "
                     f"{r['reversal_fraction']:.2f} (med {r['median_abs_step_lsb']:.0f} LSB) |")
    lines += ["", f"- max utilization {u['max_utilization']:.2f} on joint {u['binding_joint']} "
              f"(qdot_max {u['qdot_max_deg_s']} deg/s); position f99 pooled {joint['position_f99']['pooled_hz']:.2f} Hz; "
              f"velocity f99 pooled raw {joint['velocity_f99']['pooled_hz']:.1f} Hz / smoothed {joint['velocity_smoothed_f99']['pooled_hz']:.2f} Hz", ""]
    lines += [f"## Cartesian ({cart['frame']} in {cart['base_link']})", "",
              f"- path length {cart['path_length_m']:.3f} m over {cart['duration_s']:.2f} s → mean speed incl. dwell "
              f"{_fmt(cart['mean_speed_full_mps'])} m/s; active span {cart['active_span_s']:.2f} s → "
              f"{_fmt(cart['mean_speed_active_span_mps'])} m/s; moving fraction {cart['moving_fraction']:.2f}",
              f"- speed (moving): median {_fmt(sp['median'])}, p95 {_fmt(sp['p95'])}, p99 {_fmt(sp['p99'])}, max {_fmt(sp['max'])} m/s; CV {_fmt(cart['speed_cv_moving'])}",
              f"- curvature (moving): median {_fmt(cart['kappa_1_per_m']['median'], 1)}, p95 {_fmt(cart['kappa_1_per_m']['p95'], 1)}, max {_fmt(cart['kappa_1_per_m']['max'], 1)} 1/m",
              f"- lateral accel: p95 {_fmt(cart['a_lat_mps2']['p95'])}, p99 {_fmt(cart['a_lat_mps2']['p99'])}, max {_fmt(cart['a_lat_mps2']['max'])} m/s²; "
              f"tangential |a|: p99 {_fmt(cart['a_tan_mps2']['p99'])}, max {_fmt(cart['a_tan_mps2']['max'])} m/s²",
              f"- orientation rate: p95 {_fmt(cart['omega_deg_s']['p95'], 1)}, p99 {_fmt(cart['omega_deg_s']['p99'], 1)}, max {_fmt(cart['omega_deg_s']['max'], 1)} deg/s",
              f"- power law v = c·κ^(−β): β = {_fmt(pl.get('beta'))}, r² = {_fmt(pl.get('r2'))}, n = {pl.get('n')} — "
              f"NULL CONTROL (phase-randomised surrogates, same spectrum): β = {_fmt(nc.get('beta_mean'))} ± {_fmt(nc.get('beta_std'))} "
              f"(r² {_fmt(nc.get('r2_mean'))}, n = {nc.get('n_surrogates')}); the two-thirds law's 1/3 is what ANY smooth path gives",
              f"- corners (κ > {co['kappa_min_1_per_m']} 1/m, resolved): {co['count']}; min blend radius {_fmt(co['min_blend_radius_m'], 4)} m, "
              f"median {_fmt(co['median_blend_radius_m'], 4)} m; at speed ≥ {co['v_fast_mps']} m/s: {co['count_at_speed']} corners, "
              f"min radius {_fmt(co['min_blend_radius_at_speed_m'], 4)} m, max a_lat {_fmt(co['max_a_lat_at_speed_mps2'])} m/s²; "
              f"stroke-end turnarounds (radius unresolved): {co['turnarounds']['count']}",
              f"- at-speed curvature (≥ {cart['v_fast_threshold_mps']} m/s): p95 {_fmt(cart['kappa_at_speed_1_per_m']['p95'], 1)}, "
              f"p99 {_fmt(cart['kappa_at_speed_1_per_m']['p99'], 1)}, max {_fmt(cart['kappa_at_speed_1_per_m']['max'], 1)} 1/m; "
              f"a_lat at speed p99 {_fmt(cart['a_lat_at_speed_mps2']['p99'])}, max {_fmt(cart['a_lat_at_speed_mps2']['max'])} m/s²", ""]
    if co["corners"]:
        lines += ["| t s | radius m | speed m/s | a_lat m/s² |", "|---|---|---|---|"]
        for c in co["corners"][:40]:
            lines.append(f"| {c['t_s']:.2f} | {c['radius_m']:.4f} | {c['speed_mps']:.3f} | {c['a_lat_mps2']:.2f} |")
        if len(co["corners"]) > 40:
            lines.append(f"| … {len(co['corners'])-40} more | | | |")
        lines.append("")
    if geom is not None:
        pln = geom["plane"]; st = geom["strokes"]; tn = st["turns"]
        lines += ["## Motion geometry (work plane, strokes, pitch)", "",
                  f"- work plane (PCA, {pln.get('fitted_on', 'moving samples')}): normal {[round(x, 3) for x in pln['normal']]} in {cart['base_link']}, "
                  f"tilt {pln['normal_tilt_from_base_z_deg']:.1f}° from base z; out-of-plane rms {_mm(pln['out_of_plane_rms_m'])} mm, "
                  f"p95 {_mm(pln['out_of_plane_p95_m'])} mm, max {_mm(pln['out_of_plane_max_m'])} mm; in-plane extent "
                  f"{_mm(pln['extent_u_m'])} × {_mm(pln['extent_v_m'])} mm (bbox {pln['bbox_area_m2']:.3f} m²); planarity {pln['planarity']:.3f}",
                  f"- strokes (moving runs with κ < {st['kappa_stroke_1_per_m']:.0f} 1/m, ≥ {_mm(st['min_length_m'])} mm): {st['count']}"
                  f"{' (transit: #' + ', #'.join(str(k) for k in st.get('transit_strokes', [])) + ')' if st.get('transit_strokes') else ''}, "
                  f"covering {st['stroke_fraction_of_path']:.0%} of the path; length median {_mm(st['length_m']['median'])} mm "
                  f"(min {_mm(st['length_m']['min'])}, p90 {_mm(st['length_m']['p90'])}, max {_mm(st['length_m']['max'])}); "
                  f"straightness median {_fmt(st['straightness']['median'])}; bow median {_mm(st['bow_m']['median'])} mm; "
                  f"mean stroke speed median {_fmt(st['mean_speed_mps']['median'])} m/s",
                  f"- dominant stroke orientation {_fmt(st['dominant_orientation_deg'], 1)}° in-plane (alignment {_fmt(st['orientation_alignment'], 2)}; 1 = all strokes parallel)",
                  f"- pitch between consecutive parallel strokes: median {_mm(st['pitch_m']['median'])} mm "
                  f"(min {_mm(st['pitch_m']['min'])}, max {_mm(st['pitch_m']['max'])}, n = {st['pitch_count']})",
                  f"- turns between strokes: {tn['count']}, duration median {_fmt(tn['duration_s']['median'], 2)} s, "
                  f"{_pct(tn['stopped_fraction'])} of them pass through a stop", ""]
        if st["list"]:
            lines += ["| # | t0 s | length mm | chord mm | straight | bow mm | dir ° | mean v m/s | peak v m/s |", "|---|---|---|---|---|---|---|---|---|"]
            for k, s_ in enumerate(st["list"][:60]):
                lines.append(f"| {k+1} | {s_['t0_s']:.2f} | {s_['arc_length_m']*1e3:.0f} | {s_['chord_m']*1e3:.0f} | "
                             f"{_fmt(s_['straightness'], 2)} | {_mm(s_['bow_m']) if np.isfinite(s_['bow_m']) else '—'} | "
                             f"{_fmt(s_['direction_deg'], 0)} | {_fmt(s_['mean_speed_mps'])} | {s_['peak_speed_mps']:.3f} |")
            if len(st["list"]) > 60:
                lines.append(f"| … {len(st['list'])-60} more | | | | | | | | |")
            lines.append("")
    p = pathlib.Path(outdir) / "report.md"
    p.write_text("\n".join(lines) + "\n")
    return p


# ── HTML ───────────────────────────────────────────────────────────────────

_CSS = """
body{font-family:system-ui,-apple-system,Segoe UI,Roboto,sans-serif;max-width:1200px;margin:1.5rem auto;padding:0 1rem;color:#1b1b1b;background:#fff;line-height:1.45}
h1{font-size:1.5rem;border-bottom:2px solid #ddd;padding-bottom:.3rem}h2{font-size:1.15rem;margin-top:1.6rem;color:#234}
table{border-collapse:collapse;font-size:.85rem;margin:.6rem 0;display:block;overflow-x:auto}th,td{border:1px solid #ccc;padding:.2rem .5rem;text-align:right}th{background:#f2f4f7}td:first-child,th:first-child{text-align:left}
blockquote{border-left:4px solid #e0a800;background:#fff8e1;margin:.8rem 0;padding:.5rem .8rem}
figure{margin:1.2rem 0}figure img{max-width:100%;height:auto;border:1px solid #e5e5e5}figcaption{font-size:.85rem;color:#555;margin-top:.3rem}
code{background:#f3f3f3;padding:0 .2rem;border-radius:3px;font-size:.9em}.meta{color:#666;font-size:.85rem}
@media (prefers-color-scheme: dark){body{background:#151719;color:#e6e6e6}th{background:#23272b}th,td{border-color:#444}h2{color:#9cc}figure img{border-color:#333;background:#fff}blockquote{background:#3a3000;border-color:#e0a800}code{background:#2a2d31}}
"""


def _inline(s: str) -> str:
    s = html.escape(s, quote=False)
    s = re.sub(r"\*\*(.+?)\*\*", r"<strong>\1</strong>", s)
    s = re.sub(r"`([^`]+)`", r"<code>\1</code>", s)
    s = re.sub(r"\[([^\]]+)\]\(([^)]+)\)", r'<a href="\2">\1</a>', s)
    return s


def md_to_html(md: str) -> str:
    """The subset of markdown these reports use: #/## headings, - bullets,
    > quotes, | tables |, paragraphs. Anything else is a paragraph."""
    out, buf, mode = [], [], None

    def flush():
        nonlocal buf, mode
        if mode == "ul":
            out.append("<ul>" + "".join(f"<li>{_inline(x)}</li>" for x in buf) + "</ul>")
        elif mode == "table":
            rows = [r for r in buf if not re.match(r"^\|?\s*:?-{2,}", r.strip("| "))]
            cells = [[c.strip() for c in r.strip().strip("|").split("|")] for r in rows]
            if cells:
                head = "".join(f"<th>{_inline(c)}</th>" for c in cells[0])
                body = "".join("<tr>" + "".join(f"<td>{_inline(c)}</td>" for c in r) + "</tr>" for r in cells[1:])
                out.append(f"<table><thead><tr>{head}</tr></thead><tbody>{body}</tbody></table>")
        elif mode == "quote":
            out.append("<blockquote>" + _inline(" ".join(buf)) + "</blockquote>")
        elif mode == "p":
            out.append("<p>" + _inline(" ".join(buf)) + "</p>")
        buf, mode = [], None

    for line in md.splitlines():
        s = line.rstrip()
        if not s.strip():
            flush(); continue
        if s.startswith("# "):
            flush(); out.append(f"<h1>{_inline(s[2:])}</h1>"); continue
        if s.startswith("## "):
            flush(); out.append(f"<h2>{_inline(s[3:])}</h2>"); continue
        kind = "ul" if s.lstrip().startswith("- ") else "table" if s.lstrip().startswith("|") else "quote" if s.startswith("> ") else "p"
        if mode != kind:
            flush(); mode = kind
        buf.append(s.lstrip()[2:] if kind in ("ul", "quote") else s)
    flush()
    return "\n".join(out)


def _img_tag(path: pathlib.Path) -> str:
    data = base64.b64encode(path.read_bytes()).decode("ascii")
    return f'<img src="data:image/png;base64,{data}" alt="{html.escape(path.name)}">'


def write_html(outdir: pathlib.Path, title: str, markdown_path: pathlib.Path,
               plots: Iterable[pathlib.Path], captions: Optional[dict] = None,
               name: str = "report.html", intro_html: str = "") -> pathlib.Path:
    """One self-contained page: the markdown report rendered, then every plot
    embedded with its caption."""
    captions = captions or PLOT_CAPTIONS
    figs = []
    for p in plots:
        p = pathlib.Path(p)
        if p.is_file():
            figs.append(f"<figure>{_img_tag(p)}<figcaption><b>{html.escape(p.name)}</b> — "
                        f"{html.escape(captions.get(p.name, ''))}</figcaption></figure>")
    body = md_to_html(pathlib.Path(markdown_path).read_text())
    page = (f"<!doctype html><html><head><meta charset='utf-8'><title>{html.escape(title)}</title>"
            f"<style>{_CSS}</style></head><body>{intro_html}{body}<h2>Figures</h2>{''.join(figs)}"
            f"<p class='meta'>Self-contained: every figure is embedded. Generated by exp01 teachlab.</p></body></html>")
    out = pathlib.Path(outdir) / name
    out.write_text(page)
    return out
