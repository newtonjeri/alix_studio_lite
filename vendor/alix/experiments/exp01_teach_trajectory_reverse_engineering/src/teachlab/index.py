"""Cross-file index of every analysed teach file — the table that lets two
taught plans be read side by side, plus one comparison figure and an HTML page.

`collect(results_root)` keeps, per (source sha256, side, frame), the newest
result directory that holds a profile.yaml, so re-analysing a file replaces
its row and adding a file adds one. The index is rebuilt from the results on
disk after every analysis run (analyze_teach.py) or on demand
(`python3 src/analyze_teach.py --index-only`).
"""
from __future__ import annotations

import json
import pathlib
from typing import List, Optional

import yaml

from teachlab import __version__
from teachlab.report import PLOT_CAPTIONS, _plt, md_to_html, write_html

INDEX_COLUMNS = [
    ("file", "file"), ("side", "side"), ("samples", "samples"), ("duration_s", "dur s"),
    ("period_ms", "T ms"), ("period_status", "T status"), ("in_manifest", "manifest"),
    ("path_m", "path m"), ("mean_speed_mps", "v̄ m/s"), ("speed_p99_mps", "v p99"), ("speed_max_mps", "v max"),
    ("speed_cv", "CV"), ("corners_at_speed", "corners@v"), ("min_radius_mm", "r min mm"),
    ("a_lat_p99", "a_lat p99"), ("a_tan_p99", "a_tan p99"), ("omega_p99", "ω p99 °/s"),
    ("util_max", "util"), ("binding_joint", "j"), ("f99_hz", "f99 Hz"), ("beat_ratio", "src/poll"),
    ("strokes", "strokes"), ("stroke_len_mm", "stroke mm"), ("pitch_mm", "pitch mm"),
    ("plane_tilt_deg", "tilt °"), ("out_of_plane_mm", "oop mm"), ("result_dir", "results"),
]


def _g(d: dict, *keys, default=None):
    for k in keys:
        if not isinstance(d, dict) or k not in d or d[k] is None:
            return default
        d = d[k]
    return d


def row_from_profile(prof: dict, result_dir: pathlib.Path) -> dict:
    src = prof.get("source", {}); c = prof.get("cartesian", {}); j = prof.get("joint", {})
    geo = prof.get("geometry", {}); st = geo.get("strokes", {}); pl = geo.get("plane", {})
    mm = lambda x: None if x is None else round(x * 1e3, 1)   # noqa: E731
    return {
        "file": src.get("file"), "sha256": src.get("sha256"), "side": src.get("side"),
        "samples": src.get("samples"), "duration_s": round(src.get("duration_s", 0.0), 2),
        "period_ms": round(src.get("period_s", 0.0) * 1e3, 2), "period_status": src.get("period_status"),
        "in_manifest": src.get("in_manifest"), "frame": c.get("frame"),
        "path_m": c.get("path_length_m"), "mean_speed_mps": c.get("mean_speed_full_mps"),
        "speed_p99_mps": _g(c, "speed_mps", "p99"), "speed_max_mps": _g(c, "speed_mps", "max"),
        "speed_cv": c.get("speed_cv_moving"),
        "corners_at_speed": _g(c, "corners", "count_at_speed"), "min_radius_mm": mm(_g(c, "corners", "min_blend_radius_at_speed_m")),
        "a_lat_p99": _g(c, "a_lat_at_speed_mps2", "p99"), "a_tan_p99": _g(c, "a_tan_mps2", "p99"),
        "omega_p99": _g(c, "omega_deg_s", "p99"),
        "util_max": j.get("max_utilization"), "binding_joint": j.get("binding_joint"),
        "f99_hz": j.get("position_f99_pooled_hz"),
        "beat_ratio": _g(prof, "record_rate_evidence", "source_over_poll"),
        "strokes": st.get("count"), "stroke_len_mm": mm(_g(st, "length_m", "median")),
        "pitch_mm": mm(_g(st, "pitch_m", "median")), "plane_tilt_deg": pl.get("normal_tilt_from_base_z_deg"),
        "out_of_plane_mm": mm(pl.get("out_of_plane_rms_m")),
        "result_dir": result_dir.name, "generated_by": prof.get("generated_by"),
    }


def collect(results_root: pathlib.Path) -> List[dict]:
    """Newest profile per (sha256, side, frame), sorted by file name."""
    results_root = pathlib.Path(results_root)
    best = {}
    if not results_root.is_dir():
        return []
    for d in sorted(p for p in results_root.iterdir() if p.is_dir()):
        pf = d / "profile.yaml"
        if not pf.is_file():
            continue
        try:
            with open(pf) as fh:
                prof = yaml.safe_load(fh)
        except Exception:                                     # noqa: BLE001
            continue
        if not isinstance(prof, dict) or "source" not in prof:
            continue
        row = row_from_profile(prof, d)
        key = (row["sha256"], row["side"], row["frame"])
        prev = best.get(key)
        if prev is None or (d.name, pf.stat().st_mtime) >= (prev["result_dir"], prev["_mtime"]):
            row["_mtime"] = pf.stat().st_mtime
            best[key] = row
    rows = sorted(best.values(), key=lambda r: (str(r["file"]), str(r["side"])))
    for r in rows:
        r.pop("_mtime", None)
    return rows


def _fmt(v, nd=3):
    if v is None:
        return "—"
    if isinstance(v, bool):
        return "yes" if v else "NO"
    if isinstance(v, float):
        return f"{v:.{nd}f}" if abs(v) < 100 else f"{v:.0f}"
    return str(v)


def write_markdown(rows: List[dict], out: pathlib.Path, results_root: pathlib.Path) -> pathlib.Path:
    lines = ["# Teach-file index", "",
             f"- {len(rows)} analysed file(s); newest profile per (file, side, frame) under `{results_root.name}/`",
             "- speeds/accelerations scale with the record period while `T status` is `assumed`; path, radius, pitch and plane do not",
             "- `manifest: NO` = analysed under defaults (units inferred) — add an entry to data/teach_files/MANIFEST.yaml", ""]
    head = "| " + " | ".join(h for _, h in INDEX_COLUMNS) + " |"
    sep = "|" + "---|" * len(INDEX_COLUMNS)
    lines += [head, sep]
    for r in rows:
        lines.append("| " + " | ".join(_fmt(r.get(k)) for k, _ in INDEX_COLUMNS) + " |")
    lines.append("")
    out.write_text("\n".join(lines) + "\n")
    return out


def write_plot(rows: List[dict], out: pathlib.Path) -> Optional[pathlib.Path]:
    if not rows:
        return None
    plt = _plt()
    import numpy as np
    labels = [f"{r['file']}\n({r['side']})" for r in rows]
    x = np.arange(len(rows))
    panels = [
        ("TCP speed m/s", [("mean incl. dwell", "mean_speed_mps"), ("p99", "speed_p99_mps"), ("max", "speed_max_mps")]),
        ("corner radius mm (at speed) / stroke pitch mm", [("min blend radius", "min_radius_mm"), ("pitch median", "pitch_mm")]),
        ("acceleration m/s²", [("a_lat p99 (at speed)", "a_lat_p99"), ("a_tan p99", "a_tan_p99")]),
        ("joint utilization (peak/ceiling)", [("max", "util_max")]),
        ("orientation rate p99 °/s", [("ω p99", "omega_p99")]),
        ("strokes / corners", [("strokes", "strokes"), ("corners at speed", "corners_at_speed")]),
    ]
    fig, axes = plt.subplots(2, 3, figsize=(15, 8))
    for ax, (title, series) in zip(axes.ravel(), panels):
        w = 0.8 / len(series)
        for k, (lab, key) in enumerate(series):
            vals = [r.get(key) if r.get(key) is not None else np.nan for r in rows]
            ax.bar(x + (k - (len(series) - 1) / 2) * w, vals, w, label=lab)
        ax.set_xticks(x); ax.set_xticklabels(labels, fontsize=7); ax.set_title(title, fontsize=9); ax.legend(fontsize=7)
        if key == "util_max":
            ax.axhline(0.7, color="r", ls="--", lw=0.8); ax.set_ylim(0, 1.05)
    fig.suptitle("teach files side by side (assumed-period files: speeds/accelerations scale with the true period)")
    fig.tight_layout(); fig.savefig(out, dpi=110); plt.close(fig)
    return out


def write_index(results_root: pathlib.Path, plots: bool = True) -> dict:
    """Rebuild index.md/json/html (+ index.png unless plots=False). The plot is
    cheap and is rebuilt on every call so it can never be stale."""
    results_root = pathlib.Path(results_root)
    rows = collect(results_root)
    md = write_markdown(rows, results_root / "index.md", results_root)
    (results_root / "index.json").write_text(json.dumps({"generated_by": f"exp01 teachlab {__version__}", "rows": rows}, indent=1))
    png = None
    if plots:
        try:
            png = write_plot(rows, results_root / "index.png")
        except Exception as exc:                                  # noqa: BLE001
            print(f"index plot skipped: {exc!r}")
    figs = [png] if png else []
    # per-file geometry thumbnails give the page its meaning; embed when present
    for r in rows:
        g = results_root / r["result_dir"] / "geometry.png"
        if g.is_file():
            figs.append(g)
    caps = dict(PLOT_CAPTIONS); caps["index.png"] = "Headline metrics of every analysed teach file side by side."
    for r in rows:
        caps["geometry.png"] = "Motion geometry per file (work plane, strokes, corners)."
    html_path = write_html(results_root, "Teach-file index", md, figs, captions=caps, name="index.html")
    return {"rows": rows, "markdown": md, "html": html_path, "png": png}
