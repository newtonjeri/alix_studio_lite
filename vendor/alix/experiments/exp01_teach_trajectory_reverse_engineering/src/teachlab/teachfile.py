"""Drag-teach file reader and the record-rate evidence it can extract.

A file saved by `rm_save_trajectory()` is newline-delimited JSON, one object
per sample, seven integers for a 7-DoF arm:

    {"point":[-2145,106416,-2650,-102641,3747,-1871,179896]}

That is all it carries: no units, no timestamps, no frame, no arm. What is
KNOWN and what is INFERRED is kept apart in the `TeachFile` record:

* units — inferred `millideg` (0.001 deg/LSB): the coarsest scale under which
  every column stays inside the joint limits, corroborated by the vendor's own
  saved files and the controller's 0.001 deg joint precision; never yet
  confirmed by a controller readback (exp01 README, hypothesis 1).
* column order — joint1..7 in URDF order with URDF signs is PROVEN for the
  SDK/UDP joint array (0.007 mm TCP reproduction on hardware runs) and
  ASSUMED for this file (hypothesis 2).
* period — UNKNOWN. The file exposes only the ratio between the rate the
  controller's joint state advanced and the rate it was polled (`beat_test`);
  the absolute period comes from a timed replay (Phase C). Until then every
  derivative is computed with `period_s` marked `assumed`.

Per-file knowledge lives beside the files in `MANIFEST.yaml` (see
`load_manifest` / `interpretation_for`): which arm, which units, the measured
period once Phase C has it, who taught it and when. A file that is not in the
manifest is analysed under the defaults and flagged as such, so dropping a new
export into `data/teach_files/` is enough to get it analysed — and adding a
manifest entry is what turns its "assumed" rows into knowledge.
"""
from __future__ import annotations

import hashlib
import json
import math
import pathlib
from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

import numpy as np

#: Candidate integer -> radian scales, coarsest first (from reconstruct_tcp_path).
UNIT_SCALES = {
    "rad": 1.0,
    "deg": math.pi / 180.0,
    "decideg": math.pi / 1800.0,
    "millirad": 1e-3,
    "centideg": math.pi / 18000.0,
    "millideg": math.pi / 180000.0,
    "1e-5rad": 1e-5,
    "1e-4deg": math.pi / 1800000.0,
    "microrad": 1e-6,
}

DEFAULT_PERIOD_S = 0.010        # the working hypothesis; status 'assumed'
MIN_SAMPLES = 100               # 1 s at 10 ms: below this nothing here is a trajectory


class TeachFileError(ValueError):
    """The file cannot be read as a teach trajectory."""


@dataclass
class TeachFile:
    path: pathlib.Path
    sha256: str
    raw: np.ndarray                 # (N, 7) integers as float64, file order
    q_deg: np.ndarray               # (N, 7) degrees, joint1..7, URDF signs
    units: str
    scale_rad_per_int: float
    columns: Tuple[int, ...]
    signs: Tuple[int, ...]
    period_s: float
    period_status: str              # 'assumed' | 'measured'
    side: str                       # 'L' | 'R' — which arm's chain to use
    problems: List[str] = field(default_factory=list)
    units_source: str = "given"     # 'given' | 'inferred' (units='auto')

    @property
    def n(self) -> int:
        return int(self.q_deg.shape[0])

    @property
    def fs(self) -> float:
        return 1.0 / self.period_s

    @property
    def duration_s(self) -> float:
        return (self.n - 1) * self.period_s

    @property
    def q_rad(self) -> np.ndarray:
        return np.radians(self.q_deg)

    @property
    def t(self) -> np.ndarray:
        return np.arange(self.n) * self.period_s

    def describe(self) -> str:
        return (f"{self.path.name}: {self.n} samples x {self.q_deg.shape[1]} joints, "
                f"units={self.units} ({self.units_source}), period={self.period_s*1e3:.1f} ms "
                f"({self.period_status}), side={self.side}, "
                f"duration={self.duration_s:.2f} s, sha256={self.sha256[:12]}")


# ── parsing ────────────────────────────────────────────────────────────────

def read_points(path: pathlib.Path, field_name: str = "point") -> Tuple[List[List[float]], List[str]]:
    """Every sample in the file, with per-line problems collected (not raised).

    Accepts the NDJSON `{"point":[...]}` form, a bare JSON list per line, or
    comma/whitespace-separated numbers. Blank lines and `#` comments are
    skipped. Rows of differing width are reported as problems and dropped, so
    one corrupt line never silently shifts every later joint.
    """
    from collections import Counter
    points: List[List[float]] = []
    problems: List[str] = []
    with open(path, "r", encoding="utf-8-sig", errors="replace") as fh:   # utf-8-sig: a BOM is skipped
        for lineno, line in enumerate(fh, 1):
            s = line.strip()
            if not s or s.startswith("#"):
                continue
            try:
                if s.startswith("{"):
                    obj = json.loads(s)
                    vals = obj[field_name]
                elif s.startswith("["):
                    vals = json.loads(s)
                else:
                    vals = [float(x) for x in s.replace(",", " ").split()]
                row = [float(v) for v in vals]
                if not all(math.isfinite(v) for v in row):
                    raise ValueError("non-finite value")
                points.append(row)
            except Exception as exc:                          # noqa: BLE001
                problems.append(f"line {lineno}: {type(exc).__name__}: {exc}")
    if points:
        # the file's width is the MODAL row length: one over-wide or short
        # line is the corrupt one, not the 4411 rows that agree
        counts = Counter(len(p) for p in points)
        width, n_modal = counts.most_common(1)[0]
        if n_modal < 0.9 * len(points):
            problems.append(f"only {n_modal} of {len(points)} samples have the modal width {width} "
                            f"(widths seen: {dict(sorted(counts.items()))}) — mixed formats?")
        keep = []
        for i, p in enumerate(points):
            if len(p) != width:
                problems.append(f"sample {i}: {len(p)} values, expected {width}")
            else:
                keep.append(p)
        points = keep
    return points, problems


def sha256_of(path: pathlib.Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as fh:
        for chunk in iter(lambda: fh.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def to_degrees(raw: np.ndarray, columns: Sequence[int], signs: Sequence[int],
               scale_rad_per_int: float) -> np.ndarray:
    """`out[:, j] = raw[:, columns[j]-1] * scale * signs[j]`, in degrees."""
    if raw.shape[1] < max(columns):
        raise TeachFileError(f"file has {raw.shape[1]} values per sample but joint columns {tuple(columns)} "
                             f"need {max(columns)} — this pipeline models the 7-DoF RM75 (pass columns= for "
                             f"a wider file; a 6-DoF export is not an RM75 recording)")
    cols = [c - 1 for c in columns]
    q_rad = raw[:, cols] * float(scale_rad_per_int) * np.asarray(signs, dtype=float)
    return np.degrees(q_rad)


def infer_units(raw: np.ndarray, limits_deg: Sequence[Tuple[float, float]],
                columns: Sequence[int] = (1, 2, 3, 4, 5, 6, 7)) -> Tuple[list, Optional[str]]:
    """Rank the candidate scales; the recommendation is the COARSEST with zero
    limit violations, except that an all-integer file inside the limits under
    millideg is millideg (the controller's export format). Finer scales
    always fit — by shrinking the recording to implausibility — which is why
    the ranking is reported, not just the pick. This is a range test: a
    non-integer file with under ~3 deg of sweep cannot be classified; put its
    units in the manifest.

    Returns (rows, best) with rows = (name, scale, violations, max|q| deg,
    max sweep deg), coarsest first.
    """
    rows = []
    best = None
    lo = np.array([l for l, _ in limits_deg]); hi = np.array([h for _, h in limits_deg])
    viol_of = {}
    for name, scale in UNIT_SCALES.items():
        q = to_degrees(raw, columns, (1,) * len(columns), scale)
        viol = int(((q < lo) | (q > hi)).sum())
        viol_of[name] = viol
        rows.append((name, scale, viol, float(np.abs(q).max()),
                     float((q.max(axis=0) - q.min(axis=0)).max())))
        if viol == 0 and best is None:
            best = name
    # An all-integer file is a controller export (0.001 deg per LSB); the
    # range test alone would call a small-motion integer file "millirad"
    # or "rad". Integers + millideg inside the limits => millideg.
    sub = np.asarray(raw, dtype=float)[:, [c - 1 for c in columns]]
    if best is not None and best != "millideg" and viol_of.get("millideg") == 0 and np.all(sub == np.round(sub)):
        best = "millideg"
    return rows, best


MANIFEST_NAME = "MANIFEST.yaml"
MANIFEST_DEFAULTS = {"units": "auto", "period_s": DEFAULT_PERIOD_S, "period_status": "assumed",
                     "side": "L", "frame": "Arm_Tip", "columns": [1, 2, 3, 4, 5, 6, 7], "signs": [1, 1, 1, 1, 1, 1, 1]}
INTERPRETATION_KEYS = ("units", "period_s", "period_status", "side", "frame", "columns", "signs")


def load_manifest(directory) -> dict:
    """`MANIFEST.yaml` beside the teach files: `defaults:` plus `files:` keyed
    by file name. Missing file -> empty manifest (defaults only)."""
    import yaml
    p = pathlib.Path(directory) / MANIFEST_NAME
    if not p.is_file():
        return {"defaults": {}, "files": {}}
    with open(p) as fh:
        m = yaml.safe_load(fh) or {}
    if not isinstance(m, dict):
        raise TeachFileError(f"{p}: manifest must be a mapping")
    m.setdefault("defaults", {}); m.setdefault("files", {})
    if not isinstance(m["files"], dict):
        raise TeachFileError(f"{p}: 'files' must be a mapping of file name -> entry")
    return m


def interpretation_for(path, manifest: Optional[dict] = None, overrides: Optional[dict] = None) -> dict:
    """The kwargs `load_teach_file` needs for this file: built-in defaults,
    then the manifest's `defaults`, then its per-file entry, then explicit
    `overrides` (command-line flags; None values are ignored). Returns the
    interpretation plus `in_manifest` and `provenance` (the entry's other keys,
    carried into the profile)."""
    path = pathlib.Path(path)
    m = manifest if manifest is not None else load_manifest(path.parent)
    entry = m.get("files", {}).get(path.name) or {}
    out = dict(MANIFEST_DEFAULTS)
    for layer in (m.get("defaults") or {}, entry, {k: v for k, v in (overrides or {}).items() if v is not None}):
        for k in INTERPRETATION_KEYS:
            if k in layer:
                out[k] = layer[k]
    out["in_manifest"] = path.name in m.get("files", {})
    out["hardware"] = str(entry.get("hardware", (m.get("defaults") or {}).get("hardware", "unknown")))
    out["provenance"] = {k: v for k, v in entry.items() if k not in INTERPRETATION_KEYS}
    return out


def load_teach_file(path, *, units: str = "millideg",
                    columns: Sequence[int] = (1, 2, 3, 4, 5, 6, 7),
                    signs: Sequence[int] = (1, 1, 1, 1, 1, 1, 1),
                    period_s: float = DEFAULT_PERIOD_S, period_status: str = "assumed",
                    side: str = "L", field_name: str = "point",
                    limits_deg: Optional[Sequence[Tuple[float, float]]] = None,
                    allow_limit_violations: bool = False) -> TeachFile:
    """Read a teach file into degrees under an explicit interpretation.

    `units="auto"` takes `infer_units`' recommendation (the coarsest scale
    with no limit violation; needs `limits_deg`) and records the choice as
    a problem-free note in `TeachFile.units_source`. `limits_deg`, when
    given, is used to CHECK the chosen units (a violation is reported as a
    problem, never clamped) and to run `infer_units` so the caller can see
    whether the choice is the recommended one.
    """
    path = pathlib.Path(path)
    if units != "auto" and units not in UNIT_SCALES:
        raise TeachFileError(f"unknown units {units!r}; one of {sorted(UNIT_SCALES)} or 'auto'")
    if period_status not in ("assumed", "measured"):
        raise TeachFileError("period_status must be 'assumed' or 'measured'")
    side = str(side).upper()[0]
    if side not in ("L", "R"):
        raise TeachFileError("side must be L or R")
    columns = tuple(int(c) for c in columns); signs = tuple(int(x) for x in signs)
    if len(columns) != 7 or len(signs) != 7:
        raise TeachFileError("columns and signs must each name 7 joints (RM75)")
    points, problems = read_points(path, field_name)
    if not points:
        raise TeachFileError(f"{path}: no samples read ({len(problems)} problems)")
    raw = np.asarray(points, dtype=float)
    if raw.shape[0] < MIN_SAMPLES:
        raise TeachFileError(f"{path}: {raw.shape[0]} samples — fewer than {MIN_SAMPLES}, nothing to filter or differentiate")
    units_source = "given"
    if units == "auto":
        if limits_deg is None:
            raise TeachFileError("units='auto' needs limits_deg to rank the scales")
        _, best = infer_units(raw, limits_deg, columns)
        if best is None:
            raise TeachFileError(f"{path}: no candidate unit keeps every sample inside the joint limits")
        units, units_source = best, "inferred"
    scale = UNIT_SCALES[units]
    q_deg = to_degrees(raw, columns, signs, scale)
    if raw.shape[1] > max(columns):
        problems.append(f"file has {raw.shape[1]} values per sample; columns {tuple(columns)} are read as joints, "
                        f"the other {raw.shape[1] - 7} ignored — check the export format")
    if limits_deg is not None:
        lo = np.array([l for l, _ in limits_deg]); hi = np.array([h for _, h in limits_deg])
        viol = []
        for j in range(q_deg.shape[1]):
            under = int((q_deg[:, j] < lo[j]).sum()); over = int((q_deg[:, j] > hi[j]).sum())
            if under or over:
                viol.append(f"joint{j+1}: {under} under / {over} over the limit "
                            f"[{lo[j]:.1f}, {hi[j]:.1f}] deg under units={units}")
        _, best = infer_units(raw, limits_deg, columns)
        if best is not None and best != units:
            problems.append(f"units {units!r} chosen but infer_units recommends {best!r}")
        if viol and not allow_limit_violations:
            raise TeachFileError(f"{path.name}: joints outside the RM75 limits under units={units} — "
                                 + "; ".join(viol) + (f" (infer_units recommends {best!r})" if best and best != units else "")
                                 + "; the controller never records such a pose, so the units or the file are wrong "
                                 "(allow_limit_violations=True to analyse anyway)")
        problems.extend(viol)
    return TeachFile(path=path, sha256=sha256_of(path), raw=raw, q_deg=q_deg,
                     units=units, scale_rad_per_int=scale, columns=tuple(columns),
                     signs=tuple(signs), period_s=float(period_s),
                     period_status=period_status, side=side, problems=problems,
                     units_source=units_source)


# ── record-rate evidence ───────────────────────────────────────────────────

def beat_test(q_deg: np.ndarray, fs: float, *, fc_hz: float = 2.5,
              min_expected_deg: float = 0.15, band_hz: Tuple[float, float] = (5.0, 50.0)) -> dict:
    """Expose the 1x/2x step alternation a polled joint state shows when its
    source updates at a non-integer multiple of the poll rate.

    Two independent views:
    * a spectral line in the pooled |dq| signal (period = the beat);
    * the ratio r = dq / (v_smooth * dt): for clearly moving steps r clusters
      at two modes m1 < m2 with m2 ~ 2*m1 when the state advances by one or
      two quanta per poll. The modes are the two most prominent LOCAL maxima
      of the histogram (scipy find_peaks), the beat is declared only when
      m2/m1 is within [1.8, 2.2] and both clusters hold >= 10 % of the
      samples, and source/poll is estimated from the CLUSTER FRACTIONS —
      mean advance per poll = (f1*1 + f2*2)/(f1+f2) quanta — which is finer
      than the 0.05 bin centres (also reported as 1/m1).

    Nothing here gives the absolute period — only the ratio. `beat_detected
    = False` means "no 1x/2x beat found in the searched band", NOT that the
    derivatives are trustworthy: a source slower than the poll shows up as
    repeated rows (`largest_steps`), and a ratio above ~2 as 2x/3x steps this
    test does not look for.
    """
    from scipy.signal import butter, find_peaks, sosfiltfilt
    q = np.asarray(q_deg, dtype=float)
    n = q.shape[0]
    dt = 1.0 / fs
    dq = np.diff(q, axis=0)
    out: dict = {"fs_hz": fs, "n_steps": int(dq.shape[0]), "beat_detected": False,
                 "source_over_poll": None}

    # 1. spectral line in pooled |dq|
    x = np.abs(dq).sum(axis=1)
    x = x - x.mean()
    spec = np.abs(np.fft.rfft(x * np.hanning(x.size))) ** 2
    freqs = np.fft.rfftfreq(x.size, d=dt)
    lo, hi = band_hz[0], min(band_hz[1], 0.5 * fs * 0.999)
    band = (freqs >= lo) & (freqs <= hi)
    if band.any():
        k = int(np.argmax(spec[band]))
        f_line = float(freqs[band][k])
        p_line = float(spec[band][k])
        p_med = float(np.median(spec[band])) or 1e-300
        out.update(line_hz=f_line, line_period_samples=float(fs / f_line) if f_line else None,
                   line_prominence=p_line / p_med,
                   band_power_fraction=float(p_line / max(spec[band].sum(), 1e-300)),
                   line_window="hann")
    # 2. ratio test against the smoothed velocity
    if fc_hz < 0.5 * fs and n > 30:
        sos = butter(2, fc_hz, btype="low", fs=fs, output="sos")
        qs = sosfiltfilt(sos, q, axis=0)
        v = np.gradient(qs, dt, axis=0, edge_order=2)
        expected = 0.5 * (v[:-1] + v[1:]) * dt          # trapezoid: expected step per sample
        mask = np.abs(expected) > min_expected_deg
        r = dq[mask] / expected[mask]
        r = r[np.isfinite(r) & (r > -0.5) & (r < 3.5)]
        out["n_ratio_samples"] = int(r.size)
        if r.size > 50:
            edges = np.arange(0.0, 3.05, 0.05)
            hist, _ = np.histogram(r, bins=edges)
            centres = 0.5 * (edges[:-1] + edges[1:])
            idx, _ = find_peaks(hist, prominence=0.02 * r.size, distance=4)
            top = sorted(idx, key=lambda i: hist[i], reverse=True)[:2]
            modes = sorted(float(centres[i]) for i in top)
            fmid = float(((r > 0.9) & (r < 1.1)).mean())
            out.update(ratio_modes=modes, ratio_mean=float(r.mean()), frac_unimodal=fmid)

            def near(m):
                return float(((r >= m - 0.2) & (r <= m + 0.2)).mean())
            if len(modes) == 2 and modes[0] > 0.05:
                f1, f2 = near(modes[0]), near(modes[1])
                ratio = modes[1] / modes[0]
                out.update(frac_1x=f1, frac_2x=f2, mode_ratio=float(ratio))
                if 1.8 <= ratio <= 2.2 and f1 >= 0.10 and f2 >= 0.10 and (f1 + f2) >= 0.7:
                    out["beat_detected"] = True
                    out["source_over_poll"] = float((f1 * 1.0 + f2 * 2.0) / (f1 + f2))
                    out["source_over_poll_from_mode"] = float(1.0 / modes[0])
            else:
                out.update(frac_1x=near(1.0), frac_2x=near(2.0))
    return out


def raw_step_problems(q_deg: np.ndarray, period_s: float, qdot_max_deg_s: Sequence[float],
                      factor: float = 1.5) -> List[str]:
    """Single-sample steps that would exceed `factor` x the joint's speed
    ceiling at this period: a corrupt row, a splice, or the wrong period —
    the smoother would hide it, so it is reported before smoothing."""
    dq = np.abs(np.diff(np.asarray(q_deg, dtype=float), axis=0))
    out = []
    for j in range(dq.shape[1]):
        i = int(np.argmax(dq[:, j])); step = float(dq[i, j]); rate = step / period_s
        if rate > factor * float(qdot_max_deg_s[j]):
            out.append(f"joint{j+1}: raw step {step:.3f} deg between samples {i} and {i+1} = {rate:.0f} deg/s, "
                       f"> {factor}x its {qdot_max_deg_s[j]:.0f} deg/s ceiling — corrupt row, splice, or wrong period/units")
    return out


def largest_steps(raw: np.ndarray) -> dict:
    """Largest single-sample |dq| per joint (LSB) and how many exact repeats."""
    dq = np.diff(raw, axis=0)
    rep = int((np.abs(dq).sum(axis=1) == 0).sum())
    return {"max_abs_step_lsb": [int(v) for v in np.abs(dq).max(axis=0)],
            "p99_abs_step_lsb": [float(v) for v in np.percentile(np.abs(dq), 99, axis=0)],
            "consecutive_duplicate_rows": rep}
