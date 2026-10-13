"""The limits every optimised file must pass, judged on its integer millidegrees as written,
at the 5 ms controller clock.

Hard limits (a file that fails one cannot be uploaded):

- L1 joint limits (ALIX's G1): every line stays at least 0.5 deg inside each joint's range, and
  at every turn the overshoot ALIX's execution model predicts, max(floor, tau x arrival speed),
  still leaves 0.5 deg (arrival speeds read at the 5 ms cycle);
- L2 speed: the 20 Hz band-limited joint speed stays within 0.97 x the joint's rating;
- L3 acceleration: the 20 Hz band-limited joint acceleration stays within 600 deg/s^2 x 1.02
  (ALIX's reading, ``plan_min_time.band_limited_util``: raw finite differences of a point list are
  meaningless — one LSB at 5 ms is 2.26 rad/s^2, and a recorder's skipped sample, which doubles one
  step, reads as over 1000 deg/s^2 on recordings that run cleanly);
- L4 step: at most 10 deg between two lines;
- L5 path accuracy (ALIX's G4): the contact pad stays within 10 mm (or the region's MAX DEV)
  of the taught path, measured both ways against the raw recording;
- L6 start and end: the first and last lines are within 0.05 deg of the recording's.

Checks (shown, never blocking):

- C1 predicted path departure (ALIX's G5). ALIX's model reads 33-109 mm on the raw commode C
  recordings themselves, which ran fine on the robot, so it cannot gate a file;
- C2 jerk (band-limited, against 1500 rad/s^3);
- C3 hand speed (the contact pad's top speed);
- C4 raw line speed (lines whose unfiltered speed is over a rating; a warning risk).

L2, L3, L4 and the speed part of L1 depend on how fast the file plays, and ``motion_ok`` is
the quick test the time search uses. The joint-limit floor of L1, L5 and L6 do not change with
speed: no timing can fix them.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass, field

import numpy as np

from . import track
from .limits import LimitSet

# ───────────────────────────────────────────────────────────── instruments


def bandlimit(q: np.ndarray, dt: float, fc_hz: float = 20.0) -> np.ndarray:
    """4th-order zero-phase Butterworth low-pass, as ALIX reads every rate and acceleration
    (one LSB of jitter at this clock is 2.26 rad/s^2 raw). Short series pass unchanged."""
    from scipy.signal import butter, filtfilt

    q = np.asarray(q, dtype=float)
    if q.shape[0] <= 30:
        return q
    b, a = butter(4, min(fc_hz / (0.5 / float(dt)), 0.99), "low")
    return filtfilt(b, a, q, axis=0)


def speed_deg_s(q: np.ndarray, dt: float) -> np.ndarray:
    """(N-1, 7) band-limited joint speed magnitudes."""
    return np.abs(np.diff(bandlimit(q, dt), axis=0)) / float(dt)


def acc_deg_s2(q: np.ndarray, dt: float) -> np.ndarray:
    """(N-2, 7) band-limited joint acceleration magnitudes, as ALIX reads them
    (``plan_min_time.band_limited_util``, finding F99c): the 20 Hz zero-phase low-pass first,
    then the second difference at the 5 ms clock. Row i is centred on line i + 2 (1-based)."""
    q = np.asarray(q, dtype=float)
    if q.shape[0] < 3:
        return np.zeros((0, q.shape[1] if q.ndim == 2 else 7))
    return np.abs(np.diff(bandlimit(q, dt), n=2, axis=0)) / float(dt) ** 2


def commanded_margin_deg(q: np.ndarray, limits: LimitSet) -> np.ndarray:
    lim = np.asarray(limits.pos_deg, dtype=float)
    return np.minimum(q - lim[:, 0], lim[:, 1] - q)


def joint_turns(q: np.ndarray, limits: LimitSet) -> list[dict]:
    """ALIX's G1 at every commanded turn of every joint (``taught_tracks.path_reversals``),
    read at the 5 ms cycle. Each turn gets:

    - ``overshoot_deg``: max(floor, tau_limit x arrival speed);
    - ``exec_margin_deg``: commanded margin minus that overshoot;
    - ``geometry``: True if even the floor overshoot breaks the margin (no speed fixes it);
    - ``util``: (overshoot + eps) / commanded margin, above 1 fails.
    """
    import taught_tracks as tt

    m = limits.model
    floor, tau_l, eps = float(m["floor_deg"]), float(m["tau_limit_s"]), float(m["limit_eps_deg"])
    out = []
    for r in tt.path_reversals(q, limits.dt_gate, m):
        margin = float(r["margin_deg"])
        over = max(floor, tau_l * float(r["v_deg_s"]))
        out.append(dict(line=int(r["k"]), joint=int(r["j"]) + 1, arrival_deg_s=float(r["v_deg_s"]),
                        margin_deg=margin, overshoot_deg=over, exec_margin_deg=margin - over,
                        geometry=bool(margin - floor < eps),
                        util=float((over + eps) / margin) if margin > 0 else float("inf")))
    return out


# ─────────────────────────────────────────────────────── the quick test


def motion_ok(md: np.ndarray, limits: LimitSet) -> bool:
    """True if L2, L3, L4 and the speed part of L1 pass: the test the time search runs at
    every candidate length. Speed-independent failures are left to ``check_bytes``."""
    q = track.to_deg(md)
    dt = limits.dt_gate
    if np.abs(np.diff(q, axis=0)).max(initial=0.0) > limits.step_deg:
        return False
    rating = np.asarray(limits.rating_deg_s, dtype=float)
    if (speed_deg_s(q, dt) > limits.rate_bar * rating).any():
        return False
    if (acc_deg_s2(q, dt) > limits.acc_bar_deg_s2).any():
        return False
    return all(t["util"] <= 1.0 for t in joint_turns(q, limits) if not t["geometry"])


def util_series(md: np.ndarray, limits: LimitSet) -> dict[str, np.ndarray]:
    """Per-line utilisation of every speed-dependent hard limit (1.0 = at the bar), for the
    binding-limit report and the plots. Keys: ``speed_J1``.., ``acceleration_J1``..,
    ``joint_limit_J1``.."""
    q = track.to_deg(md)
    n = q.shape[0]
    dt = limits.dt_gate
    rating = np.asarray(limits.rating_deg_s, dtype=float)
    out: dict[str, np.ndarray] = {}
    sp = speed_deg_s(q, dt) / (limits.rate_bar * rating)            # interval i -> line i+1
    acc = acc_deg_s2(q, dt) / limits.acc_bar_deg_s2
    a0 = 1                                                           # row i is centred on line i + 1 (0-based)
    turns = joint_turns(q, limits)
    for j in range(7):
        s = np.zeros(n)
        s[1:] = sp[:, j]
        out[f"speed_J{j + 1}"] = s
        a = np.zeros(n)
        if acc.shape[0]:
            a[a0:a0 + acc.shape[0]] = acc[:, j][: max(0, n - a0)]
        out[f"acceleration_J{j + 1}"] = a
        g = np.zeros(n)
        for t in turns:
            if t["joint"] == j + 1 and not t["geometry"]:
                g[t["line"]] = max(g[t["line"]], min(t["util"], 1e3))
        out[f"joint_limit_J{j + 1}"] = g
    return out


# ─────────────────────────────────────────────────────── the full report


@dataclass
class Check:
    id: str                     # L1..L6, C1..C4
    name: str                   # plain-language label
    hard: bool
    passed: bool | None         # None: not measured
    value: float | None = None
    limit: float | None = None
    unit: str = ""
    detail: str = ""
    joint: int | None = None
    line: int | None = None
    speed_fixes: bool | None = None   # for a failure: would slowing down fix it?


@dataclass
class CheckReport:
    checks: list[Check] = field(default_factory=list)
    notes: list[str] = field(default_factory=list)

    @property
    def passed(self) -> bool:
        return all(c.passed is not False for c in self.checks if c.hard)

    def failures(self) -> list[Check]:
        return [c for c in self.checks if c.hard and c.passed is False]

    def warnings(self) -> list[Check]:
        return [c for c in self.checks if not c.hard and c.passed is False]

    def get(self, cid: str) -> Check | None:
        return next((c for c in self.checks if c.id == cid), None)

    def to_dict(self) -> dict:
        def plain(c: Check) -> dict:                       # JSON-safe: no numpy scalars
            d = asdict(c)
            d["passed"] = None if c.passed is None else bool(c.passed)
            for k in ("value", "limit"):
                if d[k] is not None:
                    d[k] = float(d[k])
            for k in ("joint", "line"):
                if d[k] is not None:
                    d[k] = int(d[k])
            if d["speed_fixes"] is not None:
                d["speed_fixes"] = bool(d["speed_fixes"])
            return d

        return {"passed": bool(self.passed), "checks": [plain(c) for c in self.checks],
                "notes": list(self.notes)}


def _fmt(x: float, nd: int = 2) -> float:
    return float(round(float(x), nd))


def check_bytes(md: np.ndarray, limits: LimitSet, *, raw_md: np.ndarray | None = None,
                with_departure: bool = True) -> CheckReport:
    """Every hard limit and check on one file. ``raw_md`` is the recording the file was made
    from (L5 and L6 need it; without it they are reported as not measured)."""
    import taught_tracks as tt

    md = np.asarray(md)
    q = track.to_deg(md)
    dt = limits.dt_gate
    rating = np.asarray(limits.rating_deg_s, dtype=float)
    rep = CheckReport()

    # L1 joint limits
    marg = commanded_margin_deg(q, limits)
    i_min = np.unravel_index(int(np.argmin(marg)), marg.shape)
    min_margin = float(marg[i_min])
    eps = float(limits.model["limit_eps_deg"])
    turns = joint_turns(q, limits)
    geometric = [t for t in turns if t["geometry"]]
    worst = min(geometric or turns, key=lambda t: t["exec_margin_deg"], default=None)
    if min_margin < eps:
        rep.checks.append(Check(
            "L1", "Joint limits", True, False, _fmt(min_margin), eps, "deg",
            f"joint {i_min[1] + 1} is commanded {min_margin:.2f} deg from its end stop at line "
            f"{i_min[0] + 1}; it must stay at least {eps:g} deg inside. Re-teach this part.",
            joint=int(i_min[1]) + 1, line=int(i_min[0]) + 1, speed_fixes=False))
    elif worst is not None and worst["exec_margin_deg"] < eps:
        geo = worst["geometry"]
        rep.checks.append(Check(
            "L1", "Joint limits", True, False, _fmt(worst["exec_margin_deg"]), eps, "deg",
            (f"joint {worst['joint']} turns {worst['margin_deg']:.2f} deg from its end stop at line "
             f"{worst['line'] + 1}; even a slow arrival overshoots into the last {eps:g} deg. "
             f"Re-teach this part.") if geo else
            (f"joint {worst['joint']} arrives at a turn {worst['margin_deg']:.2f} deg from its end "
             f"stop at {worst['arrival_deg_s']:.0f} deg/s (line {worst['line'] + 1}); the predicted "
             f"overshoot of {worst['overshoot_deg']:.2f} deg leaves less than {eps:g} deg. "
             f"A slower file fixes it."),
            joint=worst["joint"], line=worst["line"] + 1, speed_fixes=not geo))
    else:
        em = None if worst is None else worst["exec_margin_deg"]
        rep.checks.append(Check(
            "L1", "Joint limits", True, True, _fmt(min_margin if em is None else min(em, min_margin)),
            eps, "deg",
            f"closest approach {min_margin:.1f} deg (joint {i_min[1] + 1})"
            + ("" if em is None else f"; after the predicted overshoot at turns, {em:.1f} deg"),
            joint=int(i_min[1]) + 1, line=int(i_min[0]) + 1))
    if min_margin < limits.margin_warn_deg:
        rep.notes.append(f"Joint {i_min[1] + 1} comes within {min_margin:.1f} deg of its end stop "
                         f"(line {i_min[0] + 1}).")

    # L2 speed
    sp = speed_deg_s(q, dt)
    u = sp / (limits.rate_bar * rating)
    k = np.unravel_index(int(np.argmax(u)), u.shape) if u.size else (0, 0)
    peak = float(u[k]) if u.size else 0.0
    rep.checks.append(Check(
        "L2", "Joint speed", True, peak <= 1.0, _fmt(peak, 3), 1.0, "x limit",
        f"fastest: joint {k[1] + 1} at {sp[k]:.0f} deg/s (limit {limits.rate_bar * rating[k[1]]:.0f}) "
        f"at line {k[0] + 2}" if u.size else "",
        joint=int(k[1]) + 1, line=int(k[0]) + 2, speed_fixes=True if peak > 1.0 else None))

    # L3 acceleration
    acc = acc_deg_s2(q, dt)
    if acc.size:
        k = np.unravel_index(int(np.argmax(acc)), acc.shape)
        a_peak = float(acc[k])
        rep.checks.append(Check(
            "L3", "Joint acceleration", True, a_peak <= limits.acc_bar_deg_s2, _fmt(a_peak, 1),
            _fmt(limits.acc_bar_deg_s2, 1), "deg/s^2",
            f"highest: joint {k[1] + 1} at {a_peak:.0f} deg/s^2 (20 Hz band-limited) around line {k[0] + 2} "
            f"(hardware limit {limits.acc_deg_s2:.0f} deg/s^2)",
            joint=int(k[1]) + 1, line=int(k[0]) + 2,
            speed_fixes=True if a_peak > limits.acc_bar_deg_s2 else None))
    else:
        rep.checks.append(Check("L3", "Joint acceleration", True, True, 0.0,
                                _fmt(limits.acc_bar_deg_s2, 1), "deg/s^2", "file shorter than three lines"))

    # L4 step
    d = np.abs(np.diff(q, axis=0))
    k = np.unravel_index(int(np.argmax(d)), d.shape)
    rep.checks.append(Check(
        "L4", "Step size", True, float(d[k]) <= limits.step_deg, _fmt(d[k], 3), limits.step_deg, "deg",
        f"largest: joint {k[1] + 1}, {d[k]:.3f} deg between lines {k[0] + 1} and {k[0] + 2}",
        joint=int(k[1]) + 1, line=int(k[0]) + 2,
        speed_fixes=True if float(d[k]) > limits.step_deg else None))

    # L5 path accuracy and L6 ends, against the recording
    if raw_md is not None:
        raw = track.to_deg(raw_md)
        dev = tt.deviation_mm(raw, q, limits.side, limits.frame)
        dmax = float(dev["max_mm"])
        rep.checks.append(Check(
            "L5", "Path accuracy", True, dmax <= limits.max_dev_mm, _fmt(dmax), limits.max_dev_mm, "mm",
            f"the hand stays within {dmax:.1f} mm of the taught path (p95 "
            f"{max(dev['new_to_taught_mm']['p95_mm'], dev['taught_to_new_mm']['p95_mm']):.1f} mm)",
            speed_fixes=False if dmax > limits.max_dev_mm else None))
        e0 = float(np.abs(q[0] - raw[0]).max())
        e1 = float(np.abs(q[-1] - raw[-1]).max())
        e = max(e0, e1)
        rep.checks.append(Check(
            "L6", "Start and end poses", True, e <= limits.ends_tol_deg, _fmt(e, 4), limits.ends_tol_deg,
            "deg", f"start differs by {e0:.4f} deg, end by {e1:.4f} deg from the recording",
            speed_fixes=False if e > limits.ends_tol_deg else None))
    else:
        rep.checks.append(Check("L5", "Path accuracy", True, None, detail="no recording given"))
        rep.checks.append(Check("L6", "Start and end poses", True, None, detail="no recording given"))

    # C1 predicted departure (ALIX G5), estimate only
    if with_departure:
        try:
            mp = tt.model_predictions(q, limits.side, limits.frame, limits.model, limits.dt_gate,
                                      limits.max_dev_mm)
            g5 = float(mp["pred_line_dev_max_mm"])
            rep.checks.append(Check(
                "C1", "Predicted path departure (estimate)", False, g5 <= limits.max_dev_mm, _fmt(g5),
                limits.max_dev_mm, "mm",
                f"ALIX's upload model predicts up to {g5:.0f} mm (p95 {mp['pred_line_dev_p95_mm']:.0f} mm). "
                "It over-reads on taught paths, so it is shown, not enforced."))
        except Exception as exc:                                             # noqa: BLE001
            rep.checks.append(Check("C1", "Predicted path departure (estimate)", False, None,
                                    detail=f"not available: {exc!r}"))

    # C2 jerk
    qf = bandlimit(q, dt)
    jerk = np.abs(np.diff(qf, n=3, axis=0)) / dt ** 3 if q.shape[0] > 3 else np.zeros((1, 7))
    jref = np.degrees(limits.jerk_ref_rad_s3)
    jp = float(jerk.max()) if jerk.size else 0.0
    rep.checks.append(Check("C2", "Jerk", False, bool(jp <= jref), _fmt(jp / jref, 2), 1.0, "x reference",
                            f"peak {jp:.0f} deg/s^3 against the {limits.jerk_ref_rad_s3:g} rad/s^3 "
                            "reference (reported only)"))

    # C3 hand speed
    try:
        P = tt.pad_path_m(q, limits.side, limits.frame)
        vp = np.linalg.norm(np.diff(P, axis=0), axis=1) / dt
        rep.checks.append(Check("C3", "Hand speed", False, True, _fmt(vp.max(), 3), None, "m/s",
                                f"top speed {vp.max():.2f} m/s, typical {np.percentile(vp, 50):.2f} m/s"))
    except Exception as exc:                                                 # noqa: BLE001
        rep.checks.append(Check("C3", "Hand speed", False, None, detail=f"not available: {exc!r}"))

    # C4 raw line speed
    raw_over = (d / dt > rating).any(axis=1)
    n_over = int(raw_over.sum())
    rep.checks.append(Check("C4", "Raw line speed", False, n_over == 0, float(n_over), 0.0, "lines",
                            f"{n_over} line(s) step faster than a joint rating before filtering"
                            if n_over else "no line steps faster than a joint rating"))
    return rep
