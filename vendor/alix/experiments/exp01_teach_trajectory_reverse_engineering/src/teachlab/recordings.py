"""Loaders for recorded runs — the artefacts Phase C compares.

A run directory holds `stream.csv` (the UDP push, one row per frame, the
production RunRecorder header verbatim: t_mono, position1..7 deg, speed1..7
deg/s, current/voltage/temperature 1..7, en1..7, err1..7, lift_*, tcp_x/y/z m,
tcp_rx/ry/rz rad, arm_status) and `run.json` (metadata; `stages` carry
t_start/t_end in the same relative clock as t_mono). Runs written by the
exp01 recorder add `command.csv` when frames were streamed: one row per
commanded frame with its deadline, send time, send latency, joints and the
SDK return code.
"""
from __future__ import annotations

import csv
import json
import pathlib
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np


@dataclass
class Recording:
    dir: pathlib.Path
    run_id: str
    meta: dict
    header: List[str]
    t: np.ndarray                       # (N,) seconds, host arrival, relative
    pos_deg: np.ndarray                 # (N, 7)
    speed_deg_s: Optional[np.ndarray]   # (N, 7) or None when never populated
    en: Optional[np.ndarray]            # (N, 7) 0/1
    err: Optional[np.ndarray]           # (N, 7)
    tcp: Optional[np.ndarray]           # (N, 6) controller World pose
    arm_status: Optional[np.ndarray]    # (N,)
    columns: Dict[str, np.ndarray] = field(default_factory=dict)

    @property
    def n(self) -> int:
        return int(self.t.size)

    @property
    def fs_hz(self) -> float:
        """Nominal rate from the median POSITIVE inter-arrival time. Frames a
        coalescing reader stamps identically (bursts) contribute zeros that
        would otherwise make the median 0 and the rate infinite."""
        dt = np.diff(self.t)
        dt = dt[dt > 0]
        return float(1.0 / np.median(dt)) if dt.size else float("nan")

    def interarrival_stats_ms(self) -> dict:
        dt = np.diff(self.t) * 1e3
        if dt.size == 0:
            return {"n": 0}
        return {"n": int(dt.size), "mean": float(dt.mean()), "p50": float(np.median(dt)),
                "p95": float(np.percentile(dt, 95)), "p99": float(np.percentile(dt, 99)),
                "max": float(dt.max()), "min": float(dt.min()),
                "frac_over_1p5x_median": float((dt > 1.5 * np.median(dt)).mean())}

    def stages(self) -> List[dict]:
        return list(self.meta.get("stages") or [])

    def stage_names(self) -> List[str]:
        return [str(st.get("stage")) for st in self.stages() if st.get("stage") is not None]

    def stages_ok(self, names: Sequence[str]) -> Optional[bool]:
        """False when any selected stage carries ok=False (the SDK call
        failed); None when no stage carries an ok field."""
        flags = [st.get("ok") for st in self.stages() if st.get("stage") in names and "ok" in st]
        if not flags:
            return None
        return all(bool(x) for x in flags)

    def window(self, stages: Optional[Sequence[str]] = None, t0: Optional[float] = None,
               t1: Optional[float] = None) -> np.ndarray:
        """Boolean mask over samples: inside the named stages (any of them), or
        inside [t0, t1], or everything."""
        m = np.ones(self.n, dtype=bool)
        if stages:
            m[:] = False
            for st in self.stages():
                if st.get("stage") in stages and st.get("t_start") is not None and st.get("t_end") is not None:
                    m |= (self.t >= float(st["t_start"])) & (self.t <= float(st["t_end"]))
        if t0 is not None:
            m &= self.t >= t0
        if t1 is not None:
            m &= self.t <= t1
        return m


def _find_stream(run_dir: pathlib.Path, side: Optional[str]) -> pathlib.Path:
    cands = []
    if side:
        cands.append(run_dir / f"stream_{side}.csv")
    cands += [run_dir / "stream.csv"] + sorted(run_dir.glob("stream_*.csv"))
    for c in cands:
        if c.is_file():
            return c
    raise FileNotFoundError(f"no stream.csv under {run_dir}")


def load_recording(run_dir, side: Optional[str] = None) -> Recording:
    run_dir = pathlib.Path(run_dir)
    meta = {}
    rj = run_dir / "run.json"
    if rj.is_file():
        meta = json.loads(rj.read_text())
    path = _find_stream(run_dir, side)
    with open(path, newline="") as fh:
        rd = csv.reader(fh)
        header = next(rd)
        rows = [r for r in rd if r]
    if not rows:
        raise ValueError(f"{path}: no samples")
    arr = np.array([[float(x) for x in r] for r in rows], dtype=float)
    col = {name: arr[:, i] for i, name in enumerate(header)}

    def block(prefix, n=7):
        names = [f"{prefix}{i}" for i in range(1, n + 1)]
        if all(nm in col for nm in names):
            return np.stack([col[nm] for nm in names], axis=1)
        return None
    pos = block("position")
    if pos is None:
        raise ValueError(f"{path}: no position1..7 columns")
    speed = block("speed")
    if speed is not None and not np.any(speed != 0):
        speed = None                       # never populated (SIM / emulator)
    tcp = None
    if all(k in col for k in ("tcp_x", "tcp_y", "tcp_z", "tcp_rx", "tcp_ry", "tcp_rz")):
        tcp = np.stack([col[k] for k in ("tcp_x", "tcp_y", "tcp_z", "tcp_rx", "tcp_ry", "tcp_rz")], axis=1)
    return Recording(dir=run_dir, run_id=meta.get("run_id", run_dir.name), meta=meta, header=header,
                     t=col["t_mono"], pos_deg=pos, speed_deg_s=speed, en=block("en"), err=block("err"),
                     tcp=tcp, arm_status=col.get("arm_status"), columns=col)


COMMAND_HEADER = ["i", "t_deadline", "t_sent", "send_latency_ms", "call_ms"] + [f"cmd{j}" for j in range(1, 8)] + ["ret"]


@dataclass
class CommandLog:
    t_deadline: np.ndarray
    t_sent: np.ndarray
    q_deg: np.ndarray
    ret: np.ndarray
    send_latency_ms: np.ndarray
    call_ms: np.ndarray

    @property
    def n(self) -> int:
        return int(self.t_sent.size)

    def period_stats_ms(self) -> dict:
        d = np.diff(self.t_sent) * 1e3
        if d.size == 0:
            return {"n": 0}
        return {"n": int(d.size), "mean": float(d.mean()), "p50": float(np.median(d)),
                "p99": float(np.percentile(d, 99)), "max": float(d.max()),
                "late_frames": int((self.send_latency_ms > 0.5 * np.median(d)).sum()),
                "max_late_ms": float(self.send_latency_ms.max())}

    def max_step_deg(self) -> list:
        return [float(v) for v in np.abs(np.diff(self.q_deg, axis=0)).max(axis=0)] if self.n > 1 else [0.0] * 7


def load_command_csv(path) -> CommandLog:
    path = pathlib.Path(path)
    with open(path, newline="") as fh:
        rd = csv.reader(fh)
        header = next(rd)
        rows = [[float(x) for x in r] for r in rd if r]
    a = np.array(rows, dtype=float)
    ix = {h: i for i, h in enumerate(header)}
    return CommandLog(t_deadline=a[:, ix["t_deadline"]], t_sent=a[:, ix["t_sent"]],
                      q_deg=np.stack([a[:, ix[f"cmd{j}"]] for j in range(1, 8)], axis=1),
                      ret=a[:, ix["ret"]], send_latency_ms=a[:, ix["send_latency_ms"]],
                      call_ms=a[:, ix["call_ms"]])


def find_command_csv(run_dir) -> Optional[pathlib.Path]:
    p = pathlib.Path(run_dir) / "command.csv"
    return p if p.is_file() else None
