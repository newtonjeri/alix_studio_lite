"""Read and write drag-teach tracks.

A track is one JSON object per line, ``{"point":[j1,...,j7]}``, in integer millidegrees. The
teach pendant records one line every 5 ms and writes CRLF line endings; Lite writes the same
format. The control program's runner accepts LF or CRLF and builds the controller's project
file itself.
"""
from __future__ import annotations

import hashlib
import os
from pathlib import Path

import numpy as np

EOL = b"\r\n"
DOF = 7


def read_md(path) -> np.ndarray:
    """(N, 7) int64 millidegrees, through ALIX's strict upload parser, so any file Lite can
    read is one the upload path accepts. Raises ``ValueError`` on a malformed line."""
    from alix_dispatch import track as tr

    return np.asarray(tr.parse_track(Path(path)), dtype=np.int64)


def duration_s(lines) -> float | None:
    """A track's execution time as Lite states it everywhere (the owner, 20 Oct: the time travels with the
    trajectory): (lines - 1) x 5 ms, from its first sample to its last on the 5 ms clock, the same count as the
    optimiser's achieved time; None without a line count."""
    try:
        n = int(lines)
    except (TypeError, ValueError):
        return None
    return round(0.005 * (n - 1), 3) if n > 0 else None


def to_deg(md) -> np.ndarray:
    return np.asarray(md, dtype=float) / 1000.0


def to_md(q_deg) -> np.ndarray:
    """Degrees to integer millidegrees (``np.round``, as ALIX's emitters do). Manual tuning works
    on the integers themselves with JavaScript's rounding (``optimise.manual.js_round``)."""
    return np.round(np.asarray(q_deg, dtype=float) * 1000.0).astype(np.int64)


def track_bytes(md) -> bytes:
    rows = np.asarray(md)
    if rows.ndim != 2 or rows.shape[1] != DOF:
        raise ValueError(f"a track is (N, {DOF}) integers, got shape {rows.shape}")
    if rows.shape[0] < 2:
        raise ValueError(f"{rows.shape[0]} point(s) is not a track")
    if not np.issubdtype(rows.dtype, np.integer):
        raise ValueError("a track holds integer millidegrees; round first (track.to_md)")
    return b"".join(b'{"point":[' + ",".join(str(int(v)) for v in row).encode() + b"]}" + EOL
                    for row in rows)


def write_md(path, md) -> Path:
    """Write atomically (temporary file, then rename), so a reader never sees half a track."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    data = track_bytes(md)
    tmp = path.with_name(path.name + ".tmp")
    tmp.write_bytes(data)
    os.replace(tmp, path)
    return path


def sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def sha256_file(path) -> str:
    return sha256_bytes(Path(path).read_bytes())
