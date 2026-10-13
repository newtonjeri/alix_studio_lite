"""Write an optimised file with its record.

``<name>.txt`` holds the track; ``<name>.lite.json`` beside it records how it was made (the
optimiser, every setting, the library versions, the recording's sha256), what it achieved,
every check value, and the sha256 of the track bytes, so the record is tied to exactly this
file.
"""
from __future__ import annotations

import json
from datetime import datetime, timezone
from pathlib import Path

from ..motion import track
from .types import OptResult

SIDECAR_SUFFIX = ".lite.json"
SIDECAR_VERSION = 1


def sidecar_path(track_path) -> Path:
    p = Path(track_path)
    return p.with_name(p.stem + SIDECAR_SUFFIX)


def record(result: OptResult, track_sha256: str) -> dict:
    return {
        "lite_sidecar": SIDECAR_VERSION,
        "written_utc": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "track_sha256": track_sha256,
        "result": result.summary(),
        "checks": result.checks.to_dict(),
        "recipe": result.recipe,
    }


def write_result(result: OptResult, out_path) -> tuple[Path, Path]:
    out_path = Path(out_path)
    data = track.track_bytes(result.md)
    track.write_md(out_path, result.md)
    side = sidecar_path(out_path)
    tmp = side.with_name(side.name + ".tmp")
    tmp.write_text(json.dumps(record(result, track.sha256_bytes(data)), indent=2, default=float), encoding="utf-8")
    tmp.replace(side)
    return out_path, side
