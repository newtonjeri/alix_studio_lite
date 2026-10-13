"""C9's ladder gate — which rung a run is allowed to attempt.

    "a SIM truncation BLOCKS the REAL rung. This is C17's 1:1 rule doing
     precisely the work it was written for, and it is the cheapest gate in
     the ladder."

Two CLI invocations are two processes, so the gate has to be an artefact on
disk rather than session state. One append-only `ladder.jsonl` keyed by the
program FINGERPRINT: a SIM pass licenses REAL only for the identical commanded
motion, never for "that task" in general.

WHERE these land is NOT specified by the contract. C13 governs saved *plans*
(`realman_sim/`, `realman_real/`); a UDP run recording is a different artefact.
The ledger lives at `<repo>/Resource/runs/ladder.jsonl` — ratified by Newton
2026-08-24 ("move it to the Resource at repo root"), the same root `Resource/`
that holds the saved plans. `ALIX_RUNS_DIR` overrides. Nothing is removed from
`paths.plan_roots()`. Run RECORDINGS do not share that root by default: the
recorder writes cwd-relative `runs/<run_id>` (recording.py), so a row's
`run_dir` is relative to the dispatch cwd, not to this file — dispatch from the
repo root, or set `ALIX_RUNS_DIR` to put both under one root.

Until 2026-08-24 this resolved the root with `parents[4]`, which from this
file is `core/`, not the repo — so the ledger was being written to
`core/Resource/runs/ladder.jsonl` (moved here with its history). D8 exists for
exactly that shape: `parents[N]` breaks silently when a file moves.
"""

from __future__ import annotations

import json
import os
import pathlib
import time
from typing import Optional, Tuple

RUNGS = ("EMU", "SIM", "REAL")


def repo_root() -> pathlib.Path:
    """The alix repo root, by marker — never by counting `..` (D8).

    Shared with cli.py, which used to carry its own parents[4] copy."""
    here = pathlib.Path(__file__).resolve()
    for parent in here.parents:
        if (parent / "ARCHITECTURE.md").is_file() and (parent / "core").is_dir():
            return parent
    raise RuntimeError(f"repo root not found above {here}")


def runs_root(override: Optional[str] = None) -> pathlib.Path:
    if override:
        return pathlib.Path(override)
    env = os.environ.get("ALIX_RUNS_DIR")
    if env:
        return pathlib.Path(env)
    return repo_root() / "Resource" / "runs"


def _ledger(root: pathlib.Path) -> pathlib.Path:
    return pathlib.Path(root) / "ladder.jsonl"


def record_rung(root, *, fingerprint: str, rung: str, ok: bool,
                program_ok: bool, reason: str, run_dir: str, sdk: str,
                run_id: str, geometry_note: str = "") -> None:
    """Append one rung outcome. Append-only: history is the evidence.

    `ok` and `program_ok` are SEPARATE on purpose. C9 says an emulator
    *program* failure (a crash, a rejected command) gates SIM, while a
    *geometry* failure there does not — the emulator traced 236 % of the
    commanded path on the reference geometry and "cannot validate arc
    geometry", so its geometry verdict is a known limitation, not evidence.
    """
    path = _ledger(root)
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "a") as fh:
        fh.write(json.dumps({
            "t": time.strftime("%Y-%m-%dT%H:%M:%S"), "fingerprint": fingerprint,
            "rung": rung, "ok": bool(ok), "program_ok": bool(program_ok),
            "reason": reason, "run_dir": str(run_dir), "sdk": sdk,
            "run_id": run_id, "geometry_note": geometry_note,
        }) + "\n")


def latest(root, fingerprint: str, rung: str) -> Optional[dict]:
    """The most recent record for this exact commanded motion at this rung."""
    path = _ledger(root)
    if not path.exists():
        return None
    found = None
    for line in path.read_text().splitlines():
        try:
            row = json.loads(line)
        except Exception:
            continue
        if row.get("fingerprint") == fingerprint and row.get("rung") == rung:
            found = row
    return found


def gate_sim(root, fingerprint: str) -> Tuple[bool, str]:
    """May this program attempt SIM? Blocked only by an EMU PROGRAM failure.

    An ABSENT emulator record does not block. C9 says an emulator program
    failure gates SIM; it does not require an emulator run to exist, and
    inventing that requirement would be adding to the contract.
    """
    row = latest(root, fingerprint, "EMU")
    if row is None:
        return True, "no emulator record for this program — not required by C9"
    if row.get("program_ok") is False:
        return False, (f"the emulator rung failed as a PROGRAM "
                       f"({row.get('reason') or 'no reason recorded'}, "
                       f"run {row.get('run_id')}) — C9 gates SIM on that")
    return True, f"emulator rung ran as a program (run {row.get('run_id')})"


def gate_real(root, fingerprint: str, *, override_reason: Optional[str] = None
              ) -> Tuple[bool, str, Optional[str]]:
    """May this program attempt REAL? `(allowed, why, sim_run_id)`.

    A SIM record that FAILED is the measured truncation C9 says blocks REAL,
    and no override lifts it. An ABSENT SIM record is a different thing — a
    missing rung, not a failed one — and may be overridden with a recorded
    reason, because a hard block with no escape hatch gets bypassed by editing
    the source, which leaves no record at all.
    """
    row = latest(root, fingerprint, "SIM")
    if row is not None and row.get("ok") is False:
        return False, (f"the SIM rung for this exact program TRUNCATED "
                       f"({row.get('reason')}, run {row.get('run_id')}) — C9: a "
                       "SIM truncation blocks REAL, and no override lifts it"), \
            row.get("run_id")
    if row is not None and row.get("sdk") == "emulated":
        return False, ("the only SIM record for this program came from the "
                       "EMULATOR, which is not the SIM rung — C9's ladder needs "
                       "SIM on hardware"), row.get("run_id")
    if row is not None:
        return True, f"SIM passed for this exact program (run {row.get('run_id')})", \
            row.get("run_id")
    if override_reason:
        return True, (f"no SIM record for this program; proceeding on an "
                      f"explicit override: {override_reason}"), None
    return False, ("no SIM record for this exact commanded motion — run "
                   "--mode SIM first, or pass --allow-real-without-sim REASON"), None
