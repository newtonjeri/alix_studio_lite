"""The UDP run recording — C9's evidence, and the only evidence there is.

C9 makes this mandatory, not optional: "A driver that skips the recording check
on SIM is NON-CONFORMING: the distance and final-waypoint checks MUST run in
SIM even though the joint channel is dead." SIM cannot clear a rung on joint
load — it can only fail one — but it IS exact on path completion, and a SIM
truncation BLOCKS the REAL rung.

Ported from `run_recorder.py` and deliberately writing the SAME `stream.csv`
schema, so the existing RM_API2 analysers read alix recordings unchanged
(C17's 1:1 rule). Three things here that upstream does not need:

1. **Per-arm demux.** Upstream records one arm; a dual run needs both, and
   C9's gate is per-arm. The realtime callback is PROCESS-GLOBAL, so one
   callback fans out by the frame's own source IP.
2. **Frame verification.** `rm_set_realtime_push` returns 0 for a target host
   IP that is not yours and then delivers nothing. `start()` proves frames are
   actually arriving instead of trusting that return.
3. **Emulated output is redirected** so it can never land beside recorded
   hardware data.
"""

from __future__ import annotations

import json
import math
import os
import pathlib
import threading
import time
from typing import Dict, List, Optional, Tuple

_JOINT_FIELDS = ("joint_position", "joint_speed", "joint_current",
                 "joint_voltage", "joint_temperature")

#: How long `start()` waits for the first frame before calling the push dead.
FRAME_PROOF_TIMEOUT_S = 2.0


class RecordingError(RuntimeError):
    """The recording could not be started, or produced no usable evidence."""


class RunRecorder:
    """Captures the UDP stream for one run, marked by stage, per arm."""

    #: The SDK's realtime callback is one per PROCESS, so one recorder at a
    #: time. A second would silently steal the stream from the first.
    _active = None

    def __init__(self, robots: Dict[str, object], *, task: str,
                 host_ips: Dict[str, Optional[str]],
                 arm_ips: Dict[str, str],
                 udp_port: int = 8089, cycle: int = 2,
                 out_dir: Optional[str] = None, emulated: bool = False,
                 run_id: Optional[str] = None) -> None:
        self.robots = dict(robots)
        self.task = task
        self.host_ips = dict(host_ips)
        self.arm_ips = dict(arm_ips)
        self.udp_port = int(udp_port)
        self.cycle = int(cycle)
        self.emulated = bool(emulated)
        self.run_id = run_id or f"{time.strftime('%Y%m%dT%H%M%S')}_{task}"
        self.out_dir = out_dir
        #: side -> IP, for demultiplexing frames by their source.
        self._by_ip = {ip: side for side, ip in self.arm_ips.items()}
        self._lock = threading.Lock()
        self._rows: Dict[str, List[list]] = {s: [] for s in self.robots}
        self._dropped = 0
        self._unattributed: Dict[str, int] = {}
        self._first_error: Optional[str] = None
        self._t0: Optional[float] = None
        self.stages: List[dict] = []
        self.meta: Dict[str, object] = {}
        # The ctypes callback must stay referenced for the process lifetime:
        # letting it be collected crashes the SDK's receive thread.
        self._cb = None

    @property
    def dir(self) -> pathlib.Path:
        """Where this run lands — resolved LATE, because `emulated` is not
        known at construction.

        `emulated` is set by the session once the handle exists and its class
        module can be read, so a directory computed in `__init__` was always
        the hardware one. That is how a test run wrote
        `alix_dispatch/runs/…` into the source tree (2026-08-20): the redirect
        was correct and simply had not been told yet.

        EMULATED output must never land beside recorded HARDWARE data — same
        failure class as the 2026-08-08 emulated capture overwriting the real
        one. `ALIX_RUNS_DIR` overrides the hardware root.
        """
        if self.out_dir:
            return pathlib.Path(self.out_dir)
        if self.emulated:
            return (pathlib.Path(os.environ.get("TMPDIR", "/tmp"))
                    / "emu_runs" / self.run_id)
        root = os.environ.get("ALIX_RUNS_DIR")
        return (pathlib.Path(root) if root else pathlib.Path("runs")) / self.run_id

    # ── capture ───────────────────────────────────────────────────────────
    def _side_of_frame(self, data) -> Optional[str]:
        """Which arm sent this frame, by its own source address.

        The real SDK reports `arm_ip` as bytes and the emulator as str, so
        normalise both. A frame from an address this session did not open is
        COUNTED and reported, never dropped quietly (C16.4).
        """
        raw = getattr(data, "arm_ip", None)
        if raw is None:
            return next(iter(self._rows), None) if len(self._rows) == 1 else None
        if isinstance(raw, bytes):
            raw = raw.split(b"\x00", 1)[0].decode("ascii", "replace")
        ip = str(raw).strip()
        side = self._by_ip.get(ip)
        if side is None:
            self._unattributed[ip] = self._unattributed.get(ip, 0) + 1
        return side

    def _on_state(self, data):
        t = time.perf_counter()
        try:
            side = self._side_of_frame(data)
            if side is None:
                return
            js = data.joint_status
            row = [t - self._t0]
            for f in _JOINT_FIELDS:
                row += [float(x) for x in getattr(js, f)]
            row += [int(bool(x)) for x in getattr(js, "joint_en_flag", [0] * 7)]
            # REQUIRED getattr: the emulated joint_status carries
            # `joint_en_flag` but no `joint_err_code`, so a direct attribute
            # read faults on every emulated frame.
            row += [int(x) for x in getattr(js, "joint_err_code", [0] * 7)]
            ls = getattr(data, "liftState", None)
            row += ([int(getattr(ls, "height", 0)), float(getattr(ls, "pos", 0.0)),
                     int(getattr(ls, "current", 0)), int(getattr(ls, "err_flag", 0)),
                     int(getattr(ls, "en_flag", 0))] if ls else [0, 0.0, 0, 0, 0])
            wp = getattr(data, "waypoint", None)
            if wp is not None:
                row += [float(wp.position.x), float(wp.position.y),
                        float(wp.position.z), float(wp.euler.rx),
                        float(wp.euler.ry), float(wp.euler.rz)]
            else:
                row += [0.0] * 6
            row.append(int(getattr(data, "arm_current_status", 0)))
        except Exception as exc:
            # A recorder that silently records nothing is worse than one that
            # errors: count the drops, keep the FIRST reason, surface it at
            # stop() so "0 samples" is never a mystery.
            with self._lock:
                self._dropped += 1
                if self._first_error is None:
                    self._first_error = repr(exc)
            return
        with self._lock:
            self._rows[side].append(row)

    def header(self) -> List[str]:
        """The RM_API2 `stream.csv` header, verbatim, so analysers read both."""
        h = ["t_mono"]
        for f in _JOINT_FIELDS:
            h += [f"{f[6:]}{i}" for i in range(1, 8)]
        h += [f"en{i}" for i in range(1, 8)]
        h += [f"err{i}" for i in range(1, 8)]
        h += ["lift_height", "lift_pos", "lift_current", "lift_err", "lift_en"]
        h += ["tcp_x", "tcp_y", "tcp_z", "tcp_rx", "tcp_ry", "tcp_rz"]
        h += ["arm_status"]
        return h

    def rows(self, side: str) -> List[list]:
        with self._lock:
            return list(self._rows.get(side, ()))

    # ── lifecycle ─────────────────────────────────────────────────────────
    def start(self) -> Tuple[bool, str]:
        """Configure the push and PROVE frames arrive. `(ok, detail)`.

        The push's own return code is not evidence: the controller accepts any
        target host IP with ret 0 and then delivers nothing when it is not
        actually your host. The emulator reproduces that trap deliberately. So
        the gate is a frame that actually landed, not a return code.
        """
        if RunRecorder._active is not None:
            return False, ("a recorder is already active in this process — the "
                           "realtime callback is global; not starting a second")
        try:
            from Robotic_Arm.rm_ctypes_wrap import (
                rm_realtime_arm_state_callback_ptr, rm_realtime_push_config_t,
                rm_udp_custom_config_t)
        except Exception as exc:
            return False, f"SDK realtime types not importable: {exc!r}"

        missing = [s for s, ip in self.host_ips.items() if not ip]
        if missing:
            return False, (f"no route to {', '.join(sorted(missing))} — cannot "
                           "name a UDP target host IP, so nothing would arrive")

        self._t0 = time.perf_counter()
        self._cb = rm_realtime_arm_state_callback_ptr(self._on_state)
        rets = {}
        for side, robot in self.robots.items():
            if robot is None:
                continue
            try:
                robot.rm_realtime_arm_state_call_back(self._cb)
                # hand_state is deliberately NOT requested: measured all-zero.
                # lift_state IS populated and rides the same frame as the arm.
                rets[side] = robot.rm_set_realtime_push(rm_realtime_push_config_t(
                    cycle=self.cycle, enable=True, port=self.udp_port,
                    ip=self.host_ips[side],
                    custom_config=rm_udp_custom_config_t(
                        joint_speed=1, lift_state=1, arm_current_status=1)))
            except Exception as exc:
                return False, f"{side}: realtime push raised {exc!r}"
        RunRecorder._active = self

        deadline = time.perf_counter() + FRAME_PROOF_TIMEOUT_S
        while time.perf_counter() < deadline:
            with self._lock:
                if any(self._rows[s] for s in self._rows):
                    break
            time.sleep(0.05)
        with self._lock:
            landed = {s: len(r) for s, r in self._rows.items()}
        if not any(landed.values()):
            return False, (
                f"push configured (ret={rets}) but NO frames delivered in "
                f"{FRAME_PROOF_TIMEOUT_S:.0f}s — check the UDP target host IP "
                f"({self.host_ips}); the controller accepts a wrong target and "
                "pushes into the void")
        return True, f"recording -> {self.dir.name}  frames {landed}"

    def mark(self, side: Optional[str], stage: str, device: str,
             t_start: float, t_end: float, ok: Optional[bool],
             detail: str = "") -> None:
        """Record when a stage ran, so a check can be windowed to it."""
        self.stages.append({"side": side, "stage": stage, "device": device,
                            "t_start": t_start - (self._t0 or t_start),
                            "t_end": t_end - (self._t0 or t_end),
                            "ok": ok, "detail": detail})

    def stage_window(self, side: Optional[str],
                     stage: str) -> Optional[Tuple[float, float]]:
        """`(t_start, t_end)` of a marked stage, in recording time.

        Windowing is REQUIRED, not a refinement: every shipped config runs
        `execute_path -> move_to_pre_start -> forbid_collision -> move_to_rest`,
        so the last row of an unwindowed stream is the REST pose. Compared
        against the last cleaning waypoint that fails identically whether the
        run completed or truncated — the check would be inert.
        """
        hits = [st for st in self.stages
                if st["stage"] == stage and (side is None or st["side"] == side)]
        if not hits:
            return None
        return (min(h["t_start"] for h in hits), max(h["t_end"] for h in hits))

    def stop(self, extra: Optional[dict] = None) -> Dict[str, pathlib.Path]:
        """Write `stream.csv` per side plus a merged `run.json`."""
        try:
            for side, robot in self.robots.items():
                if robot is None:
                    continue
                try:
                    from Robotic_Arm.rm_ctypes_wrap import rm_realtime_push_config_t
                    robot.rm_set_realtime_push(rm_realtime_push_config_t(
                        cycle=self.cycle, enable=False, port=self.udp_port,
                        ip=self.host_ips.get(side) or ""))
                except Exception:
                    pass
            self.dir.mkdir(parents=True, exist_ok=True)
            written: Dict[str, pathlib.Path] = {}
            head = ",".join(self.header())
            with self._lock:
                per_side = {s: list(r) for s, r in self._rows.items()}
            for side, rows in per_side.items():
                path = self.dir / (f"stream_{side}.csv" if len(per_side) > 1
                                   else "stream.csv")
                with open(path, "w") as fh:
                    fh.write(head + "\n")
                    for row in rows:
                        fh.write(",".join(f"{v:.6g}" if isinstance(v, float)
                                          else str(v) for v in row) + "\n")
                written[side] = path
            meta = dict(self.meta)
            meta.update({
                "run_id": self.run_id, "task": self.task,
                "sdk": "emulated" if self.emulated else "hardware",
                "samples": {s: len(r) for s, r in per_side.items()},
                "dropped_frames": self._dropped,
                "first_drop_error": self._first_error,
                "unattributed_frames": dict(self._unattributed),
                "udp": {"port": self.udp_port, "cycle": self.cycle,
                        "host_ips": self.host_ips},
                "stages": self.stages,
            })
            if extra:
                meta.update(extra)
            run_json = self.dir / "run.json"
            run_json.write_text(json.dumps(meta, indent=2, default=str))
            written["run.json"] = run_json
            return written
        finally:
            # ALWAYS clear the process-global claim. A stop() that raised
            # before clearing would refuse every later recorder in the process.
            RunRecorder._active = None
