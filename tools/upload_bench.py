#!/usr/bin/env python3
"""Upload bench: how fast a Program file reaches a controller, whether a controller takes uploads while it runs a
Program (one, then a batch), whether it takes two at once, and what an upload does to the run (the user, 27 Nov).

    python3 tools/upload_bench.py logs                                  # what Lite's logs already show; no arm
    python3 tools/upload_bench.py slots --side left                     # what the controller's slots hold; reads only
    python3 tools/upload_bench.py live --side left --slots 90-95        # the tests; the arm runs a STILL Program
    python3 tools/upload_bench.py live --side left --slots 90-95 --external-run --tests during,batch
                                                                        # ... during runs you start in Lite
    python3 tools/upload_bench.py live --emulated --side left --slots 90-95 --yes   # dry run on ALIX's emulator

live runs these tests in this order (--tests picks some):
  idle      each test file sent --repeat times with the arm idle: the send, and the program-list reads around it
  baseline  the run alone: how long the Program plays (the reference for the next two)
  during    one upload to another slot, started --delay-s into a run
  batch     uploads back to back during one run, as many as the run and the free slots allow
  parallel  --parallel files at once, each on a connection of its own, to the same controller; with --other-slots,
            one more file to the other arm's controller at the same moment (last: the least known to the controller)

The run: --external-run sends no run at all: you start each run in Lite (its own pole and hand staging, gates and
STOP), the bench sees it start on the controller and uploads during it. --run-slot N runs a Program the controller
already holds (it moves the arm; the bench stages no pole or hand). Without either, the tool stores a STILL Program in the first of --slots: --hold-s seconds of the arm's own joints read at the start, so the
controller runs a Program and the arm does not move. rm_upload.py's own run() starts and watches every run: it
refuses an arm error or a running program, asks you to type the side, slow-stops the arm on Ctrl+C and puts the
global speed back.

The controller's data (the user, 28 Nov: nothing on the controller may be corrupted): the bench writes and deletes
only the slots in --slots (and --other-slots), and only a slot that is empty or holds a file of this tool (listed as
<slot>_UPBENCH...), checked at the start and again in the program-list read just before each send and each delete. It
sends store-only, as Lite does (rm_send_project only_save 1, auto_start 0: nothing runs, the IO default program is not
touched), and refuses to start when the IO default program is one of its slots. Before its first write it reads what
each controller it touches holds (every program's name, size and stored speed, the IO default program, the global
points) and which of those programs a file in the repository could restore (RealMan's API gives no program's points
back); after every test it reads them again, and any change to anything not its own stops it: no further write, and
no delete either. The two-at-once test runs last. Its own files are deleted at the end (--keep keeps them). The test
files are still Programs at the arm's joints, or --file FILE sent under an UPBENCH name.

How it measures: each upload makes the calls Lite makes (rm_upload.py: the program list, rm_send_project store only,
the program list again, the stored size compared), each timed, on a connection of its own in a process of its own
(the SDK library's state is process-wide). Beside the first upload connection of each test, a probe on a second
connection in the same process times a reading (rm_get_current_arm_state) and a call that waits for the library's write
lock as STOP does (rm_get_current_tool_frame; the 5 Nov lock study) every 0.1 s. A watcher in the main process reads
the run state every 50 ms on its own connection for the whole bench. Results go beside Lite's logs, in
Resources/Logs/<day>/upload_bench_<time>_<side>/ (results.jsonl, summary.json, summary.md); an emulated run writes to a
temporary folder, and its timings are the emulator's, not a controller's.
"""
from __future__ import annotations

import argparse
import contextlib
import datetime as dt
import io
import json
import multiprocessing as mp
import queue
import re
import statistics
import sys
import tempfile
import threading
import time
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

TAG = "UPBENCH"
TICK_S = 0.005                  # D26: a file of N lines lasts (N - 1) x 5 ms
LOG_KB_S = 42.0                 # the lab's 176 timed sends, 14 Oct - 13 Nov 2026 (the logs mode): used before an idle fit
MOTION = {"run", "workflow", "canfd stream", "canfd settle", "movej global point", "pole move", "hand move",
          "pole and hand"}


class BenchError(Exception):
    pass


def _median(xs):
    xs = [x for x in xs if x is not None]
    return statistics.median(xs) if xs else None


def _overlap(a0, a1, b0, b1) -> float:
    return max(0.0, min(a1, b1) - max(a0, b0))


def fit(pairs) -> dict | None:
    """Least squares over (bytes, seconds): seconds = fixed_s + bytes / rate. None for fewer than two sizes."""
    pairs = [(b, s) for b, s in pairs if b and s]
    if len(pairs) < 2 or max(b for b, _ in pairs) - min(b for b, _ in pairs) < 1024:
        return None
    mx, my = statistics.fmean(b for b, _ in pairs), statistics.fmean(s for _, s in pairs)
    slope = sum((b - mx) * (s - my) for b, s in pairs) / sum((b - mx) ** 2 for b, _ in pairs)
    if slope <= 0:
        return None
    return dict(fixed_s=round(my - slope * mx, 3), kb_s=round(1 / (slope * 1024), 1), n=len(pairs))


# ── logs: what Lite's hardware.jsonl already holds ─────────────────────────────────────────────────────────────────

def read_logs(logs: Path) -> dict:
    """Every upload Lite recorded, with the send's own time taken apart from the program-list reads (rm_upload.py's
    printed lines carry their own times: "upload ...", "sent: ret=0", "VERIFIED"), every timed motion command, and
    every program deleted."""
    uploads, motion, deletes = [], [], []
    for f in sorted(Path(logs).glob("*/hardware.jsonl")):
        day, open_ = f.parent.name, {}
        with f.open(encoding="utf-8", errors="replace") as fh:
            for ln in fh:
                if '"verb": "rm_upload"' not in ln and '"elapsed_s": null' in ln:
                    continue
                try:
                    r = json.loads(ln)
                except ValueError:
                    continue
                side, verb, t, text = r.get("side"), r.get("verb"), r.get("t"), r.get("text") or ""
                if verb == "rm_upload":
                    m = re.match(r"upload .*: (\d+) points", text)
                    if m:
                        open_[side] = dict(t_line=t, lines=int(m.group(1)))
                    elif side in open_ and text.strip().startswith("sent: ret="):
                        open_[side]["t_sent"] = t
                    elif side in open_ and "VERIFIED" in text:
                        open_[side]["t_checked"] = t
                    continue
                el = r.get("elapsed_s")
                if verb == "upload":
                    p, a = open_.pop(side, {}), r.get("args") or {}
                    t0 = t - (el or 0.0)
                    u = dict(day=day, side=side, t0=t0, t1=t, slot=a.get("program_id"), file=a.get("file"),
                             bytes=a.get("bytes"), lines=p.get("lines"), ok=bool(r.get("ok")), total_s=el)
                    if "t_sent" in p:
                        u.update(list_before_s=p["t_line"] - t0, send_s=p["t_sent"] - p["t_line"],
                                 verify_s=p.get("t_checked", t) - p["t_sent"])
                    uploads.append(u)
                elif verb in MOTION and el:
                    motion.append(dict(day=day, side=side, verb=verb, t0=t - el, t1=t))
                elif verb == "delete program" and el:
                    deletes.append(dict(day=day, side=side, s=el, ok=bool(r.get("ok"))))
    return dict(uploads=uploads, motion=motion, deletes=deletes)


def logs_summary(data: dict) -> dict:
    ups = data["uploads"]
    timed = [u for u in ups if u.get("send_s") and (u.get("bytes") or 0) > 5000]
    sides = {}
    for side in ("left", "right"):
        s = [u for u in timed if u["side"] == side]
        if s:
            rates = [u["bytes"] / 1024 / u["send_s"] for u in s]
            sides[side] = dict(n=len(s), median_kb_s=round(_median(rates), 1), min_kb_s=round(min(rates), 1),
                               max_kb_s=round(max(rates), 1), fit=fit([(u["bytes"], u["send_s"]) for u in s]))
    same, other, pairs = [], [], []
    for u in ups:
        for m in data["motion"]:
            if _overlap(u["t0"], u["t1"], m["t0"], m["t1"]) > 0:
                (same if m["side"] == u["side"] else other).append(dict(day=u["day"], upload_side=u["side"],
                                                                        slot=u["slot"], motion=m["verb"]))
    for i, a in enumerate(ups):
        for b in ups[i + 1:]:
            if _overlap(a["t0"], a["t1"], b["t0"], b["t1"]) > 0:
                pairs.append(dict(day=a["day"], a=(a["side"], a["slot"]), b=(b["side"], b["slot"])))
    big = max(ups, key=lambda u: u.get("bytes") or 0) if ups else None
    return dict(
        days=sorted({u["day"] for u in ups}), uploads=len(ups), stored=sum(u["ok"] for u in ups),
        timed=len(timed), sides=sides, fit=fit([(u["bytes"], u["send_s"]) for u in timed]),
        lines_per_s=round(_median(u["lines"] / u["send_s"] for u in timed if u.get("lines")) or 0),
        bytes_per_line=round(_median(u["bytes"] / u["lines"] for u in timed if u.get("lines")) or 0, 1),
        list_before_s=round(_median(u.get("list_before_s") for u in timed) or 0, 2),
        verify_s=round(_median(u.get("verify_s") for u in timed) or 0, 2),
        biggest=None if big is None else dict(day=big["day"], side=big["side"], bytes=big["bytes"], lines=big["lines"],
                                              send_s=round(big.get("send_s") or 0, 2), total_s=big["total_s"]),
        overlap_same_arm=same, overlap_other_arm=other, overlap_uploads=pairs,
        deletes=len(data["deletes"]), delete_s=round(_median(d["s"] for d in data["deletes"]) or 0, 2))


def logs_markdown(s: dict) -> str:
    f = s["fit"] or {}
    out = [f"# Uploads in Lite's logs ({s['days'][0]} to {s['days'][-1]})" if s["days"] else "# Uploads in Lite's logs",
           "",
           f"Written by `tools/upload_bench.py logs` from `Resources/Logs/*/hardware.jsonl` on {dt.date.today()}.",
           "",
           f"- Uploads recorded: {s['uploads']}, stored and verified: {s['stored']}. With the send timed apart from "
           f"the program-list reads: {s['timed']} (files over 5 KB).",
           f"- Send rate: {f.get('kb_s', '?')} KB/s with {f.get('fixed_s', '?')} s fixed per file (least squares over "
           f"{f.get('n', 0)} sends); {s['lines_per_s']} lines/s at {s['bytes_per_line']} bytes a line.",
           ]
    for side, v in s["sides"].items():
        out.append(f"- {side.upper()}: median {v['median_kb_s']} KB/s ({v['min_kb_s']} to {v['max_kb_s']}), "
                   f"{v['n']} sends.")
    b = s["biggest"]
    if b:
        out.append(f"- Biggest file: {b['bytes'] / 1024:.0f} KB, {b['lines']} lines, send {b['send_s']} s, "
                   f"{b['total_s']} s with the list reads ({b['day']}, {b['side']}).")
    out += [f"- Program-list read before the send: median {s['list_before_s']} s; read-back after it: "
            f"median {s['verify_s']} s.",
            f"- Programs deleted: {s['deletes']}, median {s['delete_s']} s each.",
            f"- Uploads that overlapped a motion command on the same arm: {len(s['overlap_same_arm'])}; on the other "
            f"arm: {len(s['overlap_other_arm'])}; uploads that overlapped another upload: {len(s['overlap_uploads'])}.",
            ""]
    if not (s["overlap_same_arm"] or s["overlap_other_arm"] or s["overlap_uploads"]):
        out.append("The logs answer the rate only. Every upload ran with both arms idle and alone, so whether a "
                   "controller takes an upload while it runs a Program, or two uploads at once, needs `live`.")
    return "\n".join(out) + "\n"


# ── the controller's data: whose a slot is, what it holds, what changed, what the repository could restore ──────────

TESTS = ("idle", "baseline", "during", "batch", "parallel")


def _ours(name, slot) -> bool:
    """A program of this bench: listed as <slot>_UPBENCH..., or UPBENCH... (the controllers' second form, 24 Nov)."""
    n = name or ""
    return n.startswith(f"{slot}_{TAG}") or n.startswith(TAG)


def snapshot(rmu, arm) -> dict:
    """What a controller holds that the bench must leave as it is, each read only: every program (name, size in bytes,
    stored speed), the program its IO starts (rm_get_default_run_program; 0: none) and its global points (runner v14's
    global_waypoints: name and joints). A part that cannot be read is said so, and compared as such."""
    try:
        programs = {int(k): [v[0], int(v[1]), v[2]] for k, v in rmu.slots(arm).items()}
    except SystemExit as exc:
        raise BenchError(f"the program list could not be read ({exc})") from None
    try:
        ret, d = arm.rm_get_default_run_program()
        io_default = int(d) if ret == 0 else f"not read (ret {ret})"
    except Exception as exc:                                                        # noqa: BLE001
        io_default = f"not read ({type(exc).__name__})"
    try:
        points = sorted([p["point_name"], [round(float(v), 3) for v in p["joint"][:7]]]
                        for p in _runner().global_waypoints(arm))
    except Exception as exc:                                                        # noqa: BLE001
        points = f"not read ({type(exc).__name__})"
    return dict(programs=programs, io_default=io_default, points=points)


def changes(before: dict, after: dict, mine: set, ran=(), notes=None) -> list[str]:
    """What differs between two snapshots, leaving out the bench's own slots while they hold the bench's files. For a
    program seen running (``ran``) only its name and size are compared: a run may set its stored speed (rm_set_program_id
    _run takes one); such a change goes to ``notes``."""
    out = []
    for slot in sorted(set(before["programs"]) | set(after["programs"])):
        b, a = before["programs"].get(slot), after["programs"].get(slot)
        if slot in mine and (b is None or _ours(b[0], slot)) and (a is None or _ours(a[0], slot)):
            continue
        if slot in ran and b and a and list(b[:2]) == list(a[:2]) and b[2] != a[2]:
            if notes is not None:
                notes.append(f"slot {slot}'s stored speed went from {b[2]} to {a[2]} after it ran (its name and size are "
                             f"unchanged)")
            continue
        if b != a:
            out.append(f"slot {slot}: was {b and tuple(b)}, now {a and tuple(a)}")
    if before["io_default"] != after["io_default"]:
        out.append(f"IO default program: was {before['io_default']}, now {after['io_default']}")
    if before["points"] != after["points"]:
        out.append("global points: " + (f"{len(before['points'])} before, {len(after['points'])} after"
                                        if isinstance(before["points"], list) and isinstance(after["points"], list)
                                        else f"{before['points']!r} before, {after['points']!r} after"
                                        if isinstance(before["points"], str) or isinstance(after["points"], str)
                                        else "changed"))
    return out


def _stored_size(path: Path) -> int | None:
    """The bytes rm_upload.write_project would send for this file (what the controller lists as its size)."""
    try:
        lines = _rmu().read_track(path)
    except SystemExit:
        return None
    folder = b'{"name":"Folder","num":1,"type":%d,"enabled":true,"parent_number":0}' % len(lines)
    return len(b'{"file":7}') + 2 + len(folder) + 2 + sum(len(ln) for ln in lines) + 2 * len(lines)


def restorable(programs: dict, roots) -> dict:
    """For each program (slot: [listed name, size, speed]): a file under ``roots`` that Lite could send again to give
    the same program (its name starts as the listed name does, without the slot, and it stores the same size), else
    None. The controller gives no program's points back, so these files are the only way back."""
    files = {}
    for root in roots:
        for f in Path(root).rglob("*.txt") if Path(root).is_dir() else []:
            files.setdefault(f.stem, f)
    sizes = {}
    out = {}
    for slot, (name, size, _) in programs.items():
        base = re.sub(r"\.(t|tx|txt)?$", "", name or "")
        heads = {base, base[len(f"{slot}_"):] if base.startswith(f"{slot}_") else base}
        found = None
        for stem, f in files.items():
            if any(len(h) >= 8 and stem.startswith(h) for h in heads):
                if f not in sizes:
                    sizes[f] = _stored_size(f)
                if sizes[f] == size:
                    found = str(f.relative_to(ROOT)) if f.is_relative_to(ROOT) else str(f)
                    break
        out[slot] = found
    return out


_RUNNER = None


def _runner():
    """vendor/control_app/rm_upload_fast_v14_safe_transit.py (runner v14) as a module, loaded once: its
    global_waypoints, as robot_real.global_points reads them."""
    global _RUNNER
    if _RUNNER is None:
        import importlib.util

        from alix_studio_lite import vendor_paths
        spec = importlib.util.spec_from_file_location("bench_runner_v14", vendor_paths.RUNNER_V14)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        _RUNNER = mod
    return _RUNNER


# ── live: the SDK, connections, and one timed upload ──────────────────────────────────────────────────────────────

def _setup(emulated: bool, scale: float = 1.0) -> None:
    from alix_studio_lite import vendor_paths
    vendor_paths.activate()
    if emulated:
        from alix_emulator import rm_emulator
        rm_emulator.install()
        rm_emulator.set_time_scale(scale)


_RMU = None


def _rmu():
    """vendor/control_app/rm_upload.py as a module, as robot_real._rmu loads it (its main() sets rm_send_project_t)."""
    global _RMU
    if _RMU is None:
        import importlib.util

        from alix_studio_lite import vendor_paths
        from Robotic_Arm.rm_robot_interface import rm_send_project_t

        spec = importlib.util.spec_from_file_location("bench_rm_upload", vendor_paths.RM_UPLOAD)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        mod.rm_send_project_t = rm_send_project_t
        _RMU = mod
    return _RMU


def _open(side: str, ip: str, port: int):
    from alix_dispatch import connect
    h = connect.open_arm(side, ip=ip, port=port)
    if h is None:
        raise BenchError(f"could not connect to the {side} controller at {ip}:{port}")
    return h


def _close(h) -> None:
    from alix_dispatch import connect
    connect.close_arm(h)


class Probe:
    """A second connection in an uploading process: a reading and a call that waits for the SDK library's write lock
    (rm_get_current_tool_frame hung behind the leaked lock as STOP did, 5 Nov), each timed, every 0.1 s."""

    def __init__(self, side, ip, port):
        self.h = _open(side, ip, port)
        self.calls, self.inflight = [], None
        self._stop = threading.Event()
        self._th = threading.Thread(target=self._loop, daemon=True)
        self._th.start()

    def _loop(self):
        r = self.h.robot
        while not self._stop.is_set():
            for kind, fn in (("read", r.rm_get_current_arm_state), ("write", r.rm_get_current_tool_frame)):
                t = time.monotonic()
                self.inflight = (kind, t)
                try:
                    ret = fn()[0]
                except Exception:                                                   # noqa: BLE001
                    ret = None
                self.inflight = None
                self.calls.append((kind, t, time.monotonic() - t, ret))
            self._stop.wait(0.1)

    def longest(self, t0: float, t1: float) -> dict:
        """The longest call of each kind under way at some time between t0 and t1 (a call still waiting counts by its
        age). Waits up to 3 s for a call begun after t1, so that a call held by the send has come back."""
        end = time.monotonic() + 3.0
        while time.monotonic() < end and not any(t >= t1 for _, t, _, _ in self.calls[-4:]):
            time.sleep(0.02)
        out, calls, inf = {}, list(self.calls), self.inflight
        if inf is not None:
            calls.append((inf[0], inf[1], time.monotonic() - inf[1], None))
        for kind, t, d, _ in calls:
            if t < t1 and t + d > t0:
                out[kind] = round(max(out.get(kind, 0.0), d), 3)
        return out

    def close(self):
        self._stop.set()
        self._th.join(10)
        _close(self.h)


def upload_one(robot, rmu, job: dict, probe: Probe | None = None) -> dict:
    """One file into one slot with rm_upload.upload's calls, each timed: the program list (a slot that holds anything
    but the bench's file is not written), rm_send_project (store only, stored speed 0, not the IO default: as Lite
    sends), the program list again and the stored size compared (within max(64 B, 0.1 %))."""
    rec = dict(kind="upload", **job, t_wall=time.time())
    try:
        lines = rmu.read_track(Path(job["file"]))
    except SystemExit as exc:
        return dict(rec, ok=False, why=str(exc))
    slot = int(job["slot"])
    with tempfile.TemporaryDirectory() as tmp:
        proj = Path(tmp) / f"{job['name']}.proj"
        sent = rmu.write_project(lines, proj)
        rec.update(lines=len(lines), bytes=sent)
        t0 = time.monotonic()
        try:
            held = rmu.slots(robot).get(slot)
        except SystemExit as exc:
            return dict(rec, ok=False, why=f"program list before the send: {exc}")
        if held is not None and not _ours(held[0], slot):
            return dict(rec, ok=False, not_sent=True,
                        why=f"slot {slot} now holds {held[0]!r}, not the bench's: nothing was sent")
        t1 = time.monotonic()
        ret, err_line = robot.rm_send_project(rmu.rm_send_project_t(str(proj), 0, 1, slot, 0, 0, 0))
        t2 = time.monotonic()
    got, why = None, None
    try:
        got = rmu.slots(robot).get(slot)
    except SystemExit as exc:
        why = f"program list after the send: {exc}"
    t3 = time.monotonic()
    sent_ok = ret == 0 and err_line == -1
    verified = got is not None and abs(got[1] - sent) <= max(64, sent // 1000)
    if not sent_ok:
        why = rmu.SEND_RET.get(ret, f"ret {ret}") if ret != 0 else (
            "the data length check failed" if err_line == 0 else f"the controller refused line {err_line}")
    elif not verified and why is None:
        why = f"slot {slot} is not listed" if got is None else f"slot {slot} holds {got[1]} bytes, {sent} were sent"
    rec.update(ret=ret, err_line=err_line, verified=verified, ok=sent_ok and verified, why=why,
               stored_name=got[0] if got else None, stored_bytes=got[1] if got else None,
               t_list0=t0, t_send0=t1, t_send1=t2, t_list1=t3, list_before_s=round(t1 - t0, 3),
               send_s=round(t2 - t1, 3), verify_s=round(t3 - t2, 3),
               kb_s=round(sent / 1024 / (t2 - t1), 1) if t2 > t1 else None)
    if probe is not None:
        rec["probe"] = probe.longest(t1, t2)
    return rec


def _worker(cfg: dict, jobs: list, go, abort, out) -> None:
    """One connection to one controller, in a process of its own (a thread on the emulator, whose controllers live in
    one process): says it is ready, waits for ``go``, then sends ``jobs`` one after the other, none once ``abort`` is
    set. Every result goes on ``out``; "done" last."""
    name, h, probe = cfg["name"], None, None
    try:
        if not cfg["in_process"]:
            _setup(False)
        h = _open(cfg["side"], cfg["ip"], cfg["port"])
        rmu = _rmu()
        probe = Probe(cfg["side"], cfg["ip"], cfg["port"]) if cfg["probe"] else None
        out.put(dict(kind="ready", worker=name))
        if not go.wait(cfg["wait_s"]):
            out.put(dict(kind="worker", worker=name, why="never told to start"))
            return
        for job in jobs:
            if abort.is_set():
                out.put(dict(kind="upload", **job, worker=name, ok=False, skipped=True, why="the bench was stopped"))
                continue
            out.put(upload_one(h.robot, rmu, dict(job, worker=name), probe))
    except KeyboardInterrupt:
        out.put(dict(kind="worker", worker=name, why="Ctrl+C"))
    except Exception as exc:                                                        # noqa: BLE001
        out.put(dict(kind="worker", worker=name, why=f"{type(exc).__name__}: {exc}"))
    finally:
        try:
            if probe is not None:
                probe.close()
            if h is not None:
                _close(h)
        finally:
            out.put(dict(kind="done", worker=name))


class Watcher:
    """The main process's own connection: the run state every 50 ms for the whole bench, each reading timed. It gives
    when each run started and ended (run_state 1 or 2, then 0) and how long the controller took to answer this other
    client while uploads went on."""

    def __init__(self, side, ip, port):
        self.h = _open(side, ip, port)
        self.reads, self.changes = [], []
        self.started = threading.Event()
        self._stop = threading.Event()
        self._th = threading.Thread(target=self._loop, daemon=True)
        self._th.start()

    def _loop(self):
        last = None
        while not self._stop.is_set():
            t = time.monotonic()
            try:
                ret, st = self.h.robot.rm_get_program_run_state()
            except Exception:                                                       # noqa: BLE001
                ret, st = None, {}
            d = time.monotonic() - t
            rs = (st or {}).get("run_state") if ret == 0 else None
            self.reads.append((t, d, ret, rs))
            if rs is not None and rs != last:
                self.changes.append((t, rs, (st or {}).get("id")))
                if rs in (1, 2):
                    self.started.set()
                last = rs
            self._stop.wait(0.05)

    def run_after(self, t_from: float) -> dict | None:
        """The first run seen after t_from: start, end (None while it runs), whether it paused, its slot."""
        run = None
        for t, rs, sid in self.changes:
            if t < t_from:
                continue
            if rs in (1, 2) and run is None:
                run = dict(t_start=t, t_end=None, paused=rs == 2, slot=sid)
            elif rs == 2 and run is not None:
                run["paused"] = True
            elif rs == 0 and run is not None:
                run["t_end"] = t
                return run
        return run

    def longest(self, t0, t1) -> float | None:
        ds = [d for t, d, _, _ in self.reads if t < t1 and t + d > t0]
        return round(max(ds), 3) if ds else None

    def close(self):
        self._stop.set()
        self._th.join(5)
        _close(self.h)


class Crew:
    """Upload workers started together: ``go`` starts them, ``abort`` stops them before their next file."""

    def __init__(self, workers, go, abort, q):
        self.workers, self.go, self.abort, self.q = workers, go, abort, q

    def collect(self, timeout_s: float = 600.0) -> list:
        done, got, end = set(), [], time.monotonic() + timeout_s
        while len(done) < len(self.workers) and time.monotonic() < end:
            try:
                m = self.q.get(timeout=1.0)
            except queue.Empty:
                continue
            if m["kind"] == "done":
                done.add(m["worker"])
            elif m["kind"] != "ready":
                got.append(m)
        for w in self.workers:
            w.join(10)
        return got


class _Tee(io.TextIOBase):
    """rm_upload.run's printed lines: shown as they come (its prompt too) and kept with their times."""

    def __init__(self):
        self.real, self.lines, self._part = sys.stdout, [], ""

    def write(self, s):
        self.real.write(s)
        self.real.flush()
        self._part += s
        while "\n" in self._part:
            line, self._part = self._part.split("\n", 1)
            self.lines.append((time.monotonic(), line))
        return len(s)

    def flush(self):
        self.real.flush()


def _slots_arg(text: str) -> list[int]:
    out = []
    for part in (text or "").split(","):
        part = part.strip()
        if not part:
            continue
        lo, _, hi = part.partition("-")
        out += list(range(int(lo), int(hi or lo) + 1))
    if any(not 1 <= s <= 100 for s in out) or len(set(out)) != len(out):
        raise argparse.ArgumentTypeError(f"slots are 1-100, each once: {text!r}")
    return out


def _still_file(path: Path, point: list, n: int) -> Path:
    """A Program of ``n`` lines that all hold ``point`` (millidegrees): a run of it does not move the arm."""
    line = json.dumps({"point": point}, separators=(",", ":"))
    path.write_text("\n".join([line] * n) + "\n", encoding="utf-8")
    return path


def addresses(a) -> dict:
    """Each arm's controller (ip, port): Lite's connection settings (--data), --ip/--port for --side; the emulator's own
    two addresses with --emulated."""
    if a.emulated:
        from alix_dispatch import connect
        return {s: (connect.DEFAULT_IPS[s], connect.DEFAULT_PORT) for s in ("left", "right")}
    from alix_studio_lite.server.connection import ConnectionSettings
    cs = ConnectionSettings(Path(a.data).expanduser())
    out = {s: (cs.arm(s)["ip"], int(cs.arm(s)["port"])) for s in ("left", "right")}
    if a.ip:
        out[a.side] = (a.ip, a.port or out[a.side][1])
    return out


class Bench:
    def __init__(self, a):
        self.a, self.emulated = a, a.emulated
        self.scale = a.time_scale if a.emulated else 1.0
        self.side = a.side
        self.other = "right" if a.side == "left" else "left"
        self.addr = addresses(a)
        self.ctx = None if self.emulated else mp.get_context("spawn")
        stamp = dt.datetime.now().strftime("%Y-%m-%d_%H%M%S")
        if a.out:
            self.out = Path(a.out)
        elif self.emulated:
            self.out = Path(tempfile.mkdtemp(prefix=f"upload_bench_emulated_{stamp}_"))
        else:
            self.out = ROOT / "Resources" / "Logs" / stamp[:10] / f"upload_bench_{stamp[11:]}_{a.side}"
        self.out.mkdir(parents=True, exist_ok=True)
        self.tmp = Path(tempfile.mkdtemp(prefix="upload_bench_files_"))
        self.records, self.written, self.halt = [], set(), None
        self._f = (self.out / "results.jsonl").open("a", encoding="utf-8")
        self.handles, self.arm, self.other_arm, self.watch, self.rmu = [], None, None, None, None
        self.snap, self.changed, self.unread, self.active, self.noted = {}, None, None, None, []

    def log(self, rec: dict) -> None:
        self.records.append(rec)
        self._f.write(json.dumps(rec, default=str) + "\n")
        self._f.flush()

    # ── setting up: what the slots hold, the arm's state, the test files, the still Program
    def prepare(self) -> None:
        a = self.a
        self.handles.append(_open(self.side, *self.addr[self.side]))
        self.arm = self.handles[-1].robot
        self.rmu = rmu = _rmu()
        self.watch = Watcher(self.side, *self.addr[self.side])
        held = self._slots(self.arm)
        self._check_slots(self.side, held, a.slots)
        if a.other_slots:
            self.handles.append(_open(self.other, *self.addr[self.other]))
            self.other_arm = self.handles[-1].robot
            self._check_slots(self.other, self._slots(self.other_arm), a.other_slots)
        if a.run_slot is not None:
            if a.run_slot in a.slots:
                raise BenchError(f"--run-slot {a.run_slot} is one of --slots; the bench would write over it")
            if a.run_slot not in held:
                raise BenchError(f"slot {a.run_slot} is empty on the {self.side} controller; nothing to run")
        joints, errs = rmu.arm_state(self.arm)
        if joints is None:
            raise BenchError(f"the {self.side} arm's state could not be read")
        ret, st = self.arm.rm_get_program_run_state()
        if ret != 0 or (st or {}).get("run_state") in (1, 2):
            raise BenchError(f"a program is running or paused on the {self.side} controller (slot {(st or {}).get('id')}),"
                             f" or its run state could not be read (ret {ret}); the bench needs it idle")
        runs = {"baseline", "during", "batch"} & set(a.tests)
        if errs and runs and not a.external_run:
            raise BenchError(f"the {self.side} arm reports {rmu.codes_text(errs)}; clear it at the pendant first")
        self.point = [int(round(j * 1000)) for j in joints]
        if a.file:
            files = [dict(path=Path(p), name=f"{TAG}_{i + 1}_{Path(p).stem}"[:28]) for i, p in enumerate(a.file)]
        else:
            files = [dict(path=_still_file(self.tmp / f"{TAG}_{n}.txt", self.point, n), name=f"{TAG}_{n}")
                     for n in a.lines]
        for f in files:
            f["lines"] = len(rmu.read_track(f["path"]))
        self.files = files
        self.big = max(files, key=lambda f: f["lines"])
        self.hold_slot = None if a.run_slot is not None or a.external_run else a.slots[0]
        self.free = [s for s in a.slots if s != self.hold_slot]
        need = max(a.parallel if "parallel" in a.tests else 1, 1)
        if len(self.free) < need:
            raise BenchError(f"--slots gives {len(self.free)} slot(s) for uploads; the tests need {need}"
                             + (" (the first holds the still Program)" if self.hold_slot else ""))
        roots = [ROOT / "Resources" / "Programs", ROOT / "Resources" / "Sequences",
                 *sorted((ROOT / "Resources" / "Projects").glob("*/controller_exports"))]
        for side, arm, slots in ((self.side, self.arm, a.slots), (self.other, self.other_arm, a.other_slots)):
            if arm is None:
                continue
            snap = self.snap[side] = snapshot(rmu, arm)
            if snap["io_default"] in slots:
                raise BenchError(f"slot {snap['io_default']} is the {side} controller's IO default program; leave it out "
                                 f"of the bench's slots")
            lab = {k: v for k, v in snap["programs"].items() if not (k in slots and _ours(v[0], k))}
            back = restorable(lab, roots)
            self.log(dict(kind="snapshot", side=side, when="before", **snap, restorable=back))
            gone = [f"{k} {lab[k][0]}" for k, f in sorted(back.items()) if f is None]
            print(f"{side} controller: {len(lab)} programs not the bench's; IO default {snap['io_default']}; global points "
                  f"{len(snap['points']) if isinstance(snap['points'], list) else snap['points']}. A file in the repository "
                  f"could restore {len(lab) - len(gone)} of the {len(lab)} programs"
                  + (f"; not these: {'; '.join(gone)}" if gone else ""))
        self.log(dict(kind="setup", t_wall=time.time(), emulated=self.emulated, side=self.side,
                      address=self.addr[self.side], slots=a.slots, other_slots=a.other_slots, run_slot=a.run_slot,
                      programs_held=len(held), joints_mdeg=self.point, tests=a.tests,
                      files=[dict(name=f["name"], lines=f["lines"], path=str(f["path"])) for f in files]))
        print(f"{self.side} controller {self.addr[self.side][0]}: {len(held)} programs held; the bench writes slot(s) "
              f"{', '.join(map(str, a.slots))}" + (f" and {self.other} slot(s) {', '.join(map(str, a.other_slots))}"
                                                  if a.other_slots else ""))
        if not a.yes and input("  go on? [y/N] ").strip().lower() != "y":
            raise BenchError("not started")

    def _slots(self, arm) -> dict:
        """The controller's program list (rm_upload.slots), its sys.exit on a failed read made a BenchError."""
        try:
            return self.rmu.slots(arm)
        except SystemExit as exc:
            raise BenchError(str(exc)) from None

    def check_controllers(self, after: str) -> bool:
        """Each controller the bench touches read again and compared with what it held before the first write. A change
        to anything not the bench's stops the bench: no further write, no delete. True when nothing changed."""
        for side, arm, slots in ((self.side, self.arm, self.a.slots), (self.other, self.other_arm, self.a.other_slots)):
            if arm is None or side not in self.snap:
                continue
            before, now = self.snap[side], None
            for attempt in (1, 2):                   # a read that fails is tried once more, a second later
                try:
                    now = snapshot(self.rmu, arm)
                except BenchError as exc:
                    now = dict(programs=None, why=str(exc))
                unread = (["the program list"] if now["programs"] is None else []) + [
                    what for key, what in (("io_default", "the IO default program"), ("points", "the global points"))
                    if now.get(key) is not None and isinstance(now[key], str) and not isinstance(before[key], str)]
                if not unread or attempt == 2:
                    break
                time.sleep(1.0)
            notes = []
            ran = {sid for _, rs, sid in self.watch.changes if rs in (1, 2) and sid} if side == self.side else set()
            if now["programs"] is None:
                diff = []
            else:                                    # a part not read is not compared (it is reported below)
                diff = changes(before, {k: (before[k] if isinstance(now[k], str) and not isinstance(before[k], str)
                                            else now[k]) for k in ("programs", "io_default", "points")}, set(slots),
                               ran, notes)
            self.log(dict(kind="snapshot", side=side, when=f"after {after}", **now, changes=diff, unread=unread,
                          notes=notes))
            for n in notes:
                if n not in self.noted:
                    self.noted.append(n)
                    print(f"  note: the {side} controller: {n}")
            if diff:
                self.changed = f"the {side} controller's data changed after {after}: " + "; ".join(diff)
                self.halt = self.changed
                print(f"\n** {self.changed}\n** the bench writes and deletes nothing more")
                return False
            if unread:
                self.halt = self.unread = (f"the {side} controller's {' and '.join(unread)} could not be read after {after}, so it "
                             f"is not known to be unchanged")
                print(f"\n** {self.halt}\n** the bench sends nothing more (its own files are still deleted, each slot "
                      f"read again first)")
                return False
        return True

    def _check_slots(self, side, held, slots) -> None:
        bad = [f"slot {s} holds {held[s][0]!r}" for s in slots if s in held and not _ours(held[s][0], s)]
        if bad:
            raise BenchError(f"the bench writes only empty slots or its own; on the {side} controller "
                             + "; ".join(bad) + ". Pick others (python3 tools/upload_bench.py slots --side "
                             f"{side} lists them).")

    # ── running workers and Programs
    def job(self, test, side, slot, f) -> dict:
        self.written.add((side, slot))
        return dict(test=test, side=side, slot=slot, file=str(f["path"]), name=f["name"], file_lines=f["lines"])

    def crew(self, plans) -> Crew:
        """One worker per (side, jobs) in ``plans``, the first of them with a probe, all connected before this returns."""
        if self.emulated:
            go, abort, q = threading.Event(), threading.Event(), queue.Queue()
            make = lambda **kw: threading.Thread(daemon=True, **kw)                 # noqa: E731
        else:
            go, abort, q = self.ctx.Event(), self.ctx.Event(), self.ctx.Queue()
            make = lambda **kw: self.ctx.Process(daemon=True, **kw)                 # noqa: E731
        workers = []
        for i, (side, jobs) in enumerate(plans):
            ip, port = self.addr[side]
            cfg = dict(name=f"{side}-{i + 1}", side=side, ip=ip, port=port, probe=i == 0 and not self.a.no_probe,
                       in_process=self.emulated, wait_s=900.0)
            w = make(target=_worker, args=(cfg, jobs, go, abort, q))
            w.start()
            workers.append(w)
        ready, end = set(), time.monotonic() + 60.0
        while len(ready) < len(workers):
            try:
                m = q.get(timeout=max(0.1, end - time.monotonic()))
            except queue.Empty:
                abort.set(), go.set()
                raise BenchError("the upload connections did not open within 60 s") from None
            if m["kind"] == "ready":
                ready.add(m["worker"])
            elif m["kind"] == "worker":
                abort.set(), go.set()
                raise BenchError(f"upload connection {m['worker']}: {m['why']}")
        self.active = Crew(workers, go, abort, q)
        return self.active

    def drain(self) -> None:
        """Uploads still under way (Ctrl+C) stopped before their next file and waited for, so that nothing is deleted
        while a send to that slot goes on."""
        if self.active is not None:
            self.active.abort.set()
            self.active.go.set()
            print("waiting for any upload still sending to end")
            self.take(self.active.collect(120.0))
            self.active = None

    def take(self, got: list) -> list:
        """Worker results logged, each upload with what the watcher saw while it was sending."""
        ups = []
        for m in got:
            if m.get("kind") == "upload" and m.get("t_send0") is not None:
                m["watcher_longest_s"] = self.watch.longest(m["t_send0"], m["t_send1"])
            self.log(m)
            if m.get("kind") == "upload":
                ups.append(m)
                self._say(m)
            else:
                print(f"  ** {m.get('worker')}: {m.get('why')}")
        return ups

    def _say(self, u) -> None:
        if u.get("skipped"):
            print(f"  {u['test']:<8} slot {u['slot']:>3}  not sent: {u['why']}")
            return
        size = f"{u.get('lines', 0):>6} lines {u.get('bytes', 0) / 1024:7.1f} KB"
        if u.get("send_s") is None:
            print(f"  {u['test']:<8} slot {u['slot']:>3}  {size}  ** {u.get('why')}")
            return
        print(f"  {u['test']:<8} {u['side']:<5} slot {u['slot']:>3}  {size}  send {u['send_s']:6.2f} s "
              f"({u['kb_s'] or 0:5.1f} KB/s)  lists {u['list_before_s']:.2f} + {u['verify_s']:.2f} s  "
              + ("stored" if u["ok"] else f"** NOT STORED: {u['why']}"))

    def run_program(self, test: str, plans=None) -> dict:
        """One run of the run slot through rm_upload.run; ``plans`` start --delay-s after the watcher sees it start."""
        crew = self.crew(plans) if plans else None
        self.watch.started.clear()
        t_call = time.monotonic()

        def conductor():
            if self.watch.started.wait(60.0) and not crew.abort.is_set():
                time.sleep(self.a.delay_s / self.scale)
                if not crew.abort.is_set():
                    crew.go.set()

        if crew is not None:
            threading.Thread(target=conductor, daemon=True).start()
        moving = self.a.run_slot is not None
        ra = SimpleNamespace(yes=self.a.yes and not moving, side=self.side, run_speed=self.a.run_speed,
                             restore_speed=self.a.restore_speed, timeout=self.run_timeout)
        tee, rc = _Tee(), 1
        n_up = f" with {sum(len(j) for _, j in plans)} upload(s)" if plans else ""
        print(f"\n[{test}] " + (f"a run started in Lite{n_up}" if self.a.external_run else f"run slot {self.run_slot}{n_up}"))
        try:
            with contextlib.redirect_stdout(tee):
                if self.a.external_run:
                    rc = self._wait_external(t_call)
                else:
                    rc = self.rmu.run(self.arm, ra, self.addr[self.side][0], self.run_slot, self.first_point)
        except KeyboardInterrupt:
            print("  ** stopped (Ctrl+C)")
        except SystemExit as exc:                        # rm_upload's program list did not answer
            print(f"  ** {exc}")
        finally:
            got = []
            if crew is not None:
                crew.abort.set()
                crew.go.set()
                got, self.active = crew.collect(), None
        w = self.watch.run_after(t_call)
        play = (w["t_end"] - w["t_start"]) if w and w["t_end"] else None
        rec = dict(kind="run", test=test, slot=w["slot"] if w and self.a.external_run else self.run_slot, rc=rc,
                   external=bool(self.a.external_run), complete=rc == 0, started=w is not None,
                   paused=bool(w and w["paused"]), play_s=round(play, 3) if play else None,
                   t_start=w and w["t_start"], t_end=w and w["t_end"],
                   ms_per_line=round(play * 1000 / (self.run_lines - 1), 3) if play and self.run_lines else None,
                   arm_reports=[ln for _, ln in tee.lines if "!!" in ln or "**" in ln],
                   printed=[[round(t - t_call, 2), ln] for t, ln in tee.lines])
        for u in got:
            if u.get("kind") == "upload" and u.get("t_send0") is not None:
                inside = _overlap(u["t_send0"], u["t_send1"], w["t_start"], w["t_end"] or float("inf")) if w else 0.0
                u["in_run"] = ("whole send" if inside >= u["t_send1"] - u["t_send0"] - 1e-6 else
                               "part of the send" if inside > 0 else "no")
        self.take(got)
        self.log(rec)
        print(f"  [{test}] played {rec['play_s']} s; rm_upload.run {'complete' if rc == 0 else 'NOT complete'}")
        if rc != 0:
            self.halt = f"the {test} run did not complete"
        return rec

    def _wait_external(self, t_call: float) -> int:
        """A run started by someone else (Lite): waited for, then watched to its end, the arm's error codes read after
        it. The bench sends no motion command and no STOP; 0 when the run ended with no error (rm_upload.run's rule)."""
        print(f"  start a Program on the {self.side} arm in Lite now; waiting up to {self.a.wait_s:g} s "
              f"(this bench sends no motion command)")
        if not self.watch.started.wait(self.a.wait_s):
            print(f"  ** no run started within {self.a.wait_s:g} s")
            return 1
        print("  running")
        end = time.monotonic() + self.a.run_timeout
        while time.monotonic() < end:
            w = self.watch.run_after(t_call)
            if w and w["t_end"]:
                _, errs = self.rmu.arm_state(self.arm)
                bad = sorted(set(errs or []) - self.rmu.WARN_ONLY)
                print(f"  the run ended ({w['t_end'] - w['t_start']:.1f} s)"
                      + (f"; ** the arm reports {self.rmu.codes_text(bad)}" if bad else ", no error"))
                return 1 if bad else 0
            time.sleep(0.2)
        print(f"  ** the run had not ended after {self.a.run_timeout:g} s; the bench stops waiting (it does not stop "
              f"the arm)")
        return 1

    # ── the tests
    def t_idle(self):
        print("\n[idle] the arm idle, one connection")
        jobs, k = [], 0
        for _ in range(self.a.repeat):
            for f in self.files:
                jobs.append(self.job("idle", self.side, self.free[k % len(self.free)], f))
                k += 1
        crew = self.crew([(self.side, jobs)])
        crew.go.set()
        got, self.active = crew.collect(), None
        self.take(got)

    def t_parallel(self):
        n = self.a.parallel
        print(f"\n[parallel] {n} at once to the {self.side} controller"
              + (f", 1 to the {self.other} controller" if self.a.other_slots else ""))
        plans = [(self.side, [self.job("parallel", self.side, self.free[i], self.big)]) for i in range(n)]
        if self.a.other_slots:
            plans.append((self.other, [self.job("parallel", self.other, self.a.other_slots[0], self.big)]))
        crew = self.crew(plans)
        crew.go.set()
        got, self.active = crew.collect(), None
        self.take(got)

    def t_baseline(self):
        self.baseline = self.run_program("baseline")

    def _send_s(self, f) -> float:
        """File f's upload predicted in this bench's seconds: the idle fit (it measured this bench, the emulator's
        scale and all), else the logs' rate; with 0.5 s for the two program-list reads."""
        ups = [r for r in self.records if r.get("kind") == "upload" and r.get("ok")]
        idle = fit([(r["bytes"], r["send_s"]) for r in ups if r.get("test") == "idle"])
        kb = next((r["bytes"] / 1024 for r in ups if r.get("lines") == f["lines"]), f["lines"] * 57 / 1024)
        if idle:
            return idle["fixed_s"] + kb / idle["kb_s"] + 0.5 / self.scale
        return (kb / LOG_KB_S + 0.5) / self.scale

    def _fitting(self):
        """(the file to send during a run, the seconds the run leaves for sends): the run's length (the baseline's,
        else the still Program's) less --delay-s and 2 s to spare, and the biggest test file whose send fits in it,
        else the smallest. A lab Program with no baseline: the biggest file, the room unknown (None)."""
        run_s = (self.baseline or {}).get("play_s") or (self.a.hold_s / self.scale if self.hold_slot else None)
        if run_s is None:
            return self.big, None
        room = run_s - (self.a.delay_s + 2.0) / self.scale
        fits = [f for f in self.files if self._send_s(f) <= room]
        return (max(fits, key=lambda f: f["lines"]) if fits else min(self.files, key=lambda f: f["lines"])), room

    def t_during(self):
        f, _ = self._fitting()
        self.run_program("during", [(self.side, [self.job("during", self.side, self.free[0], f)])])

    def t_batch(self):
        """Sends back to back, one per free slot: --batch of them, else as many as fit in the run (two when the run's
        length is unknown)."""
        f, room = self._fitting()
        k = self.a.batch or (2 if room is None else max(1, int(room / self._send_s(f))))
        k = min(len(self.free), k)
        self.run_program("batch", [(self.side, [self.job("batch", self.side, self.free[i], f) for i in range(k)])])

    def go(self) -> None:
        a = self.a
        self.prepare()
        self.baseline = None
        if {"baseline", "during", "batch"} & set(a.tests):
            if self.hold_slot is not None:
                n = int(round(a.hold_s / TICK_S)) + 1
                still = _still_file(self.tmp / f"{TAG}_hold.txt", self.point, n)
                self.log(upload_one(self.arm, self.rmu, self.job("hold", self.side, self.hold_slot,
                                                                 dict(path=still, name=f"{TAG}_hold", lines=n))))
                self._say(self.records[-1])
                if not self.records[-1]["ok"]:
                    raise BenchError(f"the still Program was not stored in slot {self.hold_slot}")
                self.check_controllers("storing the still Program")         # a change: every test is skipped
                self.run_slot, self.run_lines, self.first_point = self.hold_slot, n, self.point
                self.run_timeout = (n - 1) * TICK_S * 1.5 + 60.0
            else:
                self.run_slot, self.run_lines, self.first_point = a.run_slot, None, None
                self.run_timeout = a.run_timeout
        for name in TESTS:
            if name not in a.tests:
                continue
            if self.halt:
                print(f"\n[{name}] not run: {self.halt}")
                self.log(dict(kind="skipped", test=name, why=self.halt))
                continue
            getattr(self, f"t_{name}")()
            self.check_controllers(name)

    def cleanup(self) -> None:
        """Every slot of --slots and --other-slots that holds a file of this bench deleted (one left by an earlier bench
        too), each slot read again just before its delete. Nothing is deleted once the controller's data changed."""
        if self.a.keep:
            print(f"\nkept: {sorted(self.written)}")
            return
        if self.changed:
            print(f"\nnot deleted (the controller's data changed; nothing more is written): {sorted(self.written)}")
            return
        for side, arm, slots in ((self.side, self.arm, self.a.slots), (self.other, self.other_arm, self.a.other_slots)):
            if arm is None:
                continue
            for slot in slots:
                now = self._slots(arm).get(slot)
                if now is None or not _ours(now[0], slot):
                    continue
                t = time.monotonic()
                fn = getattr(arm, "rm_delete_program_trajectory", None)
                ret = fn(slot) if fn is not None else None
                self.log(dict(kind="delete", side=side, slot=slot, ret=ret, s=round(time.monotonic() - t, 3),
                              why=None if fn is not None else "this SDK surface has no delete (the emulator)"))
        dels = [r for r in self.records if r.get("kind") == "delete"]
        left = [f"{r['side']} {r['slot']}" for r in dels if r.get("ret") != 0]
        print(f"\ndeleted {len(dels) - len(left)} of the bench's {len(dels)} programs"
              + (f"; still held: {', '.join(left)} ({dels[0].get('why') or 'see results.jsonl'})" if left else ""))
        self.check_controllers("deleting the bench's files")

    def close(self) -> None:
        if self.watch is not None:
            self.watch.close()
        for h in self.handles:
            _close(h)
        self._f.close()
        for f in self.tmp.glob("*.txt"):
            f.unlink()
        with contextlib.suppress(OSError):
            self.tmp.rmdir()


# ── the summary: plain answers from the records ────────────────────────────────────────────────────────────────────

def summarise(records: list) -> dict:
    ups = [dict(r) for r in records if r.get("kind") == "upload" and not r.get("skipped") and r.get("test") != "hold"]
    setup = next((r for r in records if r.get("kind") == "setup"), {})
    runs = {r["test"]: r for r in records if r.get("kind") == "run"}
    idle = [u for u in ups if u["test"] == "idle" and u.get("ok")]
    f = fit([(u["bytes"], u["send_s"]) for u in idle])

    def predict(b):
        return (f["fixed_s"] + b / 1024 / f["kb_s"]) if f else b / 1024 / LOG_KB_S

    for u in ups:
        if u.get("send_s") and u.get("bytes"):
            u["vs_idle"] = round(u["send_s"] / predict(u["bytes"]), 2)
    sizes = {}
    for u in idle:
        s = sizes.setdefault(u["lines"], dict(lines=u["lines"], bytes=u["bytes"], sends=[], before=[], after=[]))
        s["sends"].append(u["send_s"]), s["before"].append(u["list_before_s"]), s["after"].append(u["verify_s"])
    sizes = [dict(lines=s["lines"], kb=round(s["bytes"] / 1024, 1), n=len(s["sends"]),
                  send_s=round(_median(s["sends"]), 3), kb_s=round(s["bytes"] / 1024 / _median(s["sends"]), 1),
                  list_before_s=round(_median(s["before"]), 3), verify_s=round(_median(s["after"]), 3))
             for s in sorted(sizes.values(), key=lambda s: s["lines"])]

    par = [u for u in ups if u["test"] == "parallel"]
    par_side = {}
    for side in sorted({u["side"] for u in par}):
        p = [u for u in par if u["side"] == side and u.get("t_send0") is not None]
        if len(p) >= 2:
            span = max(u["t_send1"] for u in p) - min(u["t_send0"] for u in p)
            par_side[side] = dict(n=len(p), stored=sum(bool(u["ok"]) for u in p),
                                  speedup=round(sum(predict(u["bytes"]) for u in p) / span, 2),
                                  all_sending_s=round(max(0.0, min(u["t_send1"] for u in p) - max(u["t_send0"] for u in p)), 3),
                                  span_s=round(span, 3), kb_s_together=round(sum(u["bytes"] for u in p) / 1024 / span, 1),
                                  vs_idle=[u.get("vs_idle") for u in p])
    cross = None
    mine = [u for u in par if u["side"] == setup.get("side") and u.get("t_send0") is not None]
    theirs = [u for u in par if u["side"] != setup.get("side") and u.get("t_send0") is not None]
    if mine and theirs:
        cross = dict(stored=all(u["ok"] for u in par), vs_idle=[u.get("vs_idle") for u in mine + theirs],
                     overlap_s=round(max(_overlap(x["t_send0"], x["t_send1"], y["t_send0"], y["t_send1"])
                                         for x in mine for y in theirs), 3))

    def in_run(test):
        us = [u for u in ups if u["test"] == test]
        return dict(n=len(us), stored=sum(bool(u.get("ok")) for u in us),
                    whole_send_in_run=sum(u.get("in_run") == "whole send" for u in us),
                    refused=[dict(slot=u["slot"], ret=u.get("ret"), err_line=u.get("err_line"), why=u.get("why"))
                             for u in us if not u.get("ok")],
                    vs_idle=[u.get("vs_idle") for u in us], send_s=[u.get("send_s") for u in us]) if us else None

    probes = [dict(test=u["test"], send_s=u["send_s"], **u["probe"]) for u in ups if "probe" in u and u.get("send_s")]
    deletes = [r for r in records if r.get("kind") == "delete"]
    base = runs.get("baseline") or {}
    snaps = [r for r in records if r.get("kind") == "snapshot"]
    data = {}
    for side in sorted({r["side"] for r in snaps}):
        first = next(r for r in snaps if r["side"] == side and r["when"] == "before")
        later = [r for r in snaps if r["side"] == side and r["when"] != "before"]
        back = first.get("restorable") or {}
        data[side] = dict(programs=len(back), restorable=sum(1 for f in back.values() if f),
                          not_read=[what for key, what in (("io_default", "its IO default program"),
                                                           ("points", "its global points"))
                                    if isinstance(first[key], str)],
                          unread_later=sorted({u for r in later for u in r.get("unread") or []}),
                          notes=sorted({n for r in later for n in r.get("notes") or []}),
                          not_restorable=[f"{k} {first['programs'][k][0]}" for k, f in sorted(back.items(), key=lambda x: int(x[0]))
                                          if not f],
                          io_default=first["io_default"],
                          points=len(first["points"]) if isinstance(first["points"], list) else first["points"],
                          checks=len(later), last_check=later[-1]["when"] if later else None,
                          changes=next((r["changes"] for r in later if r.get("changes")), []))
    out = dict(
        emulated=setup.get("emulated"), side=setup.get("side"), controller_data=data,
        idle_fit=f, idle_sizes=sizes, parallel=par_side, both_controllers=cross,
        during=in_run("during"), batch=in_run("batch"),
        runs={k: dict(play_s=v["play_s"], complete=v["complete"], paused=v["paused"], ms_per_line=v["ms_per_line"],
                      arm_reports=v["arm_reports"],
                      vs_baseline_s=round(v["play_s"] - base["play_s"], 3) if v.get("play_s") and base.get("play_s")
                      and k != "baseline" else None) for k, v in runs.items()},
        library_lock=dict(sends=len(probes),
                          longest_write_s=max((p["write"] for p in probes if "write" in p), default=None),
                          longest_read_s=max((p["read"] for p in probes if "read" in p), default=None),
                          longest_send_s=max((p["send_s"] for p in probes), default=None), each=probes),
        watcher=dict(longest_during_sends_s=max((u.get("watcher_longest_s") or 0 for u in ups), default=None)),
        deletes=dict(n=len(deletes), deleted=sum(r.get("ret") == 0 for r in deletes),
                     median_s=_median(r["s"] for r in deletes if r.get("ret") == 0),
                     why=sorted({r.get("why") or f"ret {r.get('ret')}" for r in deletes if r.get("ret") != 0})),
        skipped=[r for r in records if r.get("kind") == "skipped"])
    return out


def summary_markdown(s: dict) -> str:
    L = ["# Upload bench" + (" (EMULATED: the emulator's timings, not a controller's)" if s["emulated"] else ""), ""]
    for side, d in s.get("controller_data", {}).items():
        if not d["checks"]:
            L.append(f"- **The {side} controller's data:** read at the start; the bench wrote nothing to it.")
        elif d["changes"]:
            L.append(f"- **The {side} controller's data CHANGED** (the bench stopped there and wrote nothing more): "
                     + "; ".join(d["changes"]) + ".")
        else:
            same = [f"its {d['programs']} programs that are not the bench's (name, size, stored speed)"]
            same += [] if "its IO default program" in d["not_read"] else [f"its IO default program ({d['io_default']})"]
            same += [] if "its global points" in d["not_read"] else [f"its {d['points']} global points"]
            L.append(f"- **The {side} controller's data:** no change found. Read before the first write and after each "
                     f"of {d['checks']} steps (the last: {d['last_check']}): " + "; ".join(same) + "."
                     + (f" Not compared, as this controller did not give them: {' and '.join(d['not_read'])}."
                        if d["not_read"] else "")
                     + (f" Not read at some check, so the bench stopped sending: {', '.join(d['unread_later'])}."
                        if d["unread_later"] else "")
                     + (f" Note: {'; '.join(d['notes'])}." if d["notes"] else ""))
        L.append(f"  - A file in the repository could restore {d['restorable']} of those {d['programs']} programs"
                 + (f"; not these: {'; '.join(d['not_restorable'])}." if d["not_restorable"] else "."))
    f = s["idle_fit"]
    if f:
        L.append(f"- **Upload rate, arm idle:** {f['kb_s']} KB/s, plus {f['fixed_s']} s fixed per file "
                 f"(least squares over {f['n']} sends). The program-list reads before and after add the times below.")
    for z in s["idle_sizes"]:
        L.append(f"  - {z['lines']} lines ({z['kb']} KB): send {z['send_s']} s ({z['kb_s']} KB/s), list read "
                 f"{z['list_before_s']} s before, read-back {z['verify_s']} s after; {z['n']} sends.")

    def verdict(name, d, what):
        if not d:
            return f"- **{what}:** not tested."
        if d["stored"] == d["n"] and d["whole_send_in_run"] == d["n"]:
            head = f"yes: {d['n']} of {d['n']} stored and verified, every send wholly inside the run"
        elif d["stored"] == d["n"]:
            head = (f"stored {d['n']} of {d['n']}, but only {d['whole_send_in_run']} sent wholly inside the run "
                    f"(the run ended first)")
        else:
            head = f"NO for {d['n'] - d['stored']} of {d['n']}: " + "; ".join(
                f"slot {r['slot']} ret {r['ret']} err_line {r['err_line']} ({r['why']})" for r in d["refused"])
        return f"- **{what}:** {head}. Send time against an idle send of that size: {d['vs_idle']} x."

    L.append(verdict("during", s["during"], "One upload while a Program runs"))
    L.append(verdict("batch", s["batch"], "A batch while a Program runs"))
    for side, p in s["parallel"].items():
        gain = ("each went at full speed: sending at once saves time" if p["speedup"] >= 0.8 * p["n"] else
                "no time saved: together they went at the speed of one" if p["speedup"] <= 1.2 else
                "some time saved")
        L.append(f"- **{p['n']} uploads at once to the {side} controller:** {p['stored']} of {p['n']} stored. All were "
                 f"sending together for {p['all_sending_s']} s; together {p['kb_s_together']} KB/s, {p['speedup']} x "
                 f"the idle rate of one ({gain}); each send against an idle send: {p['vs_idle']} x.")
    if s["both_controllers"]:
        c = s["both_controllers"]
        L.append(f"- **Both controllers at once:** {'all stored' if c['stored'] else 'NOT all stored'}; the sends "
                 f"overlapped {c['overlap_s']} s; each against an idle send: {c['vs_idle']} x.")
    for k, r in s["runs"].items():
        L.append(f"- **Run ({k}):** played {r['play_s']} s"
                 + (f" ({r['ms_per_line']} ms a line, the move to the first point included)" if r["ms_per_line"] else "")
                 + (f", {r['vs_baseline_s']:+.2f} s against the baseline" if r["vs_baseline_s"] is not None else "")
                 + (", complete" if r["complete"] else ", NOT complete") + (", PAUSED at some point" if r["paused"] else "")
                 + (f"; the arm reported: {r['arm_reports']}" if r["arm_reports"] else "; no arm error") + ".")
    lk = s["library_lock"]
    if lk["sends"] and lk["longest_write_s"] is None:
        L.append("- **SDK library during a send:** no probe call fell inside a send (the sends were too short to judge).")
    elif lk["sends"]:
        w, sd = lk["longest_write_s"] or 0, lk["longest_send_s"] or 0
        judged = ("the library is held for the whole send: a STOP sent from the uploading process (Lite sends an arm's "
                  "STOP from that arm's own process) would wait for the upload to end" if sd and w >= 0.8 * sd else
                  "the library is held for part of a send" if w > 0.5 else
                  "the library is not held: other calls in the uploading process went on")
        L.append(f"- **SDK library during a send:** the longest write-lock call (tool frame) took {w} s and the longest "
                 f"reading {lk['longest_read_s']} s, against sends of up to {sd} s: {judged}.")
    if s["watcher"]["longest_during_sends_s"] is not None:
        L.append(f"- **The controller and another client:** the run-state reading from the main process took at most "
                 f"{s['watcher']['longest_during_sends_s']} s while uploads were sending.")
    d = s["deletes"]
    if d["n"]:
        L.append(f"- **Delete:** {d['deleted']} of {d['n']} of the bench's programs deleted"
                 + (f", median {d['median_s']:.2f} s each" if d["median_s"] is not None else "")
                 + (f"; not deleted: {'; '.join(d['why'])}" if d["why"] else "") + ".")
    for k in s["skipped"]:
        L.append(f"- **{k['test']}:** not run, {k['why']}.")
    return "\n".join(L) + "\n"


# ── the command line ───────────────────────────────────────────────────────────────────────────────────────────────

def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="mode", required=True)
    lg = sub.add_parser("logs", help="what Lite's logs already show about uploads (no arm)")
    lg.add_argument("--logs", type=Path, default=ROOT / "Resources" / "Logs")
    lg.add_argument("--out", type=Path, help="also write the summary here (.md, and .json beside it)")
    for name in ("slots", "live"):
        p = sub.add_parser(name)
        p.add_argument("--side", choices=("left", "right"), required=True)
        p.add_argument("--ip", help="the controller's address (default: Lite's connection settings)")
        p.add_argument("--port", type=int)
        p.add_argument("--data", default="~/ALIX-Lite", help="Lite's data folder, which holds connection.json")
        p.add_argument("--emulated", action="store_true", help="ALIX's emulator, not a controller (a dry run)")
        p.add_argument("--time-scale", type=float, default=10.0, help="emulated: runs play this many times faster")
    lv = sub.choices["live"]
    lv.add_argument("--slots", type=_slots_arg, required=True, help="slots the bench may write, e.g. 90-95")
    lv.add_argument("--other-slots", type=_slots_arg, default=[], help="the other arm's slots for the parallel test")
    lv.add_argument("--tests", default=",".join(TESTS))
    lv.add_argument("--lines", default="1000,4000,11000", help="the still test files' lengths")
    lv.add_argument("--file", action="append", help="send this Program file instead (repeatable)")
    lv.add_argument("--repeat", type=int, default=3, help="idle: sends of each file")
    lv.add_argument("--parallel", type=int, default=2, help="parallel: connections to the same controller")
    lv.add_argument("--run-slot", type=int, help="run this held Program (it moves the arm) instead of a still one")
    lv.add_argument("--external-run", action="store_true",
                    help="send no run: wait for a run started in Lite and upload during it")
    lv.add_argument("--wait-s", type=float, default=300.0, help="--external-run: how long to wait for each run to start")
    lv.add_argument("--batch", type=int, help="batch: this many sends (at most the free slots)")
    lv.add_argument("--hold-s", type=float, default=60.0, help="the still Program's length, seconds")
    lv.add_argument("--delay-s", type=float, default=2.0, help="uploads start this long into a run")
    lv.add_argument("--run-speed", type=int, default=20, help="the controller's move to the first point, %%")
    lv.add_argument("--restore-speed", type=int, default=50, help="the global speed put back after each run, %%")
    lv.add_argument("--run-timeout", type=float, default=600.0,
                    help="--run-slot: slow-stop after this many seconds; --external-run: stop waiting")
    lv.add_argument("--no-probe", action="store_true", help="no second connection in the uploading process")
    lv.add_argument("--keep", action="store_true", help="leave the bench's files in their slots")
    lv.add_argument("--yes", action="store_true", help="no questions (a --run-slot run still asks)")
    lv.add_argument("--out", help="the results folder")
    a = ap.parse_args(argv)

    if a.mode == "logs":
        s = logs_summary(read_logs(a.logs))
        text = logs_markdown(s)
        print(text)
        if a.out:
            Path(a.out).parent.mkdir(parents=True, exist_ok=True)
            Path(a.out).write_text(text, encoding="utf-8")
            Path(a.out).with_suffix(".json").write_text(json.dumps(s, indent=1, default=str), encoding="utf-8")
        return 0

    _setup(a.emulated, a.time_scale)
    if a.mode == "slots":
        addr = addresses(a)[a.side]
        try:
            h = _open(a.side, *addr)
        except BenchError as exc:
            print(f"** {exc}")
            return 1
        try:
            held = _rmu().slots(h.robot)
            ret, st = h.robot.rm_get_program_run_state()
            print(f"{a.side} controller {addr[0]}: {len(held)} slot(s) in use; run state "
                  f"{(st or {}).get('run_state') if ret == 0 else f'not read (ret {ret})'}")
            for sid, (name, size, speed) in sorted(held.items()):
                print(f"  slot {sid:>3}  {size / 1024:8.1f} KB  speed {speed}  {name}")
        finally:
            _close(h)
        return 0

    a.tests = [t.strip() for t in a.tests.split(",") if t.strip()]
    unknown = set(a.tests) - set(TESTS)
    if unknown:
        ap.error(f"unknown test(s): {', '.join(sorted(unknown))}")
    a.lines = [int(x) for x in a.lines.split(",") if x.strip()]
    if any(n < 2 for n in a.lines) or a.hold_s < 5 or a.parallel < 1 or a.repeat < 1:
        ap.error("--lines are 2 or more, --hold-s 5 s or more, --parallel and --repeat 1 or more")
    if a.other_slots and "parallel" not in a.tests:
        ap.error("--other-slots is for the parallel test")
    if a.external_run and a.run_slot is not None:
        ap.error("--external-run and --run-slot: give one")
    bench = Bench(a)
    rc = 0
    try:
        bench.go()
    except BenchError as exc:
        print(f"\n** {exc}")
        rc = 1
    except KeyboardInterrupt:
        print("\n** stopped (Ctrl+C)")
        rc = 1
    finally:
        try:
            bench.drain()
            if bench.arm is not None and bench.snap and bench.written:
                bench.cleanup()
        except (Exception, KeyboardInterrupt, SystemExit) as exc:                   # noqa: BLE001
            print(f"** the bench's files were not all deleted ({exc}); slots: {sorted(bench.written)}")
        if bench.changed or bench.unread:
            rc = 1
        bench.close()
        if not bench.records:                            # nothing was done: no empty results folder left behind
            (bench.out / "results.jsonl").unlink(missing_ok=True)
            with contextlib.suppress(OSError):
                bench.out.rmdir()
        else:
            s = summarise(bench.records)
            (bench.out / "summary.json").write_text(json.dumps(s, indent=1, default=str), encoding="utf-8")
            text = summary_markdown(s)
            (bench.out / "summary.md").write_text(text, encoding="utf-8")
            print("\n" + text + f"\nresults: {bench.out}")
    return rc


if __name__ == "__main__":
    sys.exit(main())
