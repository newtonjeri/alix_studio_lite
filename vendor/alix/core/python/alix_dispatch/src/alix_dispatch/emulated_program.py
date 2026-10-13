"""The controller's online-program channel, faked for the EMULATOR — attached
at the connect seam, never inside ``rm_emulator.py``.

S3 of the driver/track-channel plan (F109, 2026-09-02). ``rm_emulator.py`` is
Newton's file, verbatim-pinned by ``alix_core/tests/test_emulator_verbatim.py``,
and it has no notion of program slots: none of the five verbs the upload
channel lives on exist there. So the emulated ``RoboticArm`` is WRAPPED here,
by :func:`attach`, which :func:`alix_dispatch.connect.open_arm` calls exactly
when :func:`alix_dispatch.connect.sdk_provenance` says "emulated". Everything
the wrapper does not define is delegated untouched, and nothing here runs
against hardware — a real SDK handle is never wrapped.

What is faked, and where each shape comes from (all hardware-observed on the
Gen-3 V1.7.4 controllers, F98/F108, or read from the vendor's Python wrapper
in ``external/butterfli-realman-alix/.../Robotic_Arm/rm_ctypes_wrap.py``):

* ``rm_send_project(rm_send_project_t)`` -> ``(ret, err_line)``. The file is
  parsed the way the controller parses it: CRLF terminators (an LF-only file
  was rejected ``err_line=1`` on hardware), line 1 ``{"file":DOF}``, line 2 the
  Folder line whose ``type`` must equal the point count, then one
  ``{"point":[7 ints]}`` per line. Success is ``(0, -1)`` — the wrapper
  returns ``err_line`` only on failure. ``-5`` = file unreadable (vendor code).
  The stored item carries the vendor's ``rm_trajectory_data_t`` fields:
  ``id``, ``size`` (bytes), ``speed`` (the plan_speed sent with it),
  ``trajectory_name`` (``f"{save_id}_{basename}"`` cut to the 32-byte field —
  the controller prepends the id, MEASURED 2026-09-10). ``only_save=0``
  starts the program straight away, as on hardware.
* ``rm_get_program_trajectory_list(page_num, page_size, vague_search)`` ->
  ``(0, {page_num, page_size, list_size, vague_search, trajectory_list})`` —
  the ``rm_program_trajectorys_t.to_dict()`` shape. NOTE the item key is
  ``trajectory_name``, not ``name``; ``track.program_list`` read ``name``
  until this double was written against the vendor struct (S3 payoff #1).
* ``rm_set_program_id_run(tra_id, speed, block)``: unknown slot, a program
  already running, or a speed outside 0..100 -> ``1``. ``speed=0`` runs at the
  stored speed. The run speed becomes the PERSISTENT controller-wide global
  ratio (F98: a movej ran ~6x slower after a speed-10 program; there is no
  getter) — modelled here, and applied to every later ``rm_movej`` through
  this surface, so a missing restore is VISIBLE offline. The player performs
  the implicit movej to the first point (blocking, at the run speed), dwells
  ``DWELL_S`` (0.3-0.4 s measured 2026-09-01), then streams every remaining
  point through the controller's passthrough at :data:`REPLAY_CLOCK_S`,
  scaled by the emulator's time scale. Blocking mode (``block != 0``) returns
  ``0`` on completion, ``-4`` when the run was stopped from outside (the
  vendor's documented "stopped externally?" code), ``1`` if the transit was
  refused.
* ``rm_get_program_run_state()`` -> the ``rm_program_run_state_t.to_dict()``
  shape: running ``{run_state:1, id, edit_id, plan_num:1, total_loop:0,
  step_mode:0, plan_speed, loop_num:[], loop_cont:[]}``; idle ``run_state 0,
  id 0`` and — exactly as the vendor wrapper does — NO ``plan_num`` key.
* ``rm_set_plan_speed(pct)``: 1..100 else ``1``; sets the global ratio.
* ``rm_set_arm_stop`` / ``rm_set_arm_slow_stop``: whether the arm-level stops
  abort a running PROGRAM is an ASSUMPTION — hardware gate G4 has not run.
  :attr:`ProgramStore.stop_policy` = ``"aborts"`` (default) models the
  expected answer; ``"ignores"`` models the other, so the executives' abort
  paths can be tested against both. The emulator has no ``rm_set_arm_slow_stop``
  at all, so the slow stop delegates to its ``rm_set_arm_stop``.

Program state lives in a per-controller-address :class:`ProgramStore`, not on
the handle: slots, the global ratio and a running program all survive
``rm_delete_robot_arm`` and a reconnect, as they do on a real controller (the
program KEEPS PLAYING if the client dies — that is why the CLI sends a stop on
KeyboardInterrupt). Observers (:func:`subscribe`) receive the player's
progress so a mock ROS stack can mirror the replay onto its controllers.
"""

from __future__ import annotations

import json
import os
import threading
import time
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional

#: The player's nominal clock (F98). Duplicated from ``track.REPLAY_CLOCK_S``
#: on purpose: this module must import nothing from the analysis side.
REPLAY_CLOCK_S = 0.0055545
#: Pause between the implicit movej settling and the first replayed point
#: (0.3-0.4 s on hardware, 2026-09-01; the run record's ``dwell_s``).
DWELL_S = 0.35
#: The vendor struct's ``trajectory_name`` is ``c_char * 32``, and all 32 bytes
#: are usable — the arm returned `70_70_commode_c_toplid_right_exe`, exactly 32
#: characters with no terminator (2026-09-10). This double used to keep 31.
NAME_FIELD_BYTES = 32
#: ``project_path`` is ``c_char * 300``; ctypes raises on a longer path and so
#: does this stand-in — a deep repo path is a real way to lose an upload.
PATH_FIELD_BYTES = 300

STOP_POLICIES = ("aborts", "ignores")


class rm_send_project_t:
    """Stand-in for the vendor ctypes struct, same fields and defaults
    (``rm_ctypes_wrap.py`` line ~2269): ``project_path`` is stored as BYTES
    like the ``c_char`` array, ``plan_speed`` defaults to 20, everything else
    to 0. Installed into the emulator's ``Robotic_Arm`` modules by
    :func:`attach` only when they are the emulator's."""

    _FIELDS = ("plan_speed", "only_save", "save_id", "step_flag", "auto_start",
               "project_type")

    def __init__(self, project_path: str = None, plan_speed: int = None,
                 only_save: int = None, save_id: int = None, step_flag: int = None,
                 auto_start: int = None, project_type: int = None):
        self.project_path = b""
        self.project_path_len = 0
        for f in self._FIELDS:
            setattr(self, f, 0)
        args = (project_path, plan_speed, only_save, save_id, step_flag,
                auto_start, project_type)
        if all(a is None for a in args):
            return
        if project_path is not None:
            raw = project_path.encode("utf-8")
            if len(raw) >= PATH_FIELD_BYTES:
                raise ValueError(f"bytes too long ({len(raw)}, maximum length "
                                 f"{PATH_FIELD_BYTES - 1})")
            self.project_path = raw
            self.project_path_len = len(raw) + 1
        self.plan_speed = plan_speed if plan_speed is not None else 20
        self.only_save = only_save if only_save is not None else 0
        self.save_id = save_id if save_id is not None else 0
        self.step_flag = step_flag if step_flag is not None else 0
        self.auto_start = auto_start if auto_start is not None else 0
        self.project_type = project_type if project_type is not None else 0


def parse_project_file(path: str) -> tuple:
    """Controller-side parse. Returns ``(err_line, rows)``: ``err_line == -1``
    with the rows on success, else the 1-based offending line (``0`` for a
    length/terminator problem, as the vendor documents) and ``None``."""
    try:
        data = open(path, "rb").read()
    except OSError:
        return None, None                      # caller maps to ret -5
    if not data:
        return 0, None
    # The controller wants CRLF; an LF-only file failed on line 1 (F98 4d).
    if b"\r\n" not in data:
        return 1, None
    lines = data.split(b"\r\n")
    if lines and lines[-1] == b"":
        lines.pop()
    if not lines:
        return 0, None
    try:
        head = json.loads(lines[0])
        dof = int(head["file"])
    except Exception:                                                  # noqa: BLE001
        return 1, None
    if len(lines) < 2:
        return 2, None
    try:
        folder = json.loads(lines[1])
        declared = int(folder["type"])
        if folder.get("name") != "Folder":
            return 2, None
    except Exception:                                                  # noqa: BLE001
        return 2, None
    rows: List[List[int]] = []
    for i, ln in enumerate(lines[2:], 3):
        if b"\n" in ln:                        # a bare LF inside a CRLF file
            return i, None
        try:
            row = json.loads(ln)["point"]
        except Exception:                                                  # noqa: BLE001
            return i, None
        if len(row) != dof or any(not isinstance(v, int) for v in row):
            return i, None
        rows.append([int(v) for v in row])
    if declared != len(rows):
        return 2, None
    return -1, rows


@dataclass
class _Run:
    sid: int
    speed: int
    points: int
    thread: Optional[threading.Thread] = None
    done: threading.Event = field(default_factory=threading.Event)
    abort: threading.Event = field(default_factory=threading.Event)
    ok: Optional[bool] = None
    phase: str = "transit"
    point_index: int = 0
    t_start: float = 0.0
    t_transit_done: Optional[float] = None
    t_end: Optional[float] = None


class ProgramStore:
    """One controller's program memory: slots, global ratio, the running
    program, the stop policy, observers. Keyed by controller address in
    :data:`_STORES` so it outlives any handle."""

    def __init__(self, ip: str):
        self.ip = ip
        self.slots: Dict[int, dict] = {}
        self.global_ratio: int = 100
        self.stop_policy: str = "aborts"
        self.edit_id: int = 0
        self.run: Optional[_Run] = None
        self.observers: List[Callable[[str, dict], None]] = []
        self.lock = threading.RLock()
        #: every rm_set_program_id_run call, accepted or not: (sid, speed, ret)
        self.run_calls: List[tuple] = []
        #: every completed/aborted run: dicts with sid, speed, ok, transit_s,
        #: replay_s, points_played
        self.history: List[dict] = []

    def running(self) -> Optional[_Run]:
        r = self.run
        return r if (r is not None and not r.done.is_set()) else None

    def emit(self, event: str, payload: dict) -> None:
        for cb in list(self.observers):
            try:
                cb(event, payload)
            except Exception:                                              # noqa: BLE001
                pass


_STORES: Dict[str, ProgramStore] = {}
_STORES_LOCK = threading.Lock()


def store_for(ip: str) -> ProgramStore:
    with _STORES_LOCK:
        s = _STORES.get(ip)
        if s is None:
            s = _STORES[ip] = ProgramStore(ip)
        return s


def reset(ip: Optional[str] = None) -> None:
    """Forget program memory (tests). Aborts a running player first."""
    with _STORES_LOCK:
        targets = [_STORES[ip]] if ip in _STORES else ([] if ip else list(_STORES.values()))
    for s in targets:
        r = s.running()
        if r is not None:
            r.abort.set()
            r.done.wait(5.0)
    with _STORES_LOCK:
        if ip is None:
            _STORES.clear()
        else:
            _STORES.pop(ip, None)


def subscribe(ip: str, cb: Callable[[str, dict], None]) -> Callable[[], None]:
    """Observe the player at *ip*. ``cb(event, payload)`` with events
    ``start`` {id, speed, points}, ``transit_done`` {}, ``point`` {i, deg},
    ``end`` {ok, transit_s, replay_s}, ``aborted`` {i}. Returns the
    unsubscribe callable."""
    s = store_for(ip)
    s.observers.append(cb)

    def off():
        try:
            s.observers.remove(cb)
        except ValueError:
            pass
    return off


def global_ratio(ip: str) -> int:
    """The controller-wide ratio the double holds for *ip* — the getter the
    real controller does not have, for tests that prove the restore."""
    return store_for(ip).global_ratio


def set_stop_policy(ip: str, policy: str) -> None:
    if policy not in STOP_POLICIES:
        raise ValueError(f"stop_policy must be one of {STOP_POLICIES}")
    store_for(ip).stop_policy = policy


def _emu():
    from alix_emulator import rm_emulator
    return rm_emulator


class ProgramSurface:
    """The emulated ``RoboticArm`` plus the program verbs. Attribute access
    not defined here goes to the wrapped instance, so callers cannot tell the
    two apart — except through ``_emulated_inner``, which
    :func:`alix_dispatch.connect.sdk_provenance` unwraps."""

    def __init__(self, robot, ip: str):
        object.__setattr__(self, "_emulated_inner", robot)
        object.__setattr__(self, "_emu_ip", ip)
        object.__setattr__(self, "_store", store_for(ip))

    def __getattr__(self, name):
        return getattr(self._emulated_inner, name)

    def __setattr__(self, name, value):
        setattr(self._emulated_inner, name, value)

    # ── controller-side helpers ──
    def _ctrl(self):
        return getattr(self._emulated_inner, "_ctrl", None)

    def _latency(self) -> None:
        c = self._ctrl()
        lat = getattr(c, "command_latency_s", 0.0) if c is not None else 0.0
        if lat:
            time.sleep(_emu()._scaled(lat))

    # ── the five verbs ──
    def rm_send_project(self, sp) -> tuple:
        self._latency()
        path = sp.project_path
        if isinstance(path, bytes):
            path = path.decode("utf-8", "replace")
        if not path:
            return -4, 0
        err_line, rows = parse_project_file(path)
        if err_line is None:
            return -5, 0
        if err_line != -1:
            return 1, err_line
        sid = int(sp.save_id)
        if not 1 <= sid <= 100:
            return 1, 0
        # THE CONTROLLER PREPENDS THE PROGRAM ID (measured 2026-09-10, the first
        # program list ever fetched from an arm). This double stored the bare
        # basename, which is what our own H25 check expected — so double and
        # check agreed with each other and BOTH disagreed with the arm, the one
        # failure mode a stand-in is supposed to prevent. Hardware is the
        # authority: `id 90: 90_project.txt` proves the prefix is the id and not
        # something in the filename, and Newton's pendant programs obey it too
        # (`1_Hand_test_new.txt`). Full 32 bytes are usable, not 31:
        # `70_70_commode_c_toplid_right_exe` came back exactly 32 with no NUL.
        # And the extension is the CONTROLLER's: slot 60 holds the bytes of the
        # `.proj` we sent (50.9 KB) under the name `..._020.txt`, so it strips
        # what it was given and stores every program as a .txt.
        stem = os.path.splitext(os.path.basename(path))[0]
        name = f"{sid}_{stem}.txt".encode("utf-8")[:NAME_FIELD_BYTES]
        item = {"id": sid, "size": os.path.getsize(path),
                "speed": int(sp.plan_speed),
                "trajectory_name": name.decode("utf-8", "replace"),
                "rows": rows}
        store = self._store
        with store.lock:
            store.slots[sid] = item
            store.edit_id = sid
        if int(sp.only_save) == 0:
            ret = self.rm_set_program_id_run(sid, int(sp.plan_speed), 0)
            if ret != 0:
                return 1, 0
        return 0, -1

    def rm_get_program_trajectory_list(self, page_num: int, page_size: int,
                                       vague_search: str) -> tuple:
        self._latency()
        store = self._store
        with store.lock:
            items = [dict(id=v["id"], size=v["size"], speed=v["speed"],
                          trajectory_name=v["trajectory_name"])
                     for _, v in sorted(store.slots.items())]
        if vague_search:
            items = [it for it in items if vague_search in it["trajectory_name"]]
        page_num = max(int(page_num), 1)
        page_size = max(int(page_size), 1)
        lo = (page_num - 1) * page_size
        page = items[lo:lo + page_size]
        return 0, {"page_num": page_num, "page_size": page_size,
                   "list_size": len(page), "vague_search": str(vague_search or ""),
                   "trajectory_list": page}

    def rm_set_plan_speed(self, speed: int) -> int:
        self._latency()
        speed = int(speed)
        if not 1 <= speed <= 100:
            return 1
        with self._store.lock:
            self._store.global_ratio = speed
        return 0

    def rm_get_program_run_state(self) -> tuple:
        self._latency()
        store = self._store
        with store.lock:
            r = store.running()
            if r is not None:
                return 0, {"run_state": 1, "id": r.sid, "edit_id": store.edit_id,
                           "plan_num": 1, "total_loop": 0, "step_mode": 0,
                           "plan_speed": r.speed, "loop_num": [], "loop_cont": []}
            return 0, {"run_state": 0, "id": 0, "edit_id": store.edit_id,
                       "total_loop": 0, "step_mode": 0,
                       "plan_speed": store.global_ratio, "loop_num": [], "loop_cont": []}

    def rm_set_program_id_run(self, tra_id: int, speed: int, timeout: int) -> int:
        self._latency()
        store = self._store
        sid, speed = int(tra_id), int(speed)
        with store.lock:
            item = store.slots.get(sid)
            if item is None or store.running() is not None or not 0 <= speed <= 100:
                store.run_calls.append((sid, speed, 1))
                return 1
            speed_eff = speed if speed else int(item["speed"])
            if not 1 <= speed_eff <= 100:
                store.run_calls.append((sid, speed, 1))
                return 1
            # F98: the run speed becomes the persistent global ratio.
            store.global_ratio = speed_eff
            run = _Run(sid=sid, speed=speed_eff, points=len(item["rows"]))
            store.run = run
            store.run_calls.append((sid, speed, 0))
            rows = item["rows"]
        ctrl = self._ctrl()
        run.thread = threading.Thread(target=self._play, args=(run, rows, ctrl),
                                      name=f"emu-program-{store.ip}-{sid}", daemon=True)
        run.thread.start()
        if timeout:
            # Triple-thread mode: block=1 waits for the end. Bounded anyway,
            # so a wedged player cannot hang a test forever.
            bound = len(rows) * REPLAY_CLOCK_S / _emu()._time_scale * 3 + 60.0
            if not run.done.wait(bound):
                return -2
            if run.abort.is_set():
                return -4
            return 0 if run.ok else 1
        return 0

    def _play(self, run: _Run, rows: List[List[int]], ctrl) -> None:
        store = self._store
        emu = _emu()
        run.t_start = time.monotonic()
        store.emit("start", {"id": run.sid, "speed": run.speed, "points": len(rows)})
        first = [v / 1000.0 for v in rows[0]]
        try:
            # The implicit movej to the first point, blocking, at the run speed.
            # Driven on the CONTROLLER object so the program keeps playing when
            # the client handle is deleted, as on hardware.
            if ctrl is not None:
                ret = ctrl.movej(first, run.speed, 1)
            else:
                ret = self._emulated_inner.rm_movej(first, run.speed, 0, 0, 1)
            if ret != 0:
                run.ok = False
                run.t_end = time.monotonic()
                store.emit("end", {"ok": False, "reason": f"transit movej ret={ret}",
                                   "transit_s": None, "replay_s": None})
                return
            if run.abort.wait(emu._scaled(DWELL_S)):
                self._finish_aborted(run, 0)
                return
            run.t_transit_done = time.monotonic()
            run.phase = "replay"
            store.emit("transit_done", {"t": run.t_transit_done - run.t_start})
            dt = emu._scaled(REPLAY_CLOCK_S)
            t0 = time.monotonic()
            for i, row in enumerate(rows):
                if run.abort.is_set():
                    self._finish_aborted(run, i)
                    return
                deg = [v / 1000.0 for v in row]
                if ctrl is not None:
                    r = ctrl.movej_canfd(deg)
                else:
                    r = self._emulated_inner.rm_movej_canfd(deg, True)
                run.point_index = i
                store.emit("point", {"i": i, "deg": deg, "ret": r})
                due = t0 + (i + 1) * dt
                rem = due - time.monotonic()
                if rem > 0:
                    if run.abort.wait(rem):
                        self._finish_aborted(run, i)
                        return
            run.ok = True
            run.t_end = time.monotonic()
            rec = {"sid": run.sid, "speed": run.speed, "ok": True,
                   "transit_s": run.t_transit_done - run.t_start,
                   "replay_s": run.t_end - run.t_transit_done,
                   "points_played": len(rows)}
            with store.lock:
                store.history.append(rec)
            store.emit("end", rec)
        finally:
            run.done.set()

    def _finish_aborted(self, run: _Run, i: int) -> None:
        run.ok = False
        run.t_end = time.monotonic()
        rec = {"sid": run.sid, "speed": run.speed, "ok": False,
               "transit_s": ((run.t_transit_done - run.t_start)
                             if run.t_transit_done else None),
               "replay_s": ((run.t_end - run.t_transit_done)
                            if run.t_transit_done else None),
               "points_played": i}
        with self._store.lock:
            self._store.history.append(rec)
        self._store.emit("aborted", {"i": i, **rec})

    # ── arm-level stops: policy-gated program abort, then the emulator's stop ──
    def _abort_if_policy(self) -> None:
        store = self._store
        if store.stop_policy != "aborts":
            return
        r = store.running()
        if r is not None:
            r.abort.set()
            r.done.wait(2.0)

    def rm_set_arm_stop(self) -> int:
        self._abort_if_policy()
        return self._emulated_inner.rm_set_arm_stop()

    def rm_set_arm_slow_stop(self) -> int:
        self._abort_if_policy()
        inner = self._emulated_inner
        fn = getattr(inner, "rm_set_arm_slow_stop", None) or inner.rm_set_arm_stop
        return fn()

    # ── the global ratio scales every planned move through this surface ──
    def rm_movej(self, joint, v, r, connect, block):
        ratio = self._store.global_ratio
        v_eff = max(1, int(round(int(v) * ratio / 100.0)))
        return self._emulated_inner.rm_movej(joint, v_eff, r, connect, block)


def attach(robot, ip: str):
    """Wrap an EMULATED handle with the program surface and make sure the
    emulator's ``Robotic_Arm`` modules carry ``rm_send_project_t``. Refuses a
    real SDK handle (the class module is the proof, as in
    :func:`alix_dispatch.connect.sdk_provenance`)."""
    if isinstance(robot, ProgramSurface):
        return robot
    if not type(robot).__module__.startswith("alix_emulator"):
        raise TypeError("attach(): refusing to fake program verbs on a non-emulated "
                        f"handle ({type(robot).__module__}.{type(robot).__name__})")
    import sys
    for modname in ("Robotic_Arm.rm_robot_interface", "Robotic_Arm.rm_ctypes_wrap"):
        mod = sys.modules.get(modname)
        if mod is None:
            continue
        ra = getattr(mod, "RoboticArm", None)
        if ra is None or not getattr(ra, "__module__", "").startswith("alix_emulator"):
            continue                       # the real SDK: never touched
        if not hasattr(mod, "rm_send_project_t"):
            setattr(mod, "rm_send_project_t", rm_send_project_t)
    return ProgramSurface(robot, ip)
