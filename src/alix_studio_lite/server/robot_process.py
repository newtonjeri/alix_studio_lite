"""Each arm's controller connection in a process of its own (the owner, 19 Oct: "find the root cause and fix it,
so the connection recovers without manually killing the process").

The lab, 17 and 19 Oct: a call into the RealMan SDK's library never came back, four times in three days. Python
cannot interrupt a thread inside a C call, and while it hung, STOP through the same handle was never confirmed and
new connections hung too: the SDK's state is process-wide. So the SDK lives in a child process per arm, the ARM
WORKER, which runs today's link and commands unchanged (``robot_link.ArmLink`` reads the controller five times a
second; ``robot_real.RealArm`` stages, teaches, uploads, runs, moves and streams), and the server talks to them
through two proxies with the same surface:

- ``LinkProcess`` stands in ``robot.links[side]`` for an ``ArmLink``: ``view()``, ``feedback``, ``read_fresh()``,
  ``stop()``, ``leave_drag()``, ``estop_now()``, ``reserved()``, ``stuck()``, ``close()``, ``abandon()``;
- ``RealArmProxy`` stands for a ``RealArm``: every command method, run in the worker, its hardware log lines and
  alerts written by this process's logbook, its runner lines (``on_line``) and STOP hooks (``stop_hook``) relayed.

One command at a time per arm, as before: a second caller waits CMD_WAIT_S for the first, then is refused "busy"
with what holds the controller (the proxy's own lock; the worker runs commands one after the other). The worker
pushes the link's view with every reading and every half second (a heartbeat), with the link's own ``stuck()``
(robot_link): a call that has not come back. The server's watch (robot_standin._recover) then gives the connection
up: ``abandon()`` KILLS the worker, which ends the stuck call with it and closes the controller's socket, and the
arm is connected again in a fresh process, with fresh SDK state, while the other arm's worker is untouched. STOP
pressed meanwhile is sent through the new one.

Three pipes per worker: commands (one at a time, with their events and result), control (STOP, leaving drag
teaching, the STOP hook, close: a thread of the worker's own, never behind a command) and the feed (views and
heartbeats). ``LITE_LINK_PROCESS=0`` keeps the links in this process (robot_standin.LINK_PROCESS: the tests of the
in-process path, or a diagnosis). ``LITE_WORKER_SETUP=<file.py>`` names a module whose ``setup()`` the worker runs
before it opens the link (the emulated Lite's shims, tests/emulated_lite.py; a fake controller in the tests).

Ctrl+C in Lite's terminal reaches every process started from it, the workers too, and it is the server's alone: its
shutdown sends STOP through each worker, gives each controller its speed back, then closes the workers. A worker
ignores SIGINT, from its very start: the server starts it with SIGINT held (``open``) and the worker sets it aside
before letting it through (the lab, 21 Oct: both workers died of the Ctrl+C with a traceback each, and the
shutdown's STOP found no process to pass through, "STOP not confirmed" on both arms).
"""
from __future__ import annotations

import multiprocessing as mp
import os
import signal
import threading
import time
from contextlib import contextmanager
from pathlib import Path

from .jobs import UserError
from .robot_link import CALL_STUCK_S, CLOSE_WAIT_S, CMD_WAIT_S

HEARTBEAT_S = 0.5                  # the worker pushes the link's view at least this often
WORKER_LOST_S = 10.0               # no view from the worker for this long: the connection's process is gone or stuck
OPEN_WAIT_S = 25.0                 # the worker's own bound on opening (robot_standin waits CONNECT_WAIT_S and kills a late one)
CLOSE_JOIN_S = 3.0

_PROCS: list = []                  # every worker started, for kill_all at a hard exit
_PROCS_LOCK = threading.Lock()


def _err(exc) -> dict:
    """An exception as it crosses to the server: a UserError whole; anything else with where it was raised."""
    import traceback

    where = None
    if not isinstance(exc, UserError):
        tb = traceback.extract_tb(exc.__traceback__)
        where = " <- ".join(f"{Path(f.filename).name}:{f.lineno} {f.name}" for f in tb[-3:]) if tb else None
    return dict(type=type(exc).__name__, text=str(exc), hint=getattr(exc, "hint", None), code=getattr(exc, "code", None), where=where)


def _raise(e: dict):
    if e.get("type") == "UserError":
        raise UserError(e["text"], e.get("hint"), code=e.get("code"))
    raise RuntimeError(f"{e.get('type')}: {e.get('text')}" + (f" (in the arm's process at {e['where']})" if e.get("where") else ""))


def kill_all() -> None:
    """Every arm worker ended at once (Lite's hard exit: nothing may hold the terminal)."""
    with _PROCS_LOCK:
        procs = list(_PROCS)
    for p in procs:
        try:
            if p.is_alive():
                p.kill()
        except Exception:                                                   # noqa: BLE001
            pass


# ── the worker process ──────────────────────────────────────────────────────────────────────────────────────────

class _LogForwarder:
    """The worker's logbook: every record sent to the server's (robot_real logs through it)."""

    def __init__(self, send):
        self._send = send

    def hardware(self, side, verb, *, args=None, ret=None, ok=None, elapsed_s=None, text=None) -> None:
        self._send(("log", "hardware", dict(side=side, verb=verb, args=args, ret=ret, ok=ok, elapsed_s=elapsed_s, text=text)))

    def alert(self, level, text, *, hint=None, side=None, source=None) -> dict:
        self._send(("log", "alert", dict(level=level, text=text, hint=hint, side=side, source=source)))
        return {}

    def activity(self, what, **kw) -> None:
        self._send(("log", "activity", dict(what=what, **{k: v for k, v in kw.items() if k in ("who", "detail", "ok")})))

    def hw_logger(self, side, prefix: str = ""):
        return lambda text: self._send(("log", "hwline", dict(side=side, prefix=prefix, text=str(text))))


def worker_main(side: str, ip: str, port: int, kind: str, setup: str | None, log_root: str | None, cmd, ctl, feed) -> None:
    """The arm worker: the link opened, then commands one after the other (``cmd``), STOP and the like on their
    own thread (``ctl``), the link's view pushed on ``feed``."""
    # Ctrl+C is the server's (the module's docstring): ignored here first, then let through, so one that came while
    # this process was starting (held since open) is dropped, never raised. A STOP mid-stream is an exception raised
    # in the streaming thread (robot_real._interrupt), not a signal; the tuning pool's worker does the same (workers.py).
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    if hasattr(signal, "pthread_sigmask"):
        signal.pthread_sigmask(signal.SIG_UNBLOCK, {signal.SIGINT})
    from .. import vendor_paths

    vendor_paths.activate()
    if setup:                                                # the shims first: they wrap what the emulator installs
        import importlib.util

        spec = importlib.util.spec_from_file_location("lite_worker_setup", setup)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        if hasattr(mod, "setup"):
            mod.setup()
    if kind == "emulated":                                   # ALIX's controller emulator, before the SDK is imported
        import alix_emulator

        alix_emulator.install()
    from .logbook import dump_threads
    from .robot_link import ArmLink
    from .robot_real import RealArm

    send_lock = threading.Lock()

    def push(conn, msg) -> None:
        with send_lock:
            try:
                conn.send(msg)
            except Exception:                                               # noqa: BLE001 - the server is gone
                pass

    state = dict(hook=None, dumped=False)

    def report() -> None:
        """The link's view to the server, with every reading and every heartbeat; the first stuck call seen writes this
        process's threads (the reading goes on during a command since 26 Oct, so a reading may see it before the heartbeat
        does, and the server gives the connection up as soon as it is told)."""
        stuck = link.stuck()
        push(feed, ("view", link.view(), stuck, link.in_flight()))
        if stuck and not state["dumped"] and log_root:               # what this process's threads were doing
            state["dumped"] = True
            dump_threads(log_root, prefix=f"threads-{side}-")

    link = ArmLink(side, ip, port, on_read=report)
    try:
        link.open()
    except BaseException as exc:                                            # noqa: BLE001
        push(cmd, ("open_error", _err(exc)))
        return
    left = None
    if not link.emulated:                        # the controller's realtime push left on toward this computer (udp_push)
        from .udp_push import left_on

        try:
            with link.busy("reading the realtime push"):
                left = left_on(link.handle)
        except Exception as exc:                                            # noqa: BLE001 - never keeps the arm from connecting
            left = dict(error=f"{type(exc).__name__}: {exc}")
    if left and left.get("cleared"):             # turned off, but this process's SDK has bound its port: a fresh process
        push(cmd, ("reopen", left))
        try:
            link.close()
        finally:
            os._exit(0)
    push(cmd, ("opened", dict(emulated=link.emulated, view=link.view(), push=left)))
    log = _LogForwarder(lambda msg: push(cmd, msg))
    arm = RealArm(link, log)
    stop_event = threading.Event()

    def heartbeat() -> None:
        while not stop_event.wait(HEARTBEAT_S):
            report()

    def control() -> None:
        while True:
            try:
                msg = ctl.recv()
            except (EOFError, OSError):
                return
            what = msg[0]
            try:
                if what == "stop":
                    out = link.stop(hard=bool(msg[1]), drag=bool(msg[2]))
                elif what == "leave_drag":
                    out = link.leave_drag()
                elif what == "estop":                # IO3 read between a command's steps: the command waits on its line meanwhile
                    out = link.estop_now()
                elif what == "hook":
                    fn = state.get("hook")
                    out = None
                    if fn is not None:
                        fn()
                elif what == "close":
                    stop_event.set()
                    link.close()
                    push(ctl, ("result", None))
                    os._exit(0)
                else:
                    out = None
                push(ctl, ("result", out))
            except BaseException as exc:                                    # noqa: BLE001
                push(ctl, ("error", _err(exc)))

    threading.Thread(target=heartbeat, name="lite-worker-heartbeat", daemon=True).start()
    threading.Thread(target=control, name="lite-worker-control", daemon=True).start()
    while True:
        try:
            msg = cmd.recv()
        except (EOFError, OSError):                                         # the server is gone: so is this process
            stop_event.set()
            try:
                link.close()
            finally:
                os._exit(0)
        if msg[0] != "call":                         # a line's ack left unread (a STOP in its handshake): not a command
            continue
        _, method, args, kwargs, want_lines, want_hook = msg
        kwargs = dict(kwargs)

        def on_line(line):
            """A runner line: sent, and the command waits until the server has acted on it (its callback runs between
            the runner's steps, as it did in one process: the E-stop read there is never alongside a call)."""
            push(cmd, ("line", str(line)))
            try:
                if cmd.poll(60.0):
                    cmd.recv()                       # ("ack",)
            except (EOFError, OSError):
                pass

        if want_lines:
            kwargs["on_line"] = on_line
        if want_hook:
            kwargs["stop_hook"] = lambda fn: state.__setitem__("hook", fn)
        try:
            target = link if method in ("read_fresh", "estop_now") else arm
            out = getattr(target, method)(*args, **kwargs)
            push(cmd, ("result", out))
        except BaseException as exc:                                        # noqa: BLE001
            push(cmd, ("error", _err(exc)))
        finally:
            state["hook"] = None


# ── the proxies, in the server ──────────────────────────────────────────────────────────────────────────────────

class LinkProcess:
    """An arm's connection in its worker process: what ``robot_link.ArmLink`` is to the server."""

    def __init__(self, side: str, ip: str, port: int = 8080, *, kind: str = "realman", on_read=None, log=None):
        self.side, self.ip, self.port, self.kind = side, ip, int(port), kind
        self.on_read, self.log = on_read, log
        self.emulated = False
        self.abandoned = False
        self.push_on: dict | None = None                 # the controller's realtime push found left on as it connected
        self._proc = None
        self._cmd = self._ctl = self._feed = None
        self._lock = threading.RLock()                   # one command at a time (busy); reentrant for reserved()
        self._ctl_lock = threading.Lock()
        self._state = threading.Lock()
        self._view: dict = dict(ip=ip, port=port, emulated=False, abandoned=False, feedback={}, quality=dict(level="none", text="not open"))
        self._stuck = None
        self._flight = None
        self._seen = 0.0
        self._busy: str | None = None                    # the command this process has in flight on the worker
        self._dead: str | None = None
        self._hooks: list = []

    # -- life
    def open(self, _again: bool = False) -> "LinkProcess":
        self.push_on = None
        ctx = mp.get_context("spawn")
        cmd_a, cmd_b = ctx.Pipe()
        ctl_a, ctl_b = ctx.Pipe()
        feed_a, feed_b = ctx.Pipe(duplex=False)
        setup = os.environ.get("LITE_WORKER_SETUP") or None
        root = str(self.log.root) if self.log is not None else None
        self._proc = ctx.Process(target=worker_main, name=f"lite-arm-{self.side}",
                                 args=(self.side, self.ip, self.port, self.kind, setup, root, cmd_b, ctl_b, feed_b), daemon=True)
        # The worker starts with Ctrl+C held (it inherits this thread's signal mask) until worker_main ignores it; this
        # thread's mask is put back at once. multiprocessing's resource tracker, which the first spawn starts, ends its
        # own start by letting SIGINT through in the calling thread (its guard for the same race, bpo-33613): it is
        # started first, outside the hold, or the worker would start with Ctrl+C let through.
        from multiprocessing import resource_tracker

        resource_tracker.ensure_running()
        held = signal.pthread_sigmask(signal.SIG_BLOCK, {signal.SIGINT}) if hasattr(signal, "pthread_sigmask") else None
        try:
            self._proc.start()
        finally:
            if held is not None:
                signal.pthread_sigmask(signal.SIG_SETMASK, held)
        with _PROCS_LOCK:
            _PROCS[:] = [p for p in _PROCS if p.is_alive()] + [self._proc]
        cmd_b.close(), ctl_b.close(), feed_b.close()
        self._cmd, self._ctl, self._feed = cmd_a, ctl_a, feed_a
        if not cmd_a.poll(OPEN_WAIT_S):
            self.abandon()
            raise UserError(f"The {self.side.upper()} controller did not answer within {OPEN_WAIT_S:g} s, so the arm is not connected.",
                            "Check its power and cable.")
        try:
            msg = cmd_a.recv()
        except (EOFError, OSError):
            self.abandon()
            raise UserError(f"The {self.side.upper()} arm's connection process ended before it connected.",
                            "Its traceback is in Lite's terminal; start Lite from a terminal to read it.") from None
        if msg[0] == "reopen" and not _again:   # the push was left on toward this computer (udp_push): turned off, and
            self._proc.join(CLOSE_JOIN_S)       # that process's SDK holds its port; a fresh one binds nothing
            if self._proc.is_alive():
                self._proc.kill()
                self._proc.join(1.0)
            for c in (cmd_a, ctl_a, feed_a):
                c.close()
            self._say_push(msg[1], reopened=True)
            return self.open(_again=True)
        if msg[0] != "opened":
            self.abandon()
            _raise(msg[1] if msg[0] != "reopen" else dict(
                type="UserError", text=f"The {self.side.upper()} controller's 5 ms feed was on toward this computer again after Lite "
                "turned it off, so the arm is not connected.", hint="Turn it off on the pendant (realtime push), then connect the arm again."))
        self.emulated = bool(msg[1]["emulated"])
        self._take_view(msg[1]["view"], None, None)
        threading.Thread(target=self._feed_loop, name=f"lite-feed-{self.side}", daemon=True).start()
        if msg[1].get("push"):
            self._say_push(msg[1]["push"], reopened=False)
        return self

    def _say_push(self, info: dict, reopened: bool) -> None:
        """What the push check found as the arm connected (udp_push.left_on): in the hardware log; an alert when the
        lid and seat tasks cannot stream on this arm."""
        from .udp_push import held_text

        if not info.get("error") and not reopened:     # left on (another program holds its port, or it would not turn off):
            self.push_on = dict(info)                    # the controller's programs run some 13 % slower (S2 Real refuses it)
        if self.log is None:
            return
        S, port = self.side.upper(), info.get("port")
        if info.get("error"):
            self.log.hardware(self.side, "realtime push", ok=False, text=f"not checked: {info['error']}")
        elif reopened:
            self.log.hardware(self.side, "realtime push", ok=True,
                              text=f"left on toward this computer (port {port}), so this arm's connection had bound the port its "
                                   "CanFD capture needs: turned off, and the arm connected again in a fresh process")
        elif info.get("holders"):
            self.log.alert("warn", f"{S}: the controller's 5 ms feed goes to this computer's port {port}, which another program "
                           f"holds ({held_text(info['holders'])}), so the lid and seat tasks cannot stream on this arm while it runs.",
                           hint="Stop it (ALIX's driver or one of its tools), then connect the arm again.", side=self.side, source="push")
        else:
            self.log.alert("warn", f"{S}: the controller's 5 ms feed was left on toward this computer (port {port}) and could not be "
                           "turned off, so the lid and seat tasks cannot stream on this arm. " + " ".join(info.get("said") or []),
                           hint="Turn it off on the pendant (realtime push), then connect the arm again.", side=self.side, source="push")

    def close(self) -> None:
        """The worker closes its link (never under a call: robot_link) and ends; a worker that does not end is killed."""
        if self._proc is None:
            return
        try:
            with self._ctl_lock:
                self._ctl.send(("close",))
                self._ctl.poll(CLOSE_JOIN_S)
        except Exception:                                                   # noqa: BLE001
            pass
        self._proc.join(CLOSE_JOIN_S)
        if self._proc.is_alive():
            self._proc.kill()
            self._proc.join(1.0)
        self._dead = self._dead or "closed"

    def abandon(self) -> None:
        """Given up: a call that never came back dies with the process, and the controller's socket closes with it."""
        self.abandoned = True
        self._dead = self._dead or "abandoned"
        if self._proc is not None and self._proc.is_alive():
            self._proc.kill()
            self._proc.join(1.0)

    # -- the feed
    def _take_view(self, view: dict, stuck, flight) -> None:
        with self._state:
            self._view, self._stuck, self._flight, self._seen = view, stuck, flight, time.monotonic()

    def _feed_loop(self) -> None:
        while True:
            try:
                msg = self._feed.recv()
            except (EOFError, OSError):
                with self._state:
                    self._dead = self._dead or "ended"
                return
            if msg[0] == "view":
                self._take_view(msg[1], msg[2], msg[3])
                if self.on_read is not None:
                    try:
                        self.on_read()
                    except Exception:                                       # noqa: BLE001
                        pass

    @property
    def feedback(self) -> dict:
        with self._state:
            return dict(self._view.get("feedback") or {})

    def in_flight(self):
        with self._state:
            return self._flight

    def stuck(self):
        """The worker's own verdict (robot_link.ArmLink.stuck), or a worker that has gone silent."""
        with self._state:
            stuck, seen, dead = self._stuck, self._seen, self._dead
        if stuck:
            return tuple(stuck)
        silent = time.monotonic() - seen if seen else 0.0
        if dead and dead != "closed":
            return ("the connection's process", silent)
        if seen and silent > max(WORKER_LOST_S, CALL_STUCK_S):
            return ("the connection's process", silent)
        return None

    def view(self) -> dict:
        with self._state:
            v = dict(self._view)
            seen, dead, busy = self._seen, self._dead, self._busy
        v["abandoned"] = self.abandoned
        silent = time.monotonic() - seen if seen else None
        v["view_age_s"] = None if silent is None else round(silent, 3)       # how long ago the worker pushed it
        if dead and dead != "closed":
            v["quality"] = dict(level="none", text="the connection's process ended" if dead == "ended" else "the connection was given up")
        elif silent is not None and silent > WORKER_LOST_S:
            v["quality"] = dict(level="none", text=f"no word from the connection's process for {silent:.0f} s" + (f" ({busy})" if busy else ""))
        return v

    def quality(self) -> dict:
        return self.view()["quality"]

    # -- commands (one at a time, as robot_link.ArmLink.busy)
    def _take(self, wait_s: float | None = None) -> None:
        if not self._lock.acquire(timeout=CMD_WAIT_S if wait_s is None else wait_s):
            raise UserError(f"The {self.side.upper()} controller is busy: {self._busy or 'another command'}.",
                            "Try again once it has finished.", code="busy")

    @contextmanager
    def reserved(self, wait_s: float | None = None):
        """This controller held for the caller's next commands (S2's TOGETHER stage takes both arms' first)."""
        self._take(wait_s)
        try:
            yield
        finally:
            self._lock.release()

    def call(self, method: str, args: tuple = (), kwargs: dict | None = None, *, label: str | None = None,
             on_line=None, stop_hook=None, wait_s: float | None = None):
        """One command in the worker: its hardware log written here as it comes, its lines relayed, its result or
        its error returned. A STOP hook the command registers is reachable through ``hook()``."""
        self._take(wait_s)
        try:
            if self._dead:
                raise UserError(f"The {self.side.upper()} controller's connection was given up ({self._dead}): the arm must be "
                                "connected again.", "Press Connect for it, or wait for Lite to connect it again.")
            if self._busy:                           # re-entered from a callback of the command in flight: never on the same pipe
                raise UserError(f"The {self.side.upper()} controller is busy: {self._busy}.", "Try again once it has finished.", code="busy")
            self._busy = label or method
            try:
                self._cmd.send(("call", method, tuple(args), dict(kwargs or {}), on_line is not None, stop_hook is not None))
                if stop_hook is not None:
                    stop_hook(self.hook)
                while True:
                    try:
                        msg = self._cmd.recv()
                    except (EOFError, OSError):
                        with self._state:
                            self._dead = self._dead or "ended"
                        raise UserError(f"The {self.side.upper()} controller's connection ended during {self._busy}: the arm must "
                                        "be connected again.", "Wait for Lite to connect it again, then try again.") from None
                    kind = msg[0]
                    if kind == "result":
                        return msg[1]
                    if kind == "error":
                        _raise(msg[1])
                    if kind == "line":               # the worker's runner waits for this to have run
                        try:
                            if on_line is not None:
                                on_line(msg[1])
                        except Exception as exc:                             # noqa: BLE001 - a callback never breaks the command
                            self._log("hardware", dict(side=self.side, verb="line callback", ok=False, text=repr(exc)))
                        finally:
                            try:
                                self._cmd.send(("ack",))
                            except (EOFError, OSError):
                                pass
                    elif kind == "log":
                        self._log(msg[1], msg[2])
            finally:
                self._busy = None
        finally:
            self._lock.release()

    def _log(self, kind: str, d: dict) -> None:
        if self.log is None:
            return
        try:
            if kind == "hardware":
                self.log.hardware(d["side"], d["verb"], args=d.get("args"), ret=d.get("ret"), ok=d.get("ok"),
                                  elapsed_s=d.get("elapsed_s"), text=d.get("text"))
            elif kind == "alert":
                self.log.alert(d["level"], d["text"], hint=d.get("hint"), side=d.get("side"), source=d.get("source"))
            elif kind == "hwline":
                self.log.hw_logger(d["side"], d.get("prefix") or "")(d["text"])
            elif kind == "activity":
                self.log.activity(d["what"], **{k: v for k, v in d.items() if k != "what"})
        except Exception:                                                   # noqa: BLE001 - a log never stops a command
            pass

    def _control(self, msg: tuple, wait_s: float):
        with self._ctl_lock:
            try:
                self._ctl.send(msg)
                if not self._ctl.poll(wait_s):
                    return None
                out = self._ctl.recv()
            except (EOFError, OSError):
                return None
        if out[0] == "error":
            _raise(out[1])
        return out[1]

    def hook(self) -> None:
        """The STOP hook of the command in flight, run in the worker (jobs.Job.on_cancel registers this)."""
        if not self._dead:
            self._control(("hook",), 5.0)

    # -- the link's own calls
    def read_fresh(self) -> dict:
        return self.call("read_fresh", label="reading the controller")

    def estop_now(self) -> str:
        """IO3 read by the worker's control thread: between a command's steps when called from its line callback (the
        command waits on that line), as the in-process link read it from the command's own thread."""
        if self._dead:
            return "UNKNOWN"
        try:
            out = self._control(("estop",), 5.0)
        except Exception:                                                   # noqa: BLE001
            return "UNKNOWN"
        return str(out) if out else "UNKNOWN"

    def stop_in_progress(self) -> bool:
        """A STOP being sent by the worker's link, as its last view said (robot_link.ArmLink.view ``stopping``)."""
        with self._state:
            return bool(self._view.get("stopping"))

    def leave_drag(self) -> int:
        out = self._control(("leave_drag",), 10.0) if not self._dead else None
        return -1 if out is None else int(out)

    def stop(self, hard: bool = True, drag: bool = False) -> dict:
        """STOP through the worker's own control thread: never behind a command, and through the link's readings' connection
        when a command's call never came back (the link decides, robot_link.ArmLink.stop; until 27 Oct night a stuck call
        refused the STOP here, and it waited for the new connection). A worker that does not answer in the time the server
        waits (STOP_WAIT_S) is said so by the server; a dead or silent one, or a link whose connections are all stuck
        (``stuck`` in the answer), is replaced (robot_standin)."""
        S = self.side.upper()
        stuck = self.stuck()
        gone = self._dead or (stuck and stuck[0] == "the connection's process")
        if gone:
            why = f"{stuck[0]} not answered for {stuck[1]:.0f} s" if stuck else self._dead
            return dict(ok=False, stuck=True, stuck_call=tuple(stuck) if stuck else (str(self._dead), 0.0),
                        text=f"{S}: its connection's process is gone ({why}), so STOP cannot pass through it")
        out = self._control(("stop", bool(hard), bool(drag)), 60.0)
        return out if isinstance(out, dict) else dict(ok=False, text=f"{S}: STOP not confirmed (no answer from its connection's process)")


class RealArmProxy:
    """A ``robot_real.RealArm`` whose commands run in the arm's worker."""

    def __init__(self, link: LinkProcess, log=None):
        self.link, self.log, self.side = link, log, link.side
        self.SIDE = link.side.upper()

    def __getattr__(self, name: str):
        if name.startswith("_"):
            raise AttributeError(name)

        def method(*args, **kwargs):
            kwargs = _by_name(name, args, kwargs)     # positional or not, the callbacks are found by their names
            on_line = kwargs.pop("on_line", None)
            stop_hook = kwargs.pop("stop_hook", None)
            wait_s = kwargs.pop("_wait_s", None)
            if name in _NEVER_RAISES:                # a job's finally: waits for a command still holding the controller, never
                try:                                 # raises (a lost connection would say itself instead of why the job ended)
                    return self.link.call(name, (), kwargs, label=_LABELS[name], wait_s=CLOSE_WAIT_S)
                except Exception as exc:                                     # noqa: BLE001
                    verb, failed = _NEVER_RAISES[name]
                    if self.log is not None:
                        self.log.hardware(self.side, verb, ok=False, text=f"not restored ({exc})")
                    return failed
            return self.link.call(name, (), kwargs, label=_LABELS.get(name, name.replace("_", " ")),
                                  on_line=on_line, stop_hook=stop_hook, wait_s=wait_s)
        return method


def _by_name(method: str, args: tuple, kwargs: dict) -> dict:
    """A command's arguments all by name (robot_real.RealArm's own signature), so what cannot cross to the worker
    (``on_line``, ``stop_hook``: callables) is taken out whether it was passed by position or by name."""
    import inspect

    from .robot_real import RealArm

    fn = getattr(RealArm, method, None)
    if fn is None:
        return dict(kwargs, _args=list(args)) if args else dict(kwargs)
    bound = inspect.signature(fn).bind(None, *args, **kwargs)
    out = dict(bound.arguments)
    out.pop("self", None)
    return out


_LABELS = {"stage": "staging the pole and the hand", "pole_to": "moving the pole", "hand_to": "moving the hand",
           "pole_and_hand": "moving the pole and the hand",
           "set_run_mode": "switching the run mode", "restore_run_mode": "restoring the run mode", "teach_start": "drag teaching",
           "teach_stop": "saving the recording", "upload": "uploading a program", "run": "running a program",
           "run_workflow": "running a sequence", "move_point": "MoveJ / MoveL to a global point", "move_to": "moving to a pose",
           "stream": "streaming a move", "read_speed": "reading the global speed", "restore_speed": "restoring the global speed",
           "keep_feed": "turning on the 5 ms feed", "drop_feed": "turning off the 5 ms feed", "keep_hand": "opening the hand",
           "drop_hand": "letting the hand go", "keep_s2_feed": "turning on the controller's feed",
           "drop_s2_feed": "turning off the controller's feed"}
#: The commands a job's ``finally`` makes, which never raise (RealArm's own never do): their hardware log verb, and what
#: they return when the worker cannot be reached.
_NEVER_RAISES = {"restore_run_mode": ("restore run mode", None), "restore_speed": ("restore global speed", False),
                 "drop_feed": ("canfd feed", None), "drop_hand": ("hand", None), "drop_s2_feed": ("s2 feed", None)}
