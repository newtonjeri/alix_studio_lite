"""ALIX Studio Lite's local web server. Start it with ``tools/run_lite.sh``."""
from __future__ import annotations

import argparse
import os
import sys
import threading
import webbrowser
from pathlib import Path

DEFAULT_PORT = 8766                     # ALIX Studio uses 8765, so both can run side by side
DEFAULT_DATA = Path(os.environ.get("ALIX_LITE_DATA", Path.home() / "ALIX-Lite"))
# Lite's own Chrome profile (_open_browser): a separate Chrome, so its switches apply even while another is open
CHROME_PROFILE = Path(os.environ.get("XDG_CACHE_HOME") or Path.home() / ".cache") / "alix-studio-lite" / "chrome"
EXIT_LIMIT_S = 30.0                     # Lite's exit (STOP, then each controller's speed back) ends by then, else it quits
STOP_WAIT_S = EXIT_LIMIT_S + 10.0       # a Lite a new start stops (_replace_running) has ended by then, else it is killed


def chrome_profile(port: int) -> Path:
    """Lite's Chrome profile for the page on ``port``: one per port, so that a new start closes that port's windows
    only (_close_windows); the default port's is the one Lite always had."""
    return CHROME_PROFILE if int(port) == DEFAULT_PORT else CHROME_PROFILE.with_name(f"{CHROME_PROFILE.name}-{int(port)}")


def _chrome() -> str | None:
    """Google Chrome or Chromium on this computer, if any (not a snap: it cannot use Lite's profile)."""
    import shutil

    for name in ("google-chrome", "google-chrome-stable", "chromium"):
        path = shutil.which(name)
        if path and "/snap/" not in path:
            return path
    return None


def _open_browser(url: str) -> None:
    """Opens Lite's page. Chrome gets a window of its own with Chrome's GPU blocklist ignored: where
    the computer's graphics run in software (Mesa llvmpipe, e.g. while the GPU driver is not
    loaded), Chrome turns WebGL off for every page and no 3D view can start, ALIX Studio's
    included (the lab laptop, 14 October: chrome://gpu "WebGL: Disabled"). ALIX Studio's own fix
    for that is the same switch (chrome://flags/#ignore-gpu-blocklist); a profile of Lite's own
    makes it apply even while another Chrome is open. Any other browser opens the page as a tab."""
    chrome = _chrome()
    if not chrome:
        webbrowser.open(url)
        return
    import subprocess
    from urllib.parse import urlsplit

    profile = chrome_profile(urlsplit(url).port or DEFAULT_PORT)
    try:
        profile.mkdir(parents=True, exist_ok=True)
        _fresh_caches(profile)
        subprocess.Popen([chrome, f"--user-data-dir={profile}", "--ignore-gpu-blocklist", "--no-first-run",
                          "--no-default-browser-check", "--new-window", url],
                         stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                         start_new_session=True)        # as webbrowser does: Ctrl+C here leaves the window open
    except OSError:
        webbrowser.open(url)


# Chrome's GPU, shader and page caches in Lite's own profile: rebuilt by Chrome as it needs them
_CACHES = ("GPUCache", "GrShaderCache", "ShaderCache", "GPUPersistentCache", "DawnCache", "DawnGraphiteCache",
           "DawnWebGPUCache", "Default/GPUCache", "Default/DawnCache", "Default/DawnGraphiteCache", "Default/DawnWebGPUCache",
           "Default/Cache", "Default/Code Cache")


def _fresh_caches(profile: Path) -> None:
    """Lite's Chrome window starts with its caches cleared (the owner, 17 Oct: the simulation lagged at
    times, and clearing the browser's cache cured it): a GPU or shader cache left by a GPU process that
    failed can keep Chrome drawing in software. Only while that Chrome is not running (its SingletonLock):
    a window already open is left as it is."""
    import shutil

    if (profile / "SingletonLock").is_symlink() or (profile / "SingletonLock").exists():
        return
    for name in _CACHES:
        shutil.rmtree(profile / name, ignore_errors=True)


def _running_lite(port: int) -> dict | None:
    """What a Lite already serving on ``port`` says about itself, or None if another program is
    there. A Lite from before /api/lite/hello still names itself in its Server header: {}."""
    import json
    import urllib.error
    import urllib.request

    try:
        with urllib.request.urlopen(f"http://127.0.0.1:{port}/api/lite/hello", timeout=2) as r:
            d = json.loads(r.read().decode("utf-8"))
        return d if isinstance(d, dict) and d.get("app") == "ALIX Studio Lite" else None
    except urllib.error.HTTPError as exc:
        return {} if (exc.headers.get("Server") or "").startswith("ALIXStudioLite/") else None
    except Exception:                                                      # noqa: BLE001
        return None


def _replace_running(port: int) -> None:
    """A new start replaces the Lite already on its port, and closes the windows showing it (the owner, 21 Oct: "Any
    existing instance on that port should be killed when a new server-side start ... begins"; "Starting the app should
    automatically close any existing front end views"). That Lite is stopped as Ctrl+C stops it (SIGTERM: STOP to the
    arms, then each controller's speed back, its arms' processes with it; it quits by EXIT_LIMIT_S), and ended at once
    if it is still there after STOP_WAIT_S. Then Lite's own Chrome windows for this port close (_close_windows); a page
    left open in another browser closes itself once it sees the new start (web/lite/status.js). Never this process, and
    never a program that is not Lite: one that holds the port is said by _port_taken, as before."""
    pid = _lite_pid(port)
    if pid and pid not in (os.getpid(), os.getppid()):
        print(f"ALIX Studio Lite is already running on port {port} (process {pid}): stopping it first, as Ctrl+C does "
              f"(STOP to the arms, then each controller's speed back; at most {STOP_WAIT_S:g} s)...", flush=True)
        print(f"   {_stop_process(pid)}", flush=True)
    if _close_windows(port):
        print(f"Closed Lite's window for port {port}: the new start opens its own.", flush=True)


def _lite_pid(port: int) -> int | None:
    """The process of the ALIX Studio Lite serving ``port`` on this computer: by its own answer (/api/lite/hello), else
    (a Lite from before that route, or one that no longer answers) the process listening on the port, when its command
    is Lite's (Linux: /proc). None when there is none, or it cannot be told."""
    running = _running_lite(port)
    if running and running.get("pid"):
        return int(running["pid"])
    from .udp_push import port_holders

    for h in port_holders(port, kind="tcp"):
        if h.get("pid") and _is_lite_command(_command_of(h["pid"])):
            return int(h["pid"])
    return None


def _is_lite_command(command: str) -> bool:
    """Whether a command line (its arguments NUL-separated, as /proc gives them) starts Lite's server: run_lite.py (what
    run_lite.sh and run_lite.bat run), ``-m alix_studio_lite.server``, or the tests' emulated_lite.py. Its interpreter is
    not looked at: any program run with Lite's own Python names the repository there."""
    args = command.split("\0") if "\0" in command else command.split()
    rest = args[1:]
    return (any(Path(x).name in ("run_lite.py", "emulated_lite.py") for x in rest)
            or any(x == "-m" and y.startswith("alix_studio_lite.server") for x, y in zip(rest, rest[1:])))


def _command_of(pid: int) -> str:
    """A process's command line: /proc on Linux, else ps (macOS). "" when it cannot be read."""
    try:
        raw = Path(f"/proc/{int(pid)}/cmdline").read_bytes()
        return "\0".join(p.decode(errors="replace") for p in raw.split(b"\0") if p)
    except OSError:
        pass
    import subprocess

    try:
        return subprocess.run(["ps", "-p", str(int(pid)), "-o", "command="], capture_output=True, text=True, timeout=5).stdout.strip()
    except (OSError, subprocess.SubprocessError):
        return ""


def _alive(pid: int) -> bool:
    """Whether process ``pid`` still runs (a zombie has ended). Windows: asked without touching it (there os.kill with
    signal 0 would end it)."""
    if os.name == "nt":
        import ctypes

        k = ctypes.windll.kernel32
        h = k.OpenProcess(0x00100000, False, int(pid))                    # SYNCHRONIZE
        if not h:
            return False
        try:
            return k.WaitForSingleObject(h, 0) == 0x102                   # WAIT_TIMEOUT: still running
        finally:
            k.CloseHandle(h)
    try:
        os.kill(int(pid), 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    try:
        return Path(f"/proc/{int(pid)}/stat").read_text().rsplit(")", 1)[1].split()[0] != "Z"
    except (OSError, IndexError):
        return True


def _stop_process(pid: int) -> str:
    """A Lite stopped as Ctrl+C stops it (SIGTERM), waited for up to STOP_WAIT_S, else ended at once. What happened."""
    import signal
    import time

    try:
        os.kill(int(pid), signal.SIGTERM)
    except ProcessLookupError:
        return "It had stopped already."
    except PermissionError:
        return "It runs as another user, so this start cannot stop it: stop it there (Ctrl+C in its terminal)."
    t0 = time.monotonic()
    while time.monotonic() - t0 < STOP_WAIT_S:
        if not _alive(pid):
            return f"It has stopped ({time.monotonic() - t0:.1f} s)."
        time.sleep(0.2)
    try:
        os.kill(int(pid), getattr(signal, "SIGKILL", signal.SIGTERM))
    except OSError:
        pass
    for _ in range(50):
        if not _alive(pid):
            break
        time.sleep(0.1)
    return f"It had not stopped after {STOP_WAIT_S:g} s, so it was ended at once."


def _close_windows(port: int) -> bool:
    """Lite's own Chrome windows for ``port`` closed: the browser of its profile (chrome_profile), found by the profile's
    SingletonLock ("<host>-<pid>") and its own command line (that profile, so never another browser's windows), asked to
    close as Chrome closes on SIGTERM (ended at once after 5 s). Whether one was open. Not on Windows (its lock names no
    process)."""
    import signal
    import socket
    import time

    if os.name == "nt":
        return False
    profile = chrome_profile(port)
    lock = profile / "SingletonLock"
    try:
        target = os.readlink(lock)
        host, _, pid = target.rpartition("-")
        pid = int(pid)
    except (OSError, ValueError):
        return False
    args = _command_of(pid).replace("\0", " ").split(" ")
    if host != socket.gethostname() or not _alive(pid) or f"--user-data-dir={profile}" not in args:
        return False
    try:
        os.kill(pid, signal.SIGTERM)
    except OSError:
        return False
    for _ in range(50):
        if not _alive(pid):
            break
        time.sleep(0.1)
    else:
        try:
            os.kill(pid, signal.SIGKILL)
        except OSError:
            pass
        time.sleep(0.2)
    try:                                         # a lock left by the browser ended: the new window's caches are cleared
        if not _alive(pid) and os.readlink(lock) == target:
            lock.unlink()
    except OSError:
        pass
    return True


def _port_taken(port: int, root: str, url: str, no_browser: bool) -> int:
    """The port is in use: open this same Lite if it is the one running there, else say what to do."""
    from .. import code_id

    running = _running_lite(port)
    if running is not None and running.get("code") == code_id():
        print(f"ALIX Studio Lite is already running at {url}", flush=True)
        print(f"   data folder {running.get('data_dir', '?')}", flush=True)
        if not no_browser:
            _open_browser(url)
            print("   opened a browser window to it", flush=True)
        return 0
    if running is not None:                     # its server still runs the code it started with
        pid = running.get("pid")
        stop = (f"taskkill /PID {pid}" if os.name == "nt" else f"kill {pid}") if pid else None
        print(f"An older ALIX Studio Lite is still running at {root} (started before this copy changed, "
              "or from another copy).", file=sys.stderr)
        print("   Stop it first: press Ctrl+C in its terminal" + (f", or run: {stop}" if stop else "")
              + ". Then start Lite again.", file=sys.stderr)
        print(f"   Or start this one beside it: --port {port + 1}.", file=sys.stderr, flush=True)
        return 2
    print(f"Cannot use port {port}: another program (not ALIX Studio Lite) is using it.", file=sys.stderr)
    if sys.platform.startswith("linux"):
        print(f"   To see which: ss -ltnp 'sport = :{port}'", file=sys.stderr)
    print(f"   Close that program, or start Lite on another port: --port {port + 1}.", file=sys.stderr, flush=True)
    return 2


def _set_pin(data: Path, role: str) -> int:
    import getpass

    from .auth import Auth, valid_pin

    pin = getpass.getpass(f"New {role} PIN (4 to 6 digits): ")
    if not valid_pin(pin):
        print("A PIN is 4 to 6 digits.", file=sys.stderr)
        return 2
    if getpass.getpass("Again: ") != pin:
        print("The two PINs differ; nothing changed.", file=sys.stderr)
        return 2
    Auth(data).set_pin(role, pin)
    print(f"The {role} PIN is changed ({data / 'auth.json'}).")
    return 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="alix_studio_lite.server", description="Run ALIX Studio Lite.")
    ap.add_argument("-p", "--port", type=int, default=DEFAULT_PORT, help=f"default {DEFAULT_PORT}")
    ap.add_argument("--no-browser", action="store_true",
                    help="do not open a browser window (Lite opens its own Chrome window, or a tab of the default browser)")
    ap.add_argument("--data", type=Path, default=DEFAULT_DATA, help=f"project data folder (default {DEFAULT_DATA})")
    ap.add_argument("--fixture", default=None,
                    help="Legacy Lite's fixture: commode_a | commode_b | commode_c | urinal_a | washtub_a "
                         "(default: the last used, else commode_c)")
    ap.add_argument("--ui", choices=("lite", "legacy"), default="lite",
                    help="the page to open: the Lite UI (default) or Legacy Lite (/legacy/)")
    ap.add_argument("--set-pin", choices=("operator", "engineer"), metavar="ROLE",
                    help="change the operator or engineer PIN (asks for it twice), then exit")
    ap.add_argument("--logs", type=Path, default=None,
                    help="where the logs go (default: Resources/Logs in the repository)")
    ap.add_argument("--projects", type=Path, default=None,
                    help="where the projects go, one folder each (default: Resources/Projects in the repository)")
    ap.add_argument("--robot", choices=("standin", "realman", "emulated"), default="standin",
                    help="standin (default): no hardware. realman: the real arms through ALIX's dispatch "
                         "(feedback of arm, pole and hand; staging, drag teaching, upload, Real and Hardware sim "
                         "runs, STOP). emulated: the same link to ALIX's emulated arms")
    ap.add_argument("--feed", choices=("20", "off"), default="20",
                    help="S2's record of the controllers' feed during Real Machine runs: 20 (default: every 20 ms, the "
                         "programs play some 3 %% slower meanwhile) or off (never recorded)")
    a = ap.parse_args(argv)

    if a.set_pin:
        return _set_pin(a.data, a.set_pin)
    _replace_running(a.port)                        # a new start: the Lite on this port stopped, its windows closed

    from .. import vendor_paths

    vendor_paths.activate()
    from . import workers
    from .app import LiteApp, make_server
    from .project import open_project

    import json

    from . import fixtures

    fixture = a.fixture
    if fixture is None:
        try:
            fixture = json.loads((a.data / "settings.json").read_text(encoding="utf-8")).get("fixture")
        except (OSError, ValueError):
            fixture = None
    if fixture not in fixtures.FIXTURES:
        if a.fixture:
            print(f"Unknown fixture {a.fixture!r}; choose one of {', '.join(fixtures.FIXTURES)}.", file=sys.stderr)
            return 2
        fixture = fixtures.DEFAULT
    if a.robot == "emulated":                       # ALIX's controller emulator, before the SDK is imported
        import alix_emulator

        alix_emulator.install()
    project = open_project(a.data, fixture)
    try:
        from .. import vendor_paths as _vp

        logs = a.logs or (_vp.LITE_ROOT / "Resources" / "Logs")
        projects = a.projects or (_vp.LITE_ROOT / "Resources" / "Projects")
        app = LiteApp(project, a.data, robot_link=None if a.robot == "standin" else a.robot, log_dir=logs,
                      projects_dir=projects, feed_ms=None if a.feed == "off" else int(a.feed))
    except RuntimeError as exc:                     # e.g. a damaged auth.json: never guess the PINs
        print(f"Lite cannot start: {exc}", file=sys.stderr, flush=True)
        return 2
    root = f"http://127.0.0.1:{a.port}/"
    url = root + ("legacy/" if a.ui == "legacy" else "")
    try:
        srv = make_server(app, a.port)
    except OSError:
        return _port_taken(a.port, root, url, a.no_browser)

    if a.robot == "realman":                         # the wrist camera's perception loaded now, not at the first check
        from . import mechanism

        threading.Thread(target=mechanism.warm_up, args=(app.robot,), name="lite-perception-warm", daemon=True).start()

    def _terminate(signum, frame):        # a service manager's stop behaves like Ctrl+C
        raise KeyboardInterrupt

    import signal

    signal.signal(signal.SIGTERM, _terminate)
    if not a.no_browser:
        threading.Timer(0.6, lambda: _open_browser(url)).start()
        print("Opening Lite in its own Chrome window..." if _chrome() else "Opening a browser tab...", flush=True)
    print(f"ALIX Studio Lite running at {root}  (Ctrl+C to stop)", flush=True)
    print(f"Legacy Lite:  {root}legacy/", flush=True)
    print(f"Data folder:  {a.data}", flush=True)
    print(f"Projects:  {app.store.root}", flush=True)
    for p in app.adopted:
        if p.get("error"):
            print(f"   NOT copied from {app.store.home}: {p['name']}: {p['error']} (tried again at the next start)", flush=True)
        else:
            print(f"   copied from {app.store.home}: {p['name']}" + (f" (was “{p['renamed']}”)" if p["renamed"] else ""), flush=True)
    print(f"Logs:  {app.log.root}  (tail -f {app.log.root}/$(date +%F)/lite.log)", flush=True)
    print(f"Python:  {sys.executable}", flush=True)
    if hasattr(signal, "SIGUSR1"):              # what each thread is doing, on demand, for a Lite that stops answering
        import faulthandler

        faulthandler.register(signal.SIGUSR1, all_threads=True)
        print(f"Process:  {os.getpid()}  (if Lite stops answering: kill -USR1 {os.getpid()} prints what each "
              "thread is doing)", flush=True)
    try:
        srv.serve_forever(poll_interval=0.25)
    except KeyboardInterrupt:
        print(f"\nShutting down: STOP to the arms, then each controller's speed back (at most {EXIT_LIMIT_S:g} s; "
              "Ctrl+C again once STOP is sent quits at once).", flush=True)
    finally:
        _close(app, srv)
    print("ALIX Studio Lite stopped.", flush=True)
    sys.stderr.flush()
    os._exit(0)                 # nothing left behind holds the process: a job still on its thread, the SDK's own threads


def _close(app, srv) -> None:
    """Lite's exit, bounded: STOP to every arm, then each controller's global speed back and the links let
    go. It never hangs (the lab, 17 Oct: Ctrl+C did not end Lite; it took the system monitor): once STOP is
    out, a second Ctrl+C (or the system monitor's End) quits at once, and after EXIT_LIMIT_S Lite quits as it
    is, first writing what each thread was doing to the day's log folder (and the terminal)."""
    import signal

    from . import workers

    sent, ended = threading.Event(), threading.Event()

    def again(signum, frame):
        if sent.is_set():
            print("\nQuitting now: the controllers' global speed may not be back where Lite found it.", flush=True)
            workers.kill_now()
            _kill_arm_workers()
            os._exit(130)

    def watchdog():
        if ended.wait(EXIT_LIMIT_S):
            return
        where = _dump_threads(app.log.root)
        print(f"\nLite did not close within {EXIT_LIMIT_S:g} s, so it quits now"
              + (f" (what each thread was doing: {where})." if where else "."), flush=True)
        workers.kill_now()
        _kill_arm_workers()
        os._exit(1)

    signal.signal(signal.SIGINT, again)
    signal.signal(signal.SIGTERM, again)
    threading.Thread(target=watchdog, name="lite-exit", daemon=True).start()
    try:                                            # each step is tried; none keeps Lite from quitting
        for step in (lambda: _stop_arms(app), sent.set, app.robot.close, srv.server_close, workers.shutdown):
            try:
                step()
            except Exception as exc:                                        # noqa: BLE001
                print(f"Closing: {exc!r}", flush=True)
    finally:
        sent.set()
        ended.set()


def _stop_arms(app) -> None:
    """Lite's exit, first: STOP to every arm, through its connection's process (robot_process: the workers leave
    Ctrl+C to the server). An arm whose controller did not confirm it is said in the terminal too: the page that
    shows that alert is closing."""
    out = app.robot.stop("all")
    for r in (out.get("controllers") or {}).values():
        if not r.get("ok"):
            print(f"  {r.get('text')}. Press the red E-stop button on the robot if it is still moving.", flush=True)


def _dump_threads(root) -> str | None:
    """Every thread's stack, to threads-HHMMSS.txt in the day's log folder and to the terminal (logbook.dump_threads)."""
    from .logbook import dump_threads

    return dump_threads(root)


def _kill_arm_workers() -> None:
    """The arms' connection processes (robot_process) ended at once with Lite's hard exit."""
    try:
        from .robot_process import kill_all

        kill_all()
    except Exception:                                                       # noqa: BLE001
        pass
