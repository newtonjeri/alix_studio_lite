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

    try:
        CHROME_PROFILE.mkdir(parents=True, exist_ok=True)
        subprocess.Popen([chrome, f"--user-data-dir={CHROME_PROFILE}", "--ignore-gpu-blocklist", "--no-first-run",
                          "--no-default-browser-check", "--new-window", url],
                         stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                         start_new_session=True)        # as webbrowser does: Ctrl+C here leaves the window open
    except OSError:
        webbrowser.open(url)


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
    a = ap.parse_args(argv)

    if a.set_pin:
        return _set_pin(a.data, a.set_pin)

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
                      projects_dir=projects)
    except RuntimeError as exc:                     # e.g. a damaged auth.json: never guess the PINs
        print(f"Lite cannot start: {exc}", file=sys.stderr, flush=True)
        return 2
    root = f"http://127.0.0.1:{a.port}/"
    url = root + ("legacy/" if a.ui == "legacy" else "")
    try:
        srv = make_server(app, a.port)
    except OSError:
        return _port_taken(a.port, root, url, a.no_browser)

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
    try:
        srv.serve_forever(poll_interval=0.25)
    except KeyboardInterrupt:
        print("\nShutting down.", flush=True)
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)       # a second Ctrl+C must not cut the shutdown short
        try:
            app.robot.stop("all")
            app.robot.close()                       # the live links, if any
        finally:
            srv.server_close()
            workers.shutdown()
    print("ALIX Studio Lite stopped.", flush=True)
    return 0
