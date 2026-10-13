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


def _running_lite(port: int) -> dict | None:
    """The state of a Lite already serving on ``port``, or None if something else is there."""
    import json
    import urllib.request

    try:
        with urllib.request.urlopen(f"http://127.0.0.1:{port}/api/lite/state", timeout=2) as r:
            d = json.loads(r.read().decode("utf-8"))
        return d if isinstance(d, dict) and "version" in d and "arms" in d else None
    except Exception:                                                      # noqa: BLE001
        return None


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="alix_studio_lite.server", description="Run ALIX Studio Lite.")
    ap.add_argument("-p", "--port", type=int, default=DEFAULT_PORT, help=f"default {DEFAULT_PORT}")
    ap.add_argument("--no-browser", action="store_true", help="do not open a browser tab")
    ap.add_argument("--data", type=Path, default=DEFAULT_DATA, help=f"project data folder (default {DEFAULT_DATA})")
    ap.add_argument("--fixture", default=None,
                    help="commode_a | commode_b | commode_c | urinal_a | washtub_a (default: the last used, else commode_c)")
    a = ap.parse_args(argv)

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
    project = open_project(a.data, fixture)
    app = LiteApp(project, a.data)
    url = f"http://127.0.0.1:{a.port}/"
    try:
        srv = make_server(app, a.port)
    except OSError as exc:
        running = _running_lite(a.port)
        if running is not None:
            # Lite is already running there: open it rather than fail
            print(f"ALIX Studio Lite is already running at {url}", flush=True)
            print(f"   project {running.get('project', {}).get('name', '?')}, data folder {running.get('data_dir', '?')}", flush=True)
            if not a.no_browser:
                webbrowser.open(url)
                print("   opened a browser tab to it", flush=True)
            return 0
        print(f"Cannot use port {a.port}: {exc.strerror}. Another program is using it. "
              f"Try --port {a.port + 1}.", file=sys.stderr, flush=True)
        return 2

    def _terminate(signum, frame):        # a service manager's stop behaves like Ctrl+C
        raise KeyboardInterrupt

    import signal

    signal.signal(signal.SIGTERM, _terminate)
    if not a.no_browser:
        threading.Timer(0.6, lambda: webbrowser.open(url)).start()
    print(f"ALIX Studio Lite running at {url}  (Ctrl+C to stop)", flush=True)
    print(f"Project: {project.meta.get('name')}  ->  {project.root}", flush=True)
    print(f"Python:  {sys.executable}", flush=True)
    try:
        srv.serve_forever(poll_interval=0.25)
    except KeyboardInterrupt:
        print("\nShutting down.", flush=True)
    finally:
        app.robot.stop("all")
        srv.server_close()
        workers.shutdown()
    return 0
