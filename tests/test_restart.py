"""A new start of Lite replaces the one on its port (the owner, 21 Oct: "Any existing instance on that port should be
killed when a new server-side start (a new .sh run) begins"; "Starting the app should automatically close any existing
front end views"): the Lite there is stopped as Ctrl+C stops it (STOP to the arms through their processes, each
controller's speed back), or ended when it no longer answers; Lite's own Chrome window for that port closes, never
another; a page of the earlier start sees the new one (``boot``) and closes itself."""
import json
import os
import signal
import socket
import subprocess
import sys
import time
import urllib.request
from pathlib import Path

import pytest

from alix_studio_lite import vendor_paths

pytestmark = pytest.mark.skipif(not sys.platform.startswith("linux"), reason="processes found through /proc")


def free_port() -> int:
    while True:
        with socket.socket() as s:
            s.bind(("127.0.0.1", 0))
            p = s.getsockname()[1]
        if p not in (8765, 8766):
            return p


def hello(port, timeout=2.0):
    try:
        with urllib.request.urlopen(f"http://127.0.0.1:{port}/api/lite/hello", timeout=timeout) as r:
            return json.loads(r.read())
    except OSError:
        return None


def start(args, tmp, name):
    out = open(tmp / f"{name}.out", "w")
    proc = subprocess.Popen([sys.executable, *args, "--no-browser", "--data", str(tmp / name), "--logs", str(tmp / name / "logs"),
                             "--projects", str(tmp / name / "projects")],
                            cwd=str(vendor_paths.LITE_ROOT), env=dict(os.environ, ALIX_LITE_NO_VENV="1"), start_new_session=True,
                            stdout=out, stderr=subprocess.STDOUT, text=True)
    return proc


def wait_for(cond, limit=60.0):
    t0 = time.time()
    while time.time() - t0 < limit:
        v = cond()
        if v:
            return v
        time.sleep(0.2)
    raise AssertionError("timed out")


def stop(proc):
    if proc.poll() is None:
        os.killpg(proc.pid, signal.SIGINT)
        try:
            proc.wait(40)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
            proc.wait()


def test_a_new_start_stops_the_lite_on_its_port_its_arms_first_and_takes_its_place(tmp_path):
    port = free_port()
    a = start([str(Path(__file__).with_name("emulated_lite.py")), "--robot", "emulated", "--port", str(port)], tmp_path, "a")
    b = None
    try:
        first = wait_for(lambda: hello(port))
        tok = None

        def call(method, path, body=None):
            h = {"Content-Type": "application/json", **({"X-Lite-Token": tok} if tok else {})}
            r = urllib.request.Request(f"http://127.0.0.1:{port}{path}", method=method, headers=h,
                                       data=None if body is None else json.dumps(body).encode())
            with urllib.request.urlopen(r, timeout=60) as resp:
                return json.loads(resp.read())

        tok = call("POST", "/api/auth/login", {"pin": "1234"})["token"]
        for side in ("left", "right"):
            assert call("POST", f"/api/robots/{side}/connect", {})["connected"]
        assert first["pid"] == a.pid
        b = start(["tools/run_lite.py", "--port", str(port)], tmp_path, "b")
        a.wait(60)                                          # the earlier start ends ...
        now = wait_for(lambda: (h := hello(port)) and h.get("boot") != first["boot"] and h)
        assert now["pid"] == b.pid and now["boot"] != first["boot"]          # ... and the new one serves its port
        said_a, said_b = (tmp_path / "a.out").read_text(), (tmp_path / "b.out").read_text()
        assert a.returncode == 0 and "Shutting down: STOP to the arms" in said_a and "ALIX Studio Lite stopped." in said_a, said_a
        assert "Traceback" not in said_a, said_a
        assert f"ALIX Studio Lite is already running on port {port} (process {a.pid}): stopping it first" in said_b, said_b
        assert "It has stopped" in said_b, said_b
        time.sleep(0.3)
        assert not subprocess.run(["pgrep", "-g", str(a.pid)], capture_output=True, text=True).stdout.split()   # its arms' too
    finally:
        stop(a)
        if b is not None:
            stop(b)


def test_a_new_start_ends_a_lite_that_no_longer_answers(tmp_path, monkeypatch, capsys):
    """A Lite that no longer answers (the lab, 19 Oct: a stuck SDK call) is found by the process on its port, whose
    command is Lite's; asked to stop, it cannot, so it is ended at once after STOP_WAIT_S."""
    from alix_studio_lite import server

    port = free_port()
    a = start(["tools/run_lite.py", "--port", str(port)], tmp_path, "a")
    try:
        wait_for(lambda: hello(port))
        os.kill(a.pid, signal.SIGSTOP)                      # it holds the port and answers nothing
        assert hello(port, timeout=1.0) is None
        monkeypatch.setattr(server, "STOP_WAIT_S", 2.0)
        monkeypatch.setattr(server, "CHROME_PROFILE", tmp_path / "chrome")
        server._replace_running(port)
        a.wait(10)
        assert a.returncode == -signal.SIGKILL
        out = capsys.readouterr().out
        assert f"(process {a.pid}): stopping it first" in out and "so it was ended at once" in out, out
    finally:
        if a.poll() is None:
            os.kill(a.pid, signal.SIGCONT)
        stop(a)


def test_a_program_that_is_not_lite_is_never_stopped(tmp_path, monkeypatch):
    from alix_studio_lite import server

    port = free_port()
    other = subprocess.Popen([sys.executable, "-m", "http.server", str(port), "--bind", "127.0.0.1"], cwd=str(tmp_path),
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        wait_for(lambda: _listening(port))
        monkeypatch.setattr(server, "CHROME_PROFILE", tmp_path / "chrome")
        server._replace_running(port)
        time.sleep(0.5)
        assert other.poll() is None
    finally:
        other.terminate()
        other.wait(10)


def _listening(port) -> bool:
    with socket.socket() as s:
        return s.connect_ex(("127.0.0.1", port)) == 0


def test_a_new_start_closes_lite_s_own_chrome_window_for_its_port_and_no_other(tmp_path, monkeypatch):
    """Lite's window is a Chrome of its own profile, one per port (chrome_profile); its SingletonLock names the browser's
    process. That browser is asked to close; another port's Lite window, or a process that merely took over the
    pid of a browser gone, is left alone."""
    from alix_studio_lite import server

    monkeypatch.setattr(server, "CHROME_PROFILE", tmp_path / "chrome")
    mine, other = server.chrome_profile(server.DEFAULT_PORT), server.chrome_profile(8800)
    assert mine == tmp_path / "chrome" and other == tmp_path / "chrome-8800"

    def browser(profile):                                   # a process with the browser's command line
        profile.mkdir(parents=True, exist_ok=True)
        p = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(60)", f"--user-data-dir={profile}"])
        os.symlink(f"{socket.gethostname()}-{p.pid}", profile / "SingletonLock")
        return p

    a, b = browser(mine), browser(other)
    stranger = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(60)"])
    try:
        time.sleep(0.3)
        assert server._close_windows(server.DEFAULT_PORT)
        a.wait(10)
        assert a.returncode == -signal.SIGTERM and b.poll() is None          # that port's window only
        (other / "SingletonLock").unlink()
        os.symlink(f"{socket.gethostname()}-{stranger.pid}", other / "SingletonLock")
        assert not server._close_windows(8800) and stranger.poll() is None   # not that profile's browser: left alone
        assert not server._close_windows(8801)                               # no window for that port
    finally:
        for p in (a, b, stranger):
            if p.poll() is None:
                p.kill()
                p.wait()


def test_the_page_knows_the_start_it_belongs_to():
    """hello says this start (``boot``), with no log-in; the page reads it and closes itself once another start
    answers (web/lite/status.js)."""
    src = (vendor_paths.LITE_ROOT / "src/alix_studio_lite/web/lite/status.js").read_text(encoding="utf-8")
    for needle in ("fetch('/api/lite/hello'", "d.boot !== boot", "staleView()", "window.close()", "Reload this window"):
        assert needle in src, needle
    from alix_studio_lite.server import app as app_mod

    assert '"boot": app.boot' in Path(app_mod.__file__).read_text(encoding="utf-8")
