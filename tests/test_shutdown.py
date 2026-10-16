"""Ctrl+C in Lite's terminal (SIGINT to every process in it, the tuning worker included) stops Lite
cleanly: no traceback, and no process left behind (the lab laptop printed the worker's
KeyboardInterrupt traceback, 14 October)."""
import json
import os
import signal
import socket
import subprocess
import sys
import time
import urllib.request

import pytest

from alix_studio_lite import vendor_paths

pytestmark = pytest.mark.skipif(os.name != "posix", reason="process groups and SIGINT are POSIX")


def test_ctrl_c_stops_lite_and_its_worker_cleanly(tmp_path):
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        port = s.getsockname()[1]
    proc = subprocess.Popen([sys.executable, str(vendor_paths.LITE_ROOT / "tools" / "run_lite.py"), "--no-browser",
                             "--port", str(port), "--data", str(tmp_path), "--logs", str(tmp_path / "logs"), "--projects", str(tmp_path / "projects")],
                            env=dict(os.environ, ALIX_LITE_NO_VENV="1"), start_new_session=True,
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    base, tok = f"http://127.0.0.1:{port}", None

    def call(method, path, body=None):
        h = {"Content-Type": "application/json", **({"X-Lite-Token": tok} if tok else {})}
        r = urllib.request.Request(base + path, method=method, headers=h,
                                   data=None if body is None else json.dumps(body).encode())
        with urllib.request.urlopen(r, timeout=60) as resp:
            return json.loads(resp.read())

    try:
        for _ in range(150):
            try:
                tok = call("POST", "/api/auth/login", {"pin": "1234"})["token"]
                break
            except OSError:
                time.sleep(0.2)
        P = "/api/projects/cc"
        call("PUT", P, {"id": "cc", "name": "C", "createdAt": "2026-10-14T10:00:00.000Z",
                        "updatedAt": "2026-10-14T10:00:00.000Z", "archived": False, "state": {"currentTask": 0}})
        lib = next(x for x in call("GET", f"{P}/units/L-T01")["recordings"] if x["source"] == "library")
        call("POST", f"{P}/units/L-T01/active", {"file": lib["file"]})
        job = call("POST", f"{P}/units/L-T01/optimise", {})               # starts the tuning worker
        while call("GET", f"/api/jobs/{job['id']}")["state"] == "running":
            time.sleep(0.2)
        group = subprocess.run(["pgrep", "-g", str(proc.pid)], capture_output=True, text=True).stdout.split()
        assert len(group) >= 2                                              # the server and its worker
        os.killpg(proc.pid, signal.SIGINT)                                  # what Ctrl+C sends
        out, _ = proc.communicate(timeout=30)
    finally:
        if proc.poll() is None:
            os.killpg(proc.pid, signal.SIGKILL)
            proc.communicate()
    assert proc.returncode == 0, out
    assert "Traceback" not in out and "KeyboardInterrupt" not in out, out
    assert "ALIX Studio Lite stopped." in out
    time.sleep(0.3)
    assert not subprocess.run(["pgrep", "-g", str(proc.pid)], capture_output=True, text=True).stdout.split()
