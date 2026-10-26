"""A Lite started on ALIX's emulated arms for the lid and seat tests (tests/emulated_lite.py): its own port, data folder
and camera script (LITE_CHECK_SCRIPT, a test harness only), logged in, both arms connected, Real mode. Each test file
starts one of its own, so the files run on separate workers (pytest -n, one file per worker)."""
import json
import os
import socket
import subprocess
import sys
import time
import urllib.request
from pathlib import Path

from alix_studio_lite import vendor_paths

PID = "alix_lid"
P = f"/api/projects/{PID}"


def start(tmp_path_factory, name: str, answers: list):
    """Start Lite; returns (ctx, proc). ``answers`` are the camera script's first answers, in order."""
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        port = s.getsockname()[1]
    data = tmp_path_factory.mktemp(name)
    script = data / "answers.json"
    script.write_text(json.dumps(list(answers)))
    env = dict(os.environ, LITE_CHECK_SCRIPT=str(script))
    proc = subprocess.Popen([sys.executable, str(Path(__file__).with_name("emulated_lite.py")), "--robot", "emulated",
                             "--no-browser", "--port", str(port), "--data", str(data), "--logs", str(data / "logs"),
                             "--projects", str(data / "projects")],
                            cwd=str(vendor_paths.LITE_ROOT), stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, env=env)
    ctx = {"base": f"http://127.0.0.1:{port}", "data": data, "script": script}
    for _ in range(150):
        try:
            ctx["tok"] = call(ctx, "POST", "/api/auth/login", {"pin": "1234"})["token"]
            break
        except OSError:
            time.sleep(0.2)
    call(ctx, "PUT", P, {"id": PID, "name": "Lid", "archived": False, "state": {"currentTask": 0}})
    for side in ("left", "right"):
        assert call(ctx, "POST", f"/api/robots/{side}/connect", {})["connected"]
    assert call(ctx, "POST", "/api/robots/mode", {"mode": "real", "confirmed": True})["mode"] == "real"
    return ctx, proc


def call(ctx, method, path, body=None):
    headers = {"Content-Type": "application/json"}
    if ctx.get("tok"):
        headers["X-Lite-Token"] = ctx["tok"]
    r = urllib.request.Request(ctx["base"] + path, method=method, headers=headers,
                               data=None if body is None else json.dumps(body).encode())
    with urllib.request.urlopen(r, timeout=120) as resp:
        return json.loads(resp.read())


def refused(ctx, method, path, body=None) -> dict:
    out = call(ctx, method, path, body)                         # Lite answers a refusal with its words, not an HTTP error
    assert "error" in out, out
    return out


def wait(ctx, job, limit=300, until=None):
    t0 = time.time()
    while True:
        j = call(ctx, "GET", f"/api/jobs/{job['id']}")
        if j["state"] != "running" or (until and until(j)):
            return j
        assert time.time() - t0 < limit, j
        time.sleep(0.2)


def hardware(ctx) -> list:
    day = time.strftime("%Y-%m-%d")
    path = Path(ctx["data"]) / "logs" / day / "hardware.jsonl"
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]
