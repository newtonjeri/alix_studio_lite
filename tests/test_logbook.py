"""Lite's logs (server/logbook.py): every activity and every refusal, never a PIN; the page's own
steps; and hardware errors and failures as alerts the page shows until they are closed."""
import json
import threading
import time
import urllib.request

import pytest


@pytest.fixture(scope="module")
def lite(tmp_path_factory):
    from alix_studio_lite import vendor_paths

    vendor_paths.activate()
    from alix_studio_lite.server.app import LiteApp, make_server
    from alix_studio_lite.server.project import open_project

    data = tmp_path_factory.mktemp("lite-log")
    app = LiteApp(open_project(data, "commode_c"), data)
    srv = make_server(app, 0)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    ctx = {"base": f"http://127.0.0.1:{srv.server_address[1]}", "app": app, "data": data}
    ctx["tok"] = call(ctx, "POST", "/api/auth/login", {"pin": "1234"})["token"]
    yield ctx
    srv.shutdown()


def call(ctx, method, path, body=None):
    headers = {"Content-Type": "application/json"}
    if ctx.get("tok"):
        headers["X-Lite-Token"] = ctx["tok"]
    r = urllib.request.Request(ctx["base"] + path, method=method, headers=headers,
                               data=None if body is None else json.dumps(body).encode())
    with urllib.request.urlopen(r, timeout=60) as resp:
        return json.loads(resp.read())


def records(ctx, stream):
    day = next((ctx["data"] / "logs").iterdir())
    return [json.loads(x) for x in (day / f"{stream}.jsonl").read_text().splitlines()]


def wait(ctx, job):
    for _ in range(300):
        j = call(ctx, "GET", f"/api/jobs/{job['id']}")
        if j["state"] != "running":
            return j
        time.sleep(0.1)
    raise AssertionError(job)


def test_every_change_and_refusal_is_logged_without_pins(lite):
    call(lite, "POST", "/api/auth/engineer", {"pin": "0000"})                    # refused
    call(lite, "POST", "/api/auth/engineer", {"pin": "1234"})
    acts = records(lite, "activity")
    login = next(r for r in acts if r["what"] == "POST /api/auth/login")
    assert login["detail"]["pin"] == "***" and login["ok"]
    bad, good = [r for r in acts if r["what"] == "POST /api/auth/engineer"][-2:]
    assert not bad["ok"] and "Incorrect" in bad["error"] and good["ok"] and good["who"].startswith("engineer:")
    text = (next((lite["data"] / "logs").iterdir()) / "lite.log").read_text()
    assert "1234" not in text and "0000" not in text and "[activity]" in text


def test_the_pages_own_steps_are_logged(lite):
    assert call(lite, "POST", "/api/activity", {"event": "setStage", "detail": {"stage": "teach"}})["ok"]
    r = records(lite, "activity")[-1]
    assert r["what"] == "page: setStage" and r["detail"] == {"stage": "teach"}


def test_hardware_failures_and_estops_are_alerts(lite):
    call(lite, "POST", "/api/auth/engineer", {"pin": "1234"})                    # the fault and E-stop tests
    for arm in ("left", "right"):
        call(lite, "POST", f"/api/robots/{arm}/connect", {})
    call(lite, "POST", "/api/robots/mode", {"mode": "hwsim"})
    call(lite, "POST", "/api/robots/left/fault", {"pole": True})                # the pole will not arrive
    job = wait(lite, call(lite, "POST", "/api/lite/unit/L-T01/prepare", {}))
    assert job["state"] == "failed"
    st = call(lite, "GET", "/api/robots/status")
    failed = [a for a in st["alerts"] if a["level"] == "error" and a["side"] == "left"]
    assert failed and "pole did not reach" in failed[-1]["text"] and st["instance"]
    call(lite, "POST", "/api/robots/left/fault", {"pole": False})

    call(lite, "POST", "/api/robots/right/estop-test", {"pressed": True})
    alert = next(a for a in call(lite, "GET", "/api/robots/status")["alerts"] if a["source"] == "estop")
    assert alert["level"] == "warn" and alert["side"] == "right"
    call(lite, "POST", "/api/robots/right/estop-test", {"pressed": False})
    call(lite, "POST", "/api/robots/right/recover", {})
    hw = records(lite, "hardware")
    assert any(r["verb"] == "ALERT ERROR" and r["side"] == "left" for r in hw)
    assert any(r["verb"] == "ALERT WARN" and r["side"] == "right" for r in hw)
    ended = [r for r in records(lite, "activity") if r["what"].startswith("job ")]
    assert ended and not ended[-1]["ok"] and "pole did not reach" in ended[-1]["error"]
