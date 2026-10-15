"""Regression tests for the execution gates' fixes: a stopped tune never becomes the candidate,
a restated stage is a new recording and a new version, every robot step keeps the mode it was
asked in, a region's MAX DEV reaches L5, the gates show the stage a run uses, a library file
keeps its arm and stage, and the smaller refusals (sequence rows, hand text, previews, setup,
stated stages, the worker pool).

The optimiser is replaced by a fake single-worker pool (the tuned file is the recording's bytes),
so the tests are about the files, the gates and the robot rules, not the optimiser."""
import hashlib
import json
import shutil
import threading
import time
import urllib.request
from concurrent.futures import Future
from pathlib import Path

import pytest

from alix_studio_lite import vendor_paths

vendor_paths.activate()

TAUGHT = vendor_paths.LITE_ROOT / "tests/data/recordings/commode_c_taught_paths"


class FakePool:
    """One worker, like workers.pool(): the tune "result" is the recording's bytes, written the
    way the optimiser writes (the file, then its record). ``gate`` holds a worker until set."""

    def __init__(self):
        self.gate = threading.Event()
        self.gate.set()
        self.calls, self.threads = [], []
        self.fail = None             # a hard check (CheckReport.to_dict() form) the tune fails
        self.optimiser = "manual"    # a manual tune is never held to it; a time-optimal one is

    def submit(self, fn, raw_path, out_path, side, settings, source_name=""):
        self.calls.append(dict(raw=raw_path, out=out_path, side=side, settings=dict(settings)))
        fut = Future()
        fut.set_running_or_notify_cancel()             # already running: a cancel cannot stop it

        def work():
            self.gate.wait(30)
            out = Path(out_path)
            shutil.copyfile(raw_path, out)
            sha = hashlib.sha256(out.read_bytes()).hexdigest()
            fail, opt = self.fail, self.optimiser
            checks = dict(passed=False, checks=[fail], notes=[]) if fail else {}
            result = dict(uploadable=not fail or opt == "manual", limits_ok=not fail, optimiser=opt, lines=2,
                          achieved_s=0.005, hints=["Use lighter path smoothing (20 mm)."] if fail else [])
            out.with_name(out.stem + ".lite.json").write_text(json.dumps(dict(
                track_sha256=sha, recipe=dict(origin=str(raw_path)), result=result, **({"checks": checks} if fail else {}))))
            fut.set_result(dict(summary=dict(result, message="fake tune"), checks=checks, recipe={},
                                util=None, progress=None))

        t = threading.Thread(target=work, daemon=True)
        t.start()
        self.threads.append(t)
        return fut

    def idle(self):
        for t in self.threads:
            t.join(30)


@pytest.fixture
def lite(tmp_path, monkeypatch, request):
    from alix_studio_lite.server import workers
    from alix_studio_lite.server.app import LiteApp, make_server
    from alix_studio_lite.server.project import open_project

    pool = FakePool()
    monkeypatch.setattr(workers, "pool", lambda: pool)
    app = LiteApp(open_project(tmp_path, "commode_c"), tmp_path)
    srv = make_server(app, 0)
    threading.Thread(target=srv.serve_forever, kwargs={"poll_interval": 0.05}, daemon=True).start()   # shuts down at once
    pid = "alix_fix_" + request.node.name[5:30].replace("[", "_").replace("]", "")
    ctx = {"base": f"http://127.0.0.1:{srv.server_address[1]}", "app": app, "pool": pool, "pid": pid,
           "data": tmp_path}
    ctx["tok"] = call(ctx, "POST", "/api/auth/login", {"pin": "1234"})["token"]
    call(ctx, "PUT", f"/api/projects/{pid}", {"id": pid, "name": "Fixes", "archived": False, "state": {}})
    yield ctx
    srv.shutdown()


def call(ctx, method, path, body=None):
    headers = {"Content-Type": "application/json", "X-Lite-Token": ctx.get("tok") or ""}
    r = urllib.request.Request(ctx["base"] + path, method=method, headers=headers,
                               data=None if body is None else json.dumps(body).encode())
    with urllib.request.urlopen(r, timeout=60) as resp:
        return json.loads(resp.read())


def unit(ctx, key, verb="", body=None):
    path = f"/api/projects/{ctx['pid']}/units/{key}" + (f"/{verb}" if verb else "")
    return call(ctx, "POST" if verb else "GET", path, body if verb else None)


def wait(ctx, job, timeout=60):
    t0 = time.time()
    while True:
        j = call(ctx, "GET", f"/api/jobs/{job['id']}")
        if j["state"] != "running":
            return j
        assert time.time() - t0 < timeout, j
        time.sleep(0.05)


def work_dir(ctx, key, arm="left"):
    return ctx["data"] / "projects" / ctx["pid"] / "trajectories" / arm / key / "work"


def load(ctx, key, file, name, pole_mm, hand, lines=None):
    """A taught file as a region's new recording (its first ``lines`` only: short uploads and runs)."""
    text = "".join((TAUGHT / file).read_text().splitlines(keepends=True)[:lines])
    body = {"source": "upload", "name": name, "content": text, "pole_mm": pole_mm, "hand": hand}
    return unit(ctx, key, "recordings", body)["unit"]


def gate(u, gid):
    return next(g for g in u["gates"] if g["id"] == gid)


def robot_ready(ctx, mode="hwsim"):
    for arm in ("left", "right"):
        call(ctx, "POST", f"/api/robots/{arm}/connect", {})
    assert call(ctx, "POST", "/api/robots/mode", {"mode": mode, "confirmed": True})["mode"] == mode


# ── #1: a stopped tune never replaces the candidate; one tune per region at a time

def test_a_stopped_tune_never_becomes_the_candidate(lite):
    key = "L-T01"                                              # FILE_10.1, pole 200, 50 % open
    r1 = unit(lite, key)["active"]
    assert wait(lite, unit(lite, key, "optimise", {}))["state"] == "done"
    work = work_dir(lite, key)
    first = (work / "candidate.txt").read_bytes()
    load(lite, key, "FILE_04.txt", "L_rim.txt", 140, "100% open")       # another recording, another stage

    lite["pool"].gate.clear()                                   # its worker holds until released
    job = unit(lite, key, "optimise", {})
    assert unit(lite, key, "optimise", {})["code"] == "busy"   # never two tunes of one region
    call(lite, "POST", f"/api/jobs/{job['id']}/cancel", {})
    assert wait(lite, job)["state"] == "cancelled"
    lite["pool"].gate.set()                                     # the stopped tune's worker still writes ...
    lite["pool"].idle()
    assert (work / "candidate.txt").read_bytes() == first       # ... but never over the candidate
    assert not list(work.glob("candidate.optimise-*"))          # and its own files are gone

    assert unit(lite, key)["fresh"] is False                    # the active recording is not the tuned one
    assert unit(lite, key, "active", {"file": r1})["fresh"] is True
    v = unit(lite, key, "keep", {})["versions"][-1]
    assert v["stage"]["pole_mm"] == 200 and v["taught_arm"] == "left"
    vfile = work.parent / "versions" / f"{v['version']}.txt"
    assert vfile.read_bytes() == first == (TAUGHT / "FILE_10.1.txt").read_bytes()


# ── #2 and #5: the same bytes with another stage are a new recording, a new version, and the
#    gates show the stage the robot steps use

def test_a_restated_stage_is_a_new_version_and_the_gates_show_it(lite, monkeypatch):
    from alix_studio_lite.server import api_units

    key = "L-R01"
    u = load(lite, key, "FILE_09.txt", "L_hinge.txt", 140, "100% open", lines=400)
    assert wait(lite, unit(lite, key, "optimise", {}))["state"] == "done"
    assert [v["stage"]["pole_mm"] for v in unit(lite, key, "keep", {})["versions"]] == [140]
    assert "cannot change" in unit(lite, key, f"recordings/{u['active']}/stage", {"pole_mm": 200, "hand": "GRIP"})["error"]

    u = load(lite, key, "FILE_09.txt", "L_hinge.txt", 200, "50% open", lines=400)   # the correction: loaded again
    assert u["fresh"] is False                                  # the old candidate is the old stage's
    assert gate(u, "stage")["text"] == "Staged first (v001): pole 140 mm, fingers 100 % open, thumb held"
    assert "changed" in unit(lite, key, "keep", {})["error"]

    monkeypatch.setattr(api_units, "checked_version", lambda units, k, arm: units.current_version(k))
    robot_ready(lite)
    assert wait(lite, unit(lite, key, "upload", {"program_id": 9}))["state"] == "done"   # v001, pole 140
    time.sleep(0.3)
    assert wait(lite, unit(lite, key, "optimise", {}))["state"] == "done"                # the same bytes again
    k = unit(lite, key, "keep", {})                             # not "already kept": another stage
    assert [(v["version"], v["stage"]["pole_mm"]) for v in k["versions"]] == [("v001", 140), ("v002", 200)]
    assert gate(k, "stage")["text"] == "Staged first (v002): pole 200 mm, fingers 50 % open, thumb held"
    assert unit(lite, key, "upload", {"program_id": 9})["unchanged"]       # same bytes: the stage follows
    assert lite["app"].robot.arm("left").programs[9]["stage"]["pole_mm"] == 200
    j = wait(lite, unit(lite, key, "run", {}))
    assert j["state"] == "done" and "Pole at 200 mm (emulated" in " | ".join(e["text"] for e in j["events"])


# ── #3: every robot step keeps the mode it was asked (and confirmed) in

def test_a_run_never_starts_after_the_mode_changed(lite, monkeypatch):
    from alix_studio_lite.server import api_units

    key = "L-R01"
    load(lite, key, "FILE_10.1.txt", "L_lid.txt", 200, "50% open", lines=400)
    assert wait(lite, unit(lite, key, "optimise", {}))["state"] == "done"
    unit(lite, key, "keep", {})
    passthrough = lambda units, k, arm: units.current_version(k)                       # noqa: E731
    monkeypatch.setattr(api_units, "checked_version", passthrough)
    robot_ready(lite, "real")
    assert wait(lite, unit(lite, key, "upload", {"program_id": 4}))["state"] == "done"
    time.sleep(0.3)

    def switch_meanwhile(units, k, arm):                        # the mode changes while the request works
        lite["app"].set_mode("hwsim")
        return passthrough(units, k, arm)

    monkeypatch.setattr(api_units, "checked_version", switch_meanwhile)
    r = unit(lite, key, "run", {"confirmed": True})
    assert r["code"] == "mode" and "nothing was staged or moved" in r["error"]
    assert call(lite, "GET", "/api/robots/status")["jobs"] == [] and unit(lite, key)["last_run"] is None

    real_identity = api_units.identity

    def flip_identity(units, k, which):
        lite["app"].set_mode("real")
        return real_identity(units, k, which)

    monkeypatch.setattr(api_units, "identity", flip_identity)  # Hardware sim asked, Real meanwhile
    assert unit(lite, key, "move-start", {"which": "version"})["code"] == "mode"


def test_staging_and_programs_check_the_mode_again(tmp_path, monkeypatch):
    from alix_studio_lite.server import robot_standin, staging

    monkeypatch.setattr(robot_standin, "TIME_SCALE", 1.0)           # the mode changes while the pole still moves
    from alix_studio_lite.server.jobs import Jobs, UserError
    from alix_studio_lite.server.robot_standin import StandInRobot

    robot = StandInRobot(tmp_path, Jobs())
    for s in ("left", "right"):
        robot.connect(s)
    robot.mode = "hwsim"
    st = staging.make_stage(290, "OPEN")                        # a short pole move, nothing for the hand
    robot.arms["left"].programs[1] = dict(name="p", lines=400, bytes=1, sha256="x", stage=st.as_dict())
    with pytest.raises(UserError, match="mode changed"):        # a Real run asked, Hardware sim now
        robot.run("left", 1, dry_run=False, title="Real run", stage=st)
    job = robot.run("left", 1, dry_run=True, title="Hardware sim run", stage=st, mode="hwsim")
    time.sleep(0.4)
    robot.mode = "real"                                         # a switch that skipped the busy check
    t0 = time.time()
    while job.state == "running" and time.time() - t0 < 20:
        time.sleep(0.05)
    assert job.state == "failed" and "the program did not start" in job.error
    assert robot.arms["left"].controller_mode == "SIM"


# ── #4: the taught set's MAX DEV tightens L5

def test_the_taught_sets_max_dev_reaches_the_optimiser(lite):
    wait(lite, unit(lite, "L-T01", "optimise", {}))             # FILE_10.1: MAX DEV 5 mm in its record
    assert lite["pool"].calls[-1]["settings"]["max_dev_mm"] == 5
    load(lite, "L-R01", "FILE_09.txt", "L_hinge.txt", 200, "50% open")
    wait(lite, unit(lite, "L-R01", "optimise", {}))             # no MAX DEV recorded: the 10 mm default
    assert lite["pool"].calls[-1]["settings"]["max_dev_mm"] is None


# ── #6: a library file keeps its arm and stage

def test_loading_a_library_file_again_never_changes_it_silently(tmp_path):
    from alix_studio_lite.server.jobs import UserError
    from alix_studio_lite.server.library import TaughtLibrary

    lib = TaughtLibrary(tmp_path)
    txt = (TAUGHT / "FILE_09.txt").read_text()
    a = lib.add("spill.txt", txt, arm="left", pole_mm=140, hand="100% open")
    for kw in (dict(arm="right"), dict(pole_mm=50, hand="GRIP"), dict(path="RIGHT/R_spill.txt")):
        with pytest.raises(UserError, match="already in the library"):
            lib.add("spill.txt", txt, **kw)
    assert lib.add("copy.txt", txt)["id"] == a["id"]            # nothing new said: the same record
    assert lib.add("spill.txt", txt, arm="left", pole_mm=140, hand="100% open")["id"] == a["id"]
    m = lib.get(a["id"])
    assert m["arm"] == "left" and m["stage"]["pole_mm"] == 140


# ── the smaller ones: #7 and #10 sequence rows, #8 setup, #9 hand text, #11 pool, #12 stated stage

def test_sequence_rows_never_guess_and_a_preview_never_approves(tmp_path):
    from alix_studio_lite.motion.limits import load_limits
    from alix_studio_lite.server import sequence, staging
    from alix_studio_lite.server.lite_units import LiteUnits, task_list

    units = LiteUnits(tmp_path / "p", "commode_c")
    n = next(i for i, t in enumerate(task_list(units), 1) if t["left"] and t["left"]["key"] == "L-T01")

    def identity(units, key, which):
        return dict(path=TAUGHT / "FILE_10.1.txt", name=key, taught_arm="left" if key.startswith("L") else "right",
                    stage=staging.make_stage(200, "50% open"))

    def plan(rows, source="version"):
        return sequence.plan(units, rows, source=source, home={}, identity=identity,
                             rating={s: load_limits(s).rating_deg_s for s in ("left", "right")})

    for row, why in (({"step": n, "rule": "RUN FIRST"}, "is not a rule"), ({"step": "3A"}, "is not a task number")):
        p = plan([row])
        assert not p["ready"] and p["rows"][0]["arms"] == {} and why in p["rows"][0]["note"] and p["blocking"]
    assert plan([{"step": n, "rule": "left run first"}])["ready"]
    p = plan([{"step": n}], source="recording")
    assert p["playable"] and not p["ready"]


def test_setup_is_kept_only_once_the_robot_accepts_it(lite):
    key = "R-R01_PART1"
    robot_ready(lite, "real")
    before = unit(lite, key)["setup"]
    assert unit(lite, key, "setup", {"pole_mm": 260, "hand": "OPEN", "apply": True})["code"] == "confirm"
    assert unit(lite, key)["setup"] == before
    out = unit(lite, key, "setup", {"pole_mm": 260, "hand": "OPEN", "apply": True, "confirmed": True})
    assert out["unit"]["setup"]["pole_mm"] == 260 and wait(lite, out["job"])["state"] == "done"


def test_hand_text_and_counts_are_refused_plainly(lite):
    from alix_studio_lite.server import sequence, staging
    from alix_studio_lite.server.jobs import UserError

    for hand in ("150% open", [float("inf"), 0, 0, 0, 0, 0], ["1e999", 0, 0, 0, 0, 0]):
        with pytest.raises(UserError):
            staging.parse_hand(hand)
    assert staging.stage_from({"pole_mm": 100, "hand": "150% open"}) is None
    r = unit(lite, "L-R01", "setup", {"pole_mm": 150, "hand": "150% open"})
    assert "cannot be 150 % open" in r["error"]
    with pytest.raises(UserError):
        sequence.check_start({"pole_mm": 100, "hand": [5000] * 6, "q_deg": [0] * 7})


def test_two_first_tunes_make_one_pool(monkeypatch):
    from alix_studio_lite.server import workers

    made = []

    class Slow:
        def __init__(self, *a, **k):
            time.sleep(0.2)
            made.append(self)

    monkeypatch.setattr(workers, "ProcessPoolExecutor", Slow)
    monkeypatch.setattr(workers, "_POOL", None)
    got = []
    ts = [threading.Thread(target=lambda: got.append(workers.pool())) for _ in range(6)]
    [t.start() for t in ts]
    [t.join() for t in ts]
    assert len(made) == 1 and len({id(p) for p in got}) == 1
    monkeypatch.setattr(workers, "_POOL", None)


def test_a_stage_that_is_not_valid_can_be_stated(tmp_path):
    from alix_studio_lite.server import staging
    from alix_studio_lite.server.jobs import UserError
    from alix_studio_lite.server.lite_units import LiteUnits

    units = LiteUnits(tmp_path / "p", "commode_c")
    dst = units.add_recording("L-R01", TAUGHT / "FILE_09.txt", source="upload", name="L_x.txt", taught_arm="left",
                              stage=staging.make_stage(140, "GRIP"))
    meta = json.loads(dst.with_suffix(".source.json").read_text())
    meta["stage"] = {"pole_mm": 400, "hand": [0, 0, 0, 0, 150, 1000]}       # e.g. an edited project file
    dst.with_suffix(".source.json").write_text(json.dumps(meta))
    units.set_recording_stage("L-R01", dst.name, staging.make_stage(150, "GRIP"))
    assert next(r for r in units.recordings("L-R01") if r["file"] == dst.name)["stage"]["pole_mm"] == 150
    with pytest.raises(UserError, match="cannot change"):
        units.set_recording_stage("L-R01", dst.name, staging.make_stage(50, "GRIP"))


def test_a_refused_keep_names_the_limits_and_the_log_says_what_each_tune_came_to(lite):
    """The lab (14 Oct): Keep was refused six times, never saying which limit failed. A time-optimal
    tune that fails one is refused, naming it with the optimiser's hint; a manual one is kept with
    what it breaks shown (the owner, 15 Oct: never enforced); the log says what each tune came to."""
    key = "L-T01"
    lite["pool"].fail = dict(id="L5", name="Path accuracy", hard=True, passed=False, value=6.2, limit=5.0,
                             unit="mm", detail="max departure 6.2 mm around line 1230")
    lite["pool"].optimiser = "toppra"
    assert wait(lite, unit(lite, key, "optimise", {}))["state"] == "done"
    r = unit(lite, key, "keep", {})
    assert r["error"].endswith("cannot go to the robot: L5 Path accuracy 6.20 mm (limit 5 mm).")
    assert r["hint"] == "Use lighter path smoothing (20 mm)."
    lite["pool"].optimiser = "manual"                           # the same limit broken by a manual tune: kept
    assert wait(lite, unit(lite, key, "optimise", {}))["state"] == "done"
    assert "error" not in unit(lite, key, "keep", {})
    lite["pool"].fail = None
    assert wait(lite, unit(lite, key, "optimise", {}))["state"] == "done"
    assert "error" not in unit(lite, key, "keep", {})
    day = next((lite["data"] / "logs").iterdir())
    tunes = [json.loads(x) for x in (day / "activity.jsonl").read_text().splitlines() if '"job optimise' in x]
    assert [t["detail"]["outcome"] for t in tunes] == ["fails L5 Path accuracy 6.20 mm (limit 5 mm)",
                                                       "breaks L5 Path accuracy 6.20 mm (limit 5 mm), shown, not enforced",
                                                       "passes every limit"]
    assert "(fails L5 Path accuracy 6.20 mm (limit 5 mm))" in (day / "lite.log").read_text()


def test_every_file_name_in_a_project_is_used_once(lite):
    """The owner (14 Oct): Save TXT refuses a name already used in the project (any region, any
    case); a file loaded again under a used name (a correction) is kept with a numbered name."""
    load(lite, "L-R01", "FILE_09.txt", "L_hinge.txt", 140, "100% open", lines=200)
    load(lite, "L-R02", "FILE_09.txt", "L_side.txt", 140, "100% open", lines=200)
    r = unit(lite, "L-R02", "name", {"name": "l_HINGE.TXT"})
    assert "already the name of a recording in this project" in r["error"]
    assert "error" not in unit(lite, "L-R01", "name", {"name": "L_hinge.txt"})          # its own name again
    again = load(lite, "L-R01", "FILE_09.txt", "L_hinge.txt", 200, "50% open", lines=200)
    names = [x["name"] for x in again["recordings"]]
    assert names.count("L_hinge.txt") == 1 and "L_hinge_2.txt" in names


def test_a_recordings_hand_can_change_after_teaching_but_never_its_pole(lite):
    """Back in Prepare (the owner, 14 Oct): on the operator's word a recording runs with another hand
    from then on; its tuning is stale until made again, and its old stage stays in its record."""
    key = "L-R01"
    u = load(lite, key, "FILE_09.txt", "L_hinge.txt", 140, "GRIP", lines=400)
    assert wait(lite, unit(lite, key, "optimise", {}))["state"] == "done"
    assert unit(lite, key)["fresh"]
    path = f"recordings/{u['active']}/hand"
    assert unit(lite, key, path, {"hand": "OPEN"})["code"] == "confirm"
    v = unit(lite, key, path, {"hand": "OPEN", "confirmed": True})
    rec = next(r for r in v["recordings"] if r["file"] == u["active"])
    assert rec["stage"]["pole_mm"] == 140 and rec["stage"]["hand"] == [1000] * 6
    assert not v["fresh"]                                                   # tuned again before it is kept
    src = work_dir(lite, key).parent / "raw" / Path(u["active"]).with_suffix(".source.json").name
    assert json.loads(src.read_text())["stage_history"][0]["was"]["hand"] == [0, 0, 0, 0, 150, 1000]


def test_tune_has_the_processors_analysis_trim_settings_and_named_tuned_files(lite):
    """Tune as the Trajectory V2 processor (the owner, 14 Oct): its automatic analysis of the active
    recording, its trim settings reaching the optimiser, no one-row region, and the tuned file kept
    under a name of the project's own (one name per file) and saved to this computer as it is."""
    from alix_studio_lite.optimise.analysis import analyse

    key = "L-R01"
    u = load(lite, key, "FILE_09.txt", "L_hinge.txt", 140, "100% open", lines=1200)
    a = call(lite, "GET", f"/api/projects/{lite['pid']}/units/{key}/analysis")
    assert a["file"] == "L_hinge.txt" and a["lines"] == 1200 and len(a["regions"]) == 5
    assert [r["start"] for r in a["regions"]][0] == 1 and a["regions"][-1]["end"] == 1200
    assert a["regions"][0]["scale_pct"] == 75 and a["regions"][0]["motion_type"] == "Approach candidate"
    assert len(analyse([[0] * 7] * 10)["regions"]) == 5

    # the HTML's fields as typed (feedback v3): kept so, and read its way (getParams) when the file is made
    from alix_studio_lite.optimise import manual

    r = unit(lite, key, "settings", {"changes": {"idle_window": 14, "idle_threshold_md": 6, "idle_min_run": 2.6}})
    s = r["settings_view"]["settings"]
    assert {k: s[k] for k in ("idle_window", "idle_threshold_md", "idle_min_run")} == \
        {"idle_window": 14, "idle_threshold_md": 6, "idle_min_run": 2.6}
    assert manual.params(s) == dict(sample_ms=5.0, idle_window=15, idle_threshold_md=6.0, idle_min_run=3, smooth_window=101)
    s0 = unit(lite, key, "settings", {"changes": {"idle_window": None, "idle_threshold_md": -4, "idle_min_run": 0}})
    assert manual.params(s0["settings_view"]["settings"]) == \
        dict(sample_ms=5.0, idle_window=11, idle_threshold_md=0.0, idle_min_run=20, smooth_window=101)   # empty, 0: its defaults
    assert unit(lite, key, "undo", {})["settings_view"]["settings"]["idle_min_run"] == 2.6
    assert "not a region" in unit(lite, key, "settings", {"changes": {"segments": [{"start": 9, "end": 9, "pct": 80}]}})["error"]
    assert wait(lite, unit(lite, key, "optimise", {}))["state"] == "done"
    assert lite["pool"].calls[-1]["settings"]["idle_window"] == 14

    assert "already the name" in unit(lite, key, "keep", {"name": "L_hinge"})["error"]      # the recording's name
    v = unit(lite, key, "keep", {"name": "L_hinge_TUNED"})
    assert v["versions"][-1]["name"] == "L_hinge_TUNED.txt"
    assert "already the name" in unit(lite, key, "name", {"name": "L_hinge_TUNED.txt"})["error"]
    req = urllib.request.Request(f"{lite['base']}/api/projects/{lite['pid']}/units/{key}/file?which=version",
                                 headers={"X-Lite-Token": lite["tok"]})
    with urllib.request.urlopen(req, timeout=30) as resp:
        body, disp = resp.read(), resp.headers.get("Content-Disposition", "")
    kept = work_dir(lite, key).parent / "versions" / v["versions"][-1]["file"]
    assert body == kept.read_bytes() and "L_hinge_TUNED.txt" in disp


def test_regions_end_with_their_recording_and_old_settings_tune_manually(lite):
    """Feedback v3, checked: a region's scale is the HTML's (empty or 0: 100 %, at least 1 %); the scale
    regions are rows of the recording they were set on (the HTML clears them with every file it loads), so
    another recording starts without them, one undo step back; the processed TXT downloads as the HTML
    writes it (LF); and settings saved for Lite's earlier algorithm ("default") read as manual tuning with
    the HTML's smoothing window, so the file that algorithm made is not current."""
    key = "L-T01"
    first = unit(lite, key)["active"]
    got = unit(lite, key, "settings", {"changes": {"segments": [{"start": 1, "end": 50, "pct": 0}, {"start": 60, "end": 90, "pct": -5}]}})
    assert [g["pct"] for g in got["settings_view"]["settings"]["segments"]] == [100.0, 1.0]
    assert "10,000 %" in unit(lite, key, "settings", {"changes": {"segments": [{"start": 1, "end": 50, "pct": 20000}]}})["error"]
    segs = [{"start": 1, "end": 2000, "pct": 75.0}, {"start": 2001, "end": 4000, "pct": 115.0}]
    unit(lite, key, "settings", {"changes": {"segments": segs}})
    load(lite, key, "FILE_04.txt", "L_rim.txt", 140, "100% open", lines=3000)
    sv = unit(lite, key)["settings_view"]
    assert sv["settings"]["segments"] == [] and sv["undo_label"] == "Scale regions cleared: another recording (L_rim.txt)"
    assert unit(lite, key, "undo", {})["settings_view"]["settings"]["segments"] == segs        # the operator's to bring back
    assert unit(lite, key, "active", {"file": first})["settings_view"]["settings"]["segments"] == []
    assert unit(lite, key, "active", {"file": first})["settings_view"]["undo_label"].startswith("Scale regions cleared")

    assert wait(lite, unit(lite, key, "optimise", {}))["state"] == "done" and unit(lite, key)["fresh"]
    req = urllib.request.Request(f"{lite['base']}/api/projects/{lite['pid']}/units/{key}/file?which=candidate",
                                 headers={"X-Lite-Token": lite["tok"]})
    with urllib.request.urlopen(req, timeout=30) as resp:
        body = resp.read()
    cand = work_dir(lite, key) / "candidate.txt"
    assert b"\r" in cand.read_bytes() and b"\r" not in body and body == cand.read_bytes().replace(b"\r\n", b"\n")

    uj = work_dir(lite, key).parent / "unit.json"                # as a region tuned before 15 Oct left it
    st = json.loads(uj.read_text())
    st["settings"] = dict(st["settings"], optimiser="default", smooth_window=41, time_mode="natural")
    uj.write_text(json.dumps(st))
    ex = cand.with_name("candidate.extra.json")
    extra = json.loads(ex.read_text())
    extra["settings"] = {k: v for k, v in st["settings"].items() if k != "program_id"}
    ex.write_text(json.dumps(extra))
    u = unit(lite, key)
    s = u["settings_view"]["settings"]
    assert (s["optimiser"], s["smooth_window"], s["time_mode"]) == ("manual", 101, "fastest") and not u["fresh"]


def test_the_suggestions_fit_an_optimised_time(lite):
    """Analyze with an optimised time (the owner, 15 Oct: "if the user types 20 seconds, it should give
    suggestions for the scale regions"): one factor on every suggestion, each kept in the HTML's 1-300 %,
    so the generated file, after the region's Trim, comes to the time asked, or as near as the range allows."""
    from alix_studio_lite.optimise.analysis import analyse, fit_target

    key = "L-R02"
    load(lite, key, "FILE_09.txt", "L_side_target.txt", 140, "100% open", lines=1500)
    a = call(lite, "GET", f"/api/projects/{lite['pid']}/units/{key}/analysis")
    base = [r["scale_pct"] for r in a["regions"]]
    want = round(a["totals"]["original_s"] * 0.6, 2)
    t = call(lite, "GET", f"/api/projects/{lite['pid']}/units/{key}/analysis?target_s={want}")
    assert t["target"]["reached"] and t["target"]["after_trim"] and abs(t["target"]["estimated_s"] - want) <= max(0.05, 0.01 * want)
    got = [r["scale_pct"] for r in t["regions"]]
    assert all(r["selected"] for r in t["regions"] if r["scale_pct"] != 100)
    assert sorted(range(len(base)), key=lambda i: base[i]) == sorted(range(len(got)), key=lambda i: got[i])   # the guidance kept
    far = fit_target(analyse([[i * 100, 0, 0, 0, 0, 0, 0] for i in range(4000)]), 0.01)
    assert not far["target"]["reached"] and {r["scale_pct"] for r in far["regions"]} == {1}
    assert "above 0" in call(lite, "GET", f"/api/projects/{lite['pid']}/units/{key}/analysis?target_s=-3")["error"]
