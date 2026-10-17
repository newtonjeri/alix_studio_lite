"""What the lab's run of 17 Oct showed, and the owner's revisions that came with it:
- Hardware sim puts every linked controller in its SIM mode (Real in REAL), all or none, read back;
- Real Test's Hardware sim step passes once the program has run to its end, whatever the arm reported (each
  report a pop-up as it comes), its length the judge: the control program said "RUN COMPLETE" of program 50,
  cut short by the controller after 4105; a real run cut short names what the arm reported;
- the lists Lite offers are the controller's, as Data Management reads it: a program deleted there is gone;
- a STOP confirmed after the 5 s its answer waits is said when it comes (the lab's "no answer" while the
  controllers answered Recover and Clear errors)."""
import time
from types import SimpleNamespace

import pytest


def robot(tmp_path):
    from alix_studio_lite.server.jobs import Jobs
    from alix_studio_lite.server.robot_standin import StandInRobot

    alerts = []
    log = SimpleNamespace(alert=lambda level, text, **k: alerts.append((level, text)), hardware=lambda *a, **k: None)
    return StandInRobot(tmp_path, Jobs(), log=log), alerts


def wait(job, limit=10):
    t0 = time.time()
    while job.state == "running":
        assert time.time() - t0 < limit
        time.sleep(0.02)
    return job


def test_hardware_sim_puts_every_controller_in_sim_or_none(tmp_path):
    from alix_studio_lite.server.jobs import UserError

    r, _ = robot(tmp_path)
    done = []

    class Real:
        def __init__(self, side, refuse=False):
            self.side, self.refuse = side, refuse

        def set_run_mode(self, mode):
            if self.refuse:
                raise UserError("not taken")
            done.append((self.side, mode))
            return {self.side: 1}

        def restore_run_mode(self, originals):
            done.append((self.side, "put back"))

    reals = {"left": Real("left"), "right": Real("right", refuse=True)}
    r._real = lambda s: reals[s]
    for s in ("left", "right"):
        r.arms[s].connected = True
    with pytest.raises(UserError, match="RIGHT controller did not switch to its SIM mode"):
        r.set_controller_modes("hwsim")
    assert done == [("left", "SIM"), ("left", "put back")]                    # all or none
    reals["right"].refuse = False
    done.clear()
    assert r.set_controller_modes("hwsim") == {"left": "SIM", "right": "SIM"}
    assert r.set_controller_modes("real") == {"left": "REAL", "right": "REAL"}
    assert r.set_controller_modes("visual") == {} and r.arms["left"].controller_mode == "REAL"   # Visual sim: as they are


REPORT = "!! the arm reports 4105 (0x1009: joint speed over its limit (a warning; the run goes on)) at 2.3 s"


class Controller:
    """A linked controller's run, as the control program's rm_upload.run prints it."""
    def __init__(self, lines, seconds, ok=True):
        self.lines, self.seconds, self.ok = lines, seconds, ok

    def programs(self):
        return [dict(id=50, name="50_L-R01_v002.txt", size=113851)]

    def set_run_mode(self, mode):
        return {"left": 1}

    def restore_run_mode(self, originals):
        pass

    def run(self, pid, first, points, on_line=None, speed=50, restore_pct=50):
        for ln in self.lines:
            on_line(ln)
        return dict(ok=self.ok, why=None if self.ok else "RUN NOT COMPLETE", seconds=self.seconds, lines=self.lines)


def run(r, real, dry):
    prog = dict(lines=2002, first_row=[0] * 7, controller_bytes=113851)       # 10.0 s of points
    return wait(r.jobs.start("run", "t", lambda job: r._run_real(job, real, "left", 50, prog, dry, None), arm="left"))


def test_the_hardware_sim_step_passes_when_the_program_runs_to_its_end(tmp_path):
    from alix_studio_lite.server.robot_standin import arm_report

    assert arm_report(REPORT) == dict(codes="4105 (0x1009: joint speed over its limit (a warning; the run goes on))", at_s=2.3, warn=True)
    r, alerts = robot(tmp_path)
    head = ["run slot 50 (50_L-R01_v002.txt) on 192.168.1.10; transit speed 50 %", "running (0.6 s)"]
    j = run(r, Controller(head + [REPORT, "RUN COMPLETE: 10.6 s, no error"], 10.6), dry=True)
    assert j.state == "done" and j.result["reported"][0]["at_s"] == 2.3                         # passed, the report said
    assert alerts == [("warn", "LEFT Hardware sim run of program 50: the arm reports 4105 (0x1009: joint speed over its limit "
                               "(a warning; the run goes on)) at 2.3 s.")]
    error = "!! the arm reports 4106 (0x100A: joint over its limit) at 4.0 s"                       # an error, the run goes to its end
    j = run(r, Controller(head + [error, "** RUN NOT COMPLETE: 4106 (0x100A)"], 10.4, ok=False), dry=True)
    assert j.state == "done" and alerts[-1][0] == "error"
    j = run(r, Controller(head + [REPORT, "RUN COMPLETE: 3.8 s, no error"], 3.8), dry=True)        # the lab's program 50
    assert j.state == "failed" and "did not run to its end: it ended after 3.8 s of the 10.0 s" in j.error and "4105" in j.error


def test_a_real_run_cut_short_names_what_the_arm_reported(tmp_path):
    r, _ = robot(tmp_path)
    head = ["run slot 50 (50_L-R01_v002.txt) on 192.168.1.10; transit speed 50 %", "running (0.6 s)"]
    j = run(r, Controller(head + [REPORT, "RUN COMPLETE: 3.8 s, no error"], 3.8), dry=False)
    assert j.state == "failed" and "the controller stopped it after the arm reported 4105" in j.error and "tuned slower" in j.hint
    j = run(r, Controller(head + ["RUN COMPLETE: 4.0 s, no error"], 4.0), dry=False)
    assert j.state == "failed" and "an E-stop, or its pendant" in j.error
    gone = Controller([], 0)
    gone.programs = lambda: []                                                  # deleted at the controller
    j = run(r, gone, dry=True)
    assert j.state == "failed" and j.error == "The LEFT controller no longer holds program 50: it was deleted there."


def test_the_lists_are_the_controllers_as_data_management_reads_it(monkeypatch):
    from alix_studio_lite.server import content

    content.forget()
    reads = []
    held = [dict(id=1, name="1_R-R01_PART1_v003.txt", bytes=175159, sha256="abc"), dict(id=2, name=None)]
    arm = SimpleNamespace(connected=True, teaching=False, programs={80: dict(name="80_R-R01_PART1_v002.txt", sha256="old"),
                                                                   1: dict(name="R_lid_again", sha256="abc")})   # S1 sends a merge's stem
    rob = SimpleNamespace(_real=lambda s: object(), arm=lambda s: arm, global_points=lambda s: [dict(name="R_HME_POS", joints=[0] * 7)],
                          programs=lambda s: (reads.append(s), held)[1])
    app = SimpleNamespace(robot=rob, jobs=SimpleNamespace(running=lambda arm=None: []))
    files = {"abc": dict(name="R_lid.txt", also=[dict(name="R_lid_again.txt")])}       # a merge made again, the same bytes
    c = content.content(app, "right", files=files)
    assert [(p["id"], p["sent"], p["file_name"]) for p in c["programs"]] == [(1, True, "R_lid_again.txt")]   # not program 80
    assert content.file_for(files, "abc")["name"] == "R_lid.txt"                     # sent as no name of the project's: the first
    assert [p["name"] for p in c["points"]] == ["R_HME_POS"] and c["read"] and c["live"]
    content.content(app, "right")
    assert reads == ["right"]                                                        # as last read, within FRESH_S
    content.content(app, "right", refresh=True)
    assert reads == ["right", "right"]                                               # a refresh reads it again
    app.jobs = SimpleNamespace(running=lambda arm=None: [SimpleNamespace(kind="run")])
    c = content.content(app, "right", refresh=True)
    assert reads == ["right", "right"] and "busy" in c["note"] and c["programs"]      # never while a run holds it
    arm.connected = False
    assert "Connect the RIGHT arm" in content.content(app, "right")["note"]
    content.forget()


def test_a_stop_confirmed_after_the_wait_is_said_when_it_comes(tmp_path, monkeypatch):
    from alix_studio_lite.server import robot_standin

    r, alerts = robot(tmp_path)
    monkeypatch.setattr(robot_standin, "STOP_WAIT_S", 0.2)

    class Slow:
        def stop(self, hard=True):
            time.sleep(0.6)
            return dict(ok=True, text="LEFT: stopped on its controller (rm_set_arm_stop), the pole stopped; the arm is still")

    r.links["left"] = Slow()
    r.arms["left"].connected = True
    out = r.stop("both")
    assert not out["ok"] and "not confirmed by its controller within 0.2 s" in out["controllers"]["left"]["text"]
    for _ in range(50):
        if any("confirmed after" in t for _, t in alerts):
            break
        time.sleep(0.05)
    assert alerts[-1][0] == "warn" and "the arm is still (confirmed after" in alerts[-1][1]
