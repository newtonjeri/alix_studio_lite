"""tools/upload_bench.py (the user, 27 Nov): the logs mode on a made-up hardware log, and the live tests as a dry run on
ALIX's emulator, its sends slowed to the lab's rate so that the probe, the watcher and the overlaps have something
to measure. The emulator's controllers live in one process, so the dry run is a process of its own."""
import importlib.util
import json
import subprocess
import sys
import textwrap
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
TOOL = ROOT / "tools" / "upload_bench.py"


def _bench():
    spec = importlib.util.spec_from_file_location("upload_bench_under_test", TOOL)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _hw(t, side, verb, text=None, args=None, elapsed=None, ok=None):
    return json.dumps(dict(t=t, side=side, verb=verb, args=args, ret=0 if ok else None, ok=ok, elapsed_s=elapsed,
                           text=text))


def test_the_logs_give_the_send_apart_from_the_list_reads_and_find_overlaps(tmp_path):
    day = tmp_path / "2026-11-12"
    day.mkdir()
    rows = []
    for t0, side, slot, kb, send in ((1000.0, "left", 90, 100, 2.2), (1010.0, "left", 91, 200, 4.4)):
        b = kb * 1024
        rows += [_hw(t0 + 0.2, side, "rm_upload", f"upload f{slot}.txt: {kb * 18} points, {kb:.1f} KB -> slot {slot} on x"),
                 _hw(t0 + 0.2 + send, side, "rm_upload", "  sent: ret=0 err_line=-1"),
                 _hw(t0 + 0.3 + send, side, "rm_upload", f"  VERIFIED: slot {slot} holds 'x', {b} bytes (sent {b})"),
                 _hw(t0 + 0.3 + send, side, "upload", args=dict(program_id=slot, file=f"f{slot}.txt", bytes=b),
                     elapsed=0.3 + send, ok=True)]
    rows += [_hw(1012.0, "right", "run", args=dict(program_id=61), elapsed=1.0, ok=True),       # under the 2nd upload
             _hw(1100.0, "left", "run", args=dict(program_id=61), elapsed=5.0, ok=True),        # after both
             _hw(1200.0, "left", "delete program", args=dict(program_id=90), elapsed=0.4, ok=True)]
    (day / "hardware.jsonl").write_text("\n".join(rows) + "\n")
    ub = _bench()
    s = ub.logs_summary(ub.read_logs(tmp_path))
    assert s["uploads"] == 2 and s["stored"] == 2 and s["timed"] == 2
    assert abs(s["fit"]["kb_s"] - 100 / 2.2) < 0.1 and abs(s["fit"]["fixed_s"]) < 0.01     # 100 KB per 2.2 s
    assert abs(s["list_before_s"] - 0.2) < 1e-6 and abs(s["verify_s"] - 0.1) < 1e-6
    assert len(s["overlap_other_arm"]) == 1 and not s["overlap_same_arm"] and not s["overlap_uploads"]
    assert s["deletes"] == 1
    assert "on the other arm: 1" in ub.logs_markdown(s)


DRIVER = textwrap.dedent("""
    import os, sys, time
    sys.path.insert(0, {tools!r})
    import upload_bench as ub
    ub._setup(True, 10.0)
    from alix_dispatch import emulated_program as ep
    send = ep.ProgramSurface.rm_send_project

    def slow(self, sp):                  # the lab's rate (46 KB/s), ten times faster like the emulated runs
        path = sp.project_path.decode() if isinstance(sp.project_path, bytes) else sp.project_path
        time.sleep(os.path.getsize(path) / 1024 / 46.0 / 10.0)
        return send(self, sp)

    ep.ProgramSurface.rm_send_project = slow
    sys.exit(ub.main(sys.argv[1:]))
""")


def test_the_live_tests_run_through_on_the_emulator(tmp_path):
    driver = tmp_path / "driver.py"
    driver.write_text(DRIVER.format(tools=str(ROOT / "tools")))
    out = tmp_path / "bench"
    proc = subprocess.run([sys.executable, str(driver), "live", "--emulated", "--side", "left", "--slots", "90-94",
                           "--other-slots", "90", "--lines", "400,2000", "--repeat", "2", "--hold-s", "30",
                           "--time-scale", "10", "--yes", "--out", str(out)],
                          capture_output=True, text=True, timeout=240, cwd=ROOT)
    assert proc.returncode == 0, proc.stdout[-3000:] + proc.stderr[-3000:]
    s = json.loads((out / "summary.json").read_text())
    assert s["emulated"] is True and s["side"] == "left"
    assert s["idle_fit"] and [z["lines"] for z in s["idle_sizes"]] == [400, 2000]
    assert s["during"]["stored"] == 1 and s["during"]["whole_send_in_run"] == 1
    assert s["batch"]["n"] >= 2 and s["batch"]["stored"] == s["batch"]["n"] == s["batch"]["whole_send_in_run"]
    assert s["parallel"]["left"]["stored"] == 2 and s["parallel"]["left"]["all_sending_s"] > 0
    assert s["both_controllers"]["stored"] and s["both_controllers"]["overlap_s"] > 0
    assert all(r["complete"] for r in s["runs"].values()) and set(s["runs"]) == {"baseline", "during", "batch"}
    assert s["library_lock"]["longest_write_s"] is not None and s["watcher"]["longest_during_sends_s"] is not None
    assert s["deletes"]["n"] == 6 and s["deletes"]["deleted"] == 0                # the emulator has no delete
    text = (out / "summary.md").read_text()
    assert "EMULATED" in text and "One upload while a Program runs:** yes" in text
    records = [json.loads(ln) for ln in (out / "results.jsonl").read_text().splitlines()]
    hold = next(r for r in records if r.get("test") == "hold")
    assert hold["ok"] and hold["lines"] == 6001                                     # 30 s at 5 ms a line, D26


LITE_RUNS = textwrap.dedent("""
    import json, threading, time
    from pathlib import Path
    from alix_dispatch import connect
    lite = connect.open_arm("left")                          # Lite: its own connection, a Program of its own in slot 80
    still = Path({tmp!r}) / "lite_program.txt"
    still.write_text('{{"point":[0,0,0,0,0,0,0]}}\\n' * 4001)
    ub.upload_one(lite.robot, ub._rmu(), dict(test="x", side="left", slot=80, file=str(still), name="lite_program"))

    def runs():                                              # once the bench has started, a run, then another
        res = Path({out!r}) / "results.jsonl"
        while not (res.exists() and '"setup"' in res.read_text()):
            time.sleep(0.05)
        for _ in range(2):
            time.sleep(0.5)
            lite.robot.rm_set_program_id_run(80, 20, 0)
            time.sleep(0.2)
            while lite.robot.rm_get_program_run_state()[1].get("run_state"):
                time.sleep(0.05)

    threading.Thread(target=runs, daemon=True).start()
""")


def test_external_runs_are_waited_for_and_uploaded_during(tmp_path):
    out = tmp_path / "bench"
    body = DRIVER.format(tools=str(ROOT / "tools")).replace(
        "sys.exit(ub.main", LITE_RUNS.format(tmp=str(tmp_path), out=str(out)) + "\nsys.exit(ub.main")
    driver = tmp_path / "driver.py"
    driver.write_text(body)
    proc = subprocess.run([sys.executable, str(driver), "live", "--emulated", "--side", "left", "--slots", "90-92",
                           "--external-run", "--tests", "during,batch", "--batch", "2", "--lines", "400,2000",
                           "--delay-s", "1", "--time-scale", "10", "--yes", "--out", str(out)],
                          capture_output=True, text=True, timeout=240, cwd=ROOT)
    assert proc.returncode == 0, proc.stdout[-3000:] + proc.stderr[-3000:]
    assert "this bench sends no motion command" in proc.stdout
    s = json.loads((out / "summary.json").read_text())
    assert s["during"]["stored"] == s["during"]["whole_send_in_run"] == 1
    assert s["batch"]["n"] == 2 and s["batch"]["stored"] == s["batch"]["whole_send_in_run"] == 2
    records = [json.loads(ln) for ln in (out / "results.jsonl").read_text().splitlines()]
    runs = [r for r in records if r.get("kind") == "run"]
    assert [r["slot"] for r in runs] == [80, 80] and all(r["external"] and r["complete"] for r in runs)
    assert not [r for r in records if r.get("test") == "hold"]                   # no still Program of its own


def test_the_bench_writes_only_empty_slots_or_its_own(tmp_path):
    driver = tmp_path / "driver.py"
    driver.write_text(textwrap.dedent(f"""
        import sys
        sys.path.insert(0, {str(ROOT / 'tools')!r})
        import upload_bench as ub
        ub._setup(True, 10.0)
        from alix_dispatch import connect
        h = connect.open_arm("left")
        f = {str(tmp_path / 'lab.txt')!r}
        open(f, "w").write('{{"point":[0,0,0,0,0,0,0]}}\\n' * 3)
        ub.upload_one(h.robot, ub._rmu(), dict(test="x", side="left", slot=91, file=f, name="lab_program"))
        sys.exit(ub.main(sys.argv[1:]))
    """))
    proc = subprocess.run([sys.executable, str(driver), "live", "--emulated", "--side", "left", "--slots", "90-92",
                           "--tests", "idle", "--yes", "--out", str(tmp_path / "bench")],
                          capture_output=True, text=True, timeout=120, cwd=ROOT)
    assert proc.returncode == 1
    assert "slot 91 holds '91_lab_program.txt'" in proc.stdout
    assert not (tmp_path / "bench").exists()                    # refused before anything: no results folder left
