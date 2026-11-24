"""A program on a controller is linked to the pole and hand it runs with, on every computer that runs the project
(server/program_links.py; the user, 15 Nov: "link a program to its taught pole/hand value, or the pole and hand value
set during upload in the trajectory viewer, or ... set manually in the trajectory view").

The lab, 14-15 Nov: the Jetson's Lite ran S2 on the arms the lab PC had sent the Programs to. It had no record of
sending them, and of the ten files its operators uploaded to its Trajectory Viewer (each with its pole and hand) it
matched three by name: the names Lite gives the programs it sends end in what RealMan's API leaves of ".txt". So no
hand step was set before the right arm's grip Programs 62 and 65, and the hand stayed open."""
import json
import threading
import time
import urllib.request
from types import SimpleNamespace

import numpy as np
import pytest

from alix_studio_lite import vendor_paths

GRIP, CLEANING, OPEN = [0, 0, 0, 0, 150, 1000], [1000, 1000, 1000, 1000, 0, 1000], [1000] * 6

#: 15 Nov, the Jetson: the sequence's Programs as the two controllers list them, and the files uploaded to its Viewer
HELD = {"left": {61: "61_61_L_P1_pole50_cleaningpose_.", 62: "62_62_L_P2_pole200_cleaningpose_", 63: "63_63_L_P2_pole200_grip_10.5s_25",
                 64: "64_64_L_P3_pole200_cleaningpose.", 65: "65_65_L_P3_pole200_grip_45.3s_s."},
        "right": {61: "61_61_R_P1_pole200_cleaningpose.", 62: "62_62_R_P2_pole200_grip_13.0s_n.", 63: "63_63_R_P2_pole200_cleaningpose.",
                  64: "64_64_1_R_P2_pole50_cleaningpose", 65: "65_65_R_P3_pole50_grip_39.8s_ne."}}
UPLOADED = {"left": {61: ("61_L_P1_pole50_cleaningpose_48.2s_20pct_steady30s_plus20.txt", 50, CLEANING),
                     62: ("62_L_P2_pole200_cleaningpose_22.6s_30pct.txt", 200, CLEANING),
                     63: ("63_L_P2_pole200_grip_10.5s_25pct_new_slowstart.txt", 200, GRIP),
                     64: ("64_L_P3_pole200_cleaningpose_19.8s_25pct_slow10-15s.txt", 200, CLEANING),
                     65: ("65_L_P3_pole200_grip_45.3s_smooth_noidle_faster.txt", 200, GRIP)},
            "right": {61: ("61_R_P1_pole200_cleaningpose_53.5s_20pct_steady40s.txt", 210, CLEANING),
                      62: ("62_R_P2_pole200_grip_13.0s_new_R04.txt", 200, GRIP),
                      63: ("63_R_P2_pole200_cleaningpose_9.2s_30pct.txt", 200, CLEANING),
                      64: ("64_1_R_P2_pole50_cleaningpose_10.4s_new_R06.txt", 50, CLEANING),
                      65: ("65_R_P3_pole50_grip_39.8s_new_R07_R08P2_30pct_slowstart.txt", 50, GRIP)}}


def _named(side):
    return {name.lower(): [dict(path=f"/nowhere/{name}", name=name, stage={"pole_mm": pole, "hand": hand},
                                source="your files in the Trajectory Viewer")]
            for name, pole, hand in UPLOADED[side].values()}


def _steps(side, rows):
    from alix_studio_lite.server import workflow

    return {s: workflow.check_steps(rows if s == side else [], s) for s in ("left", "right")}


def _prog(i):
    return {"type": "program", "id": i, "name": "", "speed": 100, "transit_speed": 30}


def _own_steps(plan, side):
    return [(x["type"], x.get("_own"), x.get("mm"), x.get("values")) for st in plan for x in (st.get(side) or {}).get("steps", [])
            if x.get("_own")]


def test_a_program_lite_named_is_found_by_its_file_on_a_computer_that_did_not_send_it():
    """The narrow fix. Lite names a program it sends by the first 28 characters of its file's name and ".txt"; with its
    slot before it, RealMan's API keeps 32 characters: "62_62_R_P2_pole200_grip_13.0s_n." (a one-digit slot ends ".t").
    Found by its start without that end; all ten of the sequence's Programs, as on the Jetson with their files there."""
    vendor_paths.activate()
    from alix_studio_lite.server import workflow

    for side in ("left", "right"):
        for pid, held in HELD[side].items():
            f = workflow.found_by_name(_named(side), pid, _prog(pid), dict(name=held, sent=False))
            assert f is not None and f["name"] == UPLOADED[side][pid][0], (side, pid, held)
            assert f["stage"]["hand"] == UPLOADED[side][pid][2]
    named = {"l_p1_r01+r02+r03+r04_pole50_cleaningpose_29.5s_v1.1.txt": [dict(path="/nowhere/a.txt", name="a", stage=None)]}
    one = workflow.found_by_name(named, 1, _prog(1), dict(name="1_L_P1_R01+R02+R03+R04_pole50_.t", sent=False))
    assert one is not None                                      # a one-digit slot: ".t" left of ".txt"
    short = workflow.found_by_name({"l_p01p02_more.txt": [dict(path="/nowhere/b.txt", name="b", stage=None)]}, 81, _prog(81),
                                   dict(name="81_L_P01P02.", sent=False))
    assert short is None                                        # not cut: never matched by its start


def test_s2_sets_the_hand_before_the_grip_programs_on_the_computer_that_did_not_send_them():
    """The Jetson's run of 15 Nov as it plans now: with the files in its Trajectory Viewer, each of the right arm's
    Programs has its own pole and hand, and the hand closes before 62 and 65 (it stayed open through five runs)."""
    vendor_paths.activate()
    from alix_studio_lite.server import workflow

    rows = [{"type": "pole", "mm": 210}, {"type": "hand", "values": CLEANING}, _prog(61), _prog(62), _prog(63), _prog(64), _prog(65)]
    steps = _steps("right", rows)
    programs = {"right": {pid: dict(id=pid, name=n, size=None, sent=False, stage=None, file=None) for pid, n in HELD["right"].items()},
                "left": {}}
    own = workflow.own_of(programs, {"right": _named("right"), "left": {}}, lambda _p: None, steps)
    assert {pid: o["how"] for pid, o in own["right"].items()} == {p: "file" for p in (61, 62, 63, 64, 65)}
    run = workflow.plan_for(workflow._numbered(steps), "right", own=own)
    assert _own_steps(run["plan"], "right") == [("pole_hand", 61, 210.0, CLEANING), ("pole_hand", 62, 200.0, GRIP),
                                                ("hand", 63, None, CLEANING), ("pole", 64, 50.0, None), ("hand", 65, None, GRIP)]


def test_the_project_keeps_a_programs_pole_and_hand_for_the_computer_that_does_not_know_it(tmp_path):
    """The wider fix. The lab PC knows Program 62's pole and hand (its record of sending it) and Program 64's (its
    file, found by name): they are kept in the project for the programs as the controller lists them. The Jetson knows
    neither and has no file: it uses the project's, for as long as the slot holds that very program."""
    vendor_paths.activate()
    from alix_studio_lite.server import program_links as pl
    from alix_studio_lite.server import workflow

    units = SimpleNamespace(root=tmp_path / "project")
    lab = {62: dict(sent=True, name="62_R_P2_pole200_grip_13.0s_n", sha256="aa", stage={"pole_mm": 200.0, "hand": GRIP},
                    held=dict(name=HELD["right"][62], bytes=482113)),
           64: dict(sent=False, name=HELD["right"][64], size=283004, stage=None, file=None),
           66: dict(sent=True, gone=True, name="66_old", stage={"pole_mm": 10.0, "hand": OPEN}),
           63: dict(sent=False, name=HELD["right"][63], size=1000, stage=None, file=None)}
    own = {64: dict(stage={"pole_mm": 50.0, "hand": CLEANING}, how="file", file_name=UPLOADED["right"][64][0]), 63: dict(stage=None)}
    kept = pl.keep(units, "right", lab, own)
    assert sorted(kept) == [62, 64]                                 # 63: nothing known; 66: the slot no longer holds it
    assert (kept[62]["by"], kept[62]["text"], kept[62]["bytes"]) == ("record", "pole 200 mm, hand grip", 482113)
    assert (kept[64]["by"], kept[64]["file"]) == ("file", UPLOADED["right"][64][0])
    doc = json.loads((tmp_path / "project/controller_exports/right/pole_and_hand.json").read_text())
    assert doc["arm"] == "right" and [e["slot"] for e in doc["programs"]] == [62, 64]

    jetson = {pid: dict(id=pid, name=HELD["right"][pid], size=size, sent=False, stage=None, file=None)
              for pid, size in ((62, 482113), (64, 283004), (65, 700000))}
    steps = _steps("right", [{"type": "pole", "mm": 200}, {"type": "hand", "values": CLEANING}, _prog(62), _prog(64), _prog(65)])
    links = {"right": pl.load(units, "right"), "left": {}}
    own = workflow.own_of({"right": jetson, "left": {}}, {"right": {}, "left": {}}, lambda _p: None, steps, links=links)
    assert own["right"][62]["stage"]["hand"] == GRIP and own["right"][62]["how"] == "link" and own["right"][62]["link"]["compared"]
    assert own["right"][64]["stage"]["pole_mm"] == 50.0 and own["right"][65] == {"stage": None, "first_deg": None}
    run = workflow.plan_for(workflow._numbered(steps), "right", own=own)
    assert _own_steps(run["plan"], "right") == [("pole_hand", 62, 200.0, GRIP), ("pole_hand", 64, 50.0, CLEANING)]
    lp = [x for st in run["plan"] for x in st["right"]["steps"] if x["type"] == "program" and x["id"] == 62][0]
    assert lp["_own_stage"]["link"]["by"] == "record"               # the simulation says whose they are

    stamp = (tmp_path / "project/controller_exports/right/pole_and_hand.json").read_bytes()
    assert pl.keep(units, "right", jetson) == kept                  # the computer that knows less changes nothing
    assert (tmp_path / "project/controller_exports/right/pole_and_hand.json").read_bytes() == stamp
    assert pl.keep(units, "right", {62: dict(sent=True, name="x", stage={"pole_mm": 1, "hand": OPEN})}) is None   # no list read
    assert pl.link_for(links["right"], 62, {})["compared"] is False                                       # used, not compared

    other = jetson | {62: dict(jetson[62], name="62_62_R_P2_pole200_allopen_9.0s.")}                      # another program now
    assert pl.link_for(links["right"], 62, other[62]) is None
    assert pl.link_for(links["right"], 62, dict(jetson[62], size=5)) is None                               # other stored bytes
    assert sorted(pl.keep(units, "right", other)) == [64]                                                  # its entry goes


def test_a_program_runs_with_its_record_then_its_file_then_what_was_stated(tmp_path):
    """The order (the user, 15 Nov): its taught pole and hand (Lite's record of sending it, this computer's or, through
    the project, another's), else the ones stated for its file at its upload, else the ones stated by hand for a
    program nothing is known of. A stated entry is replaced by a file's, a file's by a record's, never the other way."""
    vendor_paths.activate()
    from alix_studio_lite.server import program_links as pl
    from alix_studio_lite.server import workflow
    from alix_studio_lite.server.jobs import UserError

    units = SimpleNamespace(root=tmp_path / "project")
    held = dict(name=HELD["right"][62], bytes=482113)
    info = dict(id=62, name=held["name"], size=held["bytes"], sent=False, stage=None, file=None)
    with pytest.raises(UserError) as exc:
        pl.state(units, "right", 62, None, 200, "GRIP")              # the controller's list not read
    assert exc.value.code == "connect"
    said = pl.state(units, "right", 62, held, 180, "GRIP")
    assert (said["by"], said["text"], said["name"]) == ("stated", "pole 180 mm, hand grip", held["name"])
    assert pl.state(units, "right", 62, held, 200, "GRIP")["pole_mm"] == 200.0          # stated again: corrected
    link = pl.link_for(pl.load(units, "right"), 62, info)
    found = dict(name="62_R.txt", stage={"pole_mm": 190.0, "hand": GRIP})
    record = dict(info, sent=True, stage={"pole_mm": 170.0, "hand": GRIP})
    assert workflow.own_stage(info, None, link)["how"] == "link" and workflow.own_stage(info, None, link)["stage"].pole_mm == 200.0
    assert workflow.own_stage(info, found, link)["how"] == "file"                        # its file before what was stated
    assert workflow.own_stage(record, found, link)["how"] == "record"                    # its record before both
    assert workflow.own_stage(info, None, None) is None

    pl.keep(units, "right", {62: info}, {62: dict(stage=found["stage"], how="file", file_name="62_R.txt")})
    link = pl.link_for(pl.load(units, "right"), 62, info)
    assert (link["by"], link["pole_mm"]) == ("file", 190.0)                             # the file's replaces the stated one
    with pytest.raises(UserError) as exc:
        pl.state(units, "right", 62, held, 100, "OPEN")              # known now: not replaced by hand
    assert exc.value.code == "own" and "runs with its own" in str(exc.value)
    pl.keep(units, "right", {62: dict(record, held=held)})
    by_record = pl.link_for(pl.load(units, "right"), 62, info)
    assert (by_record["by"], by_record["pole_mm"]) == ("record", 170.0)
    assert workflow.own_stage(info, found, by_record)["how"] == "link"                   # another computer's record before a file here
    pl.keep(units, "right", {62: info}, {62: dict(stage=found["stage"], how="file", file_name="62_R.txt")})
    assert pl.load(units, "right")[62]["by"] == "record"                                 # a file's never replaces a record's


# ── the Trajectory Viewer and Run, through the server

PID = "alix_links_test"
P = f"/api/projects/{PID}"


@pytest.fixture(scope="module")
def lite(tmp_path_factory):
    vendor_paths.activate()
    from alix_studio_lite.server.app import LiteApp, make_server
    from alix_studio_lite.server.project import open_project

    data = tmp_path_factory.mktemp("lite-links")
    app = LiteApp(open_project(data, "commode_c"), data, projects_dir=data / "Projects")
    srv = make_server(app, 0)
    threading.Thread(target=srv.serve_forever, kwargs={"poll_interval": 0.05}, daemon=True).start()
    ctx = {"base": f"http://127.0.0.1:{srv.server_address[1]}", "app": app, "data": data}
    ctx["tok"] = call(ctx, "POST", "/api/auth/login", {"pin": "1234"})["token"]
    call(ctx, "PUT", P, {"id": PID, "name": "Links", "archived": False, "state": {"currentTask": 0}})
    assert call(ctx, "POST", "/api/robots/left/connect", {})["connected"]
    assert call(ctx, "POST", "/api/robots/mode", {"mode": "hwsim"})["mode"] == "hwsim"
    yield ctx
    srv.shutdown()


def call(ctx, method, path, body=None):
    headers = {"Content-Type": "application/json", "X-Lite-Token": ctx.get("tok") or ""}
    r = urllib.request.Request(ctx["base"] + path, method=method, headers=headers,
                               data=None if body is None else json.dumps(body).encode())
    with urllib.request.urlopen(r, timeout=120) as resp:
        return json.loads(resp.read())


def wait(ctx, job, limit=120):
    t0 = time.time()
    while True:
        j = call(ctx, "GET", f"/api/jobs/{job['id']}")
        if j["state"] != "running":
            return j
        assert time.time() - t0 < limit, j
        time.sleep(0.1)


def content(start=0.0):
    from alix_studio_lite.motion import track

    t = np.linspace(0.0, 1.0, 120)[:, None]
    md = np.round((np.array([[10, -40, 0, 80, 0, 40, 0]]) + start + 4.0 * np.sin(2 * np.pi * t)) * 1000).astype(np.int64)
    return track.track_bytes(md).decode()


def test_a_program_nothing_is_known_of_has_its_pole_and_hand_stated_in_the_viewer(lite, monkeypatch):
    """A program on a controller with no record and no file here (a new arm connected, a program another computer
    sent): its row in the Trajectory Viewer takes its pole and hand. They are kept with the open project, shown on the
    row, and staged first when it runs from its slot; one whose pole and hand are known is never set by hand."""
    from alix_studio_lite.server import api_library

    up = call(lite, "POST", "/api/robots/left/programs/upload", {"program_id": 47, "name": "L_bare.txt", "content": content(),
                                                                "confirmed": True})
    assert wait(lite, up)["state"] == "done"
    up = call(lite, "POST", "/api/robots/left/programs/upload", {"program_id": 48, "name": "L_own.txt", "content": content(1.0),
                                                                "pole_mm": 120, "hand": "GRIP", "confirmed": True})
    assert wait(lite, up)["state"] == "done"
    row = lambda n: next(p for p in call(lite, "GET", f"/api/library/programs?pid={PID}")["arms"]["left"]["programs"]     # noqa: E731
                         if p["id"] == n)
    assert not row(47)["can_state"] and row(47)["stage"] is None                    # the stand-in: no controller's list
    refused = call(lite, "POST", "/api/library/programs/left/47", {"pid": PID, "pole_mm": 80, "hand": "GRIP"})
    assert refused["code"] == "connect"
    assert call(lite, "POST", "/api/library/programs/left/47", {"pole_mm": 80, "hand": "GRIP"})["code"] == "project"

    monkeypatch.setattr(api_library, "_real_list", lambda app, side: True)        # as with a real controller's list read
    assert row(47)["can_state"] and "set them on this row" in row(47)["why"]
    assert not row(48)["can_state"] and row(48)["stage_by"] == "record"
    own = call(lite, "POST", "/api/library/programs/left/48", {"pid": PID, "pole_mm": 80, "hand": "OPEN"})
    assert own["code"] == "own" and "runs with its own pole and hand: pole 120 mm, hand grip (as Lite sent it)" in own["error"]

    done = call(lite, "POST", "/api/library/programs/left/47", {"pid": PID, "pole_mm": 80, "hand": "GRIP"})
    assert done["stated"]["text"] == "pole 80 mm, hand grip" and done["stated"]["by"] == "stated"
    r = next(p for p in done["arms"]["left"]["programs"] if p["id"] == 47)
    assert r["stage"]["text"] == "pole 80 mm, hand grip" and r["stage_by"] == "stated" and r["can_state"]
    assert "It runs with pole 80 mm, hand grip (kept with the project: stated in the Trajectory Viewer" in r["why"]
    (file,) = (lite["data"] / "Projects").rglob("controller_exports/left/pole_and_hand.json")     # in the project: it goes with it
    kept = json.loads(file.read_text())
    assert [(e["slot"], e["by"], e["pole_mm"]) for e in kept["programs"]] == [(47, "stated", 80.0)]

    run = wait(lite, call(lite, "POST", "/api/robots/left/programs/run", {"program": 47, "pid": PID, "pole_mm": 30, "hand": "OPEN",
                                                                          "confirmed": True}))
    words = " | ".join(e["text"] for e in run.get("events") or [])
    assert run["state"] == "done", run
    assert "Program 47: pole and hand pole 80 mm, hand grip (kept with the project: stated in the Trajectory Viewer" in words
    again = call(lite, "POST", "/api/library/programs/left/47", {"pid": PID, "pole_mm": 90, "hand": "CLEANING"})
    assert again["stated"]["text"] == "pole 90 mm, hand cleaning pose"              # stated again: corrected
    bare = call(lite, "POST", "/api/robots/left/programs/run", {"program": 47, "confirmed": True})
    assert bare["code"] == "stage"                                                   # no project named: asked, as before


def test_the_simulations_arm_keeps_nothing_with_the_project(lite):
    """As the controllers' programs themselves (5 Nov): only a real controller's list writes the project's entries."""
    from alix_studio_lite.server import program_links as pl

    (folder,) = (lite["data"] / "Projects").rglob("controller_exports")
    units = SimpleNamespace(root=folder.parent)
    known = {47: dict(sent=True, name="x", stage={"pole_mm": 10, "hand": OPEN}, held=dict(name="47_x.", bytes=10))}
    before = pl.load(units, "right")
    assert pl.keep_quietly(lite["app"], units, "right", known) is None and pl.load(units, "right") == before


#: 24 Nov, the lab PC: the Programs as the two controllers listed them that evening, each with the bytes its slot held
#: (the project's pole_and_hand.json). The name is the file's own first 32 characters: no slot number put before it, as
#: there is before a name Lite's own upload sends (HELD above, "61_61_L_...").
HELD_24_NOV = {"left": {61: ("61_L_P1_pole50_cleaningpose_48.2", 555156), 62: ("62_L_P2_pole200_cleaningpose_22.", 260205),
                        63: ("63_L_P2_pole200_grip_10.5s_25pct", 121219), 64: ("64_L_P3_pole200_cleaningpose_19.", 231884),
                        65: ("65_L_P3_pole200_grip_45.3s_smoot", 514605), 66: ("66_L_P3_pole20_grip_15.6s_25pct.", 176105),
                        67: ("67_L_P3_pole200_allopen_12.5s_25", 144328)},
               "right": {61: ("61_R_P1_pole200_cleaningpose_53.", 616050), 62: ("62_R_P2_pole200_grip_13.0s_new_R", 140045),
                         63: ("63_R_P2_pole200_cleaningpose_9.2", 104017), 64: ("64_1_R_P2_pole50_cleaningpose_10", 118251),
                         65: ("65_R_P3_pole50_grip_39.8s_new_R0", 458059), 66: ("66_R_P3_pole200_allopen_wait_73.", 854278)}}


def test_a_program_listed_under_its_files_own_name_is_found_by_that_name():
    """The lab, 24 Nov: the controllers listed each Program under its file's own first 32 characters
    ("61_L_P1_pole50_cleaningpose_48.2"). Lite took the leading "61_" for the slot number a controller puts before a
    name, looked for a file starting "L_P1_...", and found none of the ten the sequence runs, with or without the files
    on the computer. The name as listed is tried too."""
    vendor_paths.activate()
    from alix_studio_lite.server import workflow

    for side in ("left", "right"):
        named = _named(side)
        for pid, (name, _pole, _hand) in UPLOADED[side].items():
            held = HELD_24_NOV[side][pid][0]
            info = dict(id=pid, name=held, size=None, sent=False, stage=None, file=None)
            stale = {"id": pid, "name": f"{pid}_{pid}_{name}"[:32]}              # the row's label, of an older upload
            for step in ({"id": pid}, stale):
                got = workflow.found_by_name(named, pid, step, info)
                assert got is not None and got["name"] == name, (side, pid, step)
    old = dict(id=61, name=HELD["left"][61], size=None, sent=False, stage=None, file=None)      # 15 Nov's names: as before
    assert workflow.found_by_name(_named("left"), 61, {"id": 61}, old)["name"] == UPLOADED["left"][61][0]


def test_the_labs_programs_of_24_nov_are_drawn_on_any_computer_from_the_kept_set():
    """The user, 24 Nov: a file loaded on a controller from one computer "should be able to work throughout, in any new
    PC". A controller gives no program's points back, so the points travel with Lite: the 13 files the lab's controllers
    held on 24 Nov are a set of the Trajectory Viewer's (assets/recordings/sets.yaml), on every computer that has Lite.
    Each is found from its controller's listed name and is the very program by its stored size; the sequence that
    stopped that evening is then drawn to its end, and the MoveJ home after the right arm's Program 65 has its own time
    (0.97 to 1.27 s: the lab's two took about 1.0 s) from where that Program ends."""
    vendor_paths.activate()
    from alix_studio_lite.motion.limits import load_limits
    from alix_studio_lite.server import library, workflow

    kept = next(s for s in library.builtin_sets() if s["id"] == "lab_25_pct_19_nov")
    assert len(kept["files"]) == 13 and {f["arm"] for f in kept["files"]} == {"left", "right"}
    named = {side: workflow.files_by_name(SimpleNamespace(library=None), None, side) for side in ("left", "right")}
    programs = {side: {pid: dict(id=pid, name=n, size=b, sent=False, stage=None, file=None) for pid, (n, b) in HELD_24_NOV[side].items()}
                for side in ("left", "right")}
    for side in ("left", "right"):
        for pid, info in programs[side].items():
            got = workflow.found_by_name(named[side], pid, {"id": pid}, info)
            assert got is not None and got["how"] == "its bytes make the size the controller holds", (side, pid, got)
            assert got["name"].startswith(info["name"]) and "Lab Programs" in got["source"]

    home = [-2.52, -94.79, 1.42, 94.20, 0.04, 84.54, 6.18]                       # R_HME_POS, as the feed gave it
    rows = [{"type": "global", "name": "R_HME_POS", "move": "movej", "speed": 50, "wait_s": 0.0, "_row": 2},
            *[{"type": "program", "id": pid, "transit_speed": 30, "_row": row} for pid, row in ((61, 6), (62, 8), (63, 9), (64, 10), (65, 12))],
            {"type": "pole", "mm": 150.0, "_row": 13},
            {"type": "global", "name": "R_HME_POS", "move": "movej", "speed": 20, "wait_s": 0.0, "_row": 14}]
    sim = workflow._ArmSim("right", dict(pole_mm=150.0, hand=[1000] * 6, q_deg=home), load_limits("right").rating_deg_s, 60,
                           programs["right"], {"points": {"R_HME_POS": home}}, workflow._md_reader(None), [], named["right"])
    sim.run(rows, 0.0, 0)
    assert [a["drawn"] for a in sim.acts if a["kind"] == "program"] == [True] * 5
    assert not [f for f in sim.findings if "not drawn" in f["text"] or "not known" in f["text"]]
    last = rows[-1]["_t"]
    assert last["known"] is True and tuple(last["move_times"]) == pytest.approx((0.97, 1.14, 1.27), abs=0.01)

    # the set states no pole and hand: the ones kept with the project for the programs on its controllers still rule
    # (here the ones the lab stated in the Trajectory Viewer that evening), and a Program with none kept has none
    assert all(f["stage"] is None for f in kept["files"])
    links = {"right": {65: dict(slot=65, name=HELD_24_NOV["right"][65][0], bytes=HELD_24_NOV["right"][65][1], pole_mm=50.0,
                                 hand=GRIP, by="stated")}, "left": {}}
    steps = _steps("right", [{"type": "pole", "mm": 210}, {"type": "hand", "values": CLEANING}, _prog(64), _prog(65)])
    own = workflow.own_of(programs, named, workflow._md_reader(None), steps, links=links)
    assert own["right"][65]["how"] == "link" and own["right"][65]["stage"]["hand"] == GRIP and own["right"][65]["stage"]["pole_mm"] == 50.0
    assert own["right"][64] == {"stage": None, "first_deg": None}


# One value (the user, 24 Nov 2026). The lab's RIGHT row 3 set the pole to 210 mm; Program 61's own, stated in the
# Trajectory Viewer that evening, was 200 mm, and Lite lowered the pole to it before the Program. "For program 61, use
# 210mm, because that is where the command of the pole comes from, so editing the pole value in either the sequence or the
# file using alix studio lite, should make the change take effect"; "build it so both places hold one value".

def _lab_right(*pids):
    held = HELD_24_NOV["right"]
    programs = {"right": {pid: dict(id=pid, name=held[pid][0], size=held[pid][1], sent=False, stage=None, file=None) for pid in pids},
                "left": {}}
    stated = {61: (200.0, CLEANING), 62: (200.0, GRIP), 63: (200.0, CLEANING), 64: (50.0, CLEANING), 65: (50.0, GRIP)}
    links = {"right": {pid: dict(slot=pid, name=held[pid][0], bytes=held[pid][1], pole_mm=stated[pid][0], hand=stated[pid][1],
                                 by="stated", kept="2026-11-24 18:40:05") for pid in pids}, "left": {}}
    return programs, links


def test_a_programs_pole_is_the_one_its_pole_row_sets():
    """A Program's Pole row (its last Pole step since the Program, lid or seat task or INSPECT before it) is that
    Program's own pole: the plan takes the row's height in place of the one known before, and says which it replaced.
    The hand stays the Program's own, and a row is the pole of the first Program after it only."""
    vendor_paths.activate()
    from alix_studio_lite.server import workflow

    row = lambda *types: [{"type": t} for t in types]                                  # noqa: E731
    assert workflow.pole_row(row("pole", "hand", "meet", "wait", "global", "wait_for", "program"), 6) == 0
    assert workflow.pole_row(row("pole", "pole", "hand", "program"), 3) == 1          # the last one
    for between in ("program", "mechanism", "inspect"):
        assert workflow.pole_row(row("pole", between, "program"), 2) is None
    assert workflow.pole_row(row("hand", "program"), 1) is None

    programs, links = _lab_right(61, 62)
    steps = _steps("right", [{"type": "pole", "mm": 210}, {"type": "hand", "values": OPEN}, _prog(61), _prog(62)])
    own = workflow.own_of(programs, {"right": {}, "left": {}}, lambda _p: None, steps, links=links)
    o = own["right"][61]
    assert o["stage"]["pole_mm"] == 210.0 and o["stage"]["text"] == "pole 210 mm, hand cleaning pose"    # the hand: its own
    assert o["pole_row"] == dict(row=1, mm=210.0, was_mm=200.0, was="stated", fresh=True)
    assert own["right"][62]["stage"]["pole_mm"] == 200.0 and "pole_row" not in own["right"][62]           # row 1 is 61's
    run = workflow.plan_for(workflow._numbered(steps), "right", own=own)
    assert _own_steps(run["plan"], "right") == [("pole_hand", 61, 210.0, CLEANING), ("pole_hand", 62, 200.0, GRIP)]
    same = _steps("right", [{"type": "pole", "mm": 200}, {"type": "hand", "values": CLEANING}, _prog(61)])
    assert "pole_row" not in workflow.own_of(programs, {"right": {}, "left": {}}, lambda _p: None, same, links=links)["right"][61]


def test_the_project_keeps_the_pole_a_row_sets_for_every_computer(tmp_path):
    """Where a controller's list is read, the row's pole is kept with the project for the program in the slot ("set"),
    with what it replaced. It then comes before any computer's own record or file of the program, so the Trajectory
    Viewer and every computer that runs the project hold the row's value; a later height of the row replaces it; another
    program in the slot ends it."""
    vendor_paths.activate()
    from alix_studio_lite.server import program_links as pl
    from alix_studio_lite.server import workflow

    units = SimpleNamespace(root=tmp_path / "project")
    name, size = HELD_24_NOV["right"][61]
    lab = {61: dict(id=61, name=name, size=size, sent=False, stage=None, file=None)}
    pl.state(units, "right", 61, dict(name=name, bytes=size), 200, CLEANING)          # as on 24 Nov, 18:40
    steps = _steps("right", [{"type": "pole", "mm": 210}, {"type": "hand", "values": CLEANING}, _prog(61)])

    def own_here(known, rows=steps):
        return workflow.own_of({"right": known, "left": {}}, {"right": {}, "left": {}}, lambda _p: None, rows,
                               links={"right": pl.load(units, "right"), "left": {}})["right"]

    kept = pl.keep(units, "right", lab, own_here(lab))
    e = kept[61]
    assert (e["by"], e["pole_mm"], e["hand"], e["was"], e["was_mm"], e["row"]) == ("set", 210.0, CLEANING, "stated", 200.0, 1)
    assert e["text"] == "pole 210 mm, hand cleaning pose" and "Pole row" in pl.where_from(e)
    again = own_here(lab)[61]                                    # as kept: the row and the program hold one value
    assert again["stage"]["pole_mm"] == 210.0 and again["pole_row"] == dict(row=1, mm=210.0, was_mm=200.0, was="stated", fresh=False)
    stamp = (tmp_path / "project/controller_exports/right/pole_and_hand.json").read_bytes()

    sender = {61: dict(id=61, name="61_R_P1", sent=True, sha256="aa", stage={"pole_mm": 200.0, "hand": CLEANING},
                       held=dict(name=name, bytes=size))}        # the computer that sent it: its record says 200 mm
    there = own_here(sender)[61]
    assert there["how"] == "link" and there["stage"]["pole_mm"] == 210.0 and there["link"]["by"] == "set"
    assert pl.keep(units, "right", sender, own_here(sender)) == kept                  # its record does not take it back
    assert (tmp_path / "project/controller_exports/right/pole_and_hand.json").read_bytes() == stamp

    higher = _steps("right", [{"type": "pole", "mm": 215}, {"type": "hand", "values": CLEANING}, _prog(61)])
    e = pl.keep(units, "right", sender, own_here(sender, higher))[61]
    assert (e["by"], e["pole_mm"], e["was"], e["was_mm"]) == ("set", 215.0, "stated", 200.0)   # what it was first known as

    other = {61: dict(id=61, name="61_another_program", size=1234, sent=False, stage=None, file=None)}
    assert pl.keep(units, "right", other, own_here(other, higher)) == {}              # another program in the slot


def test_a_pole_changed_in_the_viewer_moves_the_programs_pole_row(tmp_path):
    """The other place. A program's pole stated again in the Trajectory Viewer takes the place of the one its row set
    (when nothing but a statement was known of it), and the sequence's Pole row of that Program follows, so the row
    never puts the old height back. A program Lite sent, or has a file of, is still not set by hand there."""
    import threading

    vendor_paths.activate()
    from alix_studio_lite.server import program_links as pl
    from alix_studio_lite.server import workflow
    from alix_studio_lite.server.jobs import UserError

    units = SimpleNamespace(root=tmp_path / "project", lock=threading.RLock())
    name, size = HELD_24_NOV["right"][61]
    held = dict(name=name, bytes=size)
    rows = [{"type": "pole", "mm": 210}, {"type": "hand", "values": CLEANING}, _prog(61), {"type": "pole", "mm": 50}, _prog(64),
            {"type": "pole", "mm": 150}]
    workflow.save_side(units, "right", rows)
    known = {61: dict(id=61, name=name, size=size, sent=False, stage=None, file=None)}
    pl.state(units, "right", 61, held, 200, CLEANING)
    steps = {"right": workflow.load(units)["right"], "left": []}
    own = workflow.own_of({"right": known, "left": {}}, {"right": {}, "left": {}}, lambda _p: None, steps,
                          links={"right": pl.load(units, "right"), "left": {}})["right"]
    assert pl.keep(units, "right", known, own)[61]["by"] == "set"

    e = pl.state(units, "right", 61, held, 205, CLEANING)                             # Change pole and hand, in the Viewer
    assert (e["by"], e["pole_mm"]) == ("stated", 205.0)
    assert workflow.follow_pole(units, "right", 61, 205) == [1]
    now = workflow.load(units)["right"]
    assert [x.get("mm") for x in now if x["type"] == "pole"] == [205.0, 50.0, 150.0]  # its row alone
    assert [x["uid"] for x in now] == [x["uid"] for x in steps["right"]]              # the rows are the same rows
    assert workflow.follow_pole(units, "right", 61, 205) == [] and workflow.follow_pole(units, "right", 65, 80) == []

    sent = dict(pl.load(units, "right")[61], by="set", was="record", was_mm=200.0)    # one Lite sent, its pole set by its row
    pl._write(units, "right", {61: sent})
    with pytest.raises(UserError) as refused:
        pl.state(units, "right", 61, held, 190, GRIP)
    assert refused.value.code == "own" and "Pole row" in str(refused.value.hint)


def test_the_simulation_says_a_program_its_row_takes_off_the_pole_it_was_sent_with():
    """The owner's rule of 24 Oct stood against this: a Program ran at the pole it was taught or sent with, whatever
    the row before it said. A row now sets the pole (24 Nov), so the simulation and the run's question say it every
    time a Program runs off the pole Lite's record or its file gives it; it is not a stop any more."""
    vendor_paths.activate()
    from alix_studio_lite.server import workflow

    progs = {"right": {71: dict(id=71, stage={"pole_mm": 50.0, "hand": GRIP}, file="/p.txt", sent=True, lines=41)}, "left": {}}
    steps = _steps("right", [{"type": "pole", "mm": 200}, _prog(71)])
    own = workflow.own_of(progs, {"right": {}, "left": {}}, {"/p.txt": np.zeros((41, 7))}.get, steps)
    assert own["right"][71]["pole_row"] == dict(row=1, mm=200.0, was_mm=50.0, was="record", fresh=True)
    run = workflow.plan_for(workflow._numbered(steps), "right", own=own)
    said = []
    sim = workflow._ArmSim("right", dict(pole_mm=300.0, hand=OPEN, q_deg=[0.0] * 7), [180, 180, 225, 225, 225, 225, 225], 100,
                           progs["right"], None, {"/p.txt": np.zeros((41, 7))}.get, said)
    sim.run(run["plan"][0]["right"]["steps"], 0.0, 0)
    assert sim.pole == 200.0 and not [f for f in said if f["level"] == "stop"]
    warns = [f["text"] for f in said if f["level"] == "warn"]
    assert warns == ["RIGHT row 2, program 71: it was trained at pole 50 mm (Lite's record of sending it), and its Pole row "
                     "(row 1) sets 200 mm. It runs at the set height, 200 mm."]


def test_the_sequence_and_the_viewer_hold_one_pole_for_a_program(lite, monkeypatch):
    """Through Lite's own routes (the stand-in's arm, a real controller's list stood in for where a route asks for
    one). Program 47's pole was stated in the Trajectory Viewer, Program 48 was sent by Lite with its own.
    - A pole changed in the Viewer moves the Program's Pole row in the sequence.
    - A Pole row changed in the Sequence tab is kept with the project at once (keep_row_poles), and the plan runs the
      Program at the row's height, with a warning that it was trained at another.
    - A run from the slot (Run, Run Stored) stages the pole its row set, as the Viewer shows it."""
    from alix_studio_lite.server import api_library, workflow

    for body in ({"program_id": 47, "name": "L_bare.txt", "content": content()},                       # as the test above
                 {"program_id": 48, "name": "L_own.txt", "content": content(1.0), "pole_mm": 120, "hand": "GRIP"}):
        up = call(lite, "POST", "/api/robots/left/programs/upload", dict(body, replace=True, confirmed=True))
        assert up.get("unchanged") or wait(lite, up)["state"] == "done", up
    monkeypatch.setattr(api_library, "_real_list", lambda app, side: True)
    assert call(lite, "POST", "/api/library/programs/left/47", {"pid": PID, "pole_mm": 90, "hand": "CLEANING"})["stated"]["by"] == "stated"
    rows = [{"type": "pole", "mm": 90}, {"type": "hand", "values": CLEANING}, _prog(47),
            {"type": "pole", "mm": 150}, {"type": "hand", "values": GRIP}, _prog(48)]
    kept = call(lite, "PUT", f"{P}/workflow/left", {"steps": rows})["left"]
    assert [x.get("mm") for x in kept if x["type"] == "pole"] == [90.0, 150.0]

    done = call(lite, "POST", "/api/library/programs/left/47", {"pid": PID, "pole_mm": 100, "hand": "CLEANING"})
    assert done["stated"]["pole_mm"] == 100.0 and done["rows"] == [1]                # its Pole row follows
    now = call(lite, "GET", f"{P}/workflow")["left"]
    assert [x.get("mm") for x in now if x["type"] == "pole"] == [100.0, 150.0] and [x["uid"] for x in now] == [x["uid"] for x in kept]

    asked = []
    monkeypatch.setattr(workflow, "keep_row_poles", lambda app, units, side: asked.append(side))
    now[1] = dict(now[1], values=OPEN)                                               # a Hand row: no pole changed
    now = call(lite, "PUT", f"{P}/workflow/left", {"steps": now})["left"]
    assert asked == []
    now[0] = dict(now[0], mm=110)                                                    # Program 47's Pole row
    now = call(lite, "PUT", f"{P}/workflow/left", {"steps": now})["left"]
    assert asked == ["left"] and now[0]["mm"] == 110.0

    j = wait(lite, call(lite, "POST", f"{P}/workflow/simulate", {"mode": "left"}))
    assert j["state"] == "done", j
    sim = j["result"]
    poles = [(a["row"], a["pole1"]) for a in sim["acts"]["left"] if a["kind"] == "program"]
    assert poles == [(3, 110.0), (6, 150.0)]                                         # each at its row's height
    texts = [(f["level"], f["text"]) for f in sim["findings"] if "It runs at the set height" in f["text"]]
    assert texts == [("warn", "LEFT row 3, program 47: it was trained at pole 100 mm (as stated in the Trajectory Viewer), and its "
                              "Pole row (row 1) sets 110 mm. It runs at the set height, 110 mm."),
                     ("warn", "LEFT row 6, program 48: it was trained at pole 120 mm (Lite's record of sending it), and its Pole "
                              "row (row 4) sets 150 mm. It runs at the set height, 150 mm.")]
    assert not [f for f in sim["findings"] if f["level"] == "stop"]                  # a Program at its row's pole is not off its own

    # as a real controller's list keeps it (program_links.keep): Program 48's pole, set by its row, in the project
    row48 = next(p for p in call(lite, "GET", f"/api/library/programs?pid={PID}")["arms"]["left"]["programs"] if p["id"] == 48)
    (file,) = (lite["data"] / "Projects").rglob("controller_exports/left/pole_and_hand.json")
    doc = json.loads(file.read_text())
    doc["programs"].append(dict(slot=48, name=row48["name"], bytes=row48["bytes"], pole_mm=150.0, hand=GRIP, text="pole 150 mm, hand grip",
                                by="set", row=4, was="record", was_mm=120.0, kept="2026-11-24 22:00:00"))
    file.write_text(json.dumps(doc))
    shown = next(p for p in call(lite, "GET", f"/api/library/programs?pid={PID}")["arms"]["left"]["programs"] if p["id"] == 48)
    assert shown["stage"]["text"] == "pole 150 mm, hand grip" and shown["stage_by"] == "set" and not shown["can_state"]
    assert "its pole as its Pole row in the sequence sets it (row 4)" in shown["stage_from"]
    refused = call(lite, "POST", "/api/library/programs/left/48", {"pid": PID, "pole_mm": 80, "hand": "OPEN"})
    assert refused["code"] == "own" and "change its Pole row in the sequence" in refused["hint"]
    run = wait(lite, call(lite, "POST", "/api/robots/left/programs/run", {"program": 48, "pid": PID, "confirmed": True}))
    assert run["state"] == "done", run
    assert "Program 48: pole and hand pole 150 mm, hand grip (kept with the project: its pole as its Pole row" in " | ".join(
        e["text"] for e in run.get("events") or [])
