"""Lite's lid and seat tasks and plans kept the same as ALIX's (the user, 28 Oct: "any change in either should be reflected
in the other"; server/alix_sync.py, tools/mechanism_sync.py): which side changed a file, pull and push, each plan against
its config, the sequence plans against their tasks', Lite's registry; and, where ALIX's checkout is on this computer,
the two sides the same (the suite fails while they differ: run tools/mechanism_sync.py)."""
import copy
import json
import os
import shutil
from pathlib import Path

import pytest
import yaml

from alix_studio_lite import vendor_paths
from alix_studio_lite.server import alix_sync, mechanism

PLANS, CONFIGS = alix_sync.PLANS_REL, alix_sync.CONFIGS_REL


def test_lites_registry_and_its_vendored_plans_agree_with_their_configs():
    rows = alix_sync.registry_checks() + alix_sync.plan_checks(vendor_paths.LITE_ROOT / alix_sync.VENDOR, "Lite")
    assert not [r for r in rows if r[0] == "error"], rows


def test_lites_lid_and_seat_copies_are_the_files_the_manifest_recorded():
    """A vendored file edited in Lite and not pushed to ALIX (or copied without its record) fails here."""
    man = json.loads((vendor_paths.LITE_ROOT / "vendor" / "MANIFEST.json").read_text())
    off = []
    for f in man["files"]:
        d = f["dest"]
        if d.startswith(alix_sync.VENDOR + "/") and alix_sync._ours(d[len(alix_sync.VENDOR) + 1:]):
            if alix_sync._sha(vendor_paths.LITE_ROOT / d) != (f.get("sha256_after_patch") or f["sha256"]):
                off.append(d)
    assert not off, f"edited in Lite since they were copied (push them: tools/mechanism_sync.py push): {off}"


@pytest.mark.skipif(alix_sync.alix_root() is None or os.environ.get("LITE_SYNC_SKIP") == "1",
                    reason="ALIX's checkout is not on this computer (or LITE_SYNC_SKIP=1)")
def test_alix_and_lite_hold_the_same_lid_and_seat_tasks_and_plans():
    rep = alix_sync.status()
    assert rep.in_sync, "\n".join(rep.lines() + ["Run: python3 tools/mechanism_sync.py (pull / push)"])


def _two_sides(tmp_path):
    """A small ALIX tree and a Lite tree holding the same lid_close plan files and config, and Lite's manifest for them."""
    alix, lite = tmp_path / "alix", tmp_path / "lite"
    names = [f"{PLANS}/{p.name}" for p in (vendor_paths.LITE_ROOT / alix_sync.VENDOR / PLANS).glob("lid_close_toppra.*")]
    names += [f"{CONFIGS}/lid_close.yaml"]
    man = dict(files=[])
    for rel in names:
        for root in (alix, lite / alix_sync.VENDOR):
            (root / rel).parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(vendor_paths.LITE_ROOT / alix_sync.VENDOR / rel, root / rel)
        man["files"].append(dict(dest=f"{alix_sync.VENDOR}/{rel}", source=str(alix / rel), origin="alix",
                                 sha256=alix_sync._sha(alix / rel), bytes=(alix / rel).stat().st_size))
    (lite / "vendor").mkdir(parents=True, exist_ok=True)
    (lite / "vendor" / "MANIFEST.json").write_text(json.dumps(man))
    return alix, lite


def test_each_difference_says_which_side_changed_and_pull_and_push_carry_it(tmp_path):
    alix, lite = _two_sides(tmp_path)
    states, same = alix_sync.file_states(alix, lite)
    assert not [f for f in states if f.state != "same"] and same >= 8
    cfg = f"{CONFIGS}/lid_close.yaml"
    plan = f"{PLANS}/lid_close_toppra.json"
    (alix / cfg).write_text((alix / cfg).read_text() + "\n# the owner's edit\n")                 # ALIX changed it
    (lite / alix_sync.VENDOR / plan).write_text((lite / alix_sync.VENDOR / plan).read_text() + "\n")   # Lite changed it
    new = alix / PLANS / "rungs" / "lid_close_toppra.rung1.json"                                 # a new rung of one of Lite's tasks
    new.parent.mkdir(parents=True, exist_ok=True)
    new.write_text("{}")
    got = {f.path: f.state for f in alix_sync.file_states(alix, lite)[0]}
    assert got == {cfg: "alix", plan: "lite", f"{PLANS}/rungs/lid_close_toppra.rung1.json": "new_alix"}
    done = dict(alix_sync.pull(alix, lite))
    assert set(done) == {cfg, f"{PLANS}/rungs/lid_close_toppra.rung1.json"}                       # Lite's own change not undone
    assert (lite / alix_sync.VENDOR / cfg).read_text() == (alix / cfg).read_text()
    assert dict(alix_sync.push(alix, lite)) == {plan: "copied into ALIX"}
    assert (alix / plan).read_text() == (lite / alix_sync.VENDOR / plan).read_text()
    assert not alix_sync.file_states(alix, lite)[0]                                               # all the same, recorded
    (alix / cfg).write_text("a: 1\n")                                                            # both sides: a conflict
    (lite / alix_sync.VENDOR / cfg).write_text("a: 2\n")
    assert [f.state for f in alix_sync.file_states(alix, lite)[0]] == ["conflict"]
    assert not alix_sync.pull(alix, lite) and not alix_sync.push(alix, lite)                      # neither side guessed
    alix_sync.pull(alix, lite, take_alix=True)
    assert (lite / alix_sync.VENDOR / cfg).read_text() == "a: 1\n" and not alix_sync.file_states(alix, lite)[0]
    (alix / PLANS / "lid_close_toppra.g1.txt").unlink()                                         # removed in ALIX
    assert [(f.path, f.state) for f in alix_sync.file_states(alix, lite)[0]] == [(f"{PLANS}/lid_close_toppra.g1.txt", "gone_alix")]
    alix_sync.pull(alix, lite)
    assert not (lite / alix_sync.VENDOR / PLANS / "lid_close_toppra.g1.txt").exists() and not alix_sync.file_states(alix, lite)[0]


def test_a_plan_its_config_has_moved_on_from_is_found():
    """28 Oct: a config edited and its re-planning failed leaves the old plan in place. A stage added, a pose moved, a
    stroke's point moved: each shows; the plan as made passes."""
    root = vendor_paths.LITE_ROOT / alix_sync.VENDOR
    cfg = yaml.safe_load((root / CONFIGS / "seat_close.yaml").read_text())
    plan = json.loads((root / PLANS / "seat_close_toppra.json").read_text())
    assert alix_sync.plan_against_config(plan, cfg, root, "seat_close") == []
    more = copy.deepcopy(cfg)
    more["stage_sequence"].insert(3, {"stage": "hand_grasp", "arm": "left", "pose_name": "grasp", "planning_group": "inspire_hand_left"})
    assert "the plan is not its config's" in alix_sync.plan_against_config(plan, more, root, "seat_close")[0][2]
    moved = copy.deepcopy(cfg)
    moved["left"]["cartesian_poses"]["start_pose"][0] += 0.01
    texts = [r[2] for r in alix_sync.plan_against_config(plan, moved, root, "seat_close")]
    assert any("move_to_start (left) ends 10.0 mm" in t for t in texts), texts
    point = copy.deepcopy(cfg)
    point["left"]["cleaning_points"]["point5"]["translation"][2] -= 0.02
    assert any("passes its config's point5" in r[2] for r in alix_sync.plan_against_config(plan, point, root, "seat_close"))
    seq = json.loads((root / PLANS / "seat_close_seq_toppra.json").read_text())     # its own flag (false): its check gone
    assert alix_sync.plan_against_config(seq, cfg, root, "seat_close_seq") == []
    assert alix_sync.seq_against_task(seq, plan, cfg, "seat_close_seq") == []
    other = copy.deepcopy(seq)
    arm = next(t for t in other["sub_trajectories"] if t["stage_name"] == "execute_path" and t["joint_names"][0].startswith("L_"))
    arm["waypoints"][-1]["positions"][0] += 0.05                                  # 2.9 degrees of joint 1: the hand moves
    assert alix_sync.seq_against_task(other, plan, cfg, "seat_close_seq")[0][0] == "error"


def test_lites_registry_is_checked_against_itself(monkeypatch):
    monkeypatch.setitem(mechanism.ROWS, "L-T99", "flush_press")
    monkeypatch.delitem(mechanism.PHASE_AFTER, "lid_close")
    texts = [r[2] for r in alix_sync.registry_checks() if r[0] == "error"]
    assert any("L-T99 names flush_press" in t for t in texts) and any("lid_close has no commode state" in t for t in texts)
