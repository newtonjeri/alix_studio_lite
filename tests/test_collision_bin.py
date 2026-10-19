"""The robot's own bin in the collision check (server/collision_check.py; the owner, 19 Oct: "include the bin body, the
bin handles and the bin covers in the collision checking matrices"): an arm that reaches into the lowered bin is a
collision with it, named with the bin's link, in a recording's check and in S2's two-arm check; ALIX's standing pose
itself is clear of it."""
import json

import numpy as np

from alix_studio_lite import vendor_paths

vendor_paths.activate()

from alix_studio_lite.server import collision_check as C  # noqa: E402
from alix_studio_lite.server import robot_model  # noqa: E402

#: the RIGHT arm reaching back and down into the bin (J1 -150, J2 -120, the elbow straight), inside its joint limits
IN_THE_BIN = [-150.0, -120.0, 0.0, 0.0, 0.0, 79.0, 0.0]


def _home():
    model = robot_model.fk_model()
    return model, robot_model._home(model, C._rcm().index)


def test_the_bin_is_the_urdf_s_whole_bin_lowered():
    assert C.BIN_LINKS == {"bin_rail_link", "bin_carriage_link", "bin_link", "bin_handle_left", "bin_handle_right",
                           "lid_opening_cover1", "lid_opening_cover2"}
    b = C._bin()
    assert b.link_count == 6                       # every bin link with a collision mesh (the carriage has none)


def test_an_arm_in_the_bin_is_a_collision_with_it(tmp_path):
    model, home = _home()
    q_home = [float(np.degrees(home[f"R_joint{i}"])) for i in range(1, 8)]
    rows = [q_home] * 40 + [IN_THE_BIN] * 10
    md = tmp_path / "bin.txt"
    md.write_text("".join(json.dumps({"point": [int(round(v * 1000)) for v in r]}) + "\n" for r in rows))
    scene = {"offset_m": [0.0, 0.0], "walls": None}
    r = C.check(str(md), "right", dict(pole_mm=150.0, hand=[1000] * 6), home, scene, "commode_c", "lid_closed", 1400)
    assert r["checked"]["bin"].startswith("the bin, lowered")
    on_bin = [e for e in r["events"] if "bin" in e["what"]]
    assert on_bin and on_bin[0]["first_line"] == 41 and on_bin[-1]["last_line"] == 50
    pairs = [p for e in on_bin for p in e["pairs"] if p["what"] == "bin"]
    assert pairs and all(p["object"] in C.BIN_LINKS and p["link"].startswith(("R_", "IRH_")) for p in pairs)
    assert not any(e["first_line"] <= 40 for e in on_bin)                               # standing: clear of the bin
    # S2's two-arm check names it too, with the arm
    times = [i * 0.02 for i in range(20)]
    arms = {"right": dict(pole=[150.0] * 20, hand=[[1000] * 6] * 20, q=[q_home] * 10 + [IN_THE_BIN] * 10)}
    two = C.check_states(times, arms, home, scene, "commode_c", ["lid_closed"] * 20, 1400)
    hit = [e for e in two["events"] if "bin" in e["what"]]
    assert hit and hit[0]["sides"] == ["right"] and abs(hit[0]["first_t"] - 0.2) < 1e-9
    assert any(p["what"] == "bin" and p["side"] == "right" and p["object"] in C.BIN_LINKS for p in hit[0]["pairs"])
    assert two["checked"]["bin"].startswith("the bin, lowered")
