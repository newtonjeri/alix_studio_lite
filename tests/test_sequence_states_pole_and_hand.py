"""A sequence file states the pole and hand of every Program it runs (the user, 16 Nov: "any uploaded program in the
uploaded sequence JSON files should contain that information ... We cannot have a file that does not define the pole and
hand position of a task, because that will result to collision").

A controller stores a program's points only, and its name defines nothing: the lab's RIGHT Program 61 is named pole200
and runs at 210 mm. So S2's SAVE writes each Program's own pole and hand into its row (``pole_mm``, ``hand``), as Lite
knows them, with the program named as its controller lists it; a row keeps them through LOAD. ALIX Studio Demo, which has
neither Lite's records nor its files, runs by what the file states and refuses a file that states none."""
import json
from types import SimpleNamespace

import pytest

from alix_studio_lite import vendor_paths

GRIP, CLEANING = [0, 0, 0, 0, 150, 1000], [1000, 1000, 1000, 1000, 0, 1000]


def _prog(i, **more):
    return {"type": "program", "id": i, "name": f"{i}_{i}_R_P_pole200_grip", "speed": 100, "transit_speed": 30, **more}


def test_a_program_row_keeps_the_pole_and_hand_its_file_states():
    vendor_paths.activate()
    from alix_studio_lite.server import workflow

    rows = workflow.check_steps([{"type": "pole", "mm": 210}, _prog(61, pole_mm=210, hand=CLEANING), _prog(62, pole_mm=200.0, hand="GRIP"),
                                 _prog(63)], "right")
    assert (rows[1]["pole_mm"], rows[1]["hand"]) == (210.0, CLEANING)
    assert (rows[2]["pole_mm"], rows[2]["hand"]) == (200.0, GRIP)                  # a hand named as a preset is its six counts
    assert "pole_mm" not in rows[3] and "hand" not in rows[3]                      # none stated: none kept (as every file of before)
    assert "pole_mm" not in rows[0]                                                # only a Program row carries them
    again = workflow.check_steps(json.loads(json.dumps(rows)), "right")            # LOAD of what SAVE wrote: the same rows
    assert again == rows


@pytest.mark.parametrize("more, said", [
    (dict(pole_mm=200), "stated together"), (dict(hand=GRIP), "stated together"),
    (dict(pole_mm=400, hand=GRIP), "0 to 315"), (dict(pole_mm="high", hand=GRIP), "millimetres"),
    (dict(pole_mm=200, hand=[0, 0, 0]), "not ones the robot can take")])
def test_a_half_stated_or_impossible_pole_and_hand_is_refused_with_its_row(more, said):
    vendor_paths.activate()
    from alix_studio_lite.server import workflow
    from alix_studio_lite.server.jobs import UserError

    with pytest.raises(UserError) as exc:
        workflow.check_steps([_prog(61), _prog(62, **more)], "left")
    assert str(exc.value).startswith("LEFT row 2: ") and said in str(exc.value), str(exc.value)


def test_save_states_each_programs_own_pole_and_hand_and_names_it_as_its_controller_does(tmp_path, monkeypatch):
    """SAVE on the lab PC: Program 61's pole and hand from Lite's record of sending it (210 mm, whatever its name says),
    62's from the project's entry, 63 nothing is known of but the sequence's own Pole and Hand steps are right before it,
    64 nothing is known of but its row states them already. Without those steps before 63, nothing is saved: SAVE always
    writes the form ALIX Studio Demo requires (the user, 16 Nov)."""
    vendor_paths.activate()
    from alix_studio_lite.server import program_links as pl
    from alix_studio_lite.server import workflow

    units = SimpleNamespace(root=tmp_path / "project")
    held = {61: "61_61_R_P1_pole200_cleaningpose.", 62: "62_62_R_P2_pole200_grip_13.0s_n.", 63: "63_63_R_P2_pole200_cleaningpose.",
            64: "64_64_1_R_P2_pole50_cleaningpose"}
    known = {61: dict(sent=True, name="61_R_P1_pole200_cleaningpose", sha256="aa", stage={"pole_mm": 210.0, "hand": CLEANING},
                      held=dict(name=held[61], bytes=616050)),
             62: dict(sent=False, name=held[62], size=140045, stage=None, file=None),
             63: dict(sent=False, name=held[63], size=104017, stage=None, file=None),
             64: dict(sent=False, name=held[64], size=118251, stage=None, file=None)}
    pl.keep(units, "right", {62: dict(sent=True, name="x", sha256="bb", stage={"pole_mm": 200.0, "hand": GRIP},
                                      held=dict(name=held[62], bytes=140045))})
    monkeypatch.setattr(workflow, "programs_known", lambda app, u, side: known if side == "right" else {})
    monkeypatch.setattr(workflow, "files_by_name", lambda app, u, side: {})
    monkeypatch.setattr(workflow, "_md_reader", lambda root: (lambda _p: None))
    from alix_studio_lite.server.jobs import UserError

    bare = workflow.check_steps([{"type": "pole", "mm": 210}, {"type": "hand", "values": CLEANING},
                                 dict(_prog(61), name="a name the row kept from before"), _prog(62), _prog(63),
                                 _prog(64, pole_mm=50, hand=CLEANING), {"type": "wait", "seconds": 1}], "right")
    with pytest.raises(UserError) as e:                                            # 63: nothing known, nothing before it
        workflow.document_stated(None, units, "right", bare)
    assert str(e.value) == ("The RIGHT sequence was not saved: no pole and hand known for row 5 (Program 63). A saved sequence "
                            "states every Program's own pole and hand.") and "Set pole and hand" in e.value.hint
    steps = workflow.check_steps([{"type": "pole", "mm": 210}, {"type": "hand", "values": CLEANING},
                                  dict(_prog(61), name="a name the row kept from before"), _prog(62),
                                  {"type": "pole", "mm": 200}, {"type": "global", "name": "R_HME_POS", "move": "movej", "speed": 30},
                                  {"type": "hand", "values": CLEANING}, _prog(63),
                                  _prog(64, pole_mm=50, hand=CLEANING), {"type": "wait", "seconds": 1}], "right")
    doc = workflow.document_stated(None, units, "right", steps)
    assert doc["format"] == workflow.LITE_FORMAT and doc["side"] == "right" and len(doc["steps"]) == len(steps)
    by = {x["id"]: x for x in doc["steps"] if x["type"] == "program"}
    assert (by[61]["name"], by[61]["pole_mm"], by[61]["hand"]) == (held[61], 210.0, CLEANING)      # 210 mm, though it is named pole200
    assert (by[62]["name"], by[62]["pole_mm"], by[62]["hand"]) == (held[62], 200.0, GRIP)          # the project's entry
    assert (by[63]["name"], by[63]["pole_mm"], by[63]["hand"]) == (held[63], 200.0, CLEANING)      # the steps right before it
    assert (by[64]["pole_mm"], by[64]["hand"]) == (50.0, CLEANING)                                 # what its row states stays
    assert [x for x in doc["steps"] if x["type"] != "program"] == [x for x in steps if x["type"] != "program"]
    assert all("pole_mm" not in x for x in steps if x["type"] == "program" and x["id"] != 64)      # the kept rows are not changed by a SAVE
    assert workflow.check_steps(doc["steps"], "right")[2]["pole_mm"] == 210.0                      # and the file loads again, stated


def test_save_takes_only_the_pole_and_hand_steps_since_the_last_program_or_task():
    """A Program Lite knows nothing of takes the sequence's own Pole and Hand steps only from after the previous Program,
    lid or seat task or INSPECT (each of those leaves the pole and hand its own way); both are needed, all six counts."""
    vendor_paths.activate()
    from alix_studio_lite.server import workflow

    pole, hand, prog = {"type": "pole", "mm": 50}, {"type": "hand", "values": CLEANING}, _prog(61)
    rows = lambda *xs: workflow.check_steps(list(xs), "left")                       # noqa: E731
    st = workflow.sequence_stage(rows(pole, hand, {"type": "meet"}, {"type": "global", "name": "REST_POSE", "move": "movej",
                                                                      "speed": 30}, prog), 4)
    assert (st.pole_mm, list(st.hand)) == (50.0, CLEANING)
    for before in ([pole, hand, _prog(62)], [pole, hand, {"type": "mechanism", "task": "lid_open"}], [pole, hand, {"type": "inspect"}],
                   [pole], [hand], []):
        r = rows(*before, prog)
        assert workflow.sequence_stage(r, len(r) - 1) is None, before
