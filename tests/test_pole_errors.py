"""The pole's driver error said in the pole's words (pole_errors; the user, 7 Nov: "it would be more informative if the
error could be well communicated in the Lite Studio"): at the lab both poles stood at 314 mm after a controller hang,
past the limit set on the lifts, the controllers reported "lift driver 512" and seven runs were refused with "clear it on
the pendant or in Robot & mode", which clears the arm's errors and not the pole's; the vendor's app's lift panel did."""
import contextlib
from types import SimpleNamespace

from alix_studio_lite.server import pole_errors, robot_real


class Robot:
    """A controller whose arm is clean and whose pole's driver reports ``err`` with the pole at ``pos`` mm."""

    def __init__(self, err=0, pos=314):
        self.err, self.pos = err, pos

    def rm_get_current_arm_state(self):
        return 0, {"joint": [0.0] * 7, "pose": [0.0] * 6, "err": {"err": ["0"]}}

    def rm_get_joint_err_flag(self):
        return {"return_code": 0, "err_flag": [0] * 7}

    def rm_get_joint_en_state(self):
        return 0, [1] * 7

    def rm_get_lift_state(self):
        return 0, {"pos": self.pos, "current": 0, "err_flag": self.err, "mode": 0}


def arm(robot) -> robot_real.RealArm:
    a = robot_real.RealArm(SimpleNamespace(side="left", handle=SimpleNamespace(robot=robot), emulated=False,
                                           busy=lambda *a, **k: contextlib.nullcontext()))
    a.said = []
    a._hw = lambda verb, **kw: a.said.append((verb, kw.get("text")))
    return a


def test_the_position_limit_is_named_with_the_poles_height_and_what_clears_it():
    w = pole_errors.words(512, 314.0)
    assert w["name"] == "512 (Position Limit Error)" and w["limit"]
    assert w["text"] == "the pole's driver reports 512 (Position Limit Error) with the pole at 314 mm (past the limit set on its controller)"
    assert w["advice"] == ("lower the pole below its limit and clear the pole's error on the vendor's app (Extension, Lift Control); "
                           "Lite's Clear errors clears the arm's errors, not the pole's")


def test_another_code_is_named_from_the_vendors_table_without_the_limit_words():
    w = pole_errors.words(4)                                            # the lift's driver under voltage
    assert w["name"] == "4 (Undervoltage)" and not w["limit"] and w["text"] == "the pole's driver reports 4 (Undervoltage)"
    assert w["advice"].startswith("clear the pole's error on the vendor's app")
    assert pole_errors.reword("joints J6=512 (Position Limit Error),lift driver 512", 512, 300.0).endswith(
        "the pole's driver reports 512 (Position Limit Error) with the pole at 300 mm (past the limit set on its controller)")
    assert pole_errors.reword("clean", None) == "clean"


def test_read_errors_says_the_pole_and_its_advice():
    """Read errors / Clear errors (Robot & mode, the lock): the text the toast shows and the advice under it."""
    a = arm(Robot(err=512, pos=314))
    out = a.errors()
    assert out["readable"] and not out["clean"] and out["lift"] == 512
    assert out["text"] == "the pole's driver reports 512 (Position Limit Error) with the pole at 314 mm (past the limit set on its controller)"
    assert out["pole"]["name"] == "512 (Position Limit Error)" and out["advice"].startswith("lower the pole below its limit")
    clean = arm(Robot()).errors()
    assert clean["clean"] and clean["text"] == "clean" and clean["pole"] is None and clean["advice"] is None


def test_the_check_before_a_move_refuses_with_the_poles_words_and_advice():
    """S2's clear step (workflow.clear): the run is refused before the row with what the pole reports and what to do;
    nothing is cleared (not a warning), nothing moves."""
    a = arm(Robot(err=512, pos=314))
    got = a.clear_warning(frozenset({"4105"}))
    assert got["cleared"] == [] and got["readable"]
    assert got["other"] == "the pole's driver reports 512 (Position Limit Error) with the pole at 314 mm (past the limit set on its controller)"
    assert got["advice"].startswith("lower the pole below its limit and clear the pole's error on the vendor's app (Extension, Lift Control)")
    assert ("clear warning", "not cleared, not a warning: " + got["other"]) in a.said
    ok = arm(Robot()).clear_warning(frozenset({"4105"}))
    assert ok == dict(readable=True, cleared=[], other=None, advice=None)
