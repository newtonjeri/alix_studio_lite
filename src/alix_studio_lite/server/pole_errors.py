"""The pole's driver error, worded for the operator.

The lift reports its driver's error in the joints' code table (RealMan's SDK on ``rm_get_lift_state``: "err_flag: the
driver's error code; see the joint error codes"), so the words are the joints' (``error_codes``, RealMan's own tables,
vendored with the operator's tools). The lab, 7 Nov 2026: both poles stood at 314 mm when Lite reconnected after a
controller hang, past the limit set on the lifts, and each controller reported "lift driver 512" (0x0200, Position Limit
Error). Lite's Clear errors clears the arm's system and joint codes, which is what the vendor's app's Clear Error does
for the arm, and not the pole's: the run was refused seven times before the poles were lowered and their error cleared
on the vendor's app. The user: the error must be well communicated in Lite, which this does, everywhere the arm's errors
are said (the Connection page's check, the live rows, Read / Clear errors, the run's check before a move, the errors
read after a stop)."""
from __future__ import annotations

#: RealMan's joint error bit the lift raises past its limit (0x0200, "Position Limit Error").
POSITION_LIMIT = 0x0200

#: Where the vendor's app shows and clears the lift: its Extension page, Lift Control.
LIFT_PANEL = "the vendor's app (Extension, Lift Control)"


def _describe(code: int) -> str:
    from .robot_real import _dac

    _dac()                                                   # puts the operator's tools (error_codes) on the path
    from error_codes import describe_joint_err

    return describe_joint_err(code)


def words(code, pole_mm=None) -> dict:
    """What the pole's driver reports and what to do about it: ``{code, name, limit, text, advice}``. ``name`` is the
    vendor's ("512 (Position Limit Error)"); ``limit`` whether the code is the position limit; ``text`` a sentence with
    the pole's height when given; ``advice`` what clears it (the vendor's app's lift panel, not Lite's Clear errors)."""
    code = int(code)
    name = _describe(code)
    limit = bool(code & POSITION_LIMIT)
    text = f"the pole's driver reports {name}"
    if pole_mm is not None:
        text += f" with the pole at {pole_mm:.0f} mm"
    if limit:
        text += " (past the limit set on its controller)"
    advice = ((f"lower the pole below its limit and clear the pole's error on {LIFT_PANEL}" if limit
               else f"clear the pole's error on {LIFT_PANEL}")
              + "; Lite's Clear errors clears the arm's errors, not the pole's")
    return dict(code=code, name=name, limit=limit, text=text, advice=advice)


def reword(text: str, code, pole_mm=None) -> str:
    """``dual_arm_common.describe_error_state``'s "lift driver N" said as the pole's driver's words."""
    if not code:
        return text
    return text.replace(f"lift driver {code}", words(code, pole_mm)["text"])
