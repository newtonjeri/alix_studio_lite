"""The wrist camera's calibration (hand_eye_cal): plan the views, take them one at a time, solve.

- ``GET  /api/calibration``: the session (its folder, the planned views, those taken, the coverage, the result).
- ``POST /api/calibration/plan`` ``{pid, square_mm}``: a new session, its views planned (the worker process: FCL) from
  where the arms stand now, against the project's measured room. Nothing moves.
- ``POST /api/calibration/next`` ``{confirmed}``: the next planned view: the move (the real robot's confirmation, as
  every motion), the capture, the row saved. Real only.
- ``POST /api/calibration/rest`` ``{confirmed}``: the right arm back at rest.
- ``POST /api/calibration/solve``: ALIX's solve of the views taken; nothing is applied.
"""
from __future__ import annotations

from . import hand_eye_cal as hc
from .api_units import require_confirmed, require_robot_mode, units_of
from .api_v2 import session
from .app import route
from .jobs import UserError


def _session(app):
    s = hc.SESSIONS.get(id(app.robot))
    if s is None:
        raise UserError("No calibration session is open.", "Press Plan the views first.")
    return s


@route("GET", r"/api/calibration")
def get_calibration(app, q, body, m):
    session(app)
    s = hc.SESSIONS.get(id(app.robot))
    return dict(session=s.view() if s else None,
                board=dict(squares=f"{hc.BOARD_COLS + 1} x {hc.BOARD_ROWS + 1}", square_mm=hc.BOARD_SQUARE_M * 1000,
                           print="Resources/Calibration/board_10x7_30mm.svg"),
                pole_mm=hc.POLE_MM, speed_pct=hc.CAL_SPEED_PCT, poses=list(hc.BASES))


@route("POST", r"/api/calibration/plan")
def post_calibration_plan(app, q, body, m):
    from . import placement, workers
    from .routes_robot import _payload

    session(app)
    try:
        square_mm = float(body.get("square_mm") or hc.BOARD_SQUARE_M * 1000)
    except (TypeError, ValueError):
        raise UserError("The measured square size must be a number of millimetres.") from None
    if not 20.0 <= square_mm <= 40.0:
        raise UserError(f"A square of {square_mm:g} mm is not the 30 mm board.", "Measure ten squares with calipers and divide by ten.")
    pid = body.get("pid")
    if not pid:
        raise UserError("Open the project first: the room's walls are its.", "Open the project, then plan again.")
    units = units_of(app, pid)
    room = dict(home=dict(_payload(app)["home"]), scene=placement.scene(units.fixture, placement.load(units)["room"], None),
                fid=units.fixture, cubicle_mm=1400)
    state = hc.readings_now(app.robot) if app.ui["mode"] != "visual" else {}
    state.setdefault("right", {})["pole_mm"] = hc.POLE_MM
    fut = workers.pool().submit(hc.plan, state, room)
    planned = fut.result(timeout=300)
    s = hc.Session(hc.folder_for(app.robot), planned, square_mm, room)
    hc.SESSIONS[id(app.robot)] = s
    return dict(session=s.view(), plan=dict(diversity=planned["diversity"], candidates=planned["candidates"],
                                            refused=planned["refused"], seconds=planned["seconds"]))


@route("POST", r"/api/calibration/next")
def post_calibration_next(app, q, body, m):
    session(app)
    s = _session(app)
    mode = require_robot_mode(app)
    k = s.next_index if body.get("index") is None else int(body["index"])
    if 0 <= k < len(s.plan["views"]):
        v = s.plan["views"][k]
        require_confirmed(app, body, f"Moving the right arm to calibration view {k + 1} of {len(s.plan['views'])} "
                                     f"({v['base']}, turned {v['offsets_deg']})")
    return hc.run_view(app.robot, s, mode, k).view()


@route("POST", r"/api/calibration/rest")
def post_calibration_rest(app, q, body, m):
    session(app)
    s = _session(app)
    mode = require_robot_mode(app)
    require_confirmed(app, body, "Moving the right arm back to rest")
    return hc.back_to_rest(app.robot, s, mode).view()


@route("POST", r"/api/calibration/solve")
def post_calibration_solve(app, q, body, m):
    session(app)
    s = _session(app)
    return dict(result=hc.solve(s), session=s.view())
