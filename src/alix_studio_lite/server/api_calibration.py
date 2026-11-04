"""The wrist camera's calibration (hand_eye_cal): plan the views, check them offline, run them in one go, solve.

- ``GET  /api/calibration``: the session (its folder, the planned views, those taken, the coverage, the result).
- ``POST /api/calibration/plan`` ``{pid, squares, square_mm}``: a new session, its views planned (the worker process: FCL) from
  where the arms stand now, against the project's measured room (its walls), the commode with the board on its closed
  lid, the left arm. Nothing moves.
- ``POST /api/calibration/preview``: the tour as the 3D shows it (Visual sim's ``simulate-move`` timeline), nothing sent.
- ``POST /api/calibration/run`` ``{confirmed}``: every view not taken yet, back to rest, the solve, as one job (the
  user, 4 Nov: no confirmation per move; Real asks once, as every real run does). Hardware sim runs the MoveJs in the
  controllers' SIM mode.
- ``POST /api/calibration/rest`` ``{confirmed}``: the right arm back at rest (after a STOP).
- ``POST /api/calibration/solve``: ALIX's solve of the views taken again; nothing is applied.
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
                board=dict(squares=[hc.BOARD_COLS + 1, hc.BOARD_ROWS + 1], square_mm=hc.BOARD_SQUARE_M * 1000,
                           print="Resources/Calibration/board_10x7_30mm.svg"),
                pole_mm=hc.POLE_MM, speed_pct=hc.CAL_SPEED_PCT, poses=list(hc.BASES), mode=app.ui["mode"])


@route("POST", r"/api/calibration/plan")
def post_calibration_plan(app, q, body, m):
    """``{pid, squares: [along, across], square_mm}``: the board as laid on the lid (any turn), its square measured."""
    from . import placement, workers
    from .routes_robot import _payload

    session(app)
    sq = body.get("squares") or [hc.BOARD_COLS + 1, hc.BOARD_ROWS + 1]
    try:
        board = hc.board_spec(int(sq[0]), int(sq[1]), float(body.get("square_mm") or hc.BOARD_SQUARE_M * 1000))
    except (TypeError, ValueError, IndexError):
        raise UserError("The board's squares (along, across) and its measured square (mm) must be numbers.") from None
    pid = body.get("pid")
    if not pid:
        raise UserError("Open the project first: the room's walls are its.", "Open the project, then plan again.")
    units = units_of(app, pid)
    room = dict(home=dict(_payload(app)["home"]), scene=placement.scene(units.fixture, placement.load(units)["room"], None),
                fid=units.fixture, cubicle_mm=1400)
    state = hc.readings_now(app.robot) if app.ui["mode"] != "visual" else {}
    state.setdefault("right", {})["pole_mm"] = hc.POLE_MM
    planned = workers.pool().submit(hc.plan, state, room, board).result(timeout=300)
    s = hc.Session(hc.folder_for(app.robot), planned, room)
    hc.SESSIONS[id(app.robot)] = s
    return dict(session=s.view(), plan=dict(diversity=planned["diversity"], candidates=planned["candidates"],
                                            refused=planned["refused"], seconds=planned["seconds"],
                                            tolerance_mm=round(planned["slack_m"] * 1000)))


@route("POST", r"/api/calibration/preview")
def post_calibration_preview(app, q, body, m):
    """The tour in 3D: from where the right arm stands (a linked arm) or its standing pose, the pole to the run's
    height, the MoveJ to the first view, then every view held and the moves between at the run's speed, to rest."""
    from . import staging, workflow
    from ..motion.limits import load_limits
    from .routes_robot import _mesh_links, _payload

    session(app)
    s = _session(app)
    p = _payload(app)
    home = dict(p["home"])
    start, how = workflow._start_state(app, "right", home)
    rating = load_limits("right").rating_deg_s
    stage = staging.make_stage(hc.POLE_MM, start["hand"], "the calibration's pole")
    out = staging.execution_timeline("right", hc.tour_md(s, rating), stage, home=staging.start_values("right", home, start),
                                     links=staging.staged_links("right", _mesh_links(p)), rating_deg_s=rating,
                                     max_frames=5000, move_pct=hc.CAL_SPEED_PCT)
    out.update(what=f"the camera calibration's {len(s.plan['views'])} views at {hc.CAL_SPEED_PCT} %", start_from=how)
    return out


@route("POST", r"/api/calibration/run")
def post_calibration_run(app, q, body, m):
    session(app)
    s = _session(app)
    mode = require_robot_mode(app)
    left = len(s.plan["views"]) - s.next_index
    require_confirmed(app, body, f"The camera calibration ({left} view(s), then back to rest, at {hc.CAL_SPEED_PCT} %)")
    return hc.run_session(app.robot, s, mode).view()


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
