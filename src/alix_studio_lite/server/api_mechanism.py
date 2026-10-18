"""The lid and the seat (mechanism.py): ALIX's pre-recorded tasks, listed and run in the mode Lite is in.

- ``GET /api/mechanism``: the four tasks (what each does, its checks, its moves, the 250 mm standoff its plan
  was made at), the rows of Lite's catalogue each one runs for, the wrist camera (plugged in or not) and the
  placement the camera has learned this session.
- ``POST /api/mechanism/<task>/run`` ``{confirmed}``: one task as a job on both arms; Real needs the real
  robot's confirmation, as every motion does.
- ``POST /api/mechanism/camera/check``: what the wrist camera sees now, as the perception node's status_check
  answers it (the colour model, the depth state); nothing moves.
"""
from __future__ import annotations

from . import mechanism
from .api_units import require_confirmed
from .api_v2 import session
from .app import route
from .jobs import UserError


@route("GET", r"/api/mechanism")
def get_mechanism(app, q, body, m):
    session(app)
    k = mechanism.kit(app.robot)
    return dict(tasks=mechanism.catalogue(), rows=mechanism.ROWS, standoff_mm=mechanism.STANDOFF_MM,
                camera=k.camera.available(), registration=k.gap.registration_view())


@route("POST", r"/api/mechanism/([a-z_]+)/run")
def post_mechanism_run(app, q, body, m):
    session(app)
    name = m.group(1)
    if name not in mechanism.TASKS:
        raise UserError(f"There is no lid or seat task {name!r}.", f"One of: {', '.join(mechanism.TASKS)}.")
    plan = mechanism.Plan(name)
    require_confirmed(app, body, f"{plan.title} (ALIX's plan; the wrist camera checks the lid and the seat)")
    return mechanism.run(app.robot, name, app.ui["mode"]).view()


@route("POST", r"/api/mechanism/camera/check")
def post_mechanism_camera(app, q, body, m):
    """What the wrist camera sees now (the node's status_check: the colour model, the depth state); nothing moves."""
    session(app)
    return mechanism.camera_check(app.robot)


@route("POST", r"/api/mechanism/placement/forget")
def post_mechanism_forget(app, q, body, m):
    """The UGV was moved: the lid's and the seat's placement are learned again from the next closed ones."""
    session(app)
    k = mechanism.kit(app.robot)
    k.gap.forget()
    return dict(registration=k.gap.registration_view())
