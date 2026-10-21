"""Advanced Robot Controls' arm moves (the owner, 17 Oct: "the pole and the hand can be moved, but other arm
movements don't run ... moving to a global point; running a saved program, meaning an uploaded .txt file (a
project in a certain slot) ... in all three run modes"). The page's RUN tab was the designer's template (it
said COMPLETE after a timer and sent nothing); here:

- a program stored on a controller, run by its number or its name, as the control program's RUN runs a slot
  (``robot.run_stored``: the program's own pole and hand first (the owner, 21 Oct), then the controller's move
  to its first point, then the program). Hardware sim: the controller's SIM mode; Real: the arm, confirmed.
  The Trajectory Viewer runs a program from its slot through this route too;
- a .txt from this computer sent to a slot (and run), read through ALIX's strict parser first, with the pole
  and hand the operator states for it recorded as its own;
- in Visual sim nothing is sent: the move to a global point, a stored program and a chosen file are shown
  in 3D (``simulate-move``), from where the arm stands (a linked arm) or its standing pose; a stored program
  Lite sent with its pole and hand stages them first, as its run does.
"""
from __future__ import annotations

import re
import tempfile
import time
from pathlib import Path
from types import SimpleNamespace

from .api_units import require_confirmed, require_robot_mode, units_of
from .api_v2 import session
from .app import route
from .jobs import UserError

R = r"/api/robots/(left|right)"


def _program_of(app, side: str, given) -> int:
    """A program by its number (1-100) or by its name as the controller lists it ("50_L-R01_v003.txt",
    with or without its number and .txt), from the controller's list (content.py)."""
    from . import content

    text = str(given or "").strip()
    if not text:
        raise UserError("Enter a Program ID (1 to 100) or a program's name.")
    if re.fullmatch(r"\d+", text):
        pid = int(text)
        if not 1 <= pid <= 100:
            raise UserError("Program numbers are 1 to 100.")
        return pid
    stem = lambda n: re.sub(r"\.txt$", "", str(n or "").strip().lower())         # noqa: E731
    want = stem(re.sub(r"^\d+\s*//\s*", "", text))                                   # "50 // 50_L-R01_v003" as the lists show it
    for p in content.content(app, side)["programs"]:
        name = stem(p.get("name"))
        if want in (name, re.sub(rf"^{p['id']}_", "", name)):
            return int(p["id"])
    raise UserError(f"The {side.upper()} controller holds no program named “{text}”.",
                    "Refresh the list, then choose one, or enter its Program ID.")


def _stated_stage(body):
    """The pole and hand the operator states for a program Lite did not send (``pole_mm``, ``hand``), or None."""
    from . import staging

    if body.get("pole_mm") in (None, "") or not body.get("hand"):
        return None
    return staging.make_stage(body.get("pole_mm"), body.get("hand"), "as the operator stated it")


@route("POST", R + r"/programs/run")
def post_run_stored(app, q, body, m):
    """Run Stored / Run Only (and the Trajectory Viewer's Run on the robot): ``{program}`` (a number or a name), staged
    with the program's own pole and hand (robot_standin.run_stored); ``pole_mm``, ``hand``: the ones the operator
    states for a program Lite did not send (refused with code "stage" without them)."""
    session(app)
    side = m.group(1)
    mode = require_robot_mode(app)
    pid = _program_of(app, side, body.get("program"))
    stated = _stated_stage(body)
    if app.robot.stored_stage(side, pid) is None and stated is None:     # asked first, before any confirmation
        return app.robot.run_stored(side, pid, "", mode=mode)             # raises: "State the pole and hand ..."
    require_confirmed(app, body, f"Running program {pid}")
    title = f"{side.upper()} program {pid}" + (" in Hardware sim" if mode == "hwsim" else " on the real arm")
    return app.robot.run_stored(side, pid, title, mode=mode, stage=stated).view()


def _track_of(name: str, content) -> tuple[str, "object"]:
    """A .txt from this computer, read through ALIX's strict upload parser: (its name, its rows)."""
    from ..motion import track

    name = Path(str(name or "trajectory.txt")).name
    if not name.lower().endswith(".txt"):
        raise UserError(f"{name} is not a .txt trajectory.")
    data = content.encode("utf-8") if isinstance(content, str) else bytes(content or b"")
    if not data:
        raise UserError(f"{name} is empty.")
    with tempfile.TemporaryDirectory(prefix="lite-adv-") as d:
        raw = Path(d) / "raw.txt"
        raw.write_bytes(data)
        try:
            md = track.read_md(raw)
        except (ValueError, KeyError) as exc:
            raise UserError(f"{name} is not a trajectory Lite can send: {exc}.",
                            "Choose a .txt the drag teaching or the control program wrote.") from None
    if len(md) < 2:
        raise UserError(f"{name} holds {len(md)} point(s): it is not a trajectory.")
    return name, md


@route("POST", R + r"/programs/upload")
def post_upload_file(app, q, body, m):
    """Upload Only / Upload + Run: ``{program_id, name, content, replace, pole_mm, hand}``; kept in Lite's data folder
    (``advanced/<arm>/``) as Lite writes tracks, then sent as program ``program_id``, with the pole and hand the operator
    states for it recorded as its own (a run from its slot stages them: run_stored)."""
    from ..motion import track

    session(app)
    side = m.group(1)
    require_robot_mode(app)
    try:
        pid = int(body.get("program_id"))
    except (TypeError, ValueError):
        raise UserError("Enter a Program ID from 1 to 100.") from None
    if not 1 <= pid <= 100:
        raise UserError("Program numbers are 1 to 100.")
    name, md = _track_of(body.get("name"), body.get("content"))
    path = Path(app.data_dir) / "advanced" / side / f"{time.strftime('%Y%m%d_%H%M%S')}_{name}"
    track.write_md(path, md)
    stated = _stated_stage(body)
    job = app.robot.upload(side, pid, path, Path(name).stem[:28], track.sha256_file(path), bool(body.get("replace")),
                           f"Upload {name} to {side.upper()} program {pid}", stage=stated.as_dict() if stated else None)
    if job is None:
        return {"unchanged": True, "program_id": pid}
    return job.view()


@route("POST", R + r"/simulate-move")
def post_simulate_move(app, q, body, m):
    """Visual sim's arm move as the 3D shows it, nothing sent: ``{point}`` a global point of the arm's
    controller (MoveJ; a MoveL is shown as the same joint move), ``{program, pid}`` a stored program's file
    (found in project ``pid`` by its bytes when Lite sent it, else by its name), ``{name, content}`` a .txt
    from this computer. From where the arm stands (a linked arm) or its standing pose; the pole and the
    hand stay as they are, but for a program Lite sent with its own pole and hand: those are staged first,
    as its run from the slot does (robot_standin.run_stored)."""
    import numpy as np

    from . import content, staging, workflow
    from ..motion import track
    from ..motion.limits import load_limits
    from .routes_robot import _mesh_links, _payload

    session(app)
    side = m.group(1)
    p = _payload(app)
    home = dict(p["home"])
    start, how = workflow._start_state(app, side, home)
    if body.get("start") is not None:                # a sequence's next point: where the previous one left the arm
        start, how = staging.check_start(body["start"]), "after the previous step"
    what, own = "", None
    if body.get("point"):
        name = str(body["point"]).strip()
        pts = content.content(app, side)["points"]
        hit = next((x for x in pts if x.get("name") == name), None)
        if hit is None:
            raise UserError(f"The {side.upper()} controller holds no global point “{name}”" +
                            ("." if app.robot.arm(side).connected else ": connect the arm so Lite can read its points."),
                            "Refresh the list, then choose one.")
        md = np.asarray([[int(round(float(v) * 1000)) for v in hit["joints"][:7]]] * 2, dtype=np.int64)
        what = f"{'MoveL' if str(body.get('move')).lower() == 'movel' else 'MoveJ'} to {name}"
    elif body.get("program") is not None and body.get("content") is None:
        pid = _program_of(app, side, body.get("program"))
        if not body.get("pid"):
            raise UserError("Open a project: the program's file is found among its files.")
        units = units_of(app, str(body["pid"]))
        known = workflow.programs_known(app, units, side)
        info = known.get(pid)
        if info is None:
            raise UserError(f"Program {pid} is empty on the {side.upper()} arm.")
        path = info.get("file")
        if not path:                                 # sent by the control program: its file found by its name
            found = workflow._ArmSim._found(SimpleNamespace(named=workflow.files_by_name(app, units, side)), pid, {}, info)
            path = (found or {}).get("path")
        if not path:
            raise UserError(f"Lite does not have the file of program {pid} (“{info.get('name')}”), so it cannot show it in 3D.",
                            "Load the file in this project (S1's Load from this computer), or run it in Hardware sim.")
        md = track.read_md(path)
        what = f"program {pid} ({info.get('name') or Path(path).name})"
        if info.get("sent") and not info.get("gone"):
            own = staging.stage_from(info.get("stage"))
        what += (f", its own pole and hand first: {staging.describe(own)}" if own is not None
                 else ", with the pole and hand as they are (Lite does not know its own)")
    elif body.get("content") is not None:
        name, md = _track_of(body.get("name"), body.get("content"))
        what = name
    else:
        raise UserError("Say what to show: a global point, a program or a file.")
    stage = own or staging.make_stage(start["pole_mm"], start["hand"], "where the arm stands")
    out = staging.execution_timeline(side, md, stage, home=staging.start_values(side, home, start),
                                     links=staging.staged_links(side, _mesh_links(p)),
                                     rating_deg_s=load_limits(side).rating_deg_s, max_frames=5000)
    out.update(what=what, start_from=how)
    return out
