"""Where the room's walls, the commode and the UGV are (V32 feedback v2, slide 1; the owner, 15 Oct).

Measured by the operator, in millimetres, and kept in the project (``setup/placement.json``):

- the ROOM, once per project: its width (left wall to right wall) and length (the wall behind the
  commode to the front wall), and where the commode stands in it: from the left wall to its left
  edge, from its right edge to the right wall, and from its front tip to the front wall ("left" and
  "right" as seen facing the commode, the robot's left (+Y) and right (-Y));
- the UGV, once per TASK (both arms ride it): its STANDOFF, from the front face of its base to the
  commode's front tip, required for every task (no default), and the moves from there: Front / Rear
  along the robot's X (forward is towards the commode), Left / Right along its Y. No rotation.

The robot base frame (base_link) is ALIX's: X forward, Y to the robot's left, Z up. The commode is
where ALIX puts it (its calibrated pose, ``fixtures.footprint``), moved along X by the standoff's
difference from the one that pose was calibrated at, as ALIX's ``scene_pose.standoff_delta`` does, so
Lite's scene and the robot agree; the UGV's moves move the room and the commode the other way. The
walls are placed from the commode, 100 mm thick and 2 m high like ALIX's cubicle meshes, open at the
front as they are; the front wall is a line on the floor.
"""
from __future__ import annotations

import json
import time
from pathlib import Path

from .jobs import UserError

ROOM_FIELDS = ("width_mm", "length_mm", "left_mm", "right_mm", "front_mm")
TASK_FIELDS = ("standoff_mm", "front_mm", "rear_mm", "left_mm", "right_mm")
ROOM_TEXT = {"width_mm": "the room width", "length_mm": "the room length", "left_mm": "the left wall to the commode",
             "right_mm": "the commode to the right wall", "front_mm": "the commode's front to the front wall"}
WALL_THICK_M, WALL_HEIGHT_M = 0.10, 2.0          # ALIX's cubicle meshes (assets/scene/cubicles)
WIDTH_TOL_MM = 10.0                              # left + commode + right against the measured width


def _file(units) -> Path:
    return Path(units.root) / "setup" / "placement.json"


def load(units) -> dict:
    try:
        d = json.loads(_file(units).read_text(encoding="utf-8"))
    except (OSError, ValueError):
        d = {}
    return {"room": d.get("room"), "tasks": d.get("tasks") or {}}


def _save(units, d: dict) -> None:
    from .project import _atomic_write

    _file(units).parent.mkdir(parents=True, exist_ok=True)
    _atomic_write(_file(units), json.dumps(d, indent=1))


def standoff_range_mm() -> tuple[float, float]:
    """ALIX's fleet bounds for a UGV standoff (alix_taskgen.constants), in mm."""
    from alix_taskgen.constants import UGV_STANDOFF_MAX_M, UGV_STANDOFF_MIN_M

    return UGV_STANDOFF_MIN_M * 1000.0, UGV_STANDOFF_MAX_M * 1000.0


def _number(v, what: str, lo: float = 0.0, hi: float = 20000.0, required: bool = True):
    if v is None or (isinstance(v, str) and not v.strip()):
        if required:
            raise UserError(f"Enter {what} (mm).")
        return None
    try:
        n = float(v)
    except (TypeError, ValueError):
        raise UserError(f"{what[0].upper()}{what[1:]} must be a number of millimetres.") from None
    if not lo <= n <= hi:
        raise UserError(f"{what[0].upper()}{what[1:]} is {lo:g} to {hi:g} mm.")
    return round(n, 1)


def save_room(units, body: dict) -> dict:
    room = {k: _number(body.get(k), ROOM_TEXT[k], 1.0) for k in ROOM_FIELDS}
    room["at"] = time.strftime("%Y-%m-%d %H:%M:%S")
    with units.lock:
        d = load(units)
        d["room"] = room
        _save(units, d)
    return room


def save_task(units, label: str, body: dict) -> dict:
    label = str(label or "").strip()
    if not label:
        raise UserError("Say which task the UGV placement is for.")
    lo, hi = standoff_range_mm()
    task = {"standoff_mm": _number(body.get("standoff_mm"), "the UGV standoff", lo, hi)}
    for k, what in (("front_mm", "the move to the front"), ("rear_mm", "the move to the rear"),
                    ("left_mm", "the move to the left"), ("right_mm", "the move to the right")):
        task[k] = _number(body.get(k), what, 0.0, 2000.0, required=False)
    task["at"] = time.strftime("%Y-%m-%d %H:%M:%S")
    with units.lock:
        d = load(units)
        d["tasks"][label] = task
        _save(units, d)
    return task


def for_task(units, label: str | None) -> dict | None:
    """A task's UGV placement with the room it was measured in (as kept with a recording)."""
    d = load(units)
    task = d["tasks"].get(str(label or "")) if label else None
    if not task:
        return None
    return {"task": label, **{k: task.get(k) for k in TASK_FIELDS}, "room": d["room"]}


def scene(fid: str, room: dict | None, task: dict | None) -> dict:
    """The scene for a placement, in the robot base frame (metres): the commode's offset from ALIX's
    calibrated pose, the three walls (None without a measured room) and the front line, and what the
    measurements say against each other (``notes``)."""
    from . import fixtures

    fp = fixtures.footprint(fid)
    notes = []
    dx = dy = 0.0
    if task and task.get("standoff_mm") is not None:
        mv = lambda k: float(task.get(k) or 0.0) / 1000.0           # noqa: E731
        dx = (float(task["standoff_mm"]) / 1000.0 - fp["standoff_m"]) - (mv("front_mm") - mv("rear_mm"))
        dy = -(mv("left_mm") - mv("right_mm"))
    else:
        notes.append(f"No UGV standoff entered for this task: the commode is shown at ALIX's calibrated "
                     f"{fp['standoff_m'] * 1000:.0f} mm.")
    out = {"offset_m": [round(dx, 4), round(dy, 4)], "walls": None, "front_x": None, "notes": notes,
           "commode_width_mm": round((fp["left_y"] - fp["right_y"]) * 1000.0, 1),
           "standoff_mm": task.get("standoff_mm") if task else None}
    if not room:
        return out
    m = lambda k: float(room[k]) / 1000.0                             # noqa: E731
    front_tip, back = fp["front_x"] + dx, fp["back_x"] + dx
    left_y, right_y = fp["left_y"] + dy + m("left_mm"), fp["right_y"] + dy - m("right_mm")
    front_x = front_tip - m("front_mm")
    back_x = front_x + m("length_mm")
    spanned = (left_y - right_y) * 1000.0
    if abs(spanned - float(room["width_mm"])) > WIDTH_TOL_MM:
        notes.append(f"The left and right distances with the commode's {out['commode_width_mm']:.0f} mm make "
                     f"{spanned:.0f} mm, but the room width is {float(room['width_mm']):.0f} mm: check them.")
    if back_x < back - 0.005:
        notes.append(f"With this length and front distance the back wall is {(back - back_x) * 1000:.0f} mm inside "
                     "the commode: check the length and the commode's front distance.")
    t, h = WALL_THICK_M, WALL_HEIGHT_M
    depth = back_x - front_x
    out["walls"] = [
        {"name": "back", "center": [back_x + t / 2, (left_y + right_y) / 2, h / 2], "size": [t, left_y - right_y + 2 * t, h]},
        {"name": "left", "center": [front_x + depth / 2, left_y + t / 2, h / 2], "size": [depth, t, h]},
        {"name": "right", "center": [front_x + depth / 2, right_y - t / 2, h / 2], "size": [depth, t, h]},
    ]
    out["walls"] = [{**w, "center": [round(v, 4) for v in w["center"]], "size": [round(v, 4) for v in w["size"]]}
                    for w in out["walls"]]
    out["front_x"] = round(front_x, 4)
    out["room_y"] = [round(right_y, 4), round(left_y, 4)]
    return out
