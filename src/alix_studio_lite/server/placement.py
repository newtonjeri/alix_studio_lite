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
TOL_MM = 10.0     # the owner, 16 Oct: "a tolerance of + or -10mm in all measurements on the total": each room
                  # total against its parts, and two placements taken as the same (S1 merges)
STANDOFF_TOL_MM = 5.0   # the owner, 16 Oct (S1): "the UGV standoff use +-5mm, but for other measurements +-10mm"


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


def _mm(v: float) -> str:
    return f"{v:,.0f}"


def _mm1(v: float) -> str:
    """A difference: whole millimetres, or one decimal where that is what decides."""
    r = round(float(v), 1)
    return f"{r:,.0f}" if r == round(r) else f"{r:,.1f}"


def _apart(va: float, vb: float, tol: float) -> bool:
    return round(abs(float(va) - float(vb)), 1) > tol


def totals(room: dict, commode_width_mm: float, commode_depth_mm: float) -> list[dict]:
    """The room's totals against their parts, each within ±10 mm (the owner, 16 Oct: "right side from
    wall to commode is x, left side to commode side, and the known commode distance should be within
    +-10mm of the total measurement, also the other measurements, but warn of the exceeding values"):

    - width: the left distance + the commode's width (ALIX's model) + the right distance;
    - length: the commode's front distance + its depth (ALIX's model), the commode standing at the
      wall behind it.

    Each says its sum, the room's measure, the difference and, past the tolerance, a warning with the
    values. Nothing is refused: the 3D builds the walls from what was entered."""
    r = {k: float(room[k]) for k in ROOM_FIELDS}
    rows = [("width", "Room width", "the left distance + the commode's width + the right distance",
             [("left", r["left_mm"]), ("commode", commode_width_mm), ("right", r["right_mm"])], r["width_mm"]),
            ("length", "Room length", "the front distance + the commode's depth",
             [("front", r["front_mm"]), ("commode", commode_depth_mm)], r["length_mm"])]
    out = []
    for name, title, what, parts, whole in rows:
        total = sum(v for _, v in parts)
        diff = round(total - whole, 1)
        ok = abs(diff) <= TOL_MM                    # compared at 0.1 mm
        sums = " + ".join(_mm(v) for _, v in parts)
        warning = None
        if not ok:
            over = abs(diff) - TOL_MM
            warning = (f"{title}: {what} make {sums} = {_mm(total)} mm, but the room {name} is {_mm(whole)} mm: "
                       f"a difference of {_mm1(abs(diff))} mm, {_mm1(over)} mm past the ±{TOL_MM:.0f} mm tolerance.")
            if name == "length" and diff > 0:
                warning += " The wall behind would stand inside the commode."
            elif name == "length":
                warning += " Unless the commode stands that far off the wall behind it, check the length and the front distance."
            else:
                warning += " Check the three distances and the width."
        out.append(dict(name=name, title=title, parts=[dict(name=n, mm=round(v, 1)) for n, v in parts],
                        sum_mm=round(total, 1), room_mm=whole, diff_mm=diff, tol_mm=TOL_MM, ok=ok, warning=warning))
    return out


TASK_TEXT = {"standoff_mm": "UGV standoff", "front_mm": "UGV move front", "rear_mm": "UGV move rear",
             "left_mm": "UGV move left", "right_mm": "UGV move right"}
ROOM_SHORT = {"width_mm": "room width", "length_mm": "room length", "left_mm": "left wall to the commode",
              "right_mm": "the commode to the right wall", "front_mm": "the commode's front to the front wall"}


def differences(a: dict | None, b: dict | None) -> list[str]:
    """Where two placements (``for_task``: a task's UGV placement with the room it was measured in)
    are not the same, each measurement on its own: the standoff within ±5 mm, the moves and the room
    within ±10 mm (the owner, 16 Oct: S1 merges only files of the same pole, hand and "whole placement
    ... also the walls placement and the commode"; "the UGV standoff use +-5mm, but for other
    measurements +-10mm"), compared at 0.1 mm: [] when they are. A move left empty is 0; a
    placement or a room that was never recorded is never the same as one that was."""
    if not a or not b:
        return [] if not a and not b else ["one has no UGV placement recorded"]
    out = []
    for k, what in TASK_TEXT.items():
        va, vb = float(a.get(k) or 0.0), float(b.get(k) or 0.0)
        tol = STANDOFF_TOL_MM if k == "standoff_mm" else TOL_MM
        if _apart(va, vb, tol):
            out.append(f"{what} {_mm1(va)} mm vs {_mm1(vb)} mm (±{tol:.0f} mm)")
    ra, rb = a.get("room"), b.get("room")
    if bool(ra) != bool(rb):
        out.append("the room was measured for one only")
    elif ra and rb:
        for k, what in ROOM_SHORT.items():
            va, vb = float(ra.get(k) or 0.0), float(rb.get(k) or 0.0)
            if _apart(va, vb, TOL_MM):
                out.append(f"{what} {_mm1(va)} mm vs {_mm1(vb)} mm (±{TOL_MM:.0f} mm)")
    return out


def words(pl: dict | None) -> str:
    """A placement in a line: the standoff, its moves and the room."""
    if not pl:
        return "no UGV placement recorded"
    moves = [f"{TASK_TEXT[k].split()[-1]} {_mm(float(pl[k]))}" for k in ("front_mm", "rear_mm", "left_mm", "right_mm") if pl.get(k)]
    room = pl.get("room")
    return (f"standoff {_mm(float(pl['standoff_mm']))} mm" + (f" (moves: {', '.join(moves)} mm)" if moves else "")
            + (f"; room {_mm(float(room['width_mm']))} × {_mm(float(room['length_mm']))} mm" if room else "; room not measured"))


def scene(fid: str, room: dict | None, task: dict | None) -> dict:
    """The scene for a placement, in the robot base frame (metres): the commode's offset from ALIX's
    calibrated pose, the three walls (None without a measured room) and the front line, the room's
    totals against their parts (``totals``) and what the measurements say (``notes``)."""
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
    out["commode_depth_mm"] = round((fp["back_x"] - fp["front_x"]) * 1000.0, 1)
    if not room:
        return out
    out["totals"] = totals(room, out["commode_width_mm"], out["commode_depth_mm"])
    notes.extend(t["warning"] for t in out["totals"] if not t["ok"])
    m = lambda k: float(room[k]) / 1000.0                             # noqa: E731
    front_tip = fp["front_x"] + dx
    left_y, right_y = fp["left_y"] + dy + m("left_mm"), fp["right_y"] + dy - m("right_mm")
    front_x = front_tip - m("front_mm")
    back_x = front_x + m("length_mm")
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
