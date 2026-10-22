"""Lite on ALIX's emulated arms (``--robot emulated``), for tests/test_robot_real.py. ALIX's emulator
has no drag teaching, so the three SDK calls the control program uses for it are added here (and the
drag sensitivity's getter and setter, 100 % as the controller ships): the
recording is written by "the SDK" itself, as on the robot (one {"point":[...]} line of integer
millidegrees per 5 ms sample, CRLF), starting from where the arm stands. Nor has it global points:
three are stored here, made from where the arm stands at the first list (reachable by MoveJ and
MoveL), answered as the SDK answers (a page of the list, or one point by name), and saved as the SDK
saves them (rm_add_global_waypoint, rm_update_global_waypoint with its rm_waypoint_t), and deleted
(rm_delete_global_waypoint). Data Management's two other reads answer as the lab's 3rd-generation controllers do:
no IO default program, and -4 for the tracks list (a 4th-generation call), and for deleting a track. A program is
deleted from ALIX's emulated program store (rm_delete_program_trajectory: 1 for a slot that holds none).

The lab's 4105 (22 Oct), when the file LITE_EMU_4105 names holds an arm's side ("left", "right" or "both"), and a code
after it ("left 4107": the left's brake release failing as its Program 86 started, the lab, 22 Oct): that arm's
Programs latch 4105 (a joint over its speed limit; or that code) as they start, as the lab's controllers did, and its
blocking MoveJ does what RealMan's SDK does while any code is latched (libapi_c's rm_movej_block reads the arm's state
once the controller has taken the move and returns 1 on any code): the arm goes to the point and the call returns 1."""
import json
import os
import sys
import time
from pathlib import Path

POINTS = 600


def start(self, record):
    self._lite_drag = time.time()
    return 0


def stop(self):
    return 0


def save(self, path):
    ret, st = self.rm_get_current_arm_state()
    q = [int(round(v * 1000)) for v in st["joint"]]
    lines = [json.dumps({"point": [q[0] + 20 * i] + q[1:]}, separators=(",", ":")) for i in range(POINTS)]
    Path(path).write_bytes(("\r\n".join(lines) + "\r\n").encode())
    return 0, POINTS


def sensitivity(self):
    return 0, getattr(self, "_lite_sens", 100)


def set_sensitivity(self, grade):
    if not 0 <= int(grade) <= 100:
        return 1
    self._lite_sens = int(grade)
    return 0


def _points(self):
    if not hasattr(self, "_lite_points"):
        ret, st = self.rm_get_current_arm_state()
        q, pose = list(st["joint"]), list(st["pose"])
        moved = lambda j, d: [v + (d if i == j else 0) for i, v in enumerate(q)]      # noqa: E731
        self._lite_points = [dict(point_name=n, joint=jt, pose=pose, work_frame="", tool_frame="", time="2026-10-14 10:00:00")
                             for n, jt in (("STP_HOME", q), ("STP_LID_OPEN", moved(0, 10.0)), ("CLEAN_A", moved(1, -5.0)))]
    return self._lite_points


def waypoints(self, page, size, search):
    pts = [p for p in _points(self) if str(search or "") in p["point_name"]]
    return 0, {"total_size": len(pts), "list": pts[(page - 1) * size: page * size]}


def waypoint(self, name):
    p = next((p for p in _points(self) if p["point_name"] == name), None)
    return (0, dict(p)) if p else (-1, {})


class Waypoint:
    """RealMan's rm_waypoint_t as the SDK builds it: name, joints (deg), pose, work and tool frame."""

    def __init__(self, point_name=None, joint=None, pose=None, work_frame=None, tool_frame=None, time=""):
        self.d = dict(point_name=point_name, joint=list(joint or []), pose=list(pose or []), work_frame=work_frame or "",
                      tool_frame=tool_frame or "", time=time)


def add_point(self, wp):
    pts = _points(self)
    if any(p["point_name"] == wp.d["point_name"] for p in pts):
        return 1                                # the controller refuses a name it has
    pts.append(dict(wp.d, time=time.strftime("%Y-%m-%d %H:%M:%S")))
    return 0


def update_point(self, wp):
    pts = _points(self)
    i = next((i for i, p in enumerate(pts) if p["point_name"] == wp.d["point_name"]), None)
    if i is None:
        return 1
    pts[i] = dict(wp.d, time=time.strftime("%Y-%m-%d %H:%M:%S"))
    return 0


def delete_point(self, name):
    pts = _points(self)
    i = next((i for i, p in enumerate(pts) if p["point_name"] == name), None)
    if i is None:
        return 1                                # the controller refuses a name it does not have
    del pts[i]
    return 0


def delete_program(self, tra_id):
    store = self._store                         # ALIX's emulated program store (alix_dispatch.emulated_program)
    with store.lock:
        if int(tra_id) not in store.slots:
            return 1
        del store.slots[int(tra_id)]
    return 0


def delete_track(self, name):
    return -4                                   # RealMan's track calls: 4th-generation controllers only


def default_program(self):
    return 0, 0                             # the program the controller's IO starts: none


def tracks(self, page, size, search):
    return -4, {}                           # RealMan's tracks list: 4th-generation controllers only


def _code_on(ctrl) -> int | None:
    """The code this arm's Programs latch (LITE_EMU_4105's file: a side, then a code, 4105 if none), or None."""
    flag = os.environ.get("LITE_EMU_4105")
    try:
        want = (Path(flag).read_text().split() if flag else []) + ["4105"]
    except OSError:
        return None
    from alix_emulator import rm_emulator

    side = "left" if getattr(ctrl, "ip", None) == rm_emulator.EMU_LEFT_IP else "right"
    return int(want[1]) if want[0] in (side, "both") else None


def _program_run(run):
    def rm_set_program_id_run(self, tra_id, speed, timeout):
        ret = run(self, tra_id, speed, timeout)
        ctrl = self._ctrl()
        code = _code_on(ctrl) if ret == 0 and ctrl is not None else None
        if code:
            with ctrl._lock:
                ctrl.sys_err_code = code
        return ret

    return rm_set_program_id_run


def _movej(movej):
    def rm_movej(self, joint, v, r, connect, block):
        if block and self._ctrl.sys_err_code not in (0, None):
            ret = movej(self, joint, v, r, connect, 0)
            return 1 if ret == 0 else ret
        return movej(self, joint, v, r, connect, block)

    return rm_movej


def setup():
    """The shims on ALIX's emulator: run here before the server starts, and in each arm's worker process before it
    opens its link (robot_process: LITE_WORKER_SETUP names this file)."""
    from alix_emulator import rm_emulator

    if getattr(rm_emulator, "_lite_shimmed", False):
        return
    rm_emulator._lite_shimmed = True
    rm_emulator.RoboticArm.rm_start_drag_teach = start
    rm_emulator.RoboticArm.rm_stop_drag_teach = stop
    rm_emulator.RoboticArm.rm_save_trajectory = save
    rm_emulator.RoboticArm.rm_get_drag_teach_sensitivity = sensitivity
    rm_emulator.RoboticArm.rm_set_drag_teach_sensitivity = set_sensitivity
    rm_emulator.RoboticArm.rm_get_global_waypoints_list = waypoints
    rm_emulator.RoboticArm.rm_get_given_global_waypoint = waypoint
    rm_emulator.RoboticArm.rm_add_global_waypoint = add_point
    rm_emulator.RoboticArm.rm_update_global_waypoint = update_point
    rm_emulator.RoboticArm.rm_get_default_run_program = default_program
    rm_emulator.RoboticArm.rm_get_trajectory_file_list = tracks
    rm_emulator.RoboticArm.rm_delete_global_waypoint = delete_point
    rm_emulator.RoboticArm.rm_delete_trajectory_file = delete_track
    from alix_dispatch import emulated_program

    emulated_program.ProgramSurface.rm_delete_program_trajectory = delete_program
    emulated_program.ProgramSurface.rm_set_program_id_run = _program_run(emulated_program.ProgramSurface.rm_set_program_id_run)
    rm_emulator.RoboticArm.rm_movej = _movej(rm_emulator.RoboticArm.rm_movej)
    emulator_install = rm_emulator.install

    def install():                          # the SDK's waypoint structure beside the emulated arm
        ri, cw = emulator_install()
        ri.rm_waypoint_t = cw.rm_waypoint_t = Waypoint
        return ri, cw

    rm_emulator.install = install


if __name__ == "__main__":                  # Lite's tuning workers import the main module again
    import os

    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
    from alix_studio_lite import vendor_paths

    vendor_paths.activate()
    setup()
    os.environ["LITE_WORKER_SETUP"] = str(Path(__file__).resolve())      # the arm workers shim their emulator the same way
    from alix_studio_lite.server import main

    sys.exit(main(sys.argv[1:]))
