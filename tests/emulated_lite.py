"""Lite on ALIX's emulated arms (``--robot emulated``), for tests/test_robot_real.py. ALIX's emulator
has no drag teaching, so the three SDK calls the control program uses for it are added here: the
recording is written by "the SDK" itself, as on the robot (one {"point":[...]} line of integer
millidegrees per 5 ms sample, CRLF), starting from where the arm stands. Nor has it global points:
three are stored here, made from where the arm stands at the first list (reachable by MoveJ and
MoveL), answered as the SDK answers (a page of the list, or one point by name)."""
import json
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


if __name__ == "__main__":                  # Lite's tuning workers import the main module again
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
    from alix_studio_lite import vendor_paths

    vendor_paths.activate()
    from alix_emulator import rm_emulator

    rm_emulator.RoboticArm.rm_start_drag_teach = start
    rm_emulator.RoboticArm.rm_stop_drag_teach = stop
    rm_emulator.RoboticArm.rm_save_trajectory = save
    rm_emulator.RoboticArm.rm_get_global_waypoints_list = waypoints
    rm_emulator.RoboticArm.rm_get_given_global_waypoint = waypoint
    from alix_studio_lite.server import main

    sys.exit(main(sys.argv[1:]))
