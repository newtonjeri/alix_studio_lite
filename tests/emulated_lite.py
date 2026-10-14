"""Lite on ALIX's emulated arms (``--robot emulated``), for tests/test_robot_real.py. ALIX's emulator
has no drag teaching, so the three SDK calls the control program uses for it are added here: the
recording is written by "the SDK" itself, as on the robot (one {"point":[...]} line of integer
millidegrees per 5 ms sample, CRLF), starting from where the arm stands."""
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


if __name__ == "__main__":                  # Lite's tuning workers import the main module again
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
    from alix_studio_lite import vendor_paths

    vendor_paths.activate()
    from alix_emulator import rm_emulator

    rm_emulator.RoboticArm.rm_start_drag_teach = start
    rm_emulator.RoboticArm.rm_stop_drag_teach = stop
    rm_emulator.RoboticArm.rm_save_trajectory = save
    from alix_studio_lite.server import main

    sys.exit(main(sys.argv[1:]))
