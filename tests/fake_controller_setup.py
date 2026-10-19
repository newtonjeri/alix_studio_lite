"""A fake controller for an arm's worker process (robot_process: LITE_WORKER_SETUP names this file), for
tests/test_link_process.py: enough of the SDK for a reading, and one call that never answers on the connection
made while the marker file LITE_FAKE_STUCK_MARKER exists (the first; it is removed as that connection opens, so the
next one answers). Anything else answers -1."""
import os
import threading
from pathlib import Path


class Controller:
    def __init__(self, stuck: bool = False):
        self.block = threading.Event()
        if not stuck:
            self.block.set()

    def rm_get_current_arm_state(self):
        return 0, {"joint": [0.0] * 7, "pose": [0.0] * 6, "err": ["0"]}

    def rm_get_arm_run_mode(self):
        return 0, 1

    def rm_get_lift_state(self):
        return 0, {"pos": 50}

    def rm_set_modbus_mode(self, *a):
        return 0

    def rm_read_multiple_holding_registers(self, *a):
        return -1, []

    def rm_get_io_state(self, n):
        return 0, {"io_state": 1, "io_config": {"io_mode": 5}}

    def rm_set_plan_speed(self, pct):                       # the call that never comes back
        self.block.wait()
        return 0

    def rm_set_arm_stop(self):
        return 0

    def rm_set_arm_slow_stop(self):
        return 0

    def rm_delete_robot_arm(self):
        return 0

    def __getattr__(self, name):
        if name.startswith("rm_"):
            return lambda *a, **kw: (-1, None)
        raise AttributeError(name)


def setup():
    from alix_dispatch import connect

    marker = os.environ.get("LITE_FAKE_STUCK_MARKER")

    def open_arm(side, ip=None, port=None, **kw):
        stuck = bool(marker) and Path(marker).exists()
        if stuck:
            Path(marker).unlink()
        return connect.ArmHandle(side, ip or "192.168.1.103", port or 8080, Controller(stuck=stuck), 1, "127.0.0.1")

    connect.open_arm = open_arm
    connect.close_arm = lambda h: None
