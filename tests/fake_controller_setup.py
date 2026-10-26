"""A fake controller for an arm's worker process (robot_process: LITE_WORKER_SETUP names this file), for
tests/test_link_process.py: enough of the SDK for a reading, and one call that never answers on the connections
of the process whose first connection was made while the marker file LITE_FAKE_STUCK_MARKER exists (it is removed
then, so the next process's answer). Anything else answers -1. With LITE_FAKE_RUNNER, runner v14 is ``FakeRunner``.

With LITE_FAKE_PUSH_FILE, the controller's realtime push setting (a JSON file, so that it is the same controller
for every process that connects to it) and the RealMan SDK's own listener as measured with its library (triple-thread
mode, tests/test_sdk_push_port.py): connecting, or reading the setting, while the push is enabled binds its port on
this computer (every address, SO_REUSEADDR), kept for as long as the process lives."""
import json
import os
import socket
import threading
from pathlib import Path

_HELD: dict = {}                                            # port -> the SDK listener's socket, for this process's life


def _push() -> dict | None:
    f = os.environ.get("LITE_FAKE_PUSH_FILE")
    return json.loads(Path(f).read_text()) if f else None


def _sdk_listens(cfg: dict) -> None:
    port = int(cfg.get("port") or 0)
    if cfg.get("enable") and port and port not in _HELD:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind(("", port))
        _HELD[port] = s


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

    def rm_get_realtime_push(self):
        cfg = _push()
        if cfg is None:
            return -1, None
        _sdk_listens(cfg)
        return 0, dict(cfg)

    def rm_set_realtime_push(self, cfg):
        now = _push()
        if now is None:
            return -1
        ip = cfg.ip.decode() if isinstance(cfg.ip, bytes) else str(cfg.ip)
        now.update(enable=bool(cfg.enable), port=int(cfg.port), ip=ip, cycle=int(cfg.cycle))
        Path(os.environ["LITE_FAKE_PUSH_FILE"]).write_text(json.dumps(now))
        _sdk_listens(now)
        return 0

    def __getattr__(self, name):
        if name.startswith("rm_"):
            return lambda *a, **kw: (-1, None)
        raise AttributeError(name)


class FakeRunner:
    """Runner v14's sequence as far as its lines and its Ctrl+C go (LITE_FAKE_RUNNER): a Program's lines, then a Wait in
    50 ms slices; a KeyboardInterrupt is its STOP, handled and said as the runner does (its slow stop, the speed back)."""

    @staticmethod
    def normalize_workflow_step(st):
        return dict(st)

    @staticmethod
    def run_workflow_sequence(arm, a, ip, steps):
        import time

        try:
            print("WORKFLOW STEP START 1/2: PROGRAM")
            print("WORKFLOW PROGRAM 1/2: slot 71 (71_x.txt) // START/TRANSIT 30% // stored trajectory timing // overall 15%")
            print("WORKFLOW PROGRAM COMPLETE 1/2: 0.500 s")
            print("WORKFLOW STEP START 2/2: WAIT")
            t_end = time.monotonic() + 20
            while time.monotonic() < t_end:
                time.sleep(0.05)
            print("WORKFLOW COMPLETE: 2/2 step(s), 20.5 s")
            return 0
        except KeyboardInterrupt:
            print("** WORKFLOW STOP REQUESTED")
            print("  arm slow-stop ret=0")
            print("WORKFLOW SPEED RESTORE: 50% // ret=0")
            return 1


def setup():
    from alix_dispatch import connect

    if os.environ.get("LITE_FAKE_RUNNER"):
        from alix_studio_lite.server import robot_real

        robot_real._RUNNER = FakeRunner
    marker = os.environ.get("LITE_FAKE_STUCK_MARKER")
    state: dict = {}                                        # this process's connections hang (the marker was there at its first)

    def open_arm(side, ip=None, port=None, **kw):
        if "stuck" not in state:                            # the link opens two connections (26 Oct: the readings' first, then
            state["stuck"] = bool(marker) and Path(marker).exists()   # the commands'): both of this process hang the one call,
            if state["stuck"]:                              # which only the commands' connection ever makes
                Path(marker).unlink()
        stuck = state["stuck"]
        cfg = _push()
        if cfg is not None:                                 # connecting reads the push setting, as the SDK does
            _sdk_listens(cfg)
        return connect.ArmHandle(side, ip or "192.168.1.103", port or 8080, Controller(stuck=stuck), 1, "127.0.0.1")

    connect.open_arm = open_arm
    connect.close_arm = lambda h: None
