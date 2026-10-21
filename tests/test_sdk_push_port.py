"""What the RealMan SDK's own library does with the controller's realtime push (server/udp_push.py rests on it): its
libapi_c.so, vendored with ALIX, in triple-thread mode as Lite connects (ALIX's connect.open_arm), against a stand-in
controller on this computer that answers as RealMan's own session logs show a controller answering
(vendor/alix/external/butterfli-realman-alix/Demo/.../robot_log.txt).

- With the push ENABLED when the arm connects, the SDK binds its port on this computer for its own listener, and keeps
  it: turning the push off (the controller confirms it) does not let it go, and a capture's bind of that port without
  SO_REUSEADDR (ALIX's track.UdpRecorder) is refused: the lab's "UDP port 8090 is already bound ON THIS HOST", 21 Oct.
- With the push off when the arm connects, nothing is bound, and the capture binds the port.

A new SDK that behaves otherwise fails here first. Each case runs in a process of its own: the SDK is process-wide."""
import json
import platform
import subprocess
import sys
import textwrap

import pytest

from alix_studio_lite import vendor_paths

SDK = vendor_paths.LITE_ROOT / "vendor/alix/external/butterfli-realman-alix/Python"
pytestmark = pytest.mark.skipif(not (sys.platform == "linux" and platform.machine() == "x86_64"
                                     and (SDK / "Robotic_Arm/libs/linux_x86/libapi_c.so").exists()),
                                reason="the SDK's Linux x86_64 library")

CHILD = textwrap.dedent('''
    import errno, json, os, socket, sys, threading
    sys.path.insert(0, sys.argv[1]); sys.path.insert(0, sys.argv[2])
    ON, PUSH_PORT = sys.argv[3] == "on", int(sys.argv[4])
    push = dict(cycle=1, enable=ON, port=PUSH_PORT, ip="127.0.0.1")
    CUSTOM = dict(joint_speed=1, lift_state=1, expand_state=1, hand_state=1, arm_current_status=1, aloha_state=1,
                  plus_base=1, plus_state=1)
    CANNED = {   # as RealMan's own session logs show a controller answering
        "get_arm_software_info": {"Product_version": "RM65-BI", "algorithm_info": {"version": "1.3.4"}, "command": "arm_software_info",
                                  "ctrl_info": {"build_time": "2024/06/20 20:07:01", "commit_id": "95e4b22", "version": "V1.5.6"},
                                  "dynamic_info": {"model_version": "2"},
                                  "plan_info": {"build_time": "2024/06/20 20:07:16", "commit_id": "8d1269e", "version": "V1.5.6"}},
        "get_arm_current_trajectory": {"command": "arm_current_trajectory", "trajectory_type": "none", "data": [0] * 7},
        "get_current_work_frame": {"frame_name": "World", "pose": [0] * 6, "state": "current_work_frame"},
        "get_current_tool_frame": {"payload": 0, "pose": [0] * 6, "position": [0] * 3, "state": "current_tool_frame", "tool_name": "Arm_Tip"},
        "get_install_pose": {"pose": [0, 0, 0], "state": "install_pose"},
        "get_joint_min_pos": {"min_pos": [-108000, -130000, -135000, -178000, -128000, -360000], "state": "joint_min_pos"},
        "get_joint_max_pos": {"max_pos": [178000, 130000, 135000, 178000, 128000, 360000], "state": "joint_max_pos"},
        "get_joint_max_acc": {"joint_acc": [50000, 100000, 100000, 100000, 100000, 100000], "state": "joint_max_acc"},
        "get_joint_max_speed": {"joint_speed": [15000, 30000, 37500, 37500, 37500, 37500], "state": "joint_max_speed"},
    }

    def reply(cmd):
        c = cmd.get("command")
        if c == "get_realtime_push":
            return dict(command=c, force_coordinate=-1, custom=CUSTOM, **push)
        if c == "set_realtime_push":
            push.update({k: cmd[k] for k in ("cycle", "enable", "port", "ip") if k in cmd})
            return {"command": c, "state": True}
        return CANNED.get(c, {"command": c, "state": True})

    def client(conn):
        buf = b""
        while True:
            data = conn.recv(65536)
            if not data:
                return
            buf += data
            while b"\\r\\n" in buf:
                line, buf = buf.split(b"\\r\\n", 1)
                try:
                    conn.sendall((json.dumps(reply(json.loads(line))) + "\\r\\n").encode())
                except ValueError:
                    pass

    srv = socket.socket(); srv.bind(("127.0.0.1", 0)); srv.listen(2)
    threading.Thread(target=lambda: [threading.Thread(target=client, args=(srv.accept()[0],), daemon=True).start() for _ in range(4)],
                     daemon=True).start()

    from alix_studio_lite.server.udp_push import port_holders
    from Robotic_Arm.rm_ctypes_wrap import rm_thread_mode_e
    from Robotic_Arm.rm_robot_interface import RoboticArm, rm_realtime_push_config_t, rm_udp_custom_config_t

    mine = lambda: any(h["pid"] == os.getpid() for h in port_holders(PUSH_PORT))
    def capture_bind():
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            s.bind(("", PUSH_PORT)); return "ok"
        except OSError as exc:
            return errno.errorcode.get(exc.errno, str(exc.errno))
        finally:
            s.close()

    out = {}
    robot = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)          # as ALIX's connect.open_arm (THREAD_MODE_TRIPLE)
    robot.rm_create_robot_arm("127.0.0.1", srv.getsockname()[1], 3)
    out["bound_at_connect"] = mine()
    out["off_ret"] = robot.rm_set_realtime_push(rm_realtime_push_config_t(cycle=1, enable=False, port=PUSH_PORT, force_coordinate=-1,
                                                                          ip="127.0.0.1", custom_config=rm_udp_custom_config_t()))
    out["controller_off"] = push["enable"] is False
    out["bound_after_off"] = mine()
    out["capture_bind"] = capture_bind()
    print("RESULT " + json.dumps(out), flush=True)
    os._exit(0)
''')


def _free_udp_port() -> int:
    import socket

    while True:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.bind(("", 0))
            p = s.getsockname()[1]
        if p not in (8089, 8090):
            return p


def _run(tmp_path, case: str) -> dict:
    script = tmp_path / f"sdk_{case}.py"
    script.write_text(CHILD, encoding="utf-8")
    done = subprocess.run([sys.executable, "-I", str(script), str(SDK), str(vendor_paths.LITE_ROOT / "src"), case, str(_free_udp_port())],
                          capture_output=True, text=True, timeout=60)
    line = next((ln for ln in done.stdout.splitlines() if ln.startswith("RESULT ")), None)
    assert line, done.stdout[-2000:] + done.stderr[-2000:]
    return json.loads(line[len("RESULT "):])


def test_the_sdk_binds_an_enabled_push_s_port_as_it_connects_and_keeps_it(tmp_path):
    r = _run(tmp_path, "on")
    assert r["bound_at_connect"], r
    assert r["off_ret"] == 0 and r["controller_off"] and r["bound_after_off"], r      # off on the controller, still bound
    assert r["capture_bind"] == "EADDRINUSE", r                                        # the lab's refusal


def test_with_the_push_off_as_it_connects_the_sdk_binds_nothing(tmp_path):
    r = _run(tmp_path, "off")
    assert not r["bound_at_connect"] and not r["bound_after_off"] and r["capture_bind"] == "ok", r
