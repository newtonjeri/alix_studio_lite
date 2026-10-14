#!/usr/bin/env python3
"""UPLOAD A TRACK TO A CONTROLLER SLOT, CHECK IT LANDED, AND RUN IT — standalone.

    python3 rm_upload.py TRACK.txt SLOT --side left           # upload, then verify
    python3 rm_upload.py TRACK.txt SLOT --side left --run     # upload, verify, then run it
    python3 rm_upload.py --run SLOT --side left               # run what a slot holds
    python3 rm_upload.py --list --side left                   # what the controller's slots hold

Needs only RealMan's RM API2 Python package (`Robotic_Arm`) and the Python standard library. The
package is taken from --sdk DIR if given, else an installed one (`pip install Robotic_Arm`), else
the SDK copy in this repository (external/butterfli-realman-alix/Python).

TRACK.txt: one joint point per line, `{"point": [j1, ..., j7]}` in millidegrees — a drag-teach
recording or a slot file from Resource/tracks/taught/.

UPLOAD (the arm does not move):
  1. read the track; a malformed line refuses the whole file (half a file is never sent)
  2. write the controller's project file: `{"file":7}`, a Folder line whose `type` is the point
     count, then the points, every line ending CRLF (the controller rejects LF-only)
  3. show what the slot holds now, and ask before writing it
  4. rm_send_project, store only: success is ret 0 with err_line -1
  5. VERIFY: rm_get_program_trajectory_list must list the slot, with a stored size equal to the
     bytes sent (within max(64 B, 0.1 %))

RUN (the arm MOVES — nothing else may be driving it; stop the ROS driver first):
  1. refuse if the slot is empty, the arm reports an error, or a program is already running
  2. the controller first moves the arm, in its own joint move, from where it is to the program's
     first point: with a TRACK.txt the size of that move is printed; keep its path clear
  3. you type the arm side (or `run`) to start; Ctrl+C at any time slow-stops the arm
  4. rm_set_program_id_run, then watch rm_get_program_run_state to the end, and the arm's error
     codes every half second
  5. after the arm has settled, put the controller's global speed back (--restore-speed): the run
     speed sets it, it survives power cycles, and nothing can read it back

Exit code 0: done and verified / run complete with no error. 1: refused or failed.
"""
import argparse
import json
import os
import pathlib
import sys
import tempfile
import time

#: the two arms' controllers (override with --ip, or RM_LEFT_IP / RM_RIGHT_IP)
ARM_IPS = {"left": os.environ.get("RM_LEFT_IP", "192.168.1.10"),
           "right": os.environ.get("RM_RIGHT_IP", "192.168.1.103")}
PORT = 8080
#: rm_send_project's status codes (the SDK's own docstring)
SEND_RET = {1: "the controller returned false (a bad parameter, or the arm's state)",
            -1: "sending failed", -2: "no reply (receive failed or timed out)",
            -3: "the reply could not be parsed", -4: "the file name was refused",
            -5: "the file could not be read", -6: "a 4th-generation controller does not support this"}
#: arm error codes worth naming (RealMan's list: https://develop.realman-robotics.com/robot/json/errorList/)
KNOWN_ERR = {4105: "joint speed over its limit (a warning; the run goes on)", 4109: "ARM COLLISION"}
WARN_ONLY = {4105}


def load_sdk(sdk_dir):
    """RealMan's Robotic_Arm package: --sdk DIR, an installed one, or this repository's copy."""
    repo_copy = pathlib.Path(__file__).resolve().parents[1] / "external/butterfli-realman-alix/Python"
    for where in ([sdk_dir] if sdk_dir else []) + [None, repo_copy]:
        if where is not None:
            sys.path.insert(0, str(where))
        try:
            from Robotic_Arm.rm_robot_interface import RoboticArm, rm_send_project_t
            from Robotic_Arm.rm_ctypes_wrap import rm_thread_mode_e
            return RoboticArm, rm_send_project_t, rm_thread_mode_e
        except ImportError:
            if where is not None:
                sys.path.remove(str(where))
    sys.exit("RM API2 (Robotic_Arm) not found: pip install Robotic_Arm, or give --sdk DIR")


def read_track(path):
    """The track's point lines, checked: each must be {"point": [7 integers]}."""
    lines = [ln for ln in path.read_bytes().replace(b"\r", b"\n").split(b"\n") if ln.strip()]
    for i, ln in enumerate(lines, 1):
        try:
            point = json.loads(ln)["point"]
        except Exception as exc:
            sys.exit(f"{path}:{i}: not a point line ({exc})")
        if len(point) != 7 or not all(isinstance(v, int) for v in point):
            sys.exit(f"{path}:{i}: expected 7 integer millidegrees, got {point}")
    if len(lines) < 2:
        sys.exit(f"{path}: {len(lines)} point(s) — not a track")
    return lines


def write_project(lines, out):
    """The controller's project file: the header, the Folder line (type = point count), the points."""
    crlf = b"\r\n"
    header = (b'{"file":7}' + crlf
              + b'{"name":"Folder","num":1,"type":%d,"enabled":true,"parent_number":0}' % len(lines) + crlf)
    out.write_bytes(header + crlf.join(lines) + crlf)
    return out.stat().st_size


#: programs per page of the list. The SDK reads each reply into a fixed buffer: one page of 100 with
#: ~70 programs on the controller was cut off mid-entry ("Error parsing JSON", ret -3, 2026-09-23)
PAGE = 10


def slots(arm):
    """{slot: (name, size in bytes, stored speed)} of every program the controller holds, read a page
    of PAGE at a time until a page comes back short."""
    held = {}
    for page in range(1, 100 // PAGE + 2):
        ret, data = arm.rm_get_program_trajectory_list(page, PAGE, "")
        if ret != 0:
            sys.exit(f"the controller's program list could not be read (page {page}, ret={ret})")
        batch = data.get("trajectory_list") or []
        fresh = [t for t in batch if int(t["id"]) not in held]
        for t in fresh:
            held[int(t["id"])] = (t.get("trajectory_name") or "", int(t.get("size") or 0), t.get("speed"))
        if len(batch) < PAGE or not fresh:
            break
    return held


def arm_state(arm):
    """(joint angles in degrees, the arm's error codes as integers, zeros dropped), or (None, None) when
    unreadable. The SDK gives the codes as decimal TEXT — "0" when all is well — so each is converted
    before it is judged (a "0" kept as text reads as an error: 2026-09-23 on the left arm)."""
    ret, st = arm.rm_get_current_arm_state()
    if ret != 0:
        return None, None
    err = st.get("err") or {}
    raw = (err.get("err") or []) if isinstance(err, dict) else (err or [])
    codes = []
    for c in raw:
        s = str(c).strip()
        try:
            v = int(s, 16) if s.lower().startswith("0x") else int(s)
        except ValueError:
            continue
        if v:
            codes.append(v)
    return list(st.get("joint") or []), codes


def codes_text(codes):
    return ", ".join(f"{c} (0x{c:04X}{': ' + KNOWN_ERR[c] if c in KNOWN_ERR else ''})" for c in codes)


def upload(arm, a, ip, lines):
    held = slots(arm)
    with tempfile.TemporaryDirectory() as tmp:
        proj = pathlib.Path(tmp) / (a.track.stem + ".proj")
        sent = write_project(lines, proj)
        now = held.get(a.slot)
        print(f"upload {a.track.name}: {len(lines)} points, {sent / 1024:.1f} KB -> slot {a.slot} on {ip}")
        print(f"  slot {a.slot} now: " + (f"{now[0]} ({now[1] / 1024:.1f} KB) — it will be REPLACED"
                                         if now else "empty"))
        if not a.yes and input("  write it? [y/N] ").strip().lower() != "y":
            print("  not written")
            return 1
        ret, err_line = arm.rm_send_project(rm_send_project_t(str(proj), a.speed, 1, a.slot, 0, 0, 0))
    if ret != 0 or err_line != -1:
        why = SEND_RET.get(ret, "") if ret != 0 else (
            "the data length check failed" if err_line == 0 else f"the controller refused line {err_line}")
        print(f"  ** UPLOAD FAILED: ret={ret} err_line={err_line} — {why}")
        return 1
    print("  sent: ret=0 err_line=-1")
    got = slots(arm).get(a.slot)                      # VERIFY: what the controller now holds
    if got is None:
        print(f"  ** NOT VERIFIED: slot {a.slot} is not in the controller's program list")
        return 1
    name, size, _ = got
    if abs(size - sent) > max(64, sent // 1000):
        print(f"  ** NOT VERIFIED: slot {a.slot} holds {size} bytes, {sent} were sent")
        return 1
    print(f"  VERIFIED: slot {a.slot} holds {name!r}, {size} bytes (sent {sent})")
    return 0


def settle(arm, still_deg_s=2.0, timeout_s=5.0):
    """Wait until the arm has stopped moving: a program's end is reported before the arm is still."""
    t_end, last, calm = time.monotonic() + timeout_s, None, 0
    while time.monotonic() < t_end and calm < 3:
        joints, _ = arm_state(arm)
        if joints and last:
            calm = calm + 1 if max(abs(p - q) for p, q in zip(joints, last)) / 0.1 < still_deg_s else 0
        last = joints
        time.sleep(0.1)


def run(arm, a, ip, slot, first_point=None):
    held = slots(arm).get(slot)
    if held is None:
        print(f"** slot {slot} is empty on {ip} — nothing to run")
        return 1
    joints, errs = arm_state(arm)
    if joints is None:
        print("** the arm's state could not be read — not starting")
        return 1
    if errs:
        print(f"** the arm reports {codes_text(errs)} — clear it at the pendant first")
        return 1
    ret, st = arm.rm_get_program_run_state()
    if ret != 0 or st.get("run_state") in (1, 2):
        print(f"** a program is already running or paused (slot {st.get('id')}), or the state could not "
              f"be read (ret={ret}) — not starting")
        return 1
    print(f"run slot {slot} ({held[0]}) on {ip}; transit speed {a.run_speed} %")
    if first_point is not None:
        gap = [abs(j - p / 1000.0) for j, p in zip(joints, first_point)]
        k = max(range(len(gap)), key=gap.__getitem__)
        print(f"  the controller first moves the arm to the program's first point: up to {gap[k]:.1f} deg "
              f"on J{k + 1}" + (" — keep that path clear" if gap[k] > 1.0 else ""))
    else:
        print("  the controller first moves the arm, in its own joint move, to the program's first "
              "point — keep that path clear")
    want = a.side or "run"
    if not a.yes and input(f"  type '{want}' to start (Ctrl+C stops the arm): ").strip().lower() != want:
        print("  not started")
        return 1

    seen, started, ended, stopped = set(), False, False, None
    t0 = time.monotonic()
    try:
        ret = arm.rm_set_program_id_run(slot, a.run_speed, 0)
        if ret != 0:
            print(f"  ** NOT STARTED: rm_set_program_id_run ret={ret}")
            return 1
        next_check = t0
        while True:
            now = time.monotonic()
            r, st = arm.rm_get_program_run_state()
            rs = st.get("run_state") if r == 0 else None
            if rs in (1, 2) and not started:
                started = True
                print(f"  running ({now - t0:.1f} s)")
            if started and rs == 0:
                ended = True
                break
            if not started and now - t0 > 5.0:
                print("  ** the program did not start within 5 s")
                break
            if now - t0 > a.timeout:
                print(f"  ** still running after {a.timeout:g} s — stopping it")
                stopped = "the timeout"
                break
            if now >= next_check:                         # the arm's error codes, every half second
                next_check = now + 0.5
                _, errs = arm_state(arm)
                for c in errs or []:
                    if c not in seen:
                        seen.add(c)
                        print(f"  !! the arm reports {codes_text([c])} at {now - t0:.1f} s")
            time.sleep(0.1)
    except KeyboardInterrupt:
        stopped = "Ctrl+C"
    finally:
        if stopped:
            print(f"  ** slow-stopping the arm ({stopped}): ret={arm.rm_set_arm_slow_stop()}")
        if started:
            settle(arm)
        r = arm.rm_set_plan_speed(a.restore_speed)        # the global speed, back to a known value
        print(f"  global speed restored to {a.restore_speed} % (ret={r})"
              + ("" if r == 0 else " — ** RESTORE FAILED, set it at the pendant"))
    _, errs = arm_state(arm)
    bad = sorted((seen | set(errs or [])) - WARN_ONLY)
    if ended and not stopped and not bad:
        print(f"  RUN COMPLETE: {time.monotonic() - t0:.1f} s, no error")
        return 0
    print(f"  ** RUN NOT COMPLETE" + (f": {codes_text(bad)}" if bad else ""))
    return 1


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("track", nargs="?", type=pathlib.Path, help="the .txt track to upload")
    ap.add_argument("slot", nargs="?", type=int, help="the controller slot, 1-100")
    ap.add_argument("--side", choices=("left", "right"), help="the arm (sets its controller's IP)")
    ap.add_argument("--ip", help="the controller's IP (instead of --side)")
    ap.add_argument("--port", type=int, default=PORT)
    ap.add_argument("--speed", type=int, default=0,
                    help="upload: the slot's stored run speed, 0-100 (rm_send_project's plan_speed; 0, "
                         "the default, is what this repository has always uploaded with)")
    ap.add_argument("--run", nargs="?", const=0, type=int, metavar="SLOT",
                    help="run a slot: after an upload, the uploaded one (--run alone); or --run SLOT")
    ap.add_argument("--run-speed", type=int, default=20,
                    help="run: the speed of the controller's move to the first point, 1-100 (default 20); "
                         "the stored points play at their own timing")
    ap.add_argument("--restore-speed", type=int, default=50,
                    help="run: the controller's global speed to put back afterwards (default 50)")
    ap.add_argument("--timeout", type=float, default=300.0, help="run: slow-stop after this many seconds")
    ap.add_argument("--list", action="store_true", help="only print what the slots hold")
    ap.add_argument("--yes", action="store_true", help="do not ask before writing a slot or starting a run")
    ap.add_argument("--sdk", type=pathlib.Path, help="a directory holding the Robotic_Arm package")
    a = ap.parse_args()
    ip = a.ip or ARM_IPS.get(a.side)
    if not ip:
        ap.error("give --side left|right or --ip")
    lines = None
    if a.track is not None:
        if a.slot is None:
            ap.error("give TRACK.txt and SLOT")
        if not 1 <= a.slot <= 100:
            ap.error("a controller slot is 1-100")
        if not 0 <= a.speed <= 100:
            ap.error("--speed is 0-100")
        if not a.track.is_file():
            sys.exit(f"{a.track}: no such file")
        lines = read_track(a.track)
    run_slot = None
    if a.run is not None:
        run_slot = a.run or a.slot
        if not run_slot:
            ap.error("--run needs a SLOT (--run 45), or an upload before it (TRACK.txt SLOT --run)")
        if not 1 <= run_slot <= 100:
            ap.error("a controller slot is 1-100")
        if not 1 <= a.run_speed <= 100 or not 1 <= a.restore_speed <= 100:
            ap.error("--run-speed and --restore-speed are 1-100")
    if lines is None and run_slot is None and not a.list:
        ap.error("give TRACK.txt SLOT to upload, --run SLOT to run, or --list")

    global rm_send_project_t
    RoboticArm, rm_send_project_t, rm_thread_mode_e = load_sdk(a.sdk)
    arm = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
    handle = arm.rm_create_robot_arm(ip, a.port, 3)
    if getattr(handle, "id", -1) <= 0:
        sys.exit(f"cannot connect to the controller at {ip}:{a.port}")
    try:
        if hasattr(arm, "rm_set_timeout"):
            arm.rm_set_timeout(1500)                  # the reply timeout, ms (the SDK default is 500)
        if a.list:
            held = slots(arm)
            print(f"{ip}: {len(held)} slot(s) in use")
            for sid, (name, size, speed) in sorted(held.items()):
                print(f"  slot {sid:>3}  {size / 1024:8.1f} KB  speed {speed}  {name}")
        if lines is not None and upload(arm, a, ip, lines):
            return 1
        if run_slot is not None:
            first = json.loads(lines[0])["point"] if lines is not None and run_slot == a.slot else None
            return run(arm, a, ip, run_slot, first)
        return 0
    finally:
        arm.rm_delete_robot_arm()


if __name__ == "__main__":
    sys.exit(main())
