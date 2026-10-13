#!/usr/bin/env python3
"""UPLOAD A TRACK TO A CONTROLLER SLOT, CHECK IT LANDED, AND RUN IT — standalone.

    python3 rm_upload.py TRACK.txt SLOT --side left           # upload, then verify
    python3 rm_upload.py TRACK.txt SLOT --side left --run     # upload, verify, then run it
    python3 rm_upload.py --run SLOT --side left               # run what a slot holds
    python3 rm_upload.py --sequence 25 31 42 --side left       # fast multi-run, one connection
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

RUN / SEQUENCE (the arm MOVES — nothing else may be driving it; stop the ROS driver first):
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




def parse_hand_target(spec):
    """
    Parse one hand target.

    Accepted:
      blank / None
          no hand change for this sequence step

      0..100 or "50%"
          percentage open for little/ring/middle/index; both thumb channels hold
          their current position (uses hand.fingers_pct_counts)

      "v1,v2,v3,v4,v5,v6"
          explicit Inspire counts for all six channels, each 0..1000
    """
    if spec is None:
        return None, None

    if isinstance(spec, (int, float)):
        s = str(spec)
    else:
        s = str(spec).strip()

    if not s:
        return None, None

    if "," in s:
        parts = [p.strip() for p in s.split(",")]
        if len(parts) != 6:
            raise ValueError("hand counts must contain exactly 6 comma-separated values")
        values = [int(p) for p in parts]
        if any(v < 0 or v > 1000 for v in values):
            raise ValueError("each hand channel must be 0-1000")
        return values, ",".join(str(v) for v in values)

    if s.endswith("%"):
        s = s[:-1].strip()

    pct = float(s)
    if not 0 <= pct <= 100:
        raise ValueError("hand percentage must be 0-100")
    return ("percent", pct), f"{pct:g}%"


def normalize_sequence_step(item):
    """Validate one sequence-file entry and return the runner's normalized dict."""
    if not isinstance(item, dict):
        raise ValueError("each sequence program must be an object")

    slot = int(item.get("id"))
    if not 1 <= slot <= 100:
        raise ValueError(f"Program ID {slot} is outside 1-100")

    hand_raw = item.get("hand")
    _, hand_display = parse_hand_target(hand_raw)

    pole_raw = item.get("pole_mm")
    pole_mm = None
    if pole_raw not in (None, ""):
        pole_mm = float(pole_raw)
        if not 0 <= pole_mm <= 315:
            raise ValueError(f"pole height for Program {slot} must be 0-315 mm")

    wait_raw = item.get("wait_s", item.get("wait"))
    wait_s = 0.0
    if wait_raw not in (None, ""):
        wait_s = float(wait_raw)
        if not 0 <= wait_s <= 3600:
            raise ValueError(f"wait time for Program {slot} must be 0-3600 seconds")

    return {
        "id": slot,
        "name": str(item.get("name") or ""),
        "hand": hand_display,
        "pole_mm": pole_mm,
        "wait_s": wait_s,
    }


def load_sequence_file(path):
    data = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict) or data.get("format") != "realman_multi_run_sequence":
        raise ValueError("not a RealMan Multi Run sequence JSON file")

    raw = data.get("programs")
    if not isinstance(raw, list) or not raw:
        raise ValueError("sequence file contains no programs")

    return [normalize_sequence_step(item) for item in raw]


def load_aux_classes():
    """
    Load hand.py and pole.py from the same directory as this runner.

    Keeping these beside rm_upload_fast_v3.py means the UI/runner can use the
    user's tested Hand and Pole classes without duplicating their hardware code.
    """
    local_dir = pathlib.Path(__file__).resolve().parent
    if str(local_dir) not in sys.path:
        sys.path.insert(0, str(local_dir))

    try:
        from hand import Hand, fingers_pct_counts
    except ImportError as exc:
        raise RuntimeError(
            f"hand.py could not be loaded from {local_dir}: {exc}"
        ) from exc

    try:
        from pole import Pole
    except ImportError as exc:
        raise RuntimeError(
            f"pole.py could not be loaded from {local_dir}: {exc}"
        ) from exc

    return Hand, fingers_pct_counts, Pole


def run_sequence(arm, a, ip, sequence):
    """
    Run stored controller programs through ONE connection.

    Each sequence item may also define:
      hand:    hand percentage or six-channel Inspire counts
      pole_mm: lift/pole target in millimetres
      wait_s:   delay in seconds after Pole/Hand and before the trajectory

    Per-step execution order is:
      Pole -> Hand -> Wait -> Robot trajectory

    Blank Hand/Pole targets mean "keep the current position"; blank wait means 0 s.
    """
    # Backward compatibility: --sequence 20 21 22 still works.
    steps = []
    for item in sequence:
        if isinstance(item, dict):
            steps.append(normalize_sequence_step(item))
        else:
            steps.append({
                "id": int(item),
                "name": "",
                "hand": None,
                "pole_mm": None,
                "wait_s": 0.0,
            })

    total = len(steps)
    slots_requested = [step["id"] for step in steps]
    held_all = slots(arm)

    missing = [slot for slot in slots_requested if slot not in held_all]
    if missing:
        print("** SEQUENCE REFUSED: empty/missing controller slot(s): "
              + ", ".join(str(x) for x in missing))
        return 1

    joints, errs = arm_state(arm)
    if joints is None:
        print("** SEQUENCE REFUSED: the arm's state could not be read")
        return 1
    if errs:
        blocking = sorted(set(errs) - WARN_ONLY)
        warnings = sorted(set(errs) & WARN_ONLY)

        if warnings:
            print(
                "  !! warning-only arm code(s) already present: "
                + codes_text(warnings)
                + " — sequence may continue"
            )

        if blocking:
            print(
                f"** SEQUENCE REFUSED: arm reports {codes_text(blocking)} "
                "— clear it at the pendant first"
            )
            return 1

    ret, st = arm.rm_get_program_run_state()
    if ret != 0 or st.get("run_state") in (1, 2):
        print(f"** SEQUENCE REFUSED: controller is not idle "
              f"(slot {st.get('id')}, ret={ret})")
        return 1

    need_hand = any(step.get("hand") for step in steps)
    need_pole = any(step.get("pole_mm") is not None for step in steps)

    hand_dev = None
    pole_dev = None
    fingers_pct_counts = None

    try:
        if need_hand or need_pole:
            Hand, fingers_pct_counts, Pole = load_aux_classes()

        if need_hand:
            hand_dev = Hand(arm)
            hand_dev.open()
            print("AUX HAND READY: Inspire hand Modbus opened; speed=1000")

        if need_pole:
            pole_dev = Pole(arm)
            current_mm = pole_dev.height_m()
            if current_mm is None:
                print("AUX POLE READY: current height unavailable")
            else:
                print(f"AUX POLE READY: current height {current_mm * 1000.0:.1f} mm")

        print(f"SEQUENCE START: {total} program(s) on {ip}; one persistent controller connection")
        for i, step in enumerate(steps, 1):
            slot = step["id"]
            aux = []
            if step.get("hand"):
                aux.append(f"hand={step['hand']}")
            if step.get("pole_mm") is not None:
                aux.append(f"pole={step['pole_mm']:g}mm")
            if float(step.get("wait_s") or 0) > 0:
                aux.append(f"wait={float(step['wait_s']):g}s")
            suffix = " | " + ", ".join(aux) if aux else ""
            print(f"  {i:02d}. slot {slot}: {held_all[slot][0]}{suffix}")

        want = a.side or "run"
        if not a.yes:
            typed = input(
                f"  type '{want}' to start ALL {total} programs "
                "(Ctrl+C stops/cancels): "
            ).strip().lower()
            if typed != want:
                print("  sequence not started")
                return 1

        sequence_t0 = time.monotonic()
        completed = 0
        seen_all = set()
        active_started = False

        for index, step in enumerate(steps, 1):
            slot = step["id"]
            active_started = False

            # Lightweight arm safety check before auxiliary motion and trajectory.
            joints, errs = arm_state(arm)
            if joints is None:
                print(f"** SEQUENCE FAILED before {index}/{total}: arm state unreadable")
                return 1
            if errs:
                blocking = sorted(set(errs) - WARN_ONLY)
                warnings = sorted(set(errs) & WARN_ONLY)

                if warnings:
                    print(
                        f"  !! warning-only arm code before {index}/{total}: "
                        f"{codes_text(warnings)} — continuing"
                    )

                if blocking:
                    print(
                        f"** SEQUENCE FAILED before {index}/{total}: "
                        f"{codes_text(blocking)}"
                    )
                    return 1

            ret, st = arm.rm_get_program_run_state()
            if ret != 0 or st.get("run_state") in (1, 2):
                print(f"** SEQUENCE FAILED before {index}/{total}: controller not idle "
                      f"(slot {st.get('id')}, ret={ret})")
                return 1

            # ----------------------------------------------------------------
            # Auxiliary stage: POLE -> HAND -> WAIT -> ARM TRAJECTORY
            # ----------------------------------------------------------------
            if step.get("pole_mm") is not None:
                target_mm = float(step["pole_mm"])
                print(f"AUX POLE {index}/{total}: target {target_mm:.1f} mm")
                t_aux = time.monotonic()
                try:
                    ok, final_m = pole_dev.move(
                        target_mm / 1000.0,
                        speed_pct=100,
                        block=False,
                    )
                except Exception as exc:
                    print(f"** AUX POLE FAILED {index}/{total}: {exc}")
                    return 1

                final_mm = None if final_m is None else final_m * 1000.0
                if not ok:
                    print(
                        f"** AUX POLE FAILED {index}/{total}: timeout"
                        + ("" if final_mm is None else f", final {final_mm:.1f} mm")
                    )
                    return 1

                print(
                    f"AUX POLE COMPLETE {index}/{total}: "
                    f"{time.monotonic() - t_aux:.3f} s"
                    + ("" if final_mm is None else f", final {final_mm:.1f} mm")
                )

            if step.get("hand"):
                parsed, hand_display = parse_hand_target(step["hand"])
                if isinstance(parsed, tuple) and parsed[0] == "percent":
                    counts = fingers_pct_counts(parsed[1])
                else:
                    counts = parsed

                print(f"AUX HAND {index}/{total}: target {hand_display}")
                t_aux = time.monotonic()
                try:
                    ok, final_counts = hand_dev.command(counts, wait=True)
                except Exception as exc:
                    print(f"** AUX HAND FAILED {index}/{total}: {exc}")
                    return 1

                if not ok:
                    print(
                        f"** AUX HAND FAILED {index}/{total}: timeout, "
                        f"final={final_counts}"
                    )
                    return 1

                print(
                    f"AUX HAND COMPLETE {index}/{total}: "
                    f"{time.monotonic() - t_aux:.3f} s, final={final_counts}"
                )

            wait_s = float(step.get("wait_s") or 0.0)
            if wait_s > 0:
                print(f"AUX WAIT {index}/{total}: {wait_s:g} s")
                wait_t0 = time.monotonic()
                time.sleep(wait_s)
                print(
                    f"AUX WAIT COMPLETE {index}/{total}: "
                    f"{time.monotonic() - wait_t0:.3f} s"
                )

            # Re-check the arm after Pole/Hand/Wait, before starting trajectory.
            joints, errs = arm_state(arm)
            if joints is None:
                print(f"** SEQUENCE FAILED after AUX {index}/{total}: arm state unreadable")
                return 1
            if errs:
                blocking = sorted(set(errs) - WARN_ONLY)
                if blocking:
                    print(
                        f"** SEQUENCE FAILED after AUX {index}/{total}: "
                        f"{codes_text(blocking)}"
                    )
                    return 1

            name = held_all[slot][0]
            print(f"SEQUENCE PROGRAM {index}/{total}: slot {slot} ({name})")

            command_t0 = time.monotonic()
            ret = arm.rm_set_program_id_run(slot, a.run_speed, 0)
            if ret != 0:
                print(f"  ** NOT STARTED: rm_set_program_id_run ret={ret}")
                return 1

            t0 = time.monotonic()
            started = False
            ended = False
            stopped = None
            seen = set()
            next_check = t0

            while True:
                now = time.monotonic()
                r, state = arm.rm_get_program_run_state()
                rs = state.get("run_state") if r == 0 else None

                if rs in (1, 2) and not started:
                    started = True
                    active_started = True
                    print(
                        f"SEQUENCE RUNNING {index}/{total}: slot {slot} "
                        f"(start acknowledgement {now - command_t0:.3f} s)"
                    )

                if started and rs == 0:
                    ended = True
                    break

                if not started and now - t0 > 5.0:
                    print(f"  ** program {slot} did not start within 5 s")
                    break

                if now - t0 > a.timeout:
                    print(f"  ** program {slot} still running after {a.timeout:g} s — stopping it")
                    stopped = "the timeout"
                    break

                if now >= next_check:
                    next_check = now + 0.5
                    _, errs = arm_state(arm)
                    for c in errs or []:
                        if c not in seen:
                            seen.add(c)
                            seen_all.add(c)
                            print(f"  !! arm reports {codes_text([c])} at {now - t0:.1f} s")

                time.sleep(0.1)

            if stopped:
                print(f"  ** slow-stopping arm ({stopped}): ret={arm.rm_set_arm_slow_stop()}")
                if started:
                    settle(arm)
                return 1

            if not ended:
                print(f"  ** SEQUENCE PROGRAM NOT COMPLETE {index}/{total}: slot {slot}")
                return 1

            settle_t0 = time.monotonic()
            if started:
                settle(arm)
            settle_elapsed = time.monotonic() - settle_t0

            _, errs = arm_state(arm)
            bad = sorted((seen | set(errs or [])) - WARN_ONLY)
            if bad:
                print(f"  ** SEQUENCE PROGRAM NOT COMPLETE {index}/{total}: {codes_text(bad)}")
                return 1

            program_elapsed = time.monotonic() - t0
            completed += 1

            print(f"SEQUENCE PROGRAM COMPLETE {index}/{total}: {program_elapsed:.1f} s")
            print(f"SEQUENCE SETTLE {index}/{total}: {settle_elapsed:.3f} s")

        total_elapsed = time.monotonic() - sequence_t0
        bad_all = sorted(seen_all - WARN_ONLY)
        if bad_all:
            print(f"** SEQUENCE COMPLETE WITH ERRORS: {codes_text(bad_all)}")
            return 1

        print(f"SEQUENCE COMPLETE: {completed}/{total} programs, {total_elapsed:.1f} s")
        return 0

    except KeyboardInterrupt:
        print("\n** SEQUENCE STOP REQUESTED: stopping AUX/arm and cancelling remaining programs")

        if pole_dev is not None:
            try:
                print(f"  pole stop ret={pole_dev.stop()}")
            except Exception as exc:
                print(f"  pole stop failed: {exc}")

        if hand_dev is not None:
            try:
                frozen = hand_dev.freeze()
                print(f"  hand freeze ret={frozen}")
            except Exception as exc:
                print(f"  hand freeze failed: {exc}")

        try:
            print(f"  arm slow-stop ret={arm.rm_set_arm_slow_stop()}")
        finally:
            if active_started:
                try:
                    settle(arm)
                except KeyboardInterrupt:
                    pass
        return 1

    finally:
        if hand_dev is not None:
            try:
                hand_dev.close()
                print("AUX HAND CLOSED")
            except Exception as exc:
                print(f"  !! hand Modbus close failed: {exc}")

        r = arm.rm_set_plan_speed(a.restore_speed)
        print(
            f"  global speed restored to {a.restore_speed} % after sequence (ret={r})"
            + ("" if r == 0 else " — ** RESTORE FAILED, set it at the pendant")
        )



def manual_hand_control(arm, pct):
    """
    Move only the Inspire hand.

    pct:
      0   = fully closed for first four fingers
      100 = fully open for first four fingers

    The two thumb channels hold their current positions.
    """
    Hand, fingers_pct_counts, _Pole = load_aux_classes()
    hand_dev = Hand(arm)

    try:
        print(f"MANUAL HAND START: target {pct:g}% open")
        hand_dev.open()

        target = fingers_pct_counts(pct)
        ok, final_counts = hand_dev.command(target, wait=True)

        if not ok:
            print(f"** MANUAL HAND FAILED: timeout, final={final_counts}")
            return 1

        print(
            f"MANUAL HAND COMPLETE: target {pct:g}% open, "
            f"final={final_counts}"
        )
        return 0

    except KeyboardInterrupt:
        print("\n** MANUAL HAND STOP REQUESTED")
        try:
            ret = hand_dev.freeze()
            print(f"  hand frozen at current position (ret={ret})")
        except Exception as exc:
            print(f"  ** hand freeze failed: {exc}")
        return 1

    finally:
        try:
            hand_dev.close()
            print("MANUAL HAND: Modbus closed")
        except Exception as exc:
            print(f"  !! hand Modbus close failed: {exc}")



def manual_hand_raw_control(arm, values):
    """
    Move all six Inspire hand channels independently.

    Order:
      little, ring, middle, index, thumb_bend, thumb_rotate

    Each channel uses hand.py native units 0-1000.
    """
    Hand, _fingers_pct_counts, _Pole = load_aux_classes()
    hand_dev = Hand(arm)

    target = [int(v) for v in values]
    if len(target) != 6:
        print("** MANUAL HAND RAW FAILED: exactly 6 values are required")
        return 1

    if any(v < 0 or v > 1000 for v in target):
        print("** MANUAL HAND RAW FAILED: every value must be 0-1000")
        return 1

    try:
        print(
            "MANUAL HAND RAW START: "
            f"little={target[0]} ring={target[1]} middle={target[2]} "
            f"index={target[3]} thumb_bend={target[4]} "
            f"thumb_rotate={target[5]}"
        )

        hand_dev.open()
        ok, final_counts = hand_dev.command(target, wait=True)

        if not ok:
            print(
                f"** MANUAL HAND RAW FAILED: timeout, final={final_counts}"
            )
            return 1

        print(
            "MANUAL HAND RAW COMPLETE: "
            f"target={target}, final={final_counts}"
        )
        return 0

    except KeyboardInterrupt:
        print("\n** MANUAL HAND RAW STOP REQUESTED")
        try:
            ret = hand_dev.freeze()
            print(f"  hand frozen at current position (ret={ret})")
        except Exception as exc:
            print(f"  !! hand freeze failed: {exc}")
        return 130

    finally:
        try:
            hand_dev.close()
            print("AUX HAND CLOSED")
        except Exception as exc:
            print(f"  !! hand Modbus close failed: {exc}")


def manual_pole_control(arm, target_mm):
    """Move only the pole/lift to target_mm and wait until it arrives."""
    _Hand, _fingers_pct_counts, Pole = load_aux_classes()
    pole_dev = Pole(arm)

    try:
        current_m = pole_dev.height_m()
        if current_m is None:
            print(f"MANUAL POLE START: target {target_mm:g} mm; current unavailable")
        else:
            print(
                f"MANUAL POLE START: current {current_m * 1000.0:.1f} mm "
                f"-> target {target_mm:g} mm"
            )

        ok, final_m = pole_dev.move(
            float(target_mm) / 1000.0,
            speed_pct=100,
            block=False,
        )

        final_mm = None if final_m is None else final_m * 1000.0

        if not ok:
            print(
                "** MANUAL POLE FAILED: timeout"
                + ("" if final_mm is None else f", final={final_mm:.1f} mm")
            )
            return 1

        print(
            f"MANUAL POLE COMPLETE: target {target_mm:g} mm"
            + ("" if final_mm is None else f", final={final_mm:.1f} mm")
        )
        return 0

    except KeyboardInterrupt:
        print("\n** MANUAL POLE STOP REQUESTED")
        try:
            ret = pole_dev.stop()
            print(f"  pole stopped (ret={ret})")
        except Exception as exc:
            print(f"  ** pole stop failed: {exc}")
        return 1



def drag_teach_session(arm):
    """
    Start RealMan drag teaching with controller-side trajectory recording.

    This process intentionally stays alive while the user physically drags the
    arm. Ctrl+C / SIGINT is the normal STOP command from the UI; it calls
    rm_stop_drag_teach() before the connection is closed.
    """
    started = False
    try:
        print("DRAG TEACH STARTING: trajectory recording enabled")
        ret = arm.rm_start_drag_teach(1)
        if ret != 0:
            print(f"** DRAG TEACH FAILED TO START: ret={ret}")
            return 1

        started = True
        print("DRAG TEACH ACTIVE: arm is in drag mode; physically guide the arm now")
        print("DRAG TEACH ACTIVE: press the same UI button again to stop recording")

        while True:
            time.sleep(0.2)

    except KeyboardInterrupt:
        print("\nDRAG TEACH STOP REQUESTED")
        if started:
            ret = arm.rm_stop_drag_teach()
            started = False
            if ret != 0:
                print(f"** DRAG TEACH STOP FAILED: ret={ret}")
                return 1
            print("DRAG TEACH STOPPED: recorded trajectory retained in controller")
            return 0
        return 1

    finally:
        # Safety fallback if the function exits abnormally after drag mode started.
        if started:
            try:
                ret = arm.rm_stop_drag_teach()
                print(f"DRAG TEACH SAFETY STOP: ret={ret}")
            except Exception as exc:
                print(f"** DRAG TEACH SAFETY STOP FAILED: {exc}")


def drag_save_trajectory(arm, file_path):
    """Save the controller's latest drag-teach trajectory to a local text file."""
    path = pathlib.Path(file_path).expanduser().resolve()
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
    except Exception as exc:
        print(f"** DRAG SAVE FAILED: cannot create directory {path.parent}: {exc}")
        return 1

    print(f"DRAG SAVE START: {path}")
    try:
        result = arm.rm_save_trajectory(str(path))
    except Exception as exc:
        print(f"** DRAG SAVE FAILED: {exc}")
        return 1

    if isinstance(result, tuple):
        ret = int(result[0]) if result else -999
        point_count = result[1] if len(result) > 1 else None
    else:
        ret = int(result)
        point_count = None

    if ret != 0:
        print(f"** DRAG SAVE FAILED: rm_save_trajectory ret={ret}")
        return 1

    if point_count is None:
        print(f"DRAG SAVE COMPLETE: {path}")
    else:
        print(f"DRAG SAVE COMPLETE: {path} // {int(point_count)} points")
    return 0



def drag_program_name(requested, path):
    """
    Controller naming convention for Drag Teach programs.

    Prefix DT_ is intentional: rm_get_program_trajectory_list does not report
    how a program was created, so the UI uses this name prefix to label
    controller programs as DRAG TEACH.
    """
    raw = str(requested or "").strip()
    if not raw:
        raw = pathlib.Path(path).stem

    if raw.lower().endswith(".txt"):
        raw = raw[:-4]

    safe = "".join(ch if (ch.isalnum() or ch in "_-") else "_" for ch in raw)
    safe = safe.strip("_") or "trajectory"

    if not safe.upper().startswith("DT_"):
        safe = "DT_" + safe

    # Keep names compact for the controller list.
    safe = safe[:56]
    return safe + ".txt"


def upload_drag_txt_program(arm, a, ip, path, slot, requested_name=None, run_after=False):
    """
    Convert a raw rm_save_trajectory() .txt into an online-program file,
    save it in a controller Program ID, mark its name with DT_, and optionally run.

    RealMan's official workflow for Drag Teach -> controller program is:
      raw trajectory .txt -> add online-program header -> rm_send_project().
    """
    path = pathlib.Path(path).expanduser().resolve()

    if not path.is_file():
        print(f"** DRAG TXT NOT FOUND: {path}")
        return 1

    if not 1 <= int(slot) <= 100:
        print(f"** DRAG PROGRAM ID INVALID: {slot}; expected 1-100")
        return 1

    lines = read_track(path)
    controller_name = drag_program_name(requested_name, path)
    held = slots(arm)
    old = held.get(int(slot))

    with tempfile.TemporaryDirectory() as tmp:
        # Use the final desired controller name for the sent project filename.
        project = pathlib.Path(tmp) / controller_name
        sent = write_project(lines, project)

        print(
            f"DRAG CONTROLLER UPLOAD: {path.name} // {len(lines)} points "
            f"-> Program {slot} on {ip}"
        )
        print(f"DRAG CONTROLLER NAME: {controller_name}")
        print(
            f"  Program {slot} now: "
            + (
                f"{old[0]} ({old[1] / 1024:.1f} KB) — it will be REPLACED"
                if old else "empty"
            )
        )

        if not a.yes and input("  write it? [y/N] ").strip().lower() != "y":
            print("  not written")
            return 1

        # project_type=0 because this is now an online-program file with header.
        ret, err_line = arm.rm_send_project(
            rm_send_project_t(
                str(project),
                a.speed,
                1,             # only_save
                int(slot),
                0,             # step_flag
                0,             # auto_start
                0,             # project_type: online program
            )
        )

    if ret != 0 or err_line != -1:
        why = SEND_RET.get(ret, "") if ret != 0 else (
            "the data length check failed"
            if err_line == 0
            else f"the controller refused line {err_line}"
        )
        print(
            f"** DRAG CONTROLLER UPLOAD FAILED: ret={ret} "
            f"err_line={err_line} — {why}"
        )
        return 1

    got = slots(arm).get(int(slot))
    if got is None:
        print(
            f"** DRAG CONTROLLER UPLOAD NOT VERIFIED: "
            f"Program {slot} is missing from controller list"
        )
        return 1

    name, size, stored_speed = got
    if abs(size - sent) > max(64, sent // 1000):
        print(
            f"** DRAG CONTROLLER UPLOAD NOT VERIFIED: Program {slot} "
            f"holds {size} bytes, {sent} were sent"
        )
        return 1

    # Force a durable DT_ name if the controller did not keep the sent filename.
    if name != controller_name and hasattr(arm, "rm_update_program_trajectory"):
        try:
            speed_for_update = int(stored_speed or 0)
        except Exception:
            speed_for_update = 0
        if not 1 <= speed_for_update <= 100:
            speed_for_update = int(a.run_speed)

        rename_ret = arm.rm_update_program_trajectory(
            int(slot),
            speed_for_update,
            controller_name,
        )
        print(
            f"  controller rename -> {controller_name!r} "
            f"(ret={rename_ret})"
        )

        got = slots(arm).get(int(slot))
        if got:
            name, size, stored_speed = got

    print(
        f"DRAG CONTROLLER SAVE COMPLETE: Program {slot} // "
        f"{name or controller_name} // {size} bytes"
    )
    print(
        f"DRAG CONTROLLER PROGRAM: slot {int(slot)} name "
        f"{name or controller_name}"
    )

    if not run_after:
        return 0

    first_point = json.loads(lines[0])["point"]
    print(
        f"DRAG TXT RUN REQUEST: Program {slot} // "
        f"{name or controller_name}"
    )
    return run(
        arm,
        a,
        ip,
        int(slot),
        first_point=first_point,
    )


def drag_store_current_to_controller(arm, a, ip, file_path, slot, requested_name=None):
    """
    Save the controller's latest Drag Teach recording to local .txt, then convert
    and save that .txt into the controller's normal online-program list.
    """
    rc = drag_save_trajectory(arm, file_path)
    if rc:
        return rc

    return upload_drag_txt_program(
        arm,
        a,
        ip,
        file_path,
        slot,
        requested_name=requested_name,
        run_after=False,
    )


def drag_playback(arm):
    """
    Move to the recorded drag trajectory origin, then replay it.

    RealMan requires the arm to be at the drag trajectory origin before
    rm_run_drag_trajectory().
    """
    try:
        print("DRAG PLAYBACK PREP: moving arm to recorded trajectory origin")
        ret = arm.rm_drag_trajectory_origin(1)
        if ret != 0:
            print(f"** DRAG ORIGIN FAILED: ret={ret}")
            return 1

        print("DRAG ORIGIN COMPLETE: arm reached recorded start position")
        print("DRAG PLAYBACK START: replaying recorded drag trajectory")

        ret = arm.rm_run_drag_trajectory(1)
        if ret != 0:
            print(f"** DRAG PLAYBACK FAILED: ret={ret}")
            return 1

        print("DRAG PLAYBACK COMPLETE: recorded trajectory finished")
        return 0

    except KeyboardInterrupt:
        print("\nDRAG PLAYBACK STOP REQUESTED")
        try:
            ret = arm.rm_stop_drag_trajectory()
            print(f"DRAG PLAYBACK STOP: ret={ret}")
        except Exception as exc:
            print(f"** DRAG PLAYBACK STOP FAILED: {exc}")
        try:
            arm.rm_set_arm_slow_stop()
        except Exception:
            pass
        return 1



def get_arm_run_mode(arm):
    """Read current RealMan arm run mode. 0=simulation, 1=real."""
    try:
        ret, mode = arm.rm_get_arm_run_mode()
    except Exception as exc:
        print(f"** ARM MODE READ FAILED: {exc}")
        return 1
    if ret != 0:
        print(f"** ARM MODE READ FAILED: ret={ret}")
        return 1
    mode = int(mode)
    label = "SIMULATION" if mode == 0 else ("REAL" if mode == 1 else f"UNKNOWN({mode})")
    print(f"ARM RUN MODE: {label} // mode={mode}")
    return 0


def set_arm_run_mode(arm, mode):
    """Set RealMan arm run mode. 0=simulation, 1=real, then verify."""
    mode = int(mode)
    if mode not in (0, 1):
        print(f"** ARM MODE SET FAILED: unsupported mode {mode}")
        return 1
    wanted = "SIMULATION" if mode == 0 else "REAL"
    print(f"ARM MODE SET REQUEST: {wanted} // mode={mode}")
    try:
        ret = arm.rm_set_arm_run_mode(mode)
    except Exception as exc:
        print(f"** ARM MODE SET FAILED: {exc}")
        return 1
    if ret != 0:
        print(f"** ARM MODE SET FAILED: ret={ret}")
        return 1
    try:
        read_ret, actual = arm.rm_get_arm_run_mode()
    except Exception as exc:
        print(f"** ARM MODE VERIFY FAILED: {exc}")
        return 1
    if read_ret != 0:
        print(f"** ARM MODE VERIFY FAILED: ret={read_ret}")
        return 1
    actual = int(actual)
    actual_label = "SIMULATION" if actual == 0 else ("REAL" if actual == 1 else f"UNKNOWN({actual})")
    if actual != mode:
        print(f"** ARM MODE VERIFY FAILED: requested {wanted} ({mode}), controller reports {actual_label} ({actual})")
        return 1
    print(f"ARM MODE SET COMPLETE: {actual_label} // mode={actual}")
    return 0


# ---------------------------------------------------------------------------
# Global Waypoint control
# ---------------------------------------------------------------------------
GLOBAL_WAYPOINT_PAGE = 20


def _find_waypoint_list(data):
    if isinstance(data, list):
        if not data:
            return []
        if all(isinstance(x, dict) for x in data):
            if any("point_name" in x for x in data):
                return data

    if not isinstance(data, dict):
        return []

    for key in (
        "waypoint_list",
        "points_list",
        "point_list",
        "list",
        "global_waypoints",
        "waypoints",
    ):
        value = data.get(key)
        found = _find_waypoint_list(value)
        if found or value == []:
            return found

    for value in data.values():
        found = _find_waypoint_list(value)
        if found:
            return found

    return []


def _normalize_waypoint(raw):
    if not isinstance(raw, dict):
        raise ValueError("waypoint response is not a dictionary")

    if "point_name" not in raw:
        for value in raw.values():
            if isinstance(value, dict) and "point_name" in value:
                raw = value
                break

    return {
        "point_name": str(raw.get("point_name") or raw.get("name") or "").strip(),
        "joint": list(raw.get("joint") or []),
        "pose": list(raw.get("pose") or []),
        "work_frame": str(raw.get("work_frame") or raw.get("workframe") or "").strip(),
        "tool_frame": str(raw.get("tool_frame") or raw.get("toolframe") or "").strip(),
        "time": str(raw.get("time") or "").strip(),
    }


def global_waypoints(arm):
    result = []
    seen = set()

    for page in range(1, 101):
        ret, data = arm.rm_get_global_waypoints_list(
            page,
            GLOBAL_WAYPOINT_PAGE,
            "",
        )
        if ret != 0:
            raise RuntimeError(
                f"rm_get_global_waypoints_list page {page} failed, ret={ret}"
            )

        batch_raw = _find_waypoint_list(data)
        fresh = []

        for raw in batch_raw:
            try:
                point = _normalize_waypoint(raw)
            except Exception:
                continue

            name = point["point_name"]
            if not name or name in seen:
                continue

            seen.add(name)
            result.append(point)
            fresh.append(point)

        total = None
        if isinstance(data, dict):
            for key in ("total_size", "total", "total_num"):
                try:
                    if data.get(key) is not None:
                        total = int(data.get(key))
                        break
                except Exception:
                    pass

        if total is not None and len(result) >= total:
            break
        if len(batch_raw) < GLOBAL_WAYPOINT_PAGE:
            break
        if not fresh:
            break

    return result


def print_global_waypoints(arm, ip):
    points = global_waypoints(arm)
    print(f"GLOBAL POINT LIST: {len(points)} point(s) on {ip}")

    for point in points:
        print(
            "GLOBAL POINT: "
            + json.dumps(
                {
                    "point_name": point["point_name"],
                    "work_frame": point["work_frame"],
                    "tool_frame": point["tool_frame"],
                    "time": point["time"],
                },
                ensure_ascii=False,
            )
        )

    print(f"GLOBAL POINT LIST COMPLETE: {len(points)} point(s)")
    return 0


def get_global_waypoint(arm, point_name):
    ret, data = arm.rm_get_given_global_waypoint(str(point_name))
    if ret != 0:
        raise RuntimeError(
            f"global point {point_name!r} could not be read, ret={ret}"
        )

    point = _normalize_waypoint(data)
    if not point["point_name"]:
        point["point_name"] = str(point_name)

    if len(point["joint"]) < 7:
        raise RuntimeError(
            f"global point {point_name!r} has invalid joint data: "
            f"{point['joint']}"
        )

    if len(point["pose"]) < 6:
        raise RuntimeError(
            f"global point {point_name!r} has invalid pose data: "
            f"{point['pose']}"
        )

    return point


def _current_frame_name(frame_data):
    if not isinstance(frame_data, dict):
        return ""
    for key in ("frame_name", "name", "tool_name", "work_name"):
        value = frame_data.get(key)
        if value:
            return str(value)
    return ""


def _capture_current_frames(arm):
    work_name = ""
    tool_name = ""

    try:
        ret, data = arm.rm_get_current_work_frame()
        if ret == 0:
            work_name = _current_frame_name(data)
    except Exception:
        pass

    try:
        ret, data = arm.rm_get_current_tool_frame()
        if ret == 0:
            tool_name = _current_frame_name(data)
    except Exception:
        pass

    return work_name, tool_name


def _restore_frames(arm, work_name, tool_name):
    if work_name:
        try:
            ret = arm.rm_change_work_frame(work_name)
            print(f"  restore work frame {work_name!r}: ret={ret}")
        except Exception as exc:
            print(f"  !! restore work frame failed: {exc}")

    if tool_name:
        try:
            ret = arm.rm_change_tool_frame(tool_name)
            print(f"  restore tool frame {tool_name!r}: ret={ret}")
        except Exception as exc:
            print(f"  !! restore tool frame failed: {exc}")


def move_global_waypoint(
    arm,
    point_name,
    move_type="movej",
    speed=20,
    wait_s=0.0,
):
    move_type = str(move_type).strip().lower()
    if move_type not in ("movej", "movel"):
        raise ValueError("move type must be MoveJ or MoveL")

    speed = int(speed)
    if not 1 <= speed <= 100:
        raise ValueError("global point speed must be 1-100")

    wait_s = float(wait_s)
    if not 0 <= wait_s <= 3600:
        raise ValueError("global point wait must be 0-3600 seconds")

    point = get_global_waypoint(arm, point_name)
    original_work, original_tool = _capture_current_frames(arm)
    changed_frames = False

    try:
        print(
            f"GLOBAL MOVE START: {point['point_name']} // "
            f"{move_type.upper()} // speed {speed}%"
        )

        if move_type == "movej":
            target = [float(x) for x in point["joint"][:7]]
            ret = arm.rm_movej(target, speed, 0, 0, 1)

        else:
            if point["work_frame"]:
                frame_ret = arm.rm_change_work_frame(point["work_frame"])
                if frame_ret != 0:
                    raise RuntimeError(
                        f"cannot change to work frame "
                        f"{point['work_frame']!r}, ret={frame_ret}"
                    )
                print(f"  work frame -> {point['work_frame']}")
                changed_frames = True

            if point["tool_frame"]:
                frame_ret = arm.rm_change_tool_frame(point["tool_frame"])
                if frame_ret != 0:
                    raise RuntimeError(
                        f"cannot change to tool frame "
                        f"{point['tool_frame']!r}, ret={frame_ret}"
                    )
                print(f"  tool frame -> {point['tool_frame']}")
                changed_frames = True

            target = [float(x) for x in point["pose"][:6]]
            ret = arm.rm_movel(target, speed, 0, 0, 1)

        if ret != 0:
            raise RuntimeError(
                f"{move_type.upper()} to {point['point_name']!r} failed, ret={ret}"
            )

        print(
            f"GLOBAL MOVE COMPLETE: {point['point_name']} // "
            f"{move_type.upper()}"
        )

        if wait_s > 0:
            print(
                f"GLOBAL WAIT: {point['point_name']} // {wait_s:g} s"
            )
            time.sleep(wait_s)
            print(
                f"GLOBAL WAIT COMPLETE: {point['point_name']} // {wait_s:g} s"
            )

        return 0

    except KeyboardInterrupt:
        print(
            f"\nGLOBAL MOVE STOP REQUESTED: "
            f"{point['point_name']} // {move_type.upper()}"
        )
        try:
            print(f"  arm slow-stop ret={arm.rm_set_arm_slow_stop()}")
        except Exception as exc:
            print(f"  ** slow-stop failed: {exc}")
        return 1

    finally:
        if changed_frames:
            _restore_frames(arm, original_work, original_tool)


def load_global_sequence_file(path):
    data = json.loads(path.read_text(encoding="utf-8"))

    if not isinstance(data, dict):
        raise ValueError("global point sequence must be a JSON object")

    if data.get("format") != "realman_global_point_sequence":
        raise ValueError("not a RealMan global point sequence file")

    raw_steps = data.get("steps")
    if not isinstance(raw_steps, list) or not raw_steps:
        raise ValueError("global point sequence has no steps")

    steps = []
    for index, raw in enumerate(raw_steps, 1):
        if not isinstance(raw, dict):
            raise ValueError(f"step {index} is not an object")

        name = str(
            raw.get("name") or raw.get("point_name") or ""
        ).strip()
        if not name:
            raise ValueError(f"step {index} has no global point name")

        move_type = str(raw.get("move") or "movej").strip().lower()
        if move_type not in ("movej", "movel"):
            raise ValueError(
                f"step {index} motion must be MoveJ or MoveL"
            )

        speed = int(raw.get("speed", 20))
        if not 1 <= speed <= 100:
            raise ValueError(f"step {index} speed must be 1-100")

        wait_s = float(
            raw.get("wait_s", raw.get("wait", 0)) or 0
        )
        if not 0 <= wait_s <= 3600:
            raise ValueError(
                f"step {index} wait must be 0-3600 seconds"
            )

        steps.append(
            {
                "name": name,
                "move": move_type,
                "speed": speed,
                "wait_s": wait_s,
            }
        )

    return steps


def run_global_sequence(arm, steps):
    total = len(steps)
    print(f"GLOBAL SEQUENCE START: {total} point(s)")

    # Validate every controller point before moving.
    for index, step in enumerate(steps, 1):
        point = get_global_waypoint(arm, step["name"])
        print(
            f"  {index:02d}. {point['point_name']} // "
            f"{step['move'].upper()} // speed {step['speed']}% // "
            f"wait {step['wait_s']:g}s"
        )

    started = time.monotonic()

    try:
        for index, step in enumerate(steps, 1):
            print(
                f"GLOBAL SEQUENCE STEP {index}/{total}: "
                f"{step['name']} // {step['move'].upper()}"
            )

            rc = move_global_waypoint(
                arm,
                step["name"],
                step["move"],
                step["speed"],
                step["wait_s"],
            )
            if rc != 0:
                print(
                    f"** GLOBAL SEQUENCE FAILED: "
                    f"step {index}/{total} {step['name']}"
                )
                return 1

        elapsed = time.monotonic() - started
        print(
            f"GLOBAL SEQUENCE COMPLETE: "
            f"{total}/{total} point(s) // {elapsed:.1f} s"
        )
        return 0

    except KeyboardInterrupt:
        print("\nGLOBAL SEQUENCE STOP REQUESTED")
        try:
            print(f"  arm slow-stop ret={arm.rm_set_arm_slow_stop()}")
        except Exception as exc:
            print(f"  ** slow-stop failed: {exc}")
        return 1



def normalize_workflow_step(item):
    """Validate one Page-02 workflow step."""
    if not isinstance(item, dict):
        raise ValueError("each workflow step must be an object")

    kind = str(item.get("type") or "").strip().lower()

    if kind == "program":
        slot = int(item.get("id"))
        if not 1 <= slot <= 100:
            raise ValueError("program ID must be 1-100")

        # Stored controller programs play using their own recorded timing.
        # transit_speed controls ONLY the controller's move from the current
        # pose to this stored Program's first point.
        transit_speed = int(
            item.get("transit_speed", item.get("start_speed", 30))
        )
        if not 1 <= transit_speed <= 100:
            raise ValueError("program transit/start speed must be 1-100")

        # Keep the legacy field so old/new JSON remains compatible.
        legacy_speed = int(item.get("speed", 100))
        if not 1 <= legacy_speed <= 100:
            raise ValueError("legacy program speed must be 1-100")

        return {
            "type": "program",
            "id": slot,
            "name": str(item.get("name") or ""),
            "speed": legacy_speed,
            "transit_speed": transit_speed,
        }

    if kind == "pole":
        mm = float(item.get("mm"))
        if not 0 <= mm <= 315:
            raise ValueError("pole target must be 0-315 mm")
        return {"type": "pole", "mm": mm}

    if kind == "hand":
        values = item.get("values")
        if isinstance(values, str):
            values = [x.strip() for x in values.split(",")]
        if not isinstance(values, (list, tuple)) or len(values) != 6:
            raise ValueError("hand step requires exactly 6 values")
        values = [int(v) for v in values]
        if any(v < 0 or v > 1000 for v in values):
            raise ValueError("each hand channel must be 0-1000")
        return {"type": "hand", "values": values}

    if kind == "wait":
        seconds = float(item.get("seconds", item.get("wait_s", 0)))
        if not 0 <= seconds <= 3600:
            raise ValueError("wait must be 0-3600 seconds")
        return {"type": "wait", "seconds": seconds}

    if kind == "global":
        name = str(item.get("name") or "").strip()
        if not name:
            raise ValueError("global point name is required")
        move = str(item.get("move", "movej")).strip().lower()
        if move not in ("movej", "movel"):
            raise ValueError("global point move must be movej or movel")
        speed = int(item.get("speed", 20))
        if not 1 <= speed <= 100:
            raise ValueError("global point speed must be 1-100")
        wait_s = float(item.get("wait_s", 0))
        if not 0 <= wait_s <= 3600:
            raise ValueError("global point wait must be 0-3600 seconds")
        return {
            "type": "global",
            "name": name,
            "move": move,
            "speed": speed,
            "wait_s": wait_s,
        }

    raise ValueError(
        "workflow step type must be program, pole, hand, wait, or global"
    )


def load_workflow_sequence_file(path):
    data = json.loads(path.read_text(encoding="utf-8"))
    if (
        not isinstance(data, dict)
        or data.get("format") != "realman_workflow_sequence_v1"
    ):
        raise ValueError("not a RealMan workflow sequence JSON file")

    raw = data.get("steps")
    if not isinstance(raw, list) or not raw:
        raise ValueError("workflow sequence contains no steps")

    return [normalize_workflow_step(item) for item in raw]


def workflow_quick_settle(
    arm,
    still_deg_s=4.0,
    timeout_s=0.8,
    sample_s=0.05,
    calm_needed=2,
):
    """
    Short transition guard for Page-02 stored-program switching.

    The controller can report a program idle just before all joint motion is
    completely calm. This is intentionally much shorter than the legacy
    settle() while still checking joint stability before the next step.
    """
    deadline = time.monotonic() + timeout_s
    last = None
    calm = 0

    while time.monotonic() < deadline:
        joints, _ = arm_state(arm)
        if joints is None:
            return False

        if last is not None:
            speed = max(
                abs(p - q) for p, q in zip(joints, last)
            ) / max(sample_s, 1e-6)

            if speed < still_deg_s:
                calm += 1
                if calm >= calm_needed:
                    return True
            else:
                calm = 0

        last = joints
        time.sleep(sample_s)

    return False


def run_workflow_program_fast(
    arm,
    a,
    slot,
    program_name,
    transit_speed,
    index,
    total,
    transition_to_program,
):
    """
    Run one stored Program inside Page 02 without rebuilding a nested sequence.

    The parent workflow keeps the controller connection alive and restores the
    global speed only once at the end of the complete workflow.
    """
    joints, errs = arm_state(arm)
    if joints is None:
        print(
            f"** WORKFLOW PROGRAM FAILED {index}/{total}: "
            "arm state unreadable"
        )
        return 1

    blocking = sorted(set(errs or []) - WARN_ONLY)
    warnings = sorted(set(errs or []) & WARN_ONLY)

    if warnings:
        print(
            f"  !! workflow warning-only code before {index}/{total}: "
            f"{codes_text(warnings)} — continuing"
        )

    if blocking:
        print(
            f"** WORKFLOW PROGRAM FAILED {index}/{total}: "
            f"{codes_text(blocking)}"
        )
        return 1

    ret, state = arm.rm_get_program_run_state()
    if ret != 0 or state.get("run_state") in (1, 2):
        print(
            f"** WORKFLOW PROGRAM FAILED {index}/{total}: "
            f"controller not idle (slot {state.get('id')}, ret={ret})"
        )
        return 1

    overall_speed = int(a.workflow_overall_speed)

    ret_plan = arm.rm_set_plan_speed(overall_speed)
    if ret_plan != 0:
        print(
            f"** WORKFLOW PROGRAM FAILED {index}/{total}: "
            f"could not set overall speed {overall_speed}% "
            f"(ret={ret_plan})"
        )
        return 1

    print(
        f"WORKFLOW PROGRAM {index}/{total}: "
        f"slot {slot} ({program_name}) // START/TRANSIT {transit_speed}% "
        f"// stored trajectory timing // overall {overall_speed}%"
    )

    command_t0 = time.monotonic()
    ret = arm.rm_set_program_id_run(slot, int(transit_speed), 0)
    if ret != 0:
        print(
            f"** WORKFLOW PROGRAM NOT STARTED {index}/{total}: "
            f"rm_set_program_id_run ret={ret}"
        )
        return 1

    started = False
    ended = False
    seen = set()
    t0 = time.monotonic()
    next_error_check = t0

    while True:
        now = time.monotonic()

        ret_state, state = arm.rm_get_program_run_state()
        run_state = (
            state.get("run_state")
            if ret_state == 0
            else None
        )

        if run_state in (1, 2) and not started:
            started = True
            print(
                f"WORKFLOW PROGRAM RUNNING {index}/{total}: "
                f"slot {slot} // start ack "
                f"{now - command_t0:.3f} s"
            )

        if started and run_state == 0:
            ended = True
            break

        if not started and now - t0 > 5.0:
            print(
                f"** WORKFLOW PROGRAM {slot} did not start within 5 s"
            )
            break

        if now - t0 > a.timeout:
            print(
                f"** WORKFLOW PROGRAM {slot} still running after "
                f"{a.timeout:g} s — slow-stopping"
            )
            try:
                arm.rm_set_arm_slow_stop()
            except Exception:
                pass
            return 1

        if now >= next_error_check:
            next_error_check = now + 0.5
            _, current_errs = arm_state(arm)

            for code in current_errs or []:
                if code not in seen:
                    seen.add(code)
                    print(
                        f"  !! arm reports {codes_text([code])} "
                        f"at {now - t0:.1f} s"
                    )

        time.sleep(0.05)

    if not ended:
        print(
            f"** WORKFLOW PROGRAM NOT COMPLETE {index}/{total}: "
            f"slot {slot}"
        )
        return 1

    # Short transition stabilization. Consecutive Program -> Program gets the
    # shortest guard; a following Hand/Pole/Global step gets a little longer.
    settle_timeout = 0.60 if transition_to_program else 1.00
    settle_t0 = time.monotonic()

    calm = workflow_quick_settle(
        arm,
        timeout_s=settle_timeout,
        sample_s=0.05,
        calm_needed=2,
    )

    settle_elapsed = time.monotonic() - settle_t0

    _, final_errs = arm_state(arm)
    bad = sorted((seen | set(final_errs or [])) - WARN_ONLY)

    if bad:
        print(
            f"** WORKFLOW PROGRAM NOT COMPLETE {index}/{total}: "
            f"{codes_text(bad)}"
        )
        return 1

    elapsed = time.monotonic() - t0

    print(
        f"WORKFLOW PROGRAM COMPLETE {index}/{total}: "
        f"{elapsed:.3f} s"
    )
    print(
        f"WORKFLOW FAST SWITCH {index}/{total}: "
        f"{settle_elapsed:.3f} s"
        + ("" if calm else " // controller idle confirmed")
    )

    return 0


def run_workflow_sequence(arm, a, ip, steps):
    """
    Run Page-02 Program / Pole / Hand / Wait / Global Point steps through one
    persistent controller connection.

    Program-step optimization:
      - controller program list is read once,
      - no nested one-program run_sequence() per file,
      - no speed restore between Program files,
      - short stability check between files,
      - speed restore happens once after the full workflow.
    """
    total = len(steps)

    print(
        f"WORKFLOW START: {total} step(s) on {ip}; "
        f"one persistent controller connection // FAST PROGRAM SWITCH "
        f"// overall {a.workflow_overall_speed}%"
    )
    print(
        "WORKFLOW START MOTION: each stored Program uses its own "
        "start/transit speed to reach the first point; trajectory playback "
        "then follows the stored controller program timing."
    )

    program_slots = [
        int(step["id"])
        for step in steps
        if step["type"] == "program"
    ]

    held_programs = {}

    if program_slots:
        held_programs = slots(arm)

        missing = [
            slot
            for slot in program_slots
            if slot not in held_programs
        ]

        if missing:
            print(
                "** WORKFLOW REFUSED: empty/missing controller slot(s): "
                + ", ".join(str(x) for x in sorted(set(missing)))
            )
            return 1

    joints, errs = arm_state(arm)

    if joints is None:
        print("** WORKFLOW REFUSED: arm state unreadable")
        return 1

    blocking = sorted(set(errs or []) - WARN_ONLY)
    warnings = sorted(set(errs or []) & WARN_ONLY)

    if warnings:
        print(
            "  !! workflow warning-only code(s): "
            + codes_text(warnings)
            + " — continuing"
        )

    if blocking:
        print(
            "** WORKFLOW REFUSED: "
            + codes_text(blocking)
        )
        return 1

    ret, state = arm.rm_get_program_run_state()

    if ret != 0 or state.get("run_state") in (1, 2):
        print(
            "** WORKFLOW REFUSED: controller not idle "
            f"(slot {state.get('id')}, ret={ret})"
        )
        return 1

    for index, step in enumerate(steps, 1):
        kind = step["type"].upper()

        if step["type"] == "program":
            held = held_programs.get(
                int(step["id"]),
                ("", 0, None),
            )
            name = held[0] or step.get("name") or ""

            detail = (
                f"Program {step['id']} {name} "
                f"// start/transit {step['transit_speed']}% "
                f"// stored trajectory timing"
            )

        elif step["type"] == "pole":
            detail = f"{step['mm']:g} mm"

        elif step["type"] == "hand":
            detail = ",".join(
                str(v) for v in step["values"]
            )

        elif step["type"] == "wait":
            detail = f"{step['seconds']:g} s"

        else:
            detail = (
                f"{step['name']} // {step['move'].upper()} // "
                f"speed {step['speed']}% // "
                f"wait {step['wait_s']:g}s"
            )

        print(f"  {index:02d}. {kind}: {detail}")

    want = a.side or "run"

    if not a.yes:
        typed = input(
            f"  type '{want}' to start workflow "
            "(Ctrl+C stops/cancels): "
        ).strip().lower()

        if typed != want:
            print("  workflow not started")
            return 1

    overall_speed = int(a.workflow_overall_speed)

    try:
        ret = arm.rm_set_plan_speed(overall_speed)
        print(
            f"WORKFLOW OVERALL SPEED: {overall_speed}% // ret={ret}"
        )
        if ret != 0:
            print(
                f"** WORKFLOW REFUSED: overall speed set failed "
                f"(ret={ret})"
            )
            return 1
    except Exception as exc:
        print(
            f"** WORKFLOW REFUSED: overall speed set failed: {exc}"
        )
        return 1

    started_at = time.monotonic()
    completed = 0

    try:
        for index, step in enumerate(steps, 1):
            kind = step["type"]

            print(
                f"WORKFLOW STEP START {index}/{total}: "
                f"{kind.upper()}"
            )

            if kind == "program":
                slot = int(step["id"])
                stored = held_programs.get(
                    slot,
                    ("", 0, None),
                )
                name = (
                    stored[0]
                    or step.get("name")
                    or ""
                )

                next_is_program = (
                    index < total
                    and steps[index]["type"] == "program"
                )

                rc = run_workflow_program_fast(
                    arm=arm,
                    a=a,
                    slot=slot,
                    program_name=name,
                    transit_speed=int(step["transit_speed"]),
                    index=index,
                    total=total,
                    transition_to_program=next_is_program,
                )

            elif kind == "pole":
                rc = manual_pole_control(
                    arm,
                    float(step["mm"]),
                )

            elif kind == "hand":
                rc = manual_hand_raw_control(
                    arm,
                    step["values"],
                )

            elif kind == "wait":
                seconds = float(step["seconds"])

                print(
                    f"WORKFLOW WAIT: {seconds:g} s"
                )

                deadline = time.monotonic() + seconds

                while True:
                    remaining = deadline - time.monotonic()

                    if remaining <= 0:
                        break

                    time.sleep(
                        min(0.05, remaining)
                    )

                print(
                    f"WORKFLOW WAIT COMPLETE: {seconds:g} s"
                )
                rc = 0

            elif kind == "global":
                rc = move_global_waypoint(
                    arm,
                    step["name"],
                    step["move"],
                    step["speed"],
                    step["wait_s"],
                )

            else:
                print(
                    f"** WORKFLOW FAILED: unsupported step {kind}"
                )
                return 1

            if rc != 0:
                print(
                    f"** WORKFLOW FAILED: step {index}/{total} "
                    f"({kind}) returned {rc}"
                )
                return 1

            completed += 1

            print(
                f"WORKFLOW STEP COMPLETE {index}/{total}: "
                f"{kind.upper()}"
            )

        elapsed = time.monotonic() - started_at

        print(
            f"WORKFLOW COMPLETE: {completed}/{total} step(s), "
            f"{elapsed:.1f} s"
        )

        return 0

    except KeyboardInterrupt:
        print("\n** WORKFLOW STOP REQUESTED")

        try:
            print(
                f"  arm slow-stop ret="
                f"{arm.rm_set_arm_slow_stop()}"
            )
        except Exception as exc:
            print(
                f"  ** arm slow-stop failed: {exc}"
            )

        return 1

    finally:
        try:
            ret = arm.rm_set_plan_speed(
                a.restore_speed
            )

            print(
                f"WORKFLOW SPEED RESTORE: "
                f"{a.restore_speed}% // ret={ret}"
            )

        except Exception as exc:
            print(
                f"  !! workflow speed restore failed: {exc}"
            )


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
    ap.add_argument("--sequence", nargs="+", type=int, metavar="SLOT",
                    help="run multiple stored slots sequentially using one persistent controller connection")
    ap.add_argument("--sequence-file", type=pathlib.Path,
                    help="JSON sequence containing Program IDs plus optional hand/pole targets")
    ap.add_argument("--manual-hand", type=float, metavar="PCT",
                    help="move only the hand: 0=closed, 100=fully open (first four fingers)")
    ap.add_argument("--manual-hand-raw", nargs=6, type=int,
                    metavar=("LITTLE", "RING", "MIDDLE", "INDEX", "THUMB_BEND", "THUMB_ROTATE"),
                    help="move all six hand channels independently using native 0-1000 values")
    ap.add_argument("--manual-pole", type=float, metavar="MM",
                    help="move only the pole/lift to 0-315 mm")
    ap.add_argument("--drag-session", action="store_true",
                    help="start drag teaching with recording; Ctrl+C stops the drag session")
    ap.add_argument("--drag-save", type=pathlib.Path, metavar="FILE",
                    help="save the latest drag-teach trajectory to FILE")
    ap.add_argument("--drag-playback", action="store_true",
                    help="move to the latest drag trajectory origin and replay it")
    ap.add_argument("--drag-store", type=pathlib.Path, metavar="FILE",
                    help="save latest Drag Teach recording to FILE and also store it in controller")
    ap.add_argument("--drag-run-file", type=pathlib.Path, metavar="FILE",
                    help="upload an existing Drag Teach .txt to controller and run it")
    ap.add_argument("--drag-slot", type=int, metavar="ID",
                    help="controller Program ID 1-100 used by --drag-store/--drag-run-file")
    ap.add_argument("--drag-name", type=str, metavar="NAME",
                    help="controller name for Drag Teach program; DT_ prefix is added automatically")
    ap.add_argument("--get-arm-mode", action="store_true",
                    help="read controller run mode: SIMULATION or REAL")
    ap.add_argument("--set-arm-mode", choices=("simulation", "real"),
                    help="set controller run mode: simulation=0, real=1")
    ap.add_argument("--global-list", action="store_true",
                    help="list global waypoints stored in this arm controller")
    ap.add_argument("--global-point", type=str, metavar="NAME",
                    help="run one controller global waypoint by name")
    ap.add_argument("--global-move", choices=("movej", "movel"), default="movej",
                    help="motion for --global-point: movej or movel")
    ap.add_argument("--global-speed", type=int, default=20,
                    help="global waypoint speed ratio 1-100")
    ap.add_argument("--global-wait", type=float, default=0.0,
                    help="seconds to wait after reaching --global-point")
    ap.add_argument("--global-sequence-file", type=pathlib.Path,
                    help="JSON ordered global-point sequence")
    ap.add_argument("--workflow-sequence-file", type=pathlib.Path,
                    help="JSON Page-02 workflow containing program/pole/hand/wait/global steps")
    ap.add_argument("--workflow-overall-speed", type=int, default=100,
                    help="Page-02 overall/global controller speed 1-100 (default 100)")
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
    manual_actions = (
        int(a.manual_hand is not None)
        + int(a.manual_hand_raw is not None)
        + int(a.manual_pole is not None)
    )
    if manual_actions > 1:
        ap.error(
            "use only one manual action at a time: "
            "--manual-hand OR --manual-hand-raw OR --manual-pole"
        )

    drag_actions = (
        int(bool(a.drag_session))
        + int(a.drag_save is not None)
        + int(bool(a.drag_playback))
        + int(a.drag_store is not None)
        + int(a.drag_run_file is not None)
    )
    if drag_actions > 1:
        ap.error(
            "use only one drag action at a time: --drag-session, --drag-save, "
            "--drag-playback, --drag-store, or --drag-run-file"
        )

    if a.drag_store is not None or a.drag_run_file is not None:
        if a.drag_slot is None:
            ap.error("--drag-store/--drag-run-file requires --drag-slot ID")
        if not 1 <= a.drag_slot <= 100:
            ap.error("--drag-slot is 1-100")

    mode_actions = int(bool(a.get_arm_mode)) + int(a.set_arm_mode is not None)
    if mode_actions > 1:
        ap.error("use --get-arm-mode OR --set-arm-mode, not both")

    global_actions = (
        int(bool(a.global_list))
        + int(a.global_point is not None)
        + int(a.global_sequence_file is not None)
    )
    if global_actions > 1:
        ap.error(
            "use one global-point action at a time: "
            "--global-list, --global-point, or --global-sequence-file"
        )

    if not 1 <= a.global_speed <= 100:
        ap.error("--global-speed is 1-100")

    if not 1 <= a.workflow_overall_speed <= 100:
        ap.error("--workflow-overall-speed is 1-100")

    if not 0 <= a.global_wait <= 3600:
        ap.error("--global-wait is 0-3600 seconds")

    global_sequence = None
    if a.global_sequence_file is not None:
        if not a.global_sequence_file.is_file():
            ap.error(
                f"global sequence file does not exist: "
                f"{a.global_sequence_file}"
            )
        try:
            global_sequence = load_global_sequence_file(
                a.global_sequence_file
            )
        except Exception as exc:
            ap.error(f"invalid global sequence: {exc}")

    workflow_sequence = None
    if a.workflow_sequence_file is not None:
        if not a.workflow_sequence_file.is_file():
            ap.error(
                f"workflow sequence file does not exist: "
                f"{a.workflow_sequence_file}"
            )
        try:
            workflow_sequence = load_workflow_sequence_file(
                a.workflow_sequence_file
            )
        except Exception as exc:
            ap.error(f"invalid workflow sequence: {exc}")

    if a.manual_hand is not None and not 0 <= a.manual_hand <= 100:
        ap.error("--manual-hand is 0-100 percent")

    if a.manual_hand_raw is not None:
        if any(v < 0 or v > 1000 for v in a.manual_hand_raw):
            ap.error("--manual-hand-raw values must each be 0-1000")

    if a.manual_pole is not None and not 0 <= a.manual_pole <= 315:
        ap.error("--manual-pole is 0-315 mm")

    if a.sequence and a.sequence_file:
        ap.error("use --sequence OR --sequence-file, not both")

    sequence = list(a.sequence or [])

    if a.sequence_file is not None:
        if not a.sequence_file.is_file():
            ap.error(f"sequence file does not exist: {a.sequence_file}")
        try:
            sequence = load_sequence_file(a.sequence_file)
        except Exception as exc:
            ap.error(f"invalid sequence file: {exc}")

    if sequence:
        if (
            a.track is not None
            or a.run is not None
            or a.list
            or a.manual_hand is not None
            or a.manual_hand_raw is not None
            or a.manual_pole is not None
            or drag_actions
            or mode_actions
            or global_actions
        ):
            ap.error(
                "--sequence/--sequence-file cannot be combined with upload TRACK/SLOT, "
                "--run, --list, manual Hand/Pole, or Drag Teach actions"
            )

        if a.sequence:
            if any(not 1 <= slot <= 100 for slot in sequence):
                ap.error("--sequence slot IDs must all be 1-100")

        if not 1 <= a.run_speed <= 100 or not 1 <= a.restore_speed <= 100:
            ap.error("--run-speed and --restore-speed are 1-100")

    if manual_actions:
        if (
            a.track is not None
            or a.run is not None
            or a.list
            or a.sequence
            or a.sequence_file is not None
            or drag_actions
            or mode_actions
            or global_actions
        ):
            ap.error(
                "manual Hand/Pole control cannot be combined with upload, run, list, "
                "sequence, Drag Teach, arm mode, or global-point actions"
            )

    if drag_actions:
        if (
            a.track is not None
            or a.run is not None
            or a.list
            or a.sequence
            or a.sequence_file is not None
            or manual_actions
            or mode_actions
            or global_actions
        ):
            ap.error(
                "Drag Teach actions cannot be combined with upload, run, list, sequence, "
                "manual Hand/Pole, arm mode, or global-point actions"
            )

    if mode_actions:
        if (
            a.track is not None
            or a.run is not None
            or a.list
            or a.sequence
            or a.sequence_file is not None
            or manual_actions
            or drag_actions
            or global_actions
        ):
            ap.error(
                "Arm mode control cannot be combined with upload, run, list, sequence, "
                "manual Hand/Pole, Drag Teach, or global-point actions"
            )

    if global_actions:
        if (
            a.track is not None
            or a.run is not None
            or a.list
            or a.sequence
            or a.sequence_file is not None
            or manual_actions
            or drag_actions
            or mode_actions
        ):
            ap.error(
                "Global-point control cannot be combined with upload, run, list, "
                "program sequence, manual Hand/Pole, Drag Teach, or arm mode"
            )

    if workflow_sequence is not None:
        if (
            a.track is not None
            or a.run is not None
            or a.list
            or bool(a.sequence)
            or a.sequence_file is not None
            or manual_actions
            or drag_actions
            or mode_actions
            or global_actions
        ):
            ap.error(
                "--workflow-sequence-file cannot be combined with upload, run, "
                "list, other sequences, manual Hand/Pole, Drag Teach, arm mode, "
                "or global-point actions"
            )

    if (
        lines is None
        and run_slot is None
        and not sequence
        and not a.list
        and a.manual_hand is None
        and a.manual_hand_raw is None
        and a.manual_pole is None
        and not drag_actions
        and not mode_actions
        and not global_actions
        and workflow_sequence is None
    ):
        ap.error(
            "give TRACK.txt SLOT to upload, --run SLOT, "
            "--sequence SLOT [SLOT ...], --sequence-file FILE.json, "
            "--manual-hand PCT, --manual-hand-raw 6xVALUE, --manual-pole MM, "
            "--drag-session, --drag-save FILE, --drag-playback, "
            "--drag-store FILE --drag-slot ID, "
            "--drag-run-file FILE --drag-slot ID, "
            "--get-arm-mode, --set-arm-mode simulation|real, "
            "--global-list, --global-point NAME, "
            "--global-sequence-file FILE, --workflow-sequence-file FILE, or --list"
        )

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

        if a.get_arm_mode:
            return get_arm_run_mode(arm)

        if a.set_arm_mode is not None:
            return set_arm_run_mode(arm, 0 if a.set_arm_mode == "simulation" else 1)

        if workflow_sequence is not None:
            return run_workflow_sequence(
                arm,
                a,
                ip,
                workflow_sequence,
            )

        if a.global_list:
            return print_global_waypoints(arm, ip)

        if a.global_point is not None:
            return move_global_waypoint(
                arm,
                a.global_point,
                a.global_move,
                a.global_speed,
                a.global_wait,
            )

        if global_sequence is not None:
            return run_global_sequence(arm, global_sequence)

        if a.manual_hand is not None:
            return manual_hand_control(arm, float(a.manual_hand))

        if a.manual_hand_raw is not None:
            return manual_hand_raw_control(arm, a.manual_hand_raw)

        if a.manual_pole is not None:
            return manual_pole_control(arm, float(a.manual_pole))

        if a.drag_session:
            return drag_teach_session(arm)

        if a.drag_save is not None:
            return drag_save_trajectory(arm, a.drag_save)

        if a.drag_playback:
            return drag_playback(arm)

        if a.drag_store is not None:
            return drag_store_current_to_controller(
                arm,
                a,
                ip,
                a.drag_store,
                a.drag_slot,
                requested_name=a.drag_name,
            )

        if a.drag_run_file is not None:
            return upload_drag_txt_program(
                arm,
                a,
                ip,
                a.drag_run_file,
                a.drag_slot,
                requested_name=a.drag_name,
                run_after=True,
            )

        if lines is not None and upload(arm, a, ip, lines):
            return 1
        if run_slot is not None:
            first = json.loads(lines[0])["point"] if lines is not None and run_slot == a.slot else None
            return run(arm, a, ip, run_slot, first)
        if sequence:
            return run_sequence(arm, a, ip, sequence)
        return 0
    finally:
        arm.rm_delete_robot_arm()


if __name__ == "__main__":
    sys.exit(main())
