#!/usr/bin/env python3
"""Pole and hand from one file. Needs pole.py and hand.py next to it, and the RealMan SDK
(Robotic_Arm) on the path.

    python usage.py pole read                     # pole height, mm
    python usage.py pole move 200                 # pole to 200 mm (arm must be still)
    python usage.py pole move 140 --speed 50      # slower
    python usage.py pole stop

    python usage.py hand read                     # six counts, 0 = closed, 1000 = open
    python usage.py hand open                     # four fingers open, thumb held
    python usage.py hand open --thumb             # everything open
    python usage.py hand close                    # four fingers closed, thumb held
    python usage.py hand close --thumb            # everything closed (try --speed 300 first)
    python usage.py hand pct 50                   # four fingers 50 % open, thumb held
    python usage.py hand set 0 0 0 1000 - -       # per channel; '-' = hold that channel
    python usage.py hand freeze                   # stop where it is

    python usage.py prestroke --pole 200 --hand 50   # the flow's order: open, pole, hand
    python usage.py stop                          # arm, pole and hand together

    --ip 192.168.1.10 --port 8080                 # which arm (default: left)

Hand channel order: little, ring, middle, index, thumb_bend, thumb_rot.
Rules: the pole moves only with the arm still; never move pole or hand with the pad on a surface.
"""
import argparse
import sys
import time

from Robotic_Arm.rm_robot_interface import RoboticArm, rm_thread_mode_e

from hand import Hand, fingers_pct_counts
from pole import Pole


# ---------------------------------------------------------------- connection
def connect(ip, port):
    robot = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
    robot.rm_create_robot_arm(ip, port)
    return robot


def arm_at_rest(robot, still_deg_s=1.6, need=6, poll_s=0.025, timeout_s=5.0):
    """True once every joint moves < still_deg_s for `need` polls in a row."""
    t0, prev, quiet = time.monotonic(), None, 0
    while time.monotonic() - t0 < timeout_s:
        ret, st = robot.rm_get_current_arm_state()
        q = st.get("joint") if ret == 0 and isinstance(st, dict) else None
        if q is not None and prev is not None:
            fast = max(abs(a - b) for a, b in zip(q, prev)) / poll_s
            quiet = quiet + 1 if fast < still_deg_s else 0
            if quiet >= need:
                return True
        prev = q
        time.sleep(poll_s)
    return False


# ---------------------------------------------------------------- pole
def pole_read(robot):
    h = Pole(robot).height_m()
    print("pole: unreadable" if h is None else f"pole: {h * 1000:.0f} mm")


def pole_move(robot, mm, speed=100, block=False):
    if not arm_at_rest(robot):
        sys.exit("arm is moving: refusing to move the pole")
    ok, h = Pole(robot).move(mm / 1000.0, speed_pct=speed, block=block)
    at = "?" if h is None else f"{h * 1000:.0f} mm"
    print(f"pole: {'arrived' if ok else 'DID NOT ARRIVE'} at {at}")
    return ok


def pole_stop(robot):
    print(f"pole: stop ret={Pole(robot).stop()}")


# ---------------------------------------------------------------- hand
def with_hand(robot, fn, speed=1000):
    """Open the modbus session, run fn(hand), always close the session."""
    hand = Hand(robot)
    hand.open(speed_set=speed)
    try:
        return fn(hand)
    finally:
        hand.close()                     # ends the modbus session; does NOT move the fingers


def report(ok, now):
    print(f"hand: {'ok' if ok else 'TIMEOUT'} at {now}")
    return ok


def hand_read(robot):
    with_hand(robot, lambda h: print(f"hand: {h.read()}"))


def hand_open(robot, thumb=False, speed=1000):
    counts = [1000] * 6 if thumb else fingers_pct_counts(100)
    return with_hand(robot, lambda h: report(*h.command(counts)), speed)


def hand_close(robot, thumb=False, speed=1000):
    # a grip stops early on force: ok can mean "settled", not "reached 0"
    counts = [0] * 6 if thumb else fingers_pct_counts(0)
    return with_hand(robot, lambda h: report(*h.command(counts)), speed)


def hand_pct(robot, pct, speed=1000):
    if not 0 <= pct <= 100:
        sys.exit("pct is 0-100")
    return with_hand(robot, lambda h: report(*h.command(fingers_pct_counts(pct))), speed)


def hand_set(robot, values, speed=1000):
    if len(values) != 6:
        sys.exit("give six values (0-1000 or '-')")
    counts = [None if v == "-" else int(v) for v in values]
    return with_hand(robot, lambda h: report(*h.command(counts)), speed)


def hand_freeze(robot):
    with_hand(robot, lambda h: print(f"hand: freeze ret={h.freeze()}"))


# ---------------------------------------------------------------- together
def prestroke(robot, pole_mm, hand_pct_open, block=False):
    """The flow's order: hand open -> arm still -> pole -> hand to the taught state."""
    def run(hand):
        if not report(*hand.command(fingers_pct_counts(100))):
            sys.exit("hand did not open")
        if not pole_move(robot, pole_mm, block=block):
            sys.exit("pole did not arrive")
        if not report(*hand.command(fingers_pct_counts(hand_pct_open))):
            sys.exit("hand did not settle")
        print("ready for the stroke")
    with_hand(robot, run)


def stop_all(robot):
    robot.rm_set_arm_slow_stop()
    Pole(robot).stop()
    hand = Hand(robot)
    hand.open()
    try:
        hand.freeze()
    finally:
        hand.close()
    print("stopped: arm, pole, hand")


# ---------------------------------------------------------------- command line
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ip", default="192.168.1.10")
    ap.add_argument("--port", type=int, default=8080)
    sub = ap.add_subparsers(dest="device", required=True)

    p = sub.add_parser("pole").add_subparsers(dest="action", required=True)
    p.add_parser("read")
    m = p.add_parser("move")
    m.add_argument("mm", type=float)
    m.add_argument("--speed", type=int, default=100)
    m.add_argument("--block", action="store_true", help="use before a blocking arm call")
    p.add_parser("stop")

    h = sub.add_parser("hand").add_subparsers(dest="action", required=True)
    h.add_parser("read")
    h.add_parser("freeze")
    for name in ("open", "close"):
        x = h.add_parser(name)
        x.add_argument("--thumb", action="store_true", help="include the thumb")
        x.add_argument("--speed", type=int, default=1000)
    x = h.add_parser("pct")
    x.add_argument("pct", type=float)
    x.add_argument("--speed", type=int, default=1000)
    x = h.add_parser("set")
    x.add_argument("values", nargs=6)
    x.add_argument("--speed", type=int, default=1000)

    ps = sub.add_parser("prestroke")
    ps.add_argument("--pole", type=float, required=True, help="mm")
    ps.add_argument("--hand", type=float, required=True, help="% open, four fingers")
    ps.add_argument("--block", action="store_true")

    sub.add_parser("stop")

    a = ap.parse_args()
    robot = connect(a.ip, a.port)
    try:
        if a.device == "pole":
            {"read": lambda: pole_read(robot),
             "move": lambda: pole_move(robot, a.mm, a.speed, a.block),
             "stop": lambda: pole_stop(robot)}[a.action]()
        elif a.device == "hand":
            {"read": lambda: hand_read(robot),
             "freeze": lambda: hand_freeze(robot),
             "open": lambda: hand_open(robot, a.thumb, a.speed),
             "close": lambda: hand_close(robot, a.thumb, a.speed),
             "pct": lambda: hand_pct(robot, a.pct, a.speed),
             "set": lambda: hand_set(robot, a.values, a.speed)}[a.action]()
        elif a.device == "prestroke":
            prestroke(robot, a.pole, a.hand, a.block)
        elif a.device == "stop":
            stop_all(robot)
    except KeyboardInterrupt:
        stop_all(robot)
        raise
    finally:
        robot.rm_delete_robot_arm()


if __name__ == "__main__":
    main()
