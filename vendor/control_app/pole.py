import time

LIFT_TOL_M = 0.005          # pole controller's goal tolerance (5 mm)
LIFT_STROKE_M = 0.315       # full stroke
LIFT_SLEW_UP_MPS = 0.059    # measured on this robot at 100 % (vendor's 0.185 is never reached)
LIFT_SLEW_DOWN_MPS = 0.097
LIFT_CREEP_S = 1.5          # slow final approach: the last mm can take ~1 s
LIFT_TIMEOUT_MARGIN = 3.0


def lift_travel_s(dist_m, up=None, speed_pct=100):
    """Expected pole travel time; unknown direction uses the slower (up) slew."""
    slew = (LIFT_SLEW_DOWN_MPS if up is False else LIFT_SLEW_UP_MPS) * max(1, min(100, speed_pct)) / 100
    return LIFT_CREEP_S + abs(dist_m) / slew


class Pole:
    def __init__(self, robot, poll_s=0.05):
        self.robot = robot          # a connected Robotic_Arm.rm_robot_interface.RoboticArm
        self.poll_s = poll_s

    def height_m(self):
        ret, st = self.robot.rm_get_lift_state()
        return float(st["pos"]) / 1000.0 if ret == 0 and st and "pos" in st else None

    def stop(self):
        return self.robot.rm_set_lift_speed(0)

    def move(self, height_m, speed_pct=100, tol_m=LIFT_TOL_M, block=False):
        """Command height_m and wait until the pole arrives within tol_m. Returns (ok, final_m)."""
        start = self.height_m()
        dist = abs(height_m - start) if start is not None else LIFT_STROKE_M
        up = None if start is None else height_m > start
        expected = lift_travel_s(dist, up, speed_pct)
        timeout = LIFT_TIMEOUT_MARGIN * expected
        if start is not None and dist <= tol_m:
            return True, start                                  # already there

        ret = self.robot.rm_set_lift_height(int(speed_pct), int(round(height_m * 1000)), 1 if block else 0)
        if ret != 0:
            raise RuntimeError(f"rm_set_lift_height refused, ret={ret}")

        t0, pos = time.monotonic(), start
        while True:
            elapsed = time.monotonic() - t0
            ret, st = self.robot.rm_get_lift_state()
            if ret == 0 and st:
                pos = float(st["pos"]) / 1000.0
                if int(st.get("err_flag", 0) or 0):
                    raise RuntimeError(f"pole err_flag {st['err_flag']}")
                idle = int(st.get("mode", 0) or 0) == 0
                # the getter leads the real pole by up to 73 mm, so also wait out the modelled travel
                if idle and abs(pos - height_m) <= tol_m and elapsed >= expected:
                    return True, pos
            if elapsed > timeout:
                self.stop()
                return False, pos
            time.sleep(self.poll_s)
