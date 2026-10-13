import time

# Inspire RH56 over the RealMan end-port Modbus (register addresses from hand_channel.cpp)
HAND_MODBUS_PORT = 1          # end interface board RS485
HAND_MODBUS_DEVICE = 1
HAND_MODBUS_BAUD = 115200
HAND_REG_ANGLE_SET = 1486     # target angle, 6 channels
HAND_REG_FORCE_SET = 1498
HAND_REG_SPEED_SET = 1522
HAND_REG_ANGLE_ACT = 1546     # measured angle, 6 channels
HAND_DOF = 6                  # order: little, ring, middle, index, thumb_bend, thumb_rot
HAND_SPEED_SET_MAX = 1000
HAND_STROKE_K = 373.0         # stroke_ms = 115 + 373 * span / SPEED_SET (measured)
HAND_LATENCY_S = 0.115
HAND_SETTLE_COUNTS = 3        # two reads this close = settled
HAND_ARRIVE_COUNTS = 15       # this close to target = arrived
# units: 0-1000 per channel, 1000 = fully open


def _params(address, num, device=HAND_MODBUS_DEVICE):
    from Robotic_Arm.rm_ctypes_wrap import rm_peripheral_read_write_params_t
    return rm_peripheral_read_write_params_t(port=HAND_MODBUS_PORT, address=int(address),
                                             device=int(device), num=int(num))


def encode_hi_lo(values):
    out = []
    for v in values:
        out += [(int(v) >> 8) & 0xFF, int(v) & 0xFF]
    return out


def decode_hi_lo(data):
    return [((int(data[2 * i]) & 0xFF) << 8) | (int(data[2 * i + 1]) & 0xFF)
            for i in range(len(data) // 2)]


def fingers_pct_counts(pct):
    """'50 % open (first 4 fingers)' -> four fingers at pct, both thumb channels held (None)."""
    return [int(round(pct * 10))] * 4 + [None, None]


class Hand:
    def __init__(self, robot, device=HAND_MODBUS_DEVICE, baud=HAND_MODBUS_BAUD, poll_s=0.05):
        self.robot = robot          # a connected Robotic_Arm.rm_robot_interface.RoboticArm
        self.device, self.baud, self.poll_s = device, baud, poll_s
        self.speed_set = None

    def open(self, speed_set=HAND_SPEED_SET_MAX):
        """Switch the end port to Modbus mode (once per session), then set speed."""
        ret = self.robot.rm_set_modbus_mode(HAND_MODBUS_PORT, self.baud, 2)
        if ret != 0:
            raise RuntimeError(f"rm_set_modbus_mode failed, ret={ret}")
        self.set_speed(speed_set)

    def close(self):
        return self.robot.rm_close_modbus_mode(HAND_MODBUS_PORT)

    def _write(self, address, values):
        """One bulk write of all six (~63 ms); per-register fallback if it fails."""
        ret = self.robot.rm_write_registers(_params(address, len(values), self.device),
                                            encode_hi_lo(values))
        if ret == 0:
            return 0
        worst = 0
        for ch, v in enumerate(values):     # byte-addressed: two bytes per channel
            r = self.robot.rm_write_single_register(_params(address + 2 * ch, 1, self.device), int(v))
            worst = r if r != 0 else worst
        return worst

    def _read(self, address):
        ret, data = self.robot.rm_read_multiple_holding_registers(_params(address, HAND_DOF, self.device))
        if ret != 0 or not data or len(data) < 2 * HAND_DOF:
            return None
        return decode_hi_lo(list(data)[:2 * HAND_DOF])

    def set_speed(self, speed_set):
        s = max(1, min(HAND_SPEED_SET_MAX, int(speed_set)))
        ret = self._write(HAND_REG_SPEED_SET, [s] * HAND_DOF)
        if ret != 0:
            raise RuntimeError(f"SPEED_SET write failed, ret={ret}")
        self.speed_set = s

    def read(self):
        """Where the six channels actually are (0-1000)."""
        return self._read(HAND_REG_ANGLE_ACT)

    def command(self, counts, wait=True, timeout_s=None):
        """Move to six 0-1000 counts; None = hold that channel where it is. Returns (ok, final)."""
        if len(counts) != HAND_DOF:
            raise ValueError(f"need {HAND_DOF} values")
        before = self.read()
        if any(v is None for v in counts) and before is None:
            raise RuntimeError("channels to hold, but the hand does not read back")
        # all six registers are written at once, so a held channel gets its own current position
        target = [max(0, min(1000, int(before[i] if v is None else v))) for i, v in enumerate(counts)]
        span = max(abs(a - b) for a, b in zip(target, before)) if before else 1000
        law = HAND_LATENCY_S + HAND_STROKE_K * span / (self.speed_set or 1) / 1000.0
        timeout_s = timeout_s or 3.0 * law + 1.0

        ret = self._write(HAND_REG_ANGLE_SET, target)
        if ret != 0:
            raise RuntimeError(f"ANGLE_SET write failed, ret={ret}")
        if not wait:
            return True, None

        # wait until ARRIVED, or SETTLED (fingers closing on a tool stop early on force)
        t0, prev, cur = time.monotonic(), None, before
        while True:
            time.sleep(self.poll_s)
            cur = self.read()
            elapsed = time.monotonic() - t0
            if cur is not None:
                if all(abs(a - b) <= HAND_ARRIVE_COUNTS for a, b in zip(cur, target)):
                    return True, cur
                if (prev is not None and elapsed >= law
                        and all(abs(a - b) <= HAND_SETTLE_COUNTS for a, b in zip(cur, prev))):
                    return True, cur
                prev = cur
            if elapsed > timeout_s:
                return False, cur

    def freeze(self):
        """The SDK has no hand stop: command the angle it is at right now."""
        cur = self.read()
        return None if cur is None else self._write(HAND_REG_ANGLE_SET, cur)
