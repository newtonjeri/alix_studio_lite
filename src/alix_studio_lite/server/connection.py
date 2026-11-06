"""Each arm's connection details (the Connection page; the user, 6 Nov: "on a new setup, provided you enter the correct
connection details, such as IP address, UDP ports, or any other information required, it should connect as expected").

Saved on this computer (``<data>/connection.json``): they belong to the robot this computer drives, not to a project.
Anyone may change them. Every connection Lite makes uses them: the controller's address and port (robot_link, the
arm's process, runner v14's runs), the UDP port the controller's feed is recorded on (robot_real), and the E-stop's
input and the level that means pressed (robot_link). A change applies at the arm's next Connect.

The hand's Modbus settings are shown, not changed here: runner v14's own hand.py (the owner's) and ALIX's driver both
fix them (the end port's RS485, Modbus ID 1, 115200 baud), so a value changed in Lite would not reach the runner's
Hand steps. The hand itself is set to them."""
from __future__ import annotations

import ipaddress
import json
import threading
import time
from pathlib import Path

from .jobs import UserError

ARMS = ("left", "right")
# today's values (runner v14, ALIX's connect.DEFAULT_IPS, decided 13-14 Oct for the E-stop): used until saved
DEFAULTS = {
    "left": {"ip": "192.168.1.10", "port": 8080, "feed_port": 8089, "estop_io": 3, "estop_level": 0, "feed_follow_host": True},
    "right": {"ip": "192.168.1.103", "port": 8080, "feed_port": 8090, "estop_io": 3, "estop_level": 0, "feed_follow_host": True},
}
FIELDS = ("ip", "port", "feed_port", "estop_io", "estop_level", "feed_follow_host")
# feed_follow_host (the user, 6 Nov: "by default the feed uses the IP address of the host computer, so when the address
# changes it changes with it"): a controller's push found on toward any other address as the arm connects is taken
# for this computer's own, left from an address it had before (the lab's PC was 192.168.1.239 until 5 Nov, 192.168.1.20
# on 6 Nov, when the new controllers pushed to 192.168.1.12 and 192.168.120.11): pointed at this computer's address
# now and turned off until a run records (udp_push.left_on). Off: another computer's push is left alone, as ALIX's
# capture leaves it (a live driver there), and the Connection page can turn it off.
ESTOP_IOS = (1, 2, 3, 4)                    # the controller's digital IO (rm_get_io_state: 1 to 4)
FILE = "connection.json"


def hand_modbus() -> dict:
    """The hand's Modbus settings every hand command uses (ALIX's driver and runner v14's hand.py: the same)."""
    try:
        from alix_dispatch import peripherals

        port, device, baud = peripherals.HAND_MODBUS_PORT, peripherals.HAND_MODBUS_DEVICE, peripherals.HAND_MODBUS_BAUD
    except Exception:                                                               # noqa: BLE001
        port, device, baud = 1, 1, 115200
    return dict(port=int(port), port_name="the end's RS485" if int(port) == 1 else "the controller's RS485",
                device=int(device), baud=int(baud))


def check_ip(ip, what: str = "The address") -> str:
    """An IPv4 address as typed, refused plainly when it is not one (the lab, 6 Nov: "192.168.1.1099")."""
    text = str(ip or "").strip()
    try:
        addr = ipaddress.IPv4Address(text)
    except ValueError:
        raise UserError(f"{what} {text!r} is not an IPv4 address.",
                        "Four numbers from 0 to 255 with dots between them, as the vendor's app shows it "
                        "(Configuration, Communication, IP Mode Setting), for example 192.168.1.10.", code="connection") from None
    if addr.is_unspecified or addr.is_multicast:
        raise UserError(f"{what} {text} cannot be a controller's address.", "Use the address the vendor's app shows.",
                        code="connection")
    return str(addr)


def _int(value, name: str, lo: int, hi: int) -> int:
    try:
        n = int(str(value).strip())
    except (TypeError, ValueError):
        raise UserError(f"{name} must be a whole number from {lo} to {hi}.", code="connection") from None
    if not lo <= n <= hi:
        raise UserError(f"{name} must be from {lo} to {hi}.", code="connection")
    return n


def clean(side: str, values: dict) -> dict:
    """One arm's details, checked: refused with what is wrong."""
    S = side.upper()
    out = dict(DEFAULTS[side])
    out.update({k: v for k, v in values.items() if k in FIELDS and v is not None and str(v).strip() != ""})
    out["ip"] = check_ip(out["ip"], f"The {S} controller's address")
    out["port"] = _int(out["port"], f"The {S} controller's port", 1, 65535)
    out["feed_port"] = _int(out["feed_port"], f"The {S} feed's UDP port", 1024, 65535)
    out["estop_io"] = _int(out["estop_io"], f"The {S} E-stop input", 1, 4)
    out["estop_level"] = _int(out["estop_level"], f"The {S} E-stop's pressed level", 0, 1)
    out["feed_follow_host"] = _bool(out["feed_follow_host"])
    return out


def _bool(value) -> bool:
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in ("1", "true", "yes", "on")


class ConnectionSettings:
    """Both arms' details, read once and saved on every change."""

    def __init__(self, data_dir: Path | None):
        self.path = Path(data_dir) / FILE if data_dir else None
        self._lock = threading.Lock()
        self.saved_at: float | None = None
        self.values = {s: dict(DEFAULTS[s]) for s in ARMS}
        raw = {}
        if self.path is not None:
            try:
                raw = json.loads(self.path.read_text(encoding="utf-8"))
            except (OSError, ValueError):
                raw = {}
        for s in ARMS:
            try:
                self.values[s] = clean(s, raw.get(s) or {})
            except UserError:                     # a hand-edited file with a bad value: that arm's defaults
                self.values[s] = dict(DEFAULTS[s])
        self.saved_at = raw.get("saved_at") if isinstance(raw.get("saved_at"), (int, float)) else None

    def arm(self, side: str) -> dict:
        with self._lock:
            return dict(self.values[side])

    def update(self, side: str, changes: dict) -> dict:
        """``changes`` for one arm checked against the other arm (one controller cannot be both, nor one feed port
        both feeds) and saved. Returns the arm's details."""
        if side not in ARMS:
            raise UserError("Say which arm: left or right.", code="connection")
        with self._lock:
            new = clean(side, dict(self.values[side], **{k: v for k, v in changes.items() if k in FIELDS}))
            other = self.values[ARMS[1 - ARMS.index(side)]]
            if (new["ip"], new["port"]) == (other["ip"], other["port"]):
                raise UserError(f"The other arm's controller is at {new['ip']}:{new['port']}: each arm has a controller of its own.",
                                code="connection")
            if new["feed_port"] == other["feed_port"]:
                raise UserError(f"The other arm's feed uses UDP port {new['feed_port']}: each controller's feed needs a port of its own.",
                                code="connection")
            self.values[side] = new
            self.saved_at = time.time()
            self._save()
            return dict(new)

    def _save(self) -> None:
        if self.path is None:
            return
        self.path.parent.mkdir(parents=True, exist_ok=True)
        tmp = self.path.with_suffix(".tmp")
        tmp.write_text(json.dumps(dict(self.values, saved_at=self.saved_at), indent=1), encoding="utf-8")
        tmp.replace(self.path)

    def view(self) -> dict:
        with self._lock:
            return dict(arms={s: dict(v) for s, v in self.values.items()}, defaults={s: dict(v) for s, v in DEFAULTS.items()},
                        hand=hand_modbus(), saved_at=self.saved_at, file=str(self.path) if self.path else None)
