"""Who may use Lite: the operator PIN, the engineer PIN and the Lite UI's sessions.

The handoff asks for the PINs to leave the page (its section 19): they are stored here, salted
and hashed (PBKDF2-SHA256), in ``<data>/auth.json``. Until changed they are the designer's
template PINs, 1234 for both; change them with
``python -m alix_studio_lite.server --set-pin operator`` (or ``engineer``).

Logging in with the operator PIN returns a session token, which the page sends as the
``X-Lite-Token`` header. The engineer PIN raises a session to engineer until Advanced Robot
Controls is locked or closed. Five wrong PINs in a row lock that PIN for 30 s. STOP never needs
a session (see ``api_v2.py``).
"""
from __future__ import annotations

import hashlib
import hmac
import json
import os
import secrets
import threading
import time
from pathlib import Path

from .jobs import UserError

ROLES = ("operator", "engineer")
DEFAULT_PIN = "1234"                    # the designer's template PINs (handoff section 2)
ITERATIONS = 200_000
MAX_FAILS = 5
LOCK_S = 30.0
IDLE_S = 12 * 3600                      # a session left idle this long must log in again


def _digest(pin: str, salt: bytes, iterations: int) -> str:
    return hashlib.pbkdf2_hmac("sha256", pin.encode("utf-8"), salt, iterations).hex()


def _record(pin: str, default: bool = False) -> dict:
    salt = os.urandom(16)
    return {"salt": salt.hex(), "hash": _digest(pin, salt, ITERATIONS), "iterations": ITERATIONS,
            "default": default}


def valid_pin(pin: str) -> bool:
    return isinstance(pin, str) and pin.isdigit() and 4 <= len(pin) <= 6


class Auth:
    def __init__(self, data_dir: Path):
        self.path = Path(data_dir) / "auth.json"
        self._lock = threading.Lock()
        self._sessions: dict[str, dict] = {}
        self._fails = {r: [0, 0.0] for r in ROLES}          # wrong PINs in a row, locked until
        self._pins = self._load()

    def _load(self) -> dict:
        try:
            d = json.loads(self.path.read_text(encoding="utf-8"))
            if all(isinstance(d.get(r), dict) and "hash" in d[r] for r in ROLES):
                return d
        except (OSError, ValueError):
            pass
        return {r: _record(DEFAULT_PIN, default=True) for r in ROLES}

    def _save(self) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        tmp = self.path.with_name(self.path.name + ".tmp")
        tmp.write_text(json.dumps(self._pins, indent=1), encoding="utf-8")
        os.replace(tmp, self.path)

    # ── PINs

    def set_pin(self, role: str, pin: str) -> None:
        if role not in ROLES:
            raise ValueError(f"unknown role {role!r}")
        if not valid_pin(pin):
            raise ValueError("A PIN is 4 to 6 digits.")
        with self._lock:
            self._pins[role] = _record(pin)
            self._save()

    def is_default(self, role: str) -> bool:
        return bool(self._pins[role].get("default"))

    def check(self, role: str, pin) -> bool:
        """Whether ``pin`` is the role's PIN; refuses while the role is locked out."""
        now = time.monotonic()
        with self._lock:
            fails = self._fails[role]
            if fails[1] > now:
                raise UserError(f"Too many wrong PINs. Try again in {fails[1] - now:.0f} s.")
            rec = self._pins[role]
            ok = isinstance(pin, str) and hmac.compare_digest(
                _digest(pin, bytes.fromhex(rec["salt"]), int(rec["iterations"])), rec["hash"])
            if ok:
                fails[0] = 0
            else:
                fails[0] += 1
                if fails[0] >= MAX_FAILS:
                    fails[0], fails[1] = 0, now + LOCK_S
            return ok

    # ── sessions

    def login(self, pin) -> str:
        if not self.check("operator", pin):
            raise UserError("Incorrect operator PIN. Please try again.")
        token = secrets.token_urlsafe(24)
        with self._lock:
            self._sessions[token] = {"role": "operator", "engineer": False, "seen": time.monotonic()}
        return token

    def session(self, token) -> dict | None:
        with self._lock:
            s = self._sessions.get(token or "")
            if s is None:
                return None
            if time.monotonic() - s["seen"] > IDLE_S:
                del self._sessions[token]
                return None
            s["seen"] = time.monotonic()
            return s

    def elevate(self, token, pin) -> None:
        s = self.session(token)
        if s is None:
            raise UserError("Please log in again.", code="login")
        if not self.check("engineer", pin):
            raise UserError("Incorrect PIN. Try again.")
        s["engineer"] = True

    def drop_engineer(self, token) -> None:
        s = self.session(token)
        if s is not None:
            s["engineer"] = False

    def logout(self, token) -> None:
        with self._lock:
            self._sessions.pop(token or "", None)
