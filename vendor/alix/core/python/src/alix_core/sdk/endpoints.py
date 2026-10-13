"""Where the arms are — the single source of truth for addressing.

**The addresses in this file are defaults, not facts.** They change: the right
arm was re-addressed from ``192.168.1.11`` to ``192.168.1.103`` on 2026-08-05,
and at the time that one change left stale literals in nine files across four
languages, one of which (``alix.urdf.xacro``) was a *live default* that would
have connected a launch to the wrong host.

That is the problem this module exists to stop recurring. The rule:

    Code and tests import from here. They do not restate an address.

Enforced by ``alix_core/tests/test_endpoints.py``, which fails on a new
``192.168.*`` literal outside a named allowlist.

Every value is overridable by environment variable, and the variable names are
the ones the RealMan test suite has always used, so an override applies to
emulated runs, offline runs and hardware runs alike:

    RM_LEFT_IP=192.168.2.10 RM_RIGHT_IP=192.168.2.11 pytest ...

**Why this module does not import the emulator, which also holds these
defaults.** ``alix_emulator.rm_emulator`` defines ``EMU_LEFT_IP`` /
``EMU_RIGHT_IP`` / ``EMU_HOST_IPS`` from the same variables. It is *upstream
code under decision D4* — compared verbatim against RM_API2 by
``test_emulator_verbatim.py`` — so it cannot be edited to import this module,
and importing it from here would invert the dependency (the core would depend
on its own stand-in). Instead the two are held in agreement by
``test_endpoints.py::test_agrees_with_the_emulator``, which is the same pattern
``test_vendor_sdk_parity.py`` uses for the duplicated ``.so``: duplication that
cannot be removed is duplication that gets a test.
"""

from __future__ import annotations

import os
import warnings

__all__ = [
    "LEFT_IP",
    "RIGHT_IP",
    "HOST_IP",
    "ROBOT_PORT",
    "UDP_PORT",
    "ARM_IPS",
    "side_of",
    "arm_ip",
]


def _env(name: str, default: str) -> str:
    """Read an override, ignoring an empty or whitespace-only value.

    ``RM_RIGHT_IP=`` in a shell profile sets the variable to the empty string.
    Treating that as an override yields a connect to ``""`` and a socket error
    that names nothing useful, so an empty value falls back to the default.
    """
    value = os.environ.get(name, "").strip()
    return value or default


def _env_port(name: str, default: int) -> int:
    """Read a port override, falling back loudly rather than raising.

    A bare ``int(os.environ[...])`` here would make a typo in an environment
    variable raise :class:`ValueError` **at import time**. That is worse than it
    sounds: importers guard this module with ``except ImportError``
    (``sim_bridge._arm_sides``, ``run_studio._emulated_arms``), and a
    ``ValueError`` escapes those guards — so a stray ``RM_ROBOT_PORT=8O8O``
    would take out Studio's authoring path, which does not use ports at all.

    So: warn, and carry on with the default. A wrong port fails at connect,
    where the error can name the port; an unimportable module fails everywhere.
    """
    raw = os.environ.get(name, "").strip()
    if not raw:
        return default
    try:
        port = int(raw)
    except ValueError:
        warnings.warn(
            f"{name}={raw!r} is not an integer; using {default}. "
            f"Set it to a port number or unset it.",
            RuntimeWarning,
            stacklevel=2,
        )
        return default
    if not 1 <= port <= 65535:
        warnings.warn(
            f"{name}={port} is outside 1-65535; using {default}.",
            RuntimeWarning,
            stacklevel=2,
        )
        return default
    return port


# ── the arms ────────────────────────────────────────────────────────────────

#: Left arm. Stable since the robot was built.
LEFT_IP: str = _env("RM_LEFT_IP", "192.168.1.10")

#: Right arm. **Re-addressed 2026-08-05 (was .11)** and confirmed by a live
#: read-only query on 2026-08-13 (``RM_API2/docs/arm_version_info.md`` §1).
#: Anything still saying ``.11`` predates that change.
RIGHT_IP: str = _env("RM_RIGHT_IP", "192.168.1.103")

#: TCP control port. Same on both arms.
ROBOT_PORT: int = _env_port("RM_ROBOT_PORT", 8080)


# ── this host, as the arms see it ───────────────────────────────────────────

#: UDP push target — *this host's* address on the arm subnet.
#:
#: The controller **accepts any address here and reports success**, then
#: delivers nothing. A wrong value is therefore silent, which is why
#: ``TEST_PROCEDURE.md`` gate SM2 fails a run that sees no frame within 2 s
#: rather than continuing. The lease is DHCP — verify with ``ip addr`` at the
#: start of a session; do not trust this default.
HOST_IP: str = _env("RM_HOST_IP", "192.168.1.239")

#: Base UDP push port. The right arm conventionally uses ``UDP_PORT + 1``.
UDP_PORT: int = _env_port("RM_UDP_PORT", 8089)


# ── derived, so callers never rebuild these by hand ─────────────────────────

#: ``{"left": ip, "right": ip}`` — iterate this instead of writing a pair.
ARM_IPS: dict[str, str] = {"left": LEFT_IP, "right": RIGHT_IP}


def side_of(ip: str) -> str:
    """``"left"`` / ``"right"`` for a known arm address.

    Raises :class:`KeyError` rather than guessing. The emulator's own
    ``side = "left" if ip == EMU_LEFT_IP else "right"`` silently calls every
    unknown address "right", which turns a typo into a wrong-arm command.
    """
    for side, known in ARM_IPS.items():
        if ip == known:
            return side
    raise KeyError(
        f"{ip!r} is not a configured arm address "
        f"(left={LEFT_IP}, right={RIGHT_IP}). "
        f"Set RM_LEFT_IP / RM_RIGHT_IP if the robot was re-addressed."
    )


def arm_ip(side: str) -> str:
    """Address for ``"left"`` or ``"right"``, case-insensitively."""
    try:
        return ARM_IPS[side.strip().lower()]
    except KeyError:
        raise KeyError(
            f"unknown side {side!r}; expected 'left' or 'right'"
        ) from None
