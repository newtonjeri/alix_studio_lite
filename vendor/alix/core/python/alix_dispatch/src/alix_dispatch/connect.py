"""Opening a controller connection — the ONLY module here that touches the SDK.

Kept separate from `backends.py` on purpose. Four test modules import
`alix_dispatch.backends` at module scope with no emulator installed; a
module-level `Robotic_Arm` import there would fail collection for those and
succeed for the emulator tests, which reads as an environment fault rather than
a code change. `backends.py`'s own docstring states the standing rule — the
handle is injected, and nothing opens a connection as a side effect of import —
and this module keeps it by importing the SDK lazily, inside the function.

Addresses come from `alix_core.sdk.endpoints`, which is the repo's authority
for them (it carries the 2026-08-05 re-addressing of the right arm and the
`RM_*_IP` overrides). This package previously had no address handling at all
and its tests hardcoded the pair — two of them without the env override, so
`RM_LEFT_IP` never reached them.
"""

from __future__ import annotations

import os
import socket
from dataclasses import dataclass
from typing import Dict, Optional

try:                                     # the repo's own address authority
    from alix_core.sdk import endpoints as _endpoints
    DEFAULT_IPS: Dict[str, str] = dict(_endpoints.ARM_IPS)
    DEFAULT_PORT: int = _endpoints.ROBOT_PORT
except Exception:                        # pragma: no cover - packaging fallback
    _endpoints = None
    DEFAULT_IPS = {"left": os.environ.get("RM_LEFT_IP", "192.168.1.10"),
                   "right": os.environ.get("RM_RIGHT_IP", "192.168.1.103")}
    DEFAULT_PORT = int(os.environ.get("RM_ROBOT_PORT", "8080"))

DEFAULT_UDP_PORT: int = int(os.environ.get("RM_UDP_PORT", "8089"))

#: How long the SDK waits for the controller's reply to an acked call, in ms.
#: The vendor default is **500 ms** (`rm_set_timeout`, rm_interface.h:75) and
#: alix_hw sets the same through `sdk_ack_timeout_ms` — but alix_dispatch never
#: set it at all, so it ran at the default. A controller that is loading a
#: 4000-point program, streaming one, or running a Modbus transaction answers
#: slower than that occasionally: the SDK gives up, prints `<cmd> revice error`
#: and returns -2 ("data reception failed ... or the controller timed out
#: without returning", rm_interface.h:3456), and the caller reads a refusal
#: that never happened. On 2026-09-04 that ended two 12-task flows — one at
#: task 1, one at task 11 — and the same message appeared harmlessly twice on
#: `rm_get_program_run_state` mid-replay, where the polling loop just went
#: round again (F122). Set to 0 to leave the SDK default alone.
ACK_TIMEOUT_MS: int = int(os.environ.get("RM_ACK_TIMEOUT_MS", "1500"))


def set_ack_timeout(robot, ms: int = None) -> bool:
    """Raise this PROCESS's SDK reply timeout. True when it was applied.

    Process-global, not per handle (the vendor's own wording), and absent from
    the emulator — which is verbatim-pinned, so this asks rather than assumes.
    """
    ms = ACK_TIMEOUT_MS if ms is None else ms
    fn = getattr(robot, "rm_set_timeout", None)
    if not ms or fn is None:
        return False
    try:
        fn(int(ms))
        return True
    except Exception as exc:                                           # noqa: BLE001
        print(f"  (SDK ack timeout not set: {exc!r})")
        return False

#: Triple thread mode. `SdkBackend` refuses block!=0 under single-thread mode
#: because `block` becomes a TIMEOUT IN SECONDS there; every production path
#: on the ladder inits triple mode, so open connections that way.
THREAD_MODE_TRIPLE = 2


@dataclass
class ArmHandle:
    side: str
    ip: str
    port: int
    robot: object
    handle_id: int
    host_ip: str


def host_ip_for(arm_ip: str) -> Optional[str]:
    """THIS host's address on the route to `arm_ip` — the UDP push target.

    NEVER hardcode this. The controller accepts ANY target address, returns
    success, and then delivers nothing if it is not actually your host — a
    silent failure that produces an empty recording and no error. The emulator
    reproduces the same trap deliberately, dropping every frame for a target
    outside its host set, so a hardcoded address records nothing offline TOO.

    `RM_HOST_IP` wins when set (the emulator honours the same variable);
    otherwise ask the kernel which local address it would route from, which is
    the only answer that is right both offline and on hardware.
    """
    override = os.environ.get("RM_HOST_IP")
    if override:
        return override
    try:
        probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            probe.connect((arm_ip, 1))       # no packet leaves; just routing
            local = probe.getsockname()[0]
        finally:
            probe.close()
    except Exception:
        return None
    # A loopback answer means there is no route at all. But note what this
    # function CANNOT do, measured 2026-08-20 on a host with no route to the
    # arm subnet: the kernel answers with the DEFAULT-ROUTE address
    # (10.65.121.17 on wlp0s20f3) for a 192.168.1.x arm — plausible, non-
    # loopback, and unreachable from the controller. So this is a best effort,
    # and the real gate is downstream: a recording that received ZERO frames
    # refuses the run. Do not add a subnet heuristic here; the delivered-frame
    # count is the only honest evidence that the target was right.
    return None if not local or local.startswith("127.") else local


def sdk_provenance(robot) -> str:
    """'emulated' or 'hardware', from the handle's OWN class module.

    The rung a run may claim is derived from this, never from a flag: an
    emulator run stamped SIM would license a REAL rung off a stream the
    contract measures at 236 % of the commanded path.

    Deliberately NOT `"rm_emulator" in sys.modules` — the installed keys in
    this tree are `alix_emulator`, `alix_emulator.rm_emulator` and
    `Robotic_Arm*`, so that test is permanently false here.

    An emulated handle comes back from `open_arm` wrapped in the program
    surface (`emulated_program.attach`); the wrapper carries the wrapped
    instance as `_emulated_inner`, and THAT class is judged.
    """
    robot = getattr(robot, "_emulated_inner", robot)
    return ("emulated" if type(robot).__module__.startswith("alix_emulator")
            else "hardware")


def open_arm(side: str, ip: Optional[str] = None, port: Optional[int] = None, *,
             thread_mode: int = THREAD_MODE_TRIPLE,
             log_level: int = 3) -> Optional[ArmHandle]:
    """Connect one controller. Returns None when the handle is unusable.

    None is a SKIP, not a failure: it is what "no arm plugged in" looks like,
    and the caller reports it as such rather than as a refusal by the arm.

    `log_level` is the SDK's LOG level, not a thread mode — they are different
    arguments in different positions and confusing them silently changes how
    the handle behaves.
    """
    ip = ip or DEFAULT_IPS.get(side)
    if not ip:
        raise KeyError(f"no address configured for side {side!r}")
    port = DEFAULT_PORT if port is None else port
    try:
        from Robotic_Arm.rm_robot_interface import RoboticArm
        from Robotic_Arm.rm_ctypes_wrap import rm_thread_mode_e
    except Exception as exc:
        print(f"  [FAIL] {side}: SDK not importable ({exc!r})")
        return None
    try:
        robot = RoboticArm(rm_thread_mode_e(thread_mode))
        handle = robot.rm_create_robot_arm(ip, port, log_level)
    except Exception as exc:
        print(f"  [FAIL] {side}: connect to {ip}:{port} raised {exc!r}")
        return None
    handle_id = getattr(handle, "id", None)
    if handle is None or handle_id is None or handle_id <= 0:
        print(f"  [FAIL] {side}: no usable handle from {ip}:{port} "
              f"(id={handle_id})")
        return None
    set_ack_timeout(robot)
    if sdk_provenance(robot) == "emulated":
        # The emulator has no program channel (rm_emulator.py is verbatim-
        # pinned); the five verbs are faked HERE, at the seam, and only for
        # an emulated handle — attach() refuses anything else.
        from alix_dispatch import emulated_program
        robot = emulated_program.attach(robot, ip)
    return ArmHandle(side=side, ip=ip, port=port, robot=robot,
                     handle_id=int(handle_id), host_ip=host_ip_for(ip))


def close_arm(handle: Optional[ArmHandle]) -> None:
    """Release a handle. Never raises — teardown must not mask the real fault."""
    if handle is None:
        return
    try:
        handle.robot.rm_delete_robot_arm()
    except Exception:
        pass
