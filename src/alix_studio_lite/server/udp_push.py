"""The controller's realtime push and this computer's UDP ports: so that the CanFD channel and the arm's connection
coexist (the owner, 21 Oct: "Fix it so the CanFD channel and the upload channel can coexist, and both work well").

The lab, 21 Oct 15:24 (S2 Real Machine, both arms): the RIGHT controller's 5 ms feed could not be set up, "UDP port
8090 is already bound ON THIS HOST". What holds it is the arm's own connection. Measured here with the RealMan SDK's
own library (libapi_c.so 1.1.6, triple-thread mode, against a stand-in controller: tests/test_sdk_push_port.py):

- connecting reads the controller's push configuration and, when the push is ENABLED, binds that port on this
  computer for the SDK's own listener (every address, SO_REUSEADDR); reading the configuration while it is enabled
  does the same;
- the SDK keeps that port for as long as its process lives: turning the push off, or deleting the handle, does not
  let it go;
- ALIX's capture (``track.UdpRecorder``) binds the port without SO_REUSEADDR, on purpose (it never takes frames from
  a live driver), so it is refused. Its own way past a push left on toward this computer, turning it off on the way
  in (its L51), cannot work in the process whose SDK has bound the port already.

A push left on toward this computer is what ALIX's driver leaves behind when it stops (it points both controllers'
pushes here, ports 8089 and 8090, and does not turn them off: "whenever you used control, you had to do something
with the feedback channel"), and what a capture leaves when its process is killed mid-stream. So, when an arm
connects, in its own process (robot_process.worker_main): a push left on toward THIS computer whose port no program
outside Lite holds is turned off, the way ALIX's capture turns its own leftover off (``_clear_stale_self``, with its
verified write), and the arm is connected again in a fresh process, whose SDK then binds nothing; the capture binds
the port when it streams, as ALIX's does. A push toward this computer whose port another program holds (ALIX's driver
running) is a live consumer: left alone, and said. A push toward another computer is left alone too (ALIX's capture
refuses to take it). Linux only (/proc); elsewhere nothing is looked at."""
from __future__ import annotations

import os
from pathlib import Path


def lite_pids() -> set[int]:
    """This process, its parent (Lite's server, for an arm's process) and the parent's other children (the other arm,
    the tuning worker): Lite's own processes, whose ports are never another program's."""
    me, parent = os.getpid(), os.getppid()
    out = {me, parent}
    for d in _proc_dirs():
        try:
            ppid = int((d / "stat").read_text().rsplit(")", 1)[1].split()[1])
        except (OSError, ValueError, IndexError):
            continue
        if ppid == parent:
            out.add(int(d.name))
    return out


def port_holders(port: int, mine=(), kind: str = "udp") -> list[dict]:
    """The UDP sockets bound to ``port`` on this computer (``kind`` "tcp": the TCP sockets listening on it, a server's),
    outside processes ``mine``: [{pid, command}]. A socket of a process this user cannot look into has pid None. Empty
    without /proc (not Linux)."""
    inodes = set()
    for f in (f"/proc/net/{kind}", f"/proc/net/{kind}6"):
        try:
            rows = Path(f).read_text().splitlines()[1:]
        except OSError:
            continue
        for row in rows:
            parts = row.split()
            try:
                if int(parts[1].rsplit(":", 1)[1], 16) == int(port) and (kind != "tcp" or parts[3] == "0A"):   # 0A: LISTEN
                    inodes.add(parts[9])
            except (IndexError, ValueError):
                continue
    if not inodes:
        return []
    owner: dict[str, int] = {}
    for d in _proc_dirs():
        try:
            fds = list((d / "fd").iterdir())
        except OSError:
            continue
        for fd in fds:
            try:
                link = os.readlink(fd)
            except OSError:
                continue
            if link.startswith("socket:[") and link[8:-1] in inodes:
                owner.setdefault(link[8:-1], int(d.name))
    mine = set(mine)
    out = []
    for ino in sorted(inodes):
        pid = owner.get(ino)
        if pid is not None and pid in mine:
            continue
        out.append(dict(pid=pid, command=_command(pid)))
    return out


def held_text(holders: list[dict]) -> str:
    """Who holds a port, in a few words: "pid 1234 (ros2 run alix_hw ...)", or "a program this user cannot see"."""
    said = []
    for h in holders:
        said.append(f"pid {h['pid']} ({h['command']})" if h.get("pid") and h.get("command") else
                    f"pid {h['pid']}" if h.get("pid") else "a program this user cannot see")
    return ", ".join(dict.fromkeys(said))


def left_on(handle, log=None, follow_host: bool = False, port: int | None = None) -> dict | None:
    """At an arm's connection, in its own process: the controller's realtime push if it is ON toward this computer, or,
    with ``follow_host`` (connection.py's default since 6 Nov), toward any other address.

    None when it is off, toward another computer without ``follow_host``, or not read. Otherwise {port, ip, host,
    elsewhere, holders, cleared, said}: ``holders`` the programs outside Lite that hold the port (a live consumer here: the
    push is left on); ``cleared`` True when it was turned off (verified on the controller): this process's SDK has bound
    the port already, so the arm must be connected again in a fresh process; ``said`` what ALIX's code said doing it.

    ``follow_host`` (the user, 6 Nov: "by default the feed uses the IP address of the host computer, so when the address
    changes it changes with it"): a push toward another address is taken for this computer's own, left from an address it
    had before; it is pointed at this computer's address now (``host``: its address on the route to the controller) and
    this arm's feed ``port``, and turned off until a run records (ALIX's capture then turns it on toward the same address
    and puts it back off), through ALIX's verified push write. The lab, 6 Nov: the new controllers pushed to
    192.168.120.11:8089 and 192.168.1.12:8090 while the lab's PC was 192.168.1.20, and S2 recorded no feed."""
    if os.name != "posix" or not Path("/proc/net/udp").exists():
        return None
    robot, host = handle.robot, getattr(handle, "host_ip", None)
    try:
        ret, cfg = robot.rm_get_realtime_push()
    except Exception:                                                               # noqa: BLE001 - not read: nothing to do
        return None
    if ret != 0 or not isinstance(cfg, dict) or not cfg.get("enable") or not host:
        return None
    elsewhere = str(cfg.get("ip")) != str(host)
    if elsewhere and not follow_host:                # another computer's (a live driver there, ALIX's capture assumes): left
        return None
    found = int(cfg.get("port") or 0)
    out = dict(port=found, ip=str(cfg.get("ip")), host=str(host), elsewhere=elsewhere, cleared=False, said=[],
               holders=[] if elsewhere else port_holders(found, mine=lite_pids()))
    if out["holders"]:
        return out
    from alix_dispatch import track

    said = out["said"]
    to = int(port or found) if elsewhere else found
    rec = track.UdpRecorder(os.devnull, host, port=to, log=lambda m: (said.append(str(m)), log and log(str(m))))
    rec._prev = dict(cfg)
    if elsewhere:                                    # ALIX's verified write: this computer's address and port, off
        from Robotic_Arm.rm_robot_interface import rm_realtime_push_config_t, rm_udp_custom_config_t

        off = rm_realtime_push_config_t(cycle=int(cfg.get("cycle") or 1), enable=False, port=to,
                                        force_coordinate=int(cfg.get("force_coordinate", -1)), ip=str(host),
                                        custom_config=rm_udp_custom_config_t())
        ok, ret = rec._apply_push(robot, off, dict(enable=False), "point the feed push at this computer")
        out["cleared"], out["port_now"] = bool(ok), to
        if not ok:
            said.append(f"rm_set_realtime_push returned {ret} and the push still reads on")
        return out
    try:                                             # ALIX's own clear of its leftover: verified on the controller, retried
        rec._clear_stale_self(robot, f"{cfg.get('ip')}:{found}")
        out["cleared"] = bool(rec.cleared_stale and rec.cleared_stale.get("ok"))
    except RuntimeError as exc:
        said.append(str(exc))
    return out


def capture_refused_hint(port: int) -> str | None:
    """What to do when a capture cannot bind ``port`` (robot_real, in the arm's process): who holds it."""
    holders = port_holders(port)
    if not holders:
        return None
    if any(h.get("pid") == os.getpid() for h in holders):
        return ("This arm's own connection holds the port: the push was on toward this computer when it connected, or "
                "was turned on since. Connect the arm again (Robot & mode): Lite then turns the push off first.")
    return f"Another program on this computer holds it: {held_text(holders)}. Stop it (ALIX's driver or one of its tools), then run it again."


def _proc_dirs():
    try:
        return [d for d in Path("/proc").iterdir() if d.name.isdigit()]
    except OSError:
        return []


def _command(pid) -> str | None:
    if pid is None:
        return None
    try:
        raw = Path(f"/proc/{pid}/cmdline").read_bytes()
    except OSError:
        return None
    return " ".join(p.decode(errors="replace") for p in raw.split(b"\0") if p)[:160] or None
