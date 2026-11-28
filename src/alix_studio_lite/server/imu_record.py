"""The IMU on the UGV recorded alongside an S2 run's feed (the user, 28 Nov: the IMU's data "logged at the same time frame
as the arms feedback").

The IMU (a Wheeltec N100, an FDISYSTEMS DETA-10) is found by its USB chip, a CP210x (10c4:ea60), and read raw at
921600 by ``tools/imu_n100.py``'s own port and recording loop, so a run's recording is the same as a hand-made capture:
``<day>/imu/<HHMMSS>_<label>/`` with raw.bin, chunks.jsonl (each read stamped with time.time() and time.monotonic(),
the clock the feed's frames carry), meta.json, then report.txt / report.json. ``tools/imu_runs.py analyse`` lines it up
with the run's feed and Programs afterwards.

Before a run from the page, ``missing`` says why no IMU can be recorded, and the page asks the operator: connect it and
run again, or run without it (the user, 28 Nov: "the user should be notified if the imu is not detected, and should be
asked to proceed or make sure it is connected"). After that it never stands in a run's way: the IMU gone by the
time the run starts (or a run the UGV starts, with nobody to ask) or unplugged mid-run is said in the run's log, and
the run goes on. Its reads run on their own thread; the report is written after the run, on that
thread. Two runs at once (RUN LEFT and RUN RIGHT) share one recording, which ends with the last of them.
"""
from __future__ import annotations

import importlib.util
import json
import sys
import threading
import time
from pathlib import Path

from .. import vendor_paths

MAX_S = 4 * 3600.0          #: a recording never runs longer, whatever happens to the run
_LOCK = threading.Lock()
_ACTIVE: "Recording | None" = None


def n100():
    """``tools/imu_n100.py``, loaded once."""
    name = "imu_n100"
    mod = sys.modules.get(name)
    if mod is None:
        spec = importlib.util.spec_from_file_location(name, vendor_paths.LITE_ROOT / "tools" / "imu_n100.py")
        mod = importlib.util.module_from_spec(spec)
        sys.modules[name] = mod                       # its dataclasses look their module up there
        spec.loader.exec_module(mod)
    return mod


class Recording:
    def __init__(self, n, port, path: str, folder: Path, label: str, log):
        self.n, self.port, self.path, self.folder, self.label, self.log = n, port, path, folder, label, log
        self.users = 1
        self._stop = threading.Event()
        self.meta = dict(port=path, baud=n.BAUD, test="frames", label=label, started=time.time(),
                         started_local=time.strftime("%Y-%m-%d %H:%M:%S"), tool="Lite, S2 run",
                         note=f"recorded with the S2 run's feed ({label})")
        folder.mkdir(parents=True, exist_ok=True)
        (folder / "meta.json").write_text(json.dumps(self.meta, indent=1))
        self.recorded = threading.Event()             # the reads have ended (the report may still be on its way)
        self.thread = threading.Thread(target=self._run, name="imu-record", daemon=True)
        self.thread.start()

    def _run(self) -> None:
        def say(text: str) -> None:                   # the loop's warnings only, not its 10 s byte counts
            if text.startswith("No bytes"):
                self.log(f"IMU: {text}", "warn")

        try:
            self.meta.update(self.n.record(self.port, MAX_S, self.folder, say=say, stop=self._stop))
        except Exception as exc:                                       # noqa: BLE001 - unplugged mid-run: said, run goes on
            self.meta["ended"] = f"the IMU stopped answering: {exc}"
            self.log(f"IMU: the recording ended early ({exc}); what came before is kept in {self.folder}.", "warn")
        finally:
            try:
                self.port.close()
            except OSError:
                pass
            (self.folder / "meta.json").write_text(json.dumps(self.meta, indent=1))
            self.recorded.set()
        try:                                          # the same checks as a hand-made capture, after the run
            buf, chunks = self.n.load(self.folder)
            lines, rep = self.n.analyse(buf, chunks, "frames")
            self.n.write_report(self.folder, lines, rep)
        except Exception as exc:                                       # noqa: BLE001 - a report is a convenience
            (self.folder / "report.txt").write_text(f"no report: {exc}\n")

    def stop(self) -> None:
        """The run is over: the reads end within 50 ms (the report follows on the thread)."""
        global _ACTIVE
        with _LOCK:
            self.users -= 1
            if self.users > 0:
                return
            if _ACTIVE is self:
                _ACTIVE = None
        self._stop.set()
        self.recorded.wait(1.0)
        self.log(f"IMU recording ended: {self.meta.get('bytes', 0):,} bytes in {self.meta.get('seconds', 0):.0f} s, "
                 f"{self.folder}.")


def plain(reason: str) -> str:
    """The IMU tool's reasons (written for a terminal) in the operator's words."""
    if reason.startswith("No CP210x"):
        return "no IMU is plugged into this computer (its USB chip, 10c4:ea60, is not listed)"
    if reason.startswith("The IMU's USB chip is plugged in"):
        return "its USB chip is plugged in, but has no serial port: unplug it and plug it in again"
    if "is in use by another program" in reason:
        return "its port is in use by another program (an IMU capture started in a terminal): end that first"
    if reason.startswith("No permission"):
        return ("this computer's user may not open its port: add it to the dialout group (sudo usermod -aG dialout $USER), "
                "then log out and in")
    if "CP210x devices are plugged in" in reason:
        return "more than one USB serial chip like the IMU's is plugged in: unplug the others"
    return reason.rstrip(".")


def missing() -> str | None:
    """Why the IMU cannot be recorded now (None: it can), asked before a run starts: the same lookup as ``start``, the
    port opened and closed at once (so a port another program holds is found too)."""
    with _LOCK:
        if _ACTIVE is not None:
            return None                               # another run records it: this one will share that recording
    n = n100()
    try:
        path = n.find_port("auto")
        n.Port(path).close()
    except SystemExit as exc:
        return plain(str(exc))
    except OSError as exc:
        return f"its port could not be opened ({exc})"
    return None


def start(day_dir: Path, label: str, log) -> Recording | None:
    """Record the IMU into ``day_dir``/imu/<HHMMSS>_<label>/ until ``stop()``; None (and a note in ``log``) when no
    IMU can be read. ``log(text, level)`` is the run's log."""
    global _ACTIVE
    with _LOCK:
        if _ACTIVE is not None:
            _ACTIVE.users += 1
            log(f"The IMU is already being recorded (another run): {_ACTIVE.folder}.")
            return _ACTIVE
        n = n100()
        try:
            path = n.find_port("auto")
            port = n.Port(path)
        except SystemExit as exc:                     # the tool's plain reasons: not plugged in, no permission, in use
            log(f"The IMU is not recorded: {plain(str(exc))}.", "warn")
            return None
        except OSError as exc:
            log(f"The IMU is not recorded: its port {path} could not be opened ({exc}).", "warn")
            return None
        folder = Path(day_dir) / "imu" / f"{time.strftime('%H%M%S')}_{label}"
        _ACTIVE = Recording(n, port, path, folder, label, log)
        log(f"Recording the IMU ({path}) with the feed, to {folder}.")
        return _ACTIVE
