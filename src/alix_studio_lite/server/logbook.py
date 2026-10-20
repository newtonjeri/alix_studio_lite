"""Lite's logs: every activity, kept as ALIX's programs keep theirs (the owner's request, 14 October).

Four streams, one folder per day under ``Resources/Logs/YYYY-MM-DD/`` in the repository (the
owner's choice, 14 October; ``run_lite --logs DIR`` puts them elsewhere):

- ``activity.jsonl``: what the operator did (every request that changes something, and the page's
  own steps: a stage, a task, an arm, a window), with who (the session), what, and the outcome;
- ``hardware.jsonl``: every command to a controller, with its arguments, the return code, the time
  it took and what it means, plus the lines ALIX's own functions log while they run;
- ``status.jsonl``: twice a second while an arm is connected, the whole unit as read: each arm's
  joints, error codes and run mode, its pole, its hand, its E-stop input, and what Lite is doing;
- ``lite.log``: activity and hardware as plain text lines, for reading in a terminal (``tail -f``).

Hardware errors and failures are also kept as ALERTS (the newest 200), which the robot status
carries to the page: it shows them at the bottom right until they are closed (alerts.js). Never
raises: a log that cannot be written must not stop the robot or the page.
"""
from __future__ import annotations

import itertools
import json
import threading
import time
from collections import deque
from pathlib import Path

SECRET_KEYS = {"pin", "token", "password"}
BIG = 400                                     # longer values (file contents) are logged as their size


def _clean(v, depth: int = 0):
    """A request body as it may be logged: no PINs, no file contents."""
    if isinstance(v, dict):
        return {k: ("***" if k.lower() in SECRET_KEYS else _clean(x, depth + 1)) for k, x in v.items()
                if k != "_raw"} | ({"_raw": f"<{len(v['_raw'])} bytes>"} if "_raw" in v else {})
    if isinstance(v, str) and len(v) > BIG:
        return f"<{len(v)} characters>"
    if isinstance(v, list) and depth < 3:
        return [_clean(x, depth + 1) for x in v[:50]]
    return v


def dump_threads(root, prefix: str = "threads-") -> str | None:
    """Every thread's stack, to <prefix>HHMMSS.txt in the day's log folder and to the terminal: what Lite was doing
    when it could not close (server), or when a call into a controller's SDK never came back (robot_standin; an
    arm's worker process writes its own, ``threads-<side>-``). The path written, or None."""
    import faulthandler
    import sys

    try:
        d = Path(root) / time.strftime("%Y-%m-%d")
        d.mkdir(parents=True, exist_ok=True)
        p = d / f"{prefix}{time.strftime('%H%M%S')}.txt"
        with open(p, "w", encoding="utf-8") as f:
            faulthandler.dump_traceback(file=f, all_threads=True)
    except Exception:                                                       # noqa: BLE001
        return None
    try:
        faulthandler.dump_traceback(file=sys.stderr, all_threads=True)      # the terminal's copy; a captured stderr has no fd
    except Exception:                                                       # noqa: BLE001
        pass
    return str(p)


class Logbook:
    def __init__(self, root: Path):
        self.root = Path(root)
        self.instance = f"{int(time.time())}"          # alert ids restart with the server
        self._lock = threading.Lock()
        self._ids = itertools.count(1)
        self.alerts: deque = deque(maxlen=200)

    # ── writing
    def _day(self) -> Path:
        d = self.root / time.strftime("%Y-%m-%d")
        d.mkdir(parents=True, exist_ok=True)
        return d

    def _write(self, stream: str, rec: dict, text: str) -> None:
        try:
            with self._lock:
                d = self._day()
                with open(d / f"{stream}.jsonl", "a", encoding="utf-8") as f:
                    f.write(json.dumps(rec, default=str, ensure_ascii=False) + "\n")
                with open(d / "lite.log", "a", encoding="utf-8") as f:
                    f.write(f"{time.strftime('%H:%M:%S', time.localtime(rec['t']))}.{int(rec['t'] * 1000) % 1000:03d} "
                            f"{text}\n")
        except OSError:
            pass

    def status(self, rec: dict) -> None:
        """One snapshot of the unit (2 Hz): status.jsonl only, not the text log."""
        try:
            with self._lock:
                with open(self._day() / "status.jsonl", "a", encoding="utf-8") as f:
                    f.write(json.dumps(rec, default=str, ensure_ascii=False) + "\n")
        except OSError:
            pass

    # ── the three kinds of record
    def activity(self, what: str, *, who: str | None = None, detail=None, ok: bool = True,
                 error: str | None = None) -> None:
        """Something the operator did, and how it ended."""
        rec = dict(t=time.time(), who=who, what=what, detail=_clean(detail), ok=ok, error=error)
        self._write("activity", rec, f"[activity] {who or '-'} {what}" + (f" -> REFUSED: {error}" if error else ""))

    def hardware(self, side: str | None, verb: str, *, args=None, ret=None, ok: bool | None = None,
                 elapsed_s: float | None = None, text: str | None = None) -> None:
        """One command to a controller (or a line one of ALIX's functions logged while running)."""
        rec = dict(t=time.time(), side=side, verb=verb, args=_clean(args), ret=ret, ok=ok,
                   elapsed_s=None if elapsed_s is None else round(elapsed_s, 3), text=text)
        self._write("hardware", rec, f"[{(side or '-').upper()}] {verb}"
                    + ("" if args is None else f" {json.dumps(_clean(args), default=str)}")
                    + ("" if ret is None else f" ret={ret}")
                    + ("" if elapsed_s is None else f" ({elapsed_s:.3f} s)")
                    + ("" if not text else f" · {text}"))

    def alert(self, level: str, text: str, *, hint: str | None = None, side: str | None = None,
              source: str | None = None) -> dict:
        """A hardware error ("error"), a failure risk ("warn") or a good status ("good": an arm connected again) the
        page must show until closed."""
        a = dict(id=next(self._ids), at=time.time(), level=level, side=side, source=source, text=text, hint=hint)
        self.alerts.append(a)
        self.hardware(side, f"ALERT {level.upper()}", text=text + (f" ({hint})" if hint else ""))
        return a

    def alerts_view(self, since_s: float = 1800.0) -> list[dict]:
        """The alerts of the last half hour, oldest first."""
        cut = time.time() - since_s
        return [a for a in list(self.alerts) if a["at"] >= cut]

    def records(self, stream: str, t0: float, t1: float) -> list[dict]:
        """A stream's records with ``t0 <= t <= t1``, oldest first (the day folders in between)."""
        out = []
        day, last = time.strftime("%Y-%m-%d", time.localtime(t0)), time.strftime("%Y-%m-%d", time.localtime(t1))
        for d in sorted(x for x in (self.root.iterdir() if self.root.is_dir() else []) if x.is_dir() and day <= x.name <= last):
            try:
                lines = (d / f"{stream}.jsonl").read_text(encoding="utf-8").splitlines()
            except OSError:
                continue
            for line in lines:
                try:
                    r = json.loads(line)
                except ValueError:
                    continue
                if t0 <= float(r.get("t") or 0) <= t1:
                    out.append(r)
        out.sort(key=lambda r: r["t"])
        return out

    def hw_logger(self, side: str | None, prefix: str = ""):
        """A ``log=`` callable for ALIX's functions: each line they print goes to the hardware log."""
        def log(*parts):
            line = " ".join(str(p) for p in parts).strip()
            if line:
                self.hardware(side, f"{prefix}alix", text=line)
        return log
