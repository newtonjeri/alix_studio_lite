"""Long work runs as a job: the page starts it, then polls ``/api/lite/jobs/<id>``.

A job reports honest progress (stage i of n, elapsed time) and a short event log; it is never
a held HTTP request.
"""
from __future__ import annotations

import itertools
import threading
import time
import traceback
from concurrent.futures import ThreadPoolExecutor


class Cancelled(Exception):
    pass


class Job:
    def __init__(self, jid: str, kind: str, title: str, unit: str | None = None, arm: str | None = None):
        self.id, self.kind, self.title, self.unit, self.arm = jid, kind, title, unit, arm
        self.state = "running"
        self.stage, self.i, self.n = "Starting", 0, 0
        self.started = time.time()
        self.finished: float | None = None
        self.result: dict | None = None
        self.error: str | None = None
        self.hint: str | None = None
        self.events: list[dict] = []
        self.reason: tuple[str, str | None] | None = None   # why it was stopped, when not by STOP (an E-stop)
        self.alone = False                                 # runs on its arm alone: the other arm stays stopped
        self._cancel = threading.Event()
        self._lock = threading.Lock()

    def progress(self, stage: str, i: int = 0, n: int = 0) -> None:
        with self._lock:
            self.stage, self.i, self.n = stage, int(i), int(n)
        self.check()

    def log(self, text: str, level: str = "info") -> None:
        with self._lock:
            self.events.append(dict(seq=len(self.events), t=round(time.time() - self.started, 2),
                                    text=text, level=level))

    def cancel(self, reason: str | None = None, hint: str | None = None) -> None:
        """Stop the job at its next check. With ``reason`` (an E-stop), it ends as failed with that
        text, never as a plain "Stopped."; the first reason given is the one kept."""
        with self._lock:
            if reason and self.reason is None:
                self.reason = (reason, hint)
        self._cancel.set()

    @property
    def cancelled(self) -> bool:
        return self._cancel.is_set()

    def check(self) -> None:
        if self._cancel.is_set():
            raise Cancelled()

    def sleep(self, seconds: float) -> None:
        """Wait, but wake at once when the job is cancelled (STOP)."""
        if self._cancel.wait(seconds):
            raise Cancelled()

    def view(self, since: int = 0) -> dict:
        with self._lock:
            return dict(id=self.id, kind=self.kind, title=self.title, unit=self.unit, arm=self.arm,
                        state=self.state, stage=self.stage, i=self.i, n=self.n,
                        elapsed_s=round((self.finished or time.time()) - self.started, 1),
                        result=self.result, error=self.error, hint=self.hint,
                        events=[e for e in self.events if e["seq"] >= since])


class Jobs:
    def __init__(self, on_end=None):
        self.on_end = on_end                          # called with every job once it has ended (the log)
        self._jobs: dict[str, Job] = {}
        self._ids = itertools.count(1)
        self._pool = ThreadPoolExecutor(max_workers=16, thread_name_prefix="lite-job")
        self._lock = threading.Lock()

    def start(self, kind: str, title: str, fn, *, unit: str | None = None, arm: str | None = None) -> Job:
        job = Job(f"{kind}-{next(self._ids)}", kind, title, unit, arm)
        with self._lock:
            self._jobs[job.id] = job
            if len(self._jobs) > 200:                         # keep the registry small
                for k in [k for k, j in self._jobs.items() if j.state != "running"][:50]:
                    self._jobs.pop(k, None)

        def run():
            try:
                res = fn(job)
                job.result = res if isinstance(res, dict) else {"value": res}
                job.state = "done"
            except Cancelled:
                if job.reason:
                    job.state, (job.error, job.hint) = "failed", job.reason
                    job.log(job.error, "error")
                else:
                    job.state = "cancelled"
                    job.log("Stopped.", "warn")
            except UserError as exc:
                job.state, job.error, job.hint = "failed", str(exc), exc.hint
                job.log(str(exc), "error")
            except Exception as exc:                                # noqa: BLE001
                job.state, job.error = "failed", f"{type(exc).__name__}: {exc}"
                job.hint = "Details are in the server log."
                job.log(job.error, "error")
                traceback.print_exc()
            finally:
                job.finished = time.time()
                if self.on_end is not None:
                    try:
                        self.on_end(job)
                    except Exception:                                   # noqa: BLE001 - a log never stops a job
                        traceback.print_exc()

        self._pool.submit(run)
        return job

    def get(self, jid: str) -> Job | None:
        return self._jobs.get(jid)

    def running(self, *, arm: str | None = None, kind: str | None = None) -> list[Job]:
        return [j for j in list(self._jobs.values()) if j.state == "running"
                and (arm is None or j.arm in (arm, "both")) and (kind is None or j.kind == kind)]

    def cancel_arm(self, arm: str, reason: str | None = None, hint: str | None = None,
                   kinds: tuple | None = None) -> int:
        """Stop the running jobs of ``arm`` ("all" and "both" mean every arm), optionally only
        those of ``kinds``."""
        n = 0
        for j in self.running():
            if (arm in ("all", "both") or j.arm in (arm, "both")) and (kinds is None or j.kind in kinds):
                j.cancel(reason, hint)
                n += 1
        return n


class UserError(Exception):
    """A refusal the operator should read: what happened, and what to do."""

    def __init__(self, message: str, hint: str | None = None, code: str | None = None):
        super().__init__(message)
        self.hint = hint
        self.code = code            # for the page to act on, e.g. "login" when the session has ended
