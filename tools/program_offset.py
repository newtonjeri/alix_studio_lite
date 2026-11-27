#!/usr/bin/env python3
"""Move a recorded Program for the UGV's offset and look at the result: a test tool before the GUI and the backend
(the user, 25 Nov 2026). The work is alix_studio_lite.motion.program_offset; this is its command line and its page.

    .venv/bin/python tools/program_offset.py --ui                     # the page: http://127.0.0.1:8790/, opened in Chrome
                                                                      # (--no-browser: only the address is printed)
    .venv/bin/python tools/program_offset.py Resources/Programs/offset_tests/OFF_5P_TES.txt --side right --pole 150 \\
        --x -20 --y 20 --yaw 0                                        # the same from the command line

The offset is the UGV's, in base_link: --x mm forward (back is negative), --y mm left (right is negative), --yaw deg
counter-clockwise seen from above. It is where the UGV stands now against where it stood when the Program was recorded;
the commode stays, so the hand path moves the other way.

Each run is kept in Resources/Logs/<day>/program_offset/<HHMMSS>_<name>/: the input, the moved file
(<name>_x<..>_y<..>_yaw<..>.txt, the same format), and report.json. Moving a Program sends nothing to a robot.

Upload and Run (the user, 26 Nov 2026: "just show the upload and run button on this same screen, no need Lite login,
this is for testing only"): the page's buttons run the control program's own upload-and-run tool,
vendor/control_app/rm_upload.py, on the moved file, straight to the arm's controller (no ALIX Studio Lite):

- Upload: the file to a program slot, then the slot's stored size read back (VERIFIED);
- Run: the controller moves the arm to the program's first point at the move-to-start speed, then plays the program
  at its own timing; the arm's error codes are watched; the global speed is put back to 50 % at the end;
- STOP: Ctrl+C to the running tool, which slow-stops the arm (rm_set_arm_slow_stop). The red E-stop is the real stop.

Not done here, unlike Lite: the pole and the hand are not staged (set them before a run), and nothing stops another
program (Lite, the pendant) from driving the same controller at the same time: close those first.
Every upload and run is kept beside the moved file in its folder: robot_<HHMMSS>_<upload|run>.log.
"""
from __future__ import annotations

import argparse
import ipaddress
import json
import re
import signal
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
if str(ROOT / "src") not in sys.path:
    sys.path.insert(0, str(ROOT / "src"))

from alix_studio_lite.motion import program_offset as po  # noqa: E402

PAGE = Path(__file__).resolve().parent / "program_offset_ui.html"
PORT = 8790
LITE_PORTS = (8765, 8766)
MAX_BYTES = 30 * 1024 * 1024
RM_UPLOAD = ROOT / "vendor" / "control_app" / "rm_upload.py"
RM_SDK = ROOT / "vendor" / "alix" / "external" / "butterfli-realman-alix" / "Python"
ARM_IPS = {"left": "192.168.1.10", "right": "192.168.1.103"}      # rm_upload's and Lite's connection defaults
RESTORE_PCT = 50                                                  # the global speed put back after a run (rm_upload's)


def _safe(name: str) -> str:
    stem = Path(name or "program.txt").stem
    return re.sub(r"[^A-Za-z0-9._-]+", "_", stem).strip("._") or "program"


#: the moved files this server wrote, by name: the page names one to send, never a path
MOVED: dict[str, dict] = {}


def run(text: str, name: str, side: str, pole_mm: float, x_mm: float, y_mm: float, yaw_deg: float,
        out_root: str | None = None) -> dict:
    """Move the Program in ``text`` and keep everything. Returns the report, the picture, the moved file's name and text."""
    md, rows = po.read_program(text)
    r = po.shift_program(md, side, float(pole_mm) / 1000.0, po.UgvOffset(float(x_mm), float(y_mm), float(yaw_deg)))
    stem = _safe(name)
    out_name = f"{stem}_x{float(x_mm):+g}_y{float(y_mm):+g}_yaw{float(yaw_deg):+g}.txt"
    out_text = po.write_program(rows, r["md"])
    root = Path(out_root) if out_root else ROOT / "Resources" / "Logs"
    folder = root / time.strftime("%Y-%m-%d") / "program_offset" / f"{time.strftime('%H%M%S')}_{stem}"
    k = 2
    while folder.exists():
        folder = folder.with_name(f"{folder.name.split('~')[0]}~{k}")
        k += 1
    folder.mkdir(parents=True)
    (folder / f"{stem}.txt").write_text(text)
    (folder / out_name).write_text(out_text)
    report = dict(r["report"])
    picture = report.pop("picture")
    report.update(input=f"{stem}.txt", output=out_name, folder=str(folder), made=time.strftime("%Y-%m-%d %H:%M:%S"))
    (folder / "report.json").write_text(json.dumps(report, indent=1))
    MOVED[out_name] = dict(path=folder / out_name, folder=folder, verdict=report["verdict"], side=side,
                           lines=int(report["lines"]))
    return dict(report=report, picture=picture, output_name=out_name, output_text=out_text)


def summary(rep: dict) -> str:
    t, m = rep["taught"], rep["moved"]
    out = [f"{rep['verdict']}: {rep['words']}"]
    out += [f"  - {r}" for r in rep["reasons"]]
    out.append(f"  IK frame: {rep['ik_frame']}")
    out.append(f"  Arm_Tip from the moved path: {rep['hand_error_mm']} mm, {rep['hand_error_deg']} deg; lines unsolved: "
               f"{rep['lines_unsolved']} of {rep['lines']}")
    for k, label in (("speed", "speed share"), ("acceleration", "acceleration share"), ("jerk", "jerk share")):
        out.append(f"  {label}: {t[k]['share']} (J{t[k]['joint']}) -> {m[k]['share']} (J{m[k]['joint']})")
    out.append(f"  nearest joint limit: {t['joint_margin_deg']['min']} deg (J{t['joint_margin_deg']['joint']}) -> "
               f"{m['joint_margin_deg']['min']} deg (J{m['joint_margin_deg']['joint']})")
    out.append(f"  nearest singularity: {t['singular_margin_deg']['min']} -> {m['singular_margin_deg']['min']} deg "
               f"({m['singular_margin_deg']['type']})")
    out.append(f"  largest step between lines: {t['step_deg']} -> {m['step_deg']} deg; joints change up to "
               f"{max(rep['joint_change_deg'])} deg; the elbow moves up to {rep['elbow_move_mm']} mm")
    for i, tp in enumerate(rep["touches"], 1):
        out.append(f"  touch {i} ({tp['from_s']}-{tp['to_s']} s): {tp['taught_mm']} -> {tp['moved_mm']} mm, change "
                   f"{tp['change_mm']}")
    if "folder" in rep:
        out.append(f"  kept: {rep['folder']}")
    return "\n".join(out)


# ─────────────────────────────────────────────────────────────── the robot: rm_upload.py, one job at a time

class RobotJob:
    """One rm_upload.py call (upload, upload + run, or run), its printed lines kept for the page and in a log file."""
    _lock = threading.Lock()
    current: "RobotJob | None" = None
    _ids = 0

    def __init__(self, title: str, args: list[str], log_path: Path | None):
        RobotJob._ids += 1
        self.id, self.title, self.args, self.log_path = RobotJob._ids, title, args, log_path
        self.lines: list[str] = []
        self.rc: int | None = None
        self.stopped = False
        self.started = time.time()
        self.proc: subprocess.Popen | None = None

    @classmethod
    def start(cls, title: str, args: list[str], log_path: Path | None) -> "RobotJob":
        with cls._lock:
            if cls.current is not None and cls.current.rc is None:
                raise ValueError(f"{cls.current.title} is still going: wait for it, or press STOP.")
            job = cls(title, args, log_path)
            cls.current = job
        cmd = [sys.executable, "-u", str(RM_UPLOAD), *args, "--yes"] + (["--sdk", str(RM_SDK)] if RM_SDK.is_dir() else [])
        job.say(f"$ rm_upload.py {' '.join(args)}")
        try:
            # its own session: the terminal's Ctrl+C on this server is not sent to it (this server stops it on the
            # way out instead); Ctrl+C made the default again in it, so STOP's SIGINT always reaches its handler even
            # when this server was started where Ctrl+C is ignored (in the background, from a launcher)
            job.proc = subprocess.Popen(cmd, cwd=str(ROOT), stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                        stdin=subprocess.DEVNULL, text=True, bufsize=1, start_new_session=True,
                                        preexec_fn=lambda: signal.signal(signal.SIGINT, signal.SIG_DFL))
        except OSError as e:
            job.say(f"** rm_upload.py could not be started: {e}")
            job.rc = 1
            return job
        threading.Thread(target=job._read, daemon=True).start()
        return job

    def say(self, line: str) -> None:
        line = line.rstrip()
        self.lines.append(line)
        print(f"{time.strftime('%H:%M:%S')}  robot  {line}", flush=True)
        if self.log_path is not None:
            try:
                with open(self.log_path, "a", encoding="utf-8") as f:
                    f.write(f"{time.strftime('%H:%M:%S')}  {line}\n")
            except OSError:
                pass

    def _read(self) -> None:
        for line in self.proc.stdout:
            self.say(line)
        self.rc = self.proc.wait()
        self.say("DONE" if self.rc == 0 else f"** ended with exit code {self.rc}" + (" (STOP)" if self.stopped else ""))

    def stop(self) -> bool:
        """Ctrl+C to rm_upload.py: during a run it slow-stops the arm and puts the global speed back."""
        if self.rc is not None or self.proc is None:
            return False
        self.stopped = True
        self.say("** STOP pressed: Ctrl+C sent (the arm slow-stops)")
        try:
            self.proc.send_signal(signal.SIGINT)
        except OSError:
            return False
        return True

    def view(self, since: int = 0) -> dict:
        return dict(id=self.id, title=self.title, running=self.rc is None, rc=self.rc, stopped=self.stopped,
                    lines=self.lines[since:], next=len(self.lines), elapsed_s=round(time.time() - self.started, 1))


def _robot_args(b: dict) -> tuple[str, str, int, int]:
    """The side, the controller's address, the slot and the move-to-start speed, checked."""
    side = b.get("side")
    if side not in ("left", "right"):
        raise ValueError("Say which arm: left or right.")
    ip = str(b.get("ip") or ARM_IPS[side]).strip()
    try:
        ipaddress.IPv4Address(ip)
    except ValueError:
        raise ValueError(f"{ip!r} is not an IPv4 address.") from None
    try:
        slot, speed = int(b.get("slot")), int(b.get("run_speed", 20))
    except (TypeError, ValueError):
        raise ValueError("The Program ID and the speed are whole numbers.") from None
    if not 1 <= slot <= 100:
        raise ValueError("The Program ID is 1 to 100.")
    if not 1 <= speed <= 100:
        raise ValueError("The move-to-start speed is 1 to 100 %.")
    return side, ip, slot, speed


def robot_send(b: dict) -> RobotJob:
    """Upload (``run``: and run) a moved file this server wrote, named by ``name``."""
    side, ip, slot, speed = _robot_args(b)
    m = MOVED.get(str(b.get("name") or ""))
    if m is None:
        raise ValueError("Move the Program again: this page's server does not have that moved file.")
    if m["verdict"] == "CANNOT":
        raise ValueError("This moved file cannot be sent: the arm cannot follow it (CANNOT).")
    if m["side"] != side:
        raise ValueError(f"This file was moved for the {m['side'].upper()} arm.")
    go = bool(b.get("run"))
    args = [str(m["path"]), str(slot), "--side", side, "--ip", ip]
    if go:
        timeout = max(60.0, (m["lines"] - 1) * 0.005 * 1.5 + 60.0)
        args += ["--run", "--run-speed", str(speed), "--restore-speed", str(RESTORE_PCT), "--timeout", f"{timeout:.0f}"]
    what = "upload_run" if go else "upload"
    return RobotJob.start(f"{'Upload + run' if go else 'Upload'} of {m['path'].name} to {side.upper()} program {slot}",
                          args, m["folder"] / f"robot_{time.strftime('%H%M%S')}_{what}.log")


def robot_run(b: dict, out_root: str | None) -> RobotJob:
    """Run what a slot holds."""
    side, ip, slot, speed = _robot_args(b)
    root = Path(out_root) if out_root else ROOT / "Resources" / "Logs"
    folder = root / time.strftime("%Y-%m-%d") / "program_offset"
    folder.mkdir(parents=True, exist_ok=True)
    args = ["--run", str(slot), "--side", side, "--ip", ip, "--run-speed", str(speed),
            "--restore-speed", str(RESTORE_PCT), "--timeout", "900"]
    return RobotJob.start(f"Run of {side.upper()} program {slot}", args,
                          folder / f"robot_{time.strftime('%H%M%S')}_run_{side}_{slot}.log")


class Handler(BaseHTTPRequestHandler):
    out_root: str | None = None

    def log_message(self, fmt, *args):                       # quiet: one line per run is printed instead
        pass

    def _send(self, code: int, body: bytes, ctype: str):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _json(self, code: int, obj):
        self._send(code, json.dumps(obj).encode(), "application/json")

    def _host_ok(self) -> bool:
        """This computer only, and a browser page only from this server itself: another program's page must not
        reach the robot (Lite's own rule, app.py _host_ok)."""
        host = (self.headers.get("Host") or "").split(":")[0]
        if host not in ("127.0.0.1", "localhost"):
            return False
        origin = self.headers.get("Origin")
        if origin is None:
            return True
        m = re.match(r"^http://(127\.0\.0\.1|localhost)(?::(\d+))?$", origin)
        return m is not None and int(m.group(2) or 80) == self.server.server_address[1]

    def _body(self):
        n = int(self.headers.get("Content-Length") or 0)
        if n <= 0 or n > MAX_BYTES:
            raise ValueError("The request is empty or larger than 30 MB.")
        b = json.loads(self.rfile.read(n))
        if not isinstance(b, dict):
            raise ValueError("The request must be a JSON object.")
        return b

    def do_GET(self):
        if not self._host_ok():
            return self._send(403, b"forbidden", "text/plain")
        if self.path in ("/", "/index.html"):
            return self._send(200, PAGE.read_bytes(), "text/html; charset=utf-8")
        m = re.fullmatch(r"/api/robot/job(?:\?since=(\d+))?", self.path)
        if m:
            job = RobotJob.current
            return self._json(200, dict(job=job.view(int(m.group(1) or 0)) if job else None,
                                        ips=ARM_IPS, tool=RM_UPLOAD.is_file()))
        self._json(404, dict(error=f"No page {self.path}"))

    def do_POST(self):
        if not self._host_ok():
            return self._send(403, b"forbidden", "text/plain")
        if self.path == "/api/robot/stop":                   # no body needed
            job = RobotJob.current
            ok = job.stop() if job else False
            return self._json(200, dict(stopped=ok, message="STOP: Ctrl+C sent to the run; the arm slow-stops." if ok
                                        else "Nothing is running from this page. Use the red E-stop if the arm moves."))
        if self.path in ("/api/robot/send", "/api/robot/run"):
            try:
                b = self._body()
                job = robot_send(b) if self.path == "/api/robot/send" else robot_run(b, self.out_root)
            except (ValueError, TypeError) as e:
                return self._json(400, dict(error=str(e)))
            return self._json(200, dict(job=job.view()))
        if self.path != "/api/move":
            return self._json(404, dict(error=f"No route {self.path}"))
        try:
            b = self._body()
            out = run(b.get("text") or "", b.get("name") or "program.txt", b.get("side") or "right",
                      float(b.get("pole_mm", 150)), float(b.get("x_mm", 0)), float(b.get("y_mm", 0)),
                      float(b.get("yaw_deg", 0)), self.out_root)
        except po.ProgramError as e:
            return self._json(400, dict(error=str(e)))
        except (ValueError, TypeError) as e:
            return self._json(400, dict(error=f"A setting could not be read: {e}"))
        rep = out["report"]
        print(f"{time.strftime('%H:%M:%S')}  {rep['input']}  {rep['verdict']}  {rep['words']}  -> {rep['output']}",
              flush=True)
        self._json(200, out)


def serve(port: int, out_root: str | None = None, host: str = "127.0.0.1") -> ThreadingHTTPServer:
    if port in LITE_PORTS:
        raise SystemExit(f"Port {port} is Lite's; choose another (the default is {PORT}).")
    Handler.out_root = out_root
    return ThreadingHTTPServer((host, port), Handler)


def open_page(url: str) -> None:
    """The page in Chrome (or Chromium) when it is installed, else the default browser; the server is already
    listening, so the page loads at once. A computer with no browser (a headless one) just keeps the printed address."""
    import webbrowser

    for name in ("google-chrome", "google-chrome-stable", "chromium", "chromium-browser", "chrome"):
        try:
            if webbrowser.get(name).open(url, new=2):
                return
        except webbrowser.Error:
            continue
    try:
        if webbrowser.open(url, new=2):
            return
    except webbrowser.Error:
        pass
    print("No browser could be opened: open the address above yourself.", flush=True)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0], formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("file", nargs="?", help="the Program file (.txt, one {\"point\": [...]} a line)")
    ap.add_argument("--ui", action="store_true", help="open the page instead")
    ap.add_argument("--no-browser", action="store_true", help="with --ui: do not open the page in a browser")
    ap.add_argument("--port", type=int, default=PORT)
    ap.add_argument("--side", choices=["right", "left"], default="right")
    ap.add_argument("--pole", type=float, default=150.0, help="the pole height the Program runs at, mm")
    ap.add_argument("--x", type=float, default=0.0, help="UGV offset, mm forward (back: negative)")
    ap.add_argument("--y", type=float, default=0.0, help="UGV offset, mm left (right: negative)")
    ap.add_argument("--yaw", type=float, default=0.0, help="UGV offset, deg counter-clockwise seen from above")
    ap.add_argument("--out", default=None, help="the Logs folder (default: Lite's Resources/Logs)")
    a = ap.parse_args(argv)
    if a.ui:
        srv = serve(a.port, a.out)
        print(f"Program offset page: http://127.0.0.1:{a.port}/  (Ctrl-C stops it)", flush=True)
        if not RM_UPLOAD.is_file():
            print(f"Upload and Run are off: {RM_UPLOAD} is missing.", flush=True)
        if not a.no_browser:
            open_page(f"http://127.0.0.1:{a.port}/")
        try:
            srv.serve_forever()
        except KeyboardInterrupt:
            job = RobotJob.current
            if job is not None and job.stop():                # a run never outlives its page's server unstopped
                job.proc.wait(timeout=10)
        finally:
            srv.server_close()
        return 0
    if not a.file:
        ap.error("give a Program file, or --ui for the page")
    path = Path(a.file)
    try:
        out = run(path.read_text(), path.name, a.side, a.pole, a.x, a.y, a.yaw, a.out)
    except po.ProgramError as e:
        print(f"Cannot use {path}: {e}")
        return 2
    print(summary(out["report"]))
    return 0 if out["report"]["verdict"] != "CANNOT" else 1


if __name__ == "__main__":
    sys.exit(main())
