#!/usr/bin/env python3
"""Move a recorded Program for the UGV's offset and look at the result: a test tool before the GUI and the backend
(the user, 25 Nov 2026). The work is alix_studio_lite.motion.program_offset; this is its command line and its page.

    .venv/bin/python tools/program_offset.py --ui                     # the page: http://127.0.0.1:8790/
    .venv/bin/python tools/program_offset.py Resources/Programs/offset_tests/OFF_5P_TES.txt --side right --pole 150 \\
        --x -20 --y 20 --yaw 0                                        # the same from the command line

The offset is the UGV's, in base_link: --x mm forward (back is negative), --y mm left (right is negative), --yaw deg
counter-clockwise seen from above. It is where the UGV stands now against where it stood when the Program was recorded;
the commode stays, so the hand path moves the other way.

Each run is kept in Resources/Logs/<day>/program_offset/<HHMMSS>_<name>/: the input, the moved file
(<name>_x<..>_y<..>_yaw<..>.txt, the same format), and report.json. Nothing is sent to a robot.
"""
from __future__ import annotations

import argparse
import json
import re
import sys
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


def _safe(name: str) -> str:
    stem = Path(name or "program.txt").stem
    return re.sub(r"[^A-Za-z0-9._-]+", "_", stem).strip("._") or "program"


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

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            self._send(200, PAGE.read_bytes(), "text/html; charset=utf-8")
        else:
            self._json(404, dict(error=f"No page {self.path}"))

    def do_POST(self):
        if self.path != "/api/move":
            return self._json(404, dict(error=f"No route {self.path}"))
        n = int(self.headers.get("Content-Length") or 0)
        if n <= 0 or n > MAX_BYTES:
            return self._json(400, dict(error="The request is empty or larger than 30 MB."))
        try:
            b = json.loads(self.rfile.read(n))
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


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0], formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("file", nargs="?", help="the Program file (.txt, one {\"point\": [...]} a line)")
    ap.add_argument("--ui", action="store_true", help="open the page instead")
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
        try:
            srv.serve_forever()
        except KeyboardInterrupt:
            pass
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
