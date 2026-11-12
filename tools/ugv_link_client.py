#!/usr/bin/env python3
"""A stand-in for the UGV on the UGV link (server/ugv_link.py), for the lab: it asks the arm side's status, sends a goal
with the offset typed here, follows the feedback and prints the result, as the UGV's own client will. Standard library
only. It drives nothing: the arm side decides and moves.

    python3 tools/ugv_link_client.py --url http://127.0.0.1:8780 --token-file ~/ALIX-Lite/ugv_link_token.txt status
    python3 tools/ugv_link_client.py --url ... --token-file ... clean --fixture "Cubicle 1" --x-mm 3 --y-mm -2 --yaw-deg 0.4
    python3 tools/ugv_link_client.py --url ... --token-file ... result GOAL_ID
    python3 tools/ugv_link_client.py --url ... --token-file ... cancel GOAL_ID

    clean   sends one goal (a new goal_id), then polls twice a second until its result. Ctrl+C cancels the goal (STOP on
            the arms) and waits for the result. --attempt N for a retry; --no-wait: only the goal reply.
            The offsets are typed in millimetres and degrees and sent in metres and degrees, as the contract has them.
            --source lidar_fit: how the offset was measured, as the UGV will say it (default: typed).
    --token T / --token-file F / the environment's UGV_LINK_TOKEN: the arm side's token (Lite prints where it keeps it).

It ends with 0 when the goal SUCCEEDED with safe_to_move true (or the status is ready), 1 otherwise, 2 when the arm side
could not be reached or refused the request itself.
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time
import urllib.error
import urllib.request
import uuid
from pathlib import Path

PREFIX = "/link/v1"


class Unreachable(Exception):
    pass


def request(a, method: str, path: str, body: dict | None = None) -> dict:
    req = urllib.request.Request(a.url.rstrip("/") + PREFIX + path, method=method,
                                 headers={"Content-Type": "application/json", "X-Link-Token": a.token},
                                 data=None if body is None else json.dumps(body).encode())
    try:
        with urllib.request.urlopen(req, timeout=a.timeout) as resp:
            return json.loads(resp.read())
    except urllib.error.HTTPError as exc:
        try:
            said = json.loads(exc.read()).get("error")
        except ValueError:
            said = None
        raise Unreachable(f"HTTP {exc.code}: {said or exc.reason}") from None
    except (OSError, ValueError) as exc:
        raise Unreachable(f"no answer from {a.url} ({exc})") from None


def show_status(st: dict) -> None:
    print(f"mode {st['mode']}; ready: {'yes' if st['ready'] else 'no, ' + st['not_ready']}"
          + (f" ({st['detail']})" if st.get("detail") else ""))
    print(f"safe_to_move: {'yes' if st['safe_to_move'] else 'NO'} ({st.get('safe_to_move_detail', '')})")
    lim = st.get("limits") or {}
    print(f"limits: x +-{lim.get('x_m', 0) * 1000:g} mm, y +-{lim.get('y_m', 0) * 1000:g} mm, yaw +-{lim.get('yaw_deg', 0):g} deg; "
          f"attempts a visit: {st.get('max_attempts')}" + (f"; goal running: {st['active_goal_id']}" if st.get("active_goal_id") else ""))


def show_result(view: dict) -> int:
    res = view["result"]
    print(f"RESULT {res['status']}" + (f", {res['abort_code']}" if res.get("abort_code") else "")
          + f"; safe_to_move: {'yes' if res['safe_to_move'] else 'NO'}; the UGV: {res['ugv_action']}; "
          f"{res['stages_done']} stage(s) done in {res['duration_s']:g} s" + (f"; {res['detail']}" if res.get("detail") else ""))
    return 0 if res["status"] == "SUCCEEDED" and res["safe_to_move"] else 1


def follow(a, gid: str) -> int:
    """Polls the goal twice a second, printing each new stage and event, until its result; Ctrl+C cancels it."""
    since, stage, cancelled, misses = 0, None, False, 0
    while True:
        try:
            try:
                view = request(a, "GET", f"/goals/{gid}?since={since}")
                misses = 0
            except Unreachable as exc:              # the link lost: the UGV keeps holding and keeps asking
                misses += 1
                if misses == 1 or misses % 10 == 0:
                    print(f"  ... {exc}; holding, asking again", flush=True)
                time.sleep(0.5)
                continue
            for e in view.get("events") or []:
                print(f"  {e['t']:7.1f} s  {e['text']}", flush=True)
            since = view.get("seq", since)
            fb = view.get("feedback") or {}
            now = (view["status"], fb.get("stage_index"), fb.get("stage_name"))
            if now != stage and "result" not in view:
                stage = now
                print(f"{view['status']}: stage {fb.get('stage_index')} of {fb.get('stage_count')}, {fb.get('stage_name')} "
                      f"({fb.get('elapsed_s', 0):.0f} s)", flush=True)
            if "result" in view:
                return show_result(view)
            time.sleep(0.5)
        except KeyboardInterrupt:
            if cancelled:
                print("\nLeft while the goal runs on: ask for its result later with:  result " + gid)
                return 1
            cancelled = True
            print("\nCancelling (STOP on the arms); waiting for the result. Ctrl+C again leaves.", flush=True)
            try:
                request(a, "POST", f"/goals/{gid}/cancel", {})
            except Unreachable as exc:
                print(f"  the cancel did not reach the arm side: {exc}")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="A stand-in for the UGV on the UGV link.")
    ap.add_argument("--url", required=True, help="the arm side's link, for example http://127.0.0.1:8780")
    ap.add_argument("--token", default=os.environ.get("UGV_LINK_TOKEN"))
    ap.add_argument("--token-file", type=Path)
    ap.add_argument("--timeout", type=float, default=15.0, help="seconds to wait for each answer (default 15)")
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("status")
    c = sub.add_parser("clean")
    c.add_argument("--fixture", required=True)
    c.add_argument("--x-mm", type=float, default=0.0)
    c.add_argument("--y-mm", type=float, default=0.0)
    c.add_argument("--yaw-deg", type=float, default=0.0)
    c.add_argument("--attempt", type=int, default=1)
    c.add_argument("--source", default="typed")
    c.add_argument("--goal-id", default=None)
    c.add_argument("--no-wait", action="store_true")
    for name in ("result", "cancel"):
        sub.add_parser(name).add_argument("goal_id")
    a = ap.parse_args(argv)
    if a.token_file:
        try:
            a.token = a.token_file.expanduser().read_text(encoding="utf-8").strip()
        except OSError as exc:
            print(f"The token file cannot be read: {exc}", file=sys.stderr)
            return 2
    if not a.token:
        print("Give the arm side's token: --token, --token-file or UGV_LINK_TOKEN.", file=sys.stderr)
        return 2
    try:
        if a.cmd == "status":
            st = request(a, "GET", "/status")
            show_status(st)
            return 0 if st["ready"] else 1
        if a.cmd == "result":
            view = request(a, "GET", f"/goals/{a.goal_id}")
            if "result" not in view:
                print(f"{view['status']}: still running.")
                return 1
            return show_result(view)
        if a.cmd == "cancel":
            print("cancel:", request(a, "POST", f"/goals/{a.goal_id}/cancel", {})["status"])
            return follow(a, a.goal_id)
        gid = a.goal_id or str(uuid.uuid4())
        goal = dict(protocol=1, goal_id=gid, fixture_id=a.fixture, attempt=a.attempt, offset_x_m=a.x_mm / 1000.0,
                    offset_y_m=a.y_mm / 1000.0, offset_yaw_deg=a.yaw_deg, offset_source=a.source)
        print(f"goal {gid}: {a.fixture}, attempt {a.attempt}, offset x {a.x_mm:+g} mm, y {a.y_mm:+g} mm, yaw {a.yaw_deg:+g} deg")
        reply = request(a, "POST", "/goals", goal)
        if not reply["accepted"]:
            print(f"REJECTED {reply['reject_code']}; the UGV: {reply['ugv_action']}"
                  + (f"; outside: {', '.join(reply['exceeded'])}" if reply.get("exceeded") else "") + f"; {reply['detail']}")
            return 1
        print(f"ACCEPTED; the UGV: {reply['ugv_action']}"
              + (f"; expected {reply['expected_duration_s']:g} s" if reply.get("expected_duration_s") else ""))
        return 0 if a.no_wait else follow(a, gid)
    except Unreachable as exc:
        print(f"The arm side did not take the request: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
