"""Merge + Full Sequence S2, Full & Partial Sequence (feedback v5, slide 9): V53's Page 02 on Lite's
server (workflow.py). Each arm's steps are kept in the project as they are edited; V53's JSON is
saved and loaded per arm; RUN in Simulation Mode simulates, in Real Machine Mode runs on the robot."""
from __future__ import annotations

import json

from . import workflow
from .api_units import P, units_of
from .api_v2 import session
from .app import route
from .jobs import UserError

W = P + "/workflow"


def _run_args(body: dict) -> tuple[str, dict | None]:
    """``mode`` (both, left, right) and ``pick``: what is run (workflow.plan_for: the sequence, a single task, a
    dual-arm task)."""
    mode = str(body.get("mode") or "both")
    pick = body.get("pick") if isinstance(body.get("pick"), dict) else None
    return mode, pick


def _stage_programs(s: dict) -> dict:
    parts = [s["first"], s["second"]] if s["kind"] == "serial" else [s.get(x) for x in workflow.ARMS if s.get(x)]
    return {x["side"]: [dict(id=int(st["id"]), row=st.get("_row"), name=st.get("name")) for st in x["steps"] if st["type"] == "program"]
            for x in parts}


@route("GET", W)
def get_workflow(app, q, body, m):
    """Both arms' steps, the last simulation (its stages, actions, findings and collisions) and the
    S2 run or simulation going on, if any (a page opened again follows it)."""
    out = workflow.summary(units_of(app, m.group(1)))
    out["running"] = next((j.view(10 ** 9) for j in app.jobs.running() if j.title.startswith("S2 ")), None)
    return out


@route("PUT", W + r"/(left|right)")
def put_workflow_side(app, q, body, m):
    """An arm's steps as edited (each checked as runner v14 checks it, V53's Dual Rule kept)."""
    units = units_of(app, m.group(1))
    workflow.save_side(units, m.group(2), body.get("steps"))
    return workflow.summary(units)


@route("POST", W + r"/(left|right)/load")
def post_workflow_load(app, q, body, m):
    """V53's LOAD: a realman_workflow_sequence_v1 document for this arm (``document``); another arm's
    file is refused."""
    units, side = units_of(app, m.group(1)), m.group(2)
    doc = body.get("document")
    if isinstance(doc, str):
        try:
            doc = json.loads(doc)
        except ValueError:
            raise UserError("Could not load the sequence: it is not JSON.") from None
    workflow.save_side(units, side, workflow.from_document(side, doc))
    return workflow.summary(units)


@route("GET", W + r"/(left|right)/download")
def get_workflow_download(app, q, body, m):
    """V53's SAVE: the arm's steps as a realman_workflow_sequence_v1 document."""
    from .app import RawResponse

    units, side = units_of(app, m.group(1)), m.group(2)
    steps = workflow.load(units)[side]
    if not steps:
        raise UserError(f"{side.upper()} sequence is empty.")
    return RawResponse(json.dumps(workflow.document(side, steps), indent=2).encode(), "application/json",
                       f"{side}_workflow_sequence.json")


@route("GET", W + r"/(left|right)/sources")
def get_workflow_sources(app, q, body, m):
    """V53's refresh_sources: the programs and the global points to choose from. Programs: what the
    controller holds when it is linked and idle, else the ones Lite sent; points: read from a linked
    controller, else as last read."""
    units, side = units_of(app, m.group(1)), m.group(2)
    progs = workflow.programs_known(app, units, side)
    pts = workflow.points_known(app, side)
    return {"side": side,
            "programs": [dict(id=p["id"], name=p.get("name"), sent=p.get("sent"), file_name=p.get("file_name"), gone=p.get("gone"))
                         for p in sorted(progs.values(), key=lambda p: p["id"])],
            "points": sorted((pts or {}).get("points", {})), "points_read": (pts or {}).get("read")}


@route("POST", W + "/plan")
def post_workflow_plan(app, q, body, m):
    """V53's validation before anything moves: the plan's stages and counts, or why it cannot run."""
    units = units_of(app, m.group(1))
    mode, pick = _run_args(body)
    run = workflow.plan_for(workflow._numbered(workflow.load(units)), mode, pick)
    stages = [dict(index=i, kind=s["kind"], label=str(s.get("program_id") or ""), title=s.get("title"),
                   programs=_stage_programs(s) if s["kind"] in ("parallel", "serial", "single") else {}, **(
        {"first": s["first"]["side"], "second": s["second"]["side"]} if s["kind"] == "serial" else {}))
        for i, s in enumerate(run["plan"])]
    return dict(mode=run["mode"], pick=run["pick"], counts=run["counts"], stages=stages)


@route("POST", W + "/simulate")
def post_workflow_simulate(app, q, body, m):
    """Simulation Mode: the run in 3D with the collision check (a job)."""
    units = units_of(app, m.group(1))
    mode, pick = _run_args(body)
    return workflow.simulate(app, units, mode, pick).view()


@route("POST", W + "/frames")
def post_workflow_frames(app, q, body, m):
    """One arm's part of a simulated stage as 3D frames (stage -1: where it starts)."""
    units = units_of(app, m.group(1))
    side = str(body.get("side") or "")
    if side not in workflow.ARMS:
        raise UserError("Say which arm: left or right.")
    try:
        stage = int(body.get("stage"))
    except (TypeError, ValueError):
        raise UserError("Say which stage.") from None
    return workflow.frames(app, units, side, stage, str(body.get("hash") or "") or None)


@route("POST", W + "/collisions")
def post_workflow_collisions(app, q, body, m):
    """The last simulation checked with the commode in one state, for S2's 3D (``{hash, phase}``; a job)."""
    units = units_of(app, m.group(1))
    return workflow.check_in(app, units, str(body.get("hash") or ""), str(body.get("phase") or "")).view()


@route("POST", W + "/played")
def post_workflow_played(app, q, body, m):
    """The page played the simulation to its end (Real Machine Mode needs it)."""
    return workflow.played(units_of(app, m.group(1)), str(body.get("hash") or ""))


@route("POST", W + "/run")
def post_workflow_run(app, q, body, m):
    """Real Machine Mode: the run on the robot (a job), once simulated as it is."""
    session(app)
    units = units_of(app, m.group(1))
    mode, pick = _run_args(body)
    return workflow.run_real(app, units, mode, pick, bool(body.get("confirm_each", True)), body).view()
