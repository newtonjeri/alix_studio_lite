"""Merge + Sequence S2, Sequence (feedback v5, slide 9): V53's Page 02 on Lite's
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
    return {x["side"]: [dict(id=int(st["id"]), row=st.get("_row"), name=st.get("name")) for st in x["steps"] if st["type"] == "program"]
            for x in (s.get(side) for side in workflow.ARMS) if x}


@route("GET", W)
def get_workflow(app, q, body, m):
    """Both arms' steps, the last simulation (its stages, actions, findings and collisions) and the
    S2 run or simulation going on, if any (a page opened again follows it)."""
    out = workflow.summary(units_of(app, m.group(1)))
    s2 = sorted((j for j in app.jobs.running() if j.title.startswith("S2 ") and j.kind in ("run", "simulate")),
                key=lambda j: j.kind != "run")              # a run on the robot first; a collision check is not followed
    out["running"] = s2[0].view(10 ** 9) if s2 else None
    return out


@route("PUT", W + r"/(left|right)")
def put_workflow_side(app, q, body, m):
    """An arm's steps as edited (each checked as runner v14 checks it, with its uid)."""
    units = units_of(app, m.group(1))
    workflow.save_side(units, m.group(2), body.get("steps"))
    return workflow.summary(units)


@route("POST", W + r"/(left|right)/load")
def post_workflow_load(app, q, body, m):
    """V53's LOAD: a realman_workflow_sequence_v1 document for this arm (``document``); another arm's
    file is refused."""
    units, side = units_of(app, m.group(1)), m.group(2)
    doc = body.get("document")
    if isinstance(doc, str) and not doc.strip():
        raise UserError("Could not load the sequence: the file is empty.",
                        "A sequence saved before 22 Oct could be saved empty: save the sequence again, then load that file.")
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
    """V53's refresh_sources: the programs and the global points to choose from, as the controller holds them
    (content.py: Data Management's content, the owner, 17 Oct). ``refresh=1``: read the controller again,
    completely; otherwise as last read when that was within content.FRESH_S. The stand-in: what Lite sent."""
    from . import content

    units, side = units_of(app, m.group(1)), m.group(2)
    c = content.content(app, side, refresh=str((q.get("refresh") or [""])[0]) in ("1", "true"),
                        files=workflow._project_files(units, side, every=True))
    return {"side": side, "read": c["read"], "live": c["live"], "standin": c["standin"], "note": c["note"],
            "programs": [dict(id=p["id"], name=p.get("name"), sent=p.get("sent"), file_name=p.get("file_name"), lines=p.get("lines"),
                              seconds=p.get("seconds")) for p in c["programs"]],
            "points": sorted(p["name"] for p in c["points"]), "points_read": c["read"] if c["points"] or c["live"] else None}


@route("POST", W + "/plan")
def post_workflow_plan(app, q, body, m):
    """The check before anything moves: the plan's stages (each part with the rows its arms meet at after it, each lid
    or seat task with its rows) and counts, and what the plan says (rows passed over); or why it cannot run."""
    units = units_of(app, m.group(1))
    mode, pick = _run_args(body)
    run = workflow.plan_for(workflow._numbered(workflow.load(units)), mode, pick)
    stages = [dict(index=i, kind=s["kind"], label=str(s.get("program_id") or ""),
                   title=workflow.restore_title(s) if s["kind"] == "mechanism" else s.get("title"),
                   programs=_stage_programs(s) if s["kind"] in ("parallel", "single") else {},
                   **({"meet_rows": s["meet_rows"]} if s.get("meet_rows") else {}),
                   **({"task": s["task"], "rows": s.get("rows")} if s["kind"] == "mechanism" else {}))
              for i, s in enumerate(run["plan"])]
    return dict(mode=run["mode"], pick=run["pick"], counts=run["counts"], stages=stages, said=run.get("said") or [])


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
    return workflow.run_real(app, units, mode, pick, body).view()
