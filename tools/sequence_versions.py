"""Keep an S2 sequence the lab sends, with its versions, in Resources/Sequences/<name>/ (the user, 16 Nov: "a folder that
carries the sequence so that we do not lose them ... but they do not bring confusion with the sequence loaded in the
project"). Lite never loads from that folder: a version runs only when it is LOADed on the S2 page.

Each file is written as Lite's SAVE writes it (workflow.document_stated): every Program row with its own pole and hand
(``pole_mm``, ``hand``), as this computer's Lite knows them for the project, so nothing depends on a Program's name
(ALIX Studio Demo refuses a file without them). The versions of a sequence with the camera:

  original                the rows as sent
  lid_open_checks_only    no INSPECT; Open the lid keeps its mid-lift and end checks (the seat read from its end frames
                          and the Close the seat link stay); every other lid or seat task asks no camera check
  no_inspection           no INSPECT; no lid or seat task asks a camera check (the camera is never asked; Lite still
                          refuses a lid or seat task with the camera unplugged)

and, given, the pair made for running without the camera (no lid or seat task), as sent.

Each pair is checked as LOAD reads it (workflow.from_document, check_steps), planned as S2 plans it (workflow.plan_for:
the two arms' rows, a task's checks the same on both), and read back from the written file. Nothing is sent to a robot.

Usage (from the Lite repo):
  PYTHONPATH=src python3 tools/sequence_versions.py NAME --with-camera DIR [--without-camera DIR]
                [--project Resources/Projects/Commode_C_17_10_2026] [--copy-to ~/Downloads]
DIR holds left_workflow_sequence.json and right_workflow_sequence.json."""
from __future__ import annotations

import argparse
import copy
import json
import shutil
import sys
import time
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from alix_studio_lite import vendor_paths  # noqa: E402

vendor_paths.activate()

from alix_studio_lite.server import mechanism, workflow  # noqa: E402

ARMS = ("left", "right")
FILE = "{side}_workflow_sequence.json"
WORDS = {
    "original": "the rows as the lab sent them",
    "lid_open_checks_only": "no INSPECT; Open the lid keeps its mid-lift and end checks (the seat read from its end frames and "
                            "the Close the seat link stay); every other lid or seat task asks no camera check",
    "no_inspection": "no INSPECT; no lid or seat task asks a camera check (Lite still refuses a lid or seat task with the "
                     "camera unplugged; a seat that comes up with the lid is not noticed, a missed lift is not re-lifted)",
    "without_camera": "the lab's pair for running without the camera: no INSPECT and no lid or seat task, as sent",
}


def read_pair(folder: Path) -> dict:
    out = {}
    for side in ARMS:
        data = json.loads((folder / FILE.format(side=side)).read_text(encoding="utf-8"))
        out[side] = workflow.check_steps(workflow.from_document(side, data), side)       # as LOAD reads it
    return out


def kept_checks(task: str, roles: set) -> list:
    return [c["stage"] for c in mechanism.task_checks(task) if c["role"] in roles]


def version(rows: dict, which: str) -> dict:
    """``rows`` (both arms) as the version ``which``."""
    if which in ("original", "without_camera"):
        return copy.deepcopy(rows)
    out = {}
    for side, steps in rows.items():
        new = []
        for st in steps:
            if st["type"] == "inspect":
                continue
            st = copy.deepcopy(st)
            if st["type"] == "mechanism":
                st["checks"] = kept_checks(st["task"], {"lift", "end"}) if which == "lid_open_checks_only" and st["task"] == "lid_open" else []
            new.append(st)
        out[side] = workflow.check_steps(new, side)
    return out


def plan_words(rows: dict) -> str:
    run = workflow.plan_for(workflow._numbered(rows), "both")                  # as S2 plans it; refused here if it cannot run
    tasks = [st for st in run["plan"] if st["kind"] == "mechanism"]
    looks = sum(1 for st in run["plan"] if st["kind"] == "inspect")
    words = lambda c: "all its camera checks" if c is None else "no camera check" if not c else ", ".join(c)  # noqa: E731
    return (f"{len(run['plan'])} stages, {looks} INSPECT; " + ("; ".join(f"{st['title']}: {words(st.get('checks'))}" for st in tasks)
                                                              or "no lid or seat task"))


def saved(app, units, side: str, steps: list) -> dict:
    """SAVE's document, every Program row stating its own pole and hand, refused here when one does not."""
    doc = workflow.document_stated(app, units, side, steps)
    bare = [f"row {i} (Program {st['id']})" for i, st in enumerate(doc["steps"], 1)
            if st["type"] == "program" and (st.get("pole_mm") is None or st.get("hand") is None)]
    if bare:
        raise SystemExit(f"{side.upper()}: no pole and hand known for {', '.join(bare)}: state them in the Trajectory Viewer "
                         "(Set pole and hand) on this computer's project, then run this again.")
    return doc


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("name", help="the set's folder name under Resources/Sequences")
    ap.add_argument("--with-camera", type=Path, required=True)
    ap.add_argument("--without-camera", type=Path)
    ap.add_argument("--project", type=Path, default=ROOT / "Resources" / "Projects" / "Commode_C_17_10_2026")
    ap.add_argument("--out", type=Path, default=ROOT / "Resources" / "Sequences")
    ap.add_argument("--copy-to", type=Path, help="a copy of the set's folder there too")
    a = ap.parse_args(argv)

    # SAVE on a Lite whose arms are not linked: the controllers' programs as last kept with the project
    app = SimpleNamespace(robot=SimpleNamespace(_real=lambda side: None, arm=lambda side: SimpleNamespace(programs={})),
                          library=None)
    from alix_studio_lite.server.lite_units import LiteUnits      # the project as Lite opens it (read only here)

    units = LiteUnits(a.project.resolve(), "commode_c")
    sets = {"with_camera": read_pair(a.with_camera)}
    if a.without_camera:
        sets["without_camera"] = read_pair(a.without_camera)
    made = {}
    for which in ("original", "lid_open_checks_only", "no_inspection") + (("without_camera",) if a.without_camera else ()):
        rows = version(sets["without_camera" if which == "without_camera" else "with_camera"], which)
        made[which] = (rows, plan_words(rows), {s: saved(app, units, s, rows[s]) for s in ARMS})

    dest = a.out / a.name
    dest.mkdir(parents=True, exist_ok=True)
    lines = [f"# {a.name}", "",
             f"Kept {time.strftime('%Y-%m-%d %H:%M')} by tools/sequence_versions.py from `{a.with_camera}`"
             + (f" and `{a.without_camera}`" if a.without_camera else "") + ". Lite never loads from here: LOAD a version's two "
             "files on the S2 page (left into LEFT, right into RIGHT) to run it.", "",
             "Every Program row states its own pole and hand (`pole_mm`, `hand`), as Lite's SAVE writes them: here from the "
             f"project `{a.project.name}`'s kept entries for each controller slot. A file saved again from the lab's Lite "
             "states what that Lite knows.", "", "| Version | What it is | As S2 plans it |", "|---|---|---|"]
    for which, (rows, words, docs) in made.items():
        (dest / which).mkdir(exist_ok=True)
        for side in ARMS:
            path = dest / which / FILE.format(side=side)
            path.write_text(json.dumps(docs[side], indent=2), encoding="utf-8")
            back = workflow.check_steps(workflow.from_document(side, json.loads(path.read_text(encoding="utf-8"))), side)
            assert [(x.get("pole_mm"), x.get("hand"), x.get("checks")) for x in back] == \
                   [(x.get("pole_mm"), x.get("hand"), x.get("checks")) for x in docs[side]["steps"]], (which, side)
        lines.append(f"| `{which}/` | {WORDS[which]} | {words} |")
        print(f"{which}: {words}")
    lines += ["", "Each Program's pole and hand as stated in the files:", "", "| Arm | Program | Name | Pole (mm) | Hand |", "|---|---|---|---|---|"]
    seen = set()
    for rows, _w, docs in made.values():
        for side in ARMS:
            for st in docs[side]["steps"]:
                if st["type"] == "program" and (side, st["id"]) not in seen:
                    seen.add((side, st["id"]))
                    lines.append(f"| {side.upper()} | {st['id']} | `{st['name']}` | {st['pole_mm']:g} | {st['hand']} |")
    (dest / "README.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"kept: {dest}")
    if a.copy_to:
        target = a.copy_to.expanduser() / f"{a.name}_lite"
        shutil.copytree(dest, target, dirs_exist_ok=True)
        print(f"copied: {target}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
