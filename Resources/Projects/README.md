# Lite's projects

ALIX Studio Lite keeps each project of the Lite UI here, in a folder named exactly as the project was
named when it was created (`Toilet 01/`). Renaming a project in the Project Manager renames the
project, not its folder. `run_lite --projects DIR` keeps them somewhere else.

A project's folder holds its record (`project.json`, with the project's id) and its files:

| Folder | What it holds |
|---|---|
| `trajectories/left`, `trajectories/right` | Each region's recordings, kept versions, transitions and their records, by arm. |
| `merged/left`, `merged/right` | S1's merges, and the files loaded to merge. |
| `sequence/` | S2's sequences and runs. |
| `simulation/`, `controller_exports/`, `telemetry/` | The handoff's other folders. |

These folders are not committed by Lite (`.gitignore` says how to keep them out of `git status`):
deleting the clone, or `git clean -x`, deletes them. A project file (Project Manager, Save Project
File) carries a project to another computer.

What belongs to this computer stays in `~/ALIX-Lite/projects/`: the last project opened, the
projects deleted (`.trash/`), and the projects Lite kept there by id before 16 October, which the
first start of this version copied here (`copied.json` lists them).
