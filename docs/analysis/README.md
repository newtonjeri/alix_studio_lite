# Analysis notes behind the ALIX Studio Lite documents

> **Note.** Lite uses the 5 ms controller cycle as its only time base (decision D26, 13 Oct 2026). Per-line averages such as 5.55 or 5.67 ms in these notes are ALIX's measurements of upload playback, kept as evidence; Lite does not use them.

These are the working notes from the 13 October 2026 analysis. The two documents in `../` summarise them:
`ALIX_Studio_Lite_Requirements_Revised.md` and `ALIX_Studio_Lite_Implementation_Plan.md`.
They are kept for the engineers (and agents) who implement Lite, because they hold the line-level evidence
(`path:line`) behind every finding.

| Folder | What it holds |
|---|---|
| `source_maps/` | 13 evidence maps, one per source: ALIX Studio UI and theme (01), simulation and collisions (02), regions/pole/hand (03), server architecture (04), kinematics and collision (05), motion and upload research (06), the trajectory prototype (07), control-app versions (08), runner and SDK (09), data formats and regions (10), robot description and emulator (11), the Operations-UI checklist (12), the regions deck catalogue (13). |
| `requirement_analyses/` | The 8 verified requirement analyses (C1–C8) that the requirement sections condense: C1 = R1, R7 · C2 = R2, R6, R11, R12, INT-TRAJ · C3 = R3, R5, R10 · C4 = R4, R13 · C5 = R8 · C6 = R9, R14 · C7 = X1, X2 and the NFRs · C8 = INT-CTRL. They predate the 13 October decisions; where they disagree with the documents, the documents win. |
| `research/` | The two research notes behind the 13 October decisions: the RealMan IO E-stop (F4) and the two optimisers with an operator target time (F2). |
| `architecture_proposals/` | The three architecture proposals (reuse-first, operator-first, safety-first) scored in the plan's §3. |
| `scripts/` | Read-only analysis scripts used during the work: regions-deck extraction (`pptx_*.py`, `arm_crops.py`, `overlay_norm.py`), V6 segment mapping by forward kinematics (`fk_*.py`, `v6_fk_*.py`), and track statistics (`txtstats.py`, `spikes.py`, `margins.py`, `seqgaps.py`, `findbridges.py`, `recon.py`). They are reference material, not Lite code. |

Path aliases used in the notes: `alix/`, `ALIX/` = `~/alix_ws/src/alix`; `lite/`, `LITE/` = `~/alix_studio_lite_ws`;
`studio/`, `TG/` = the ALIX Studio package `alix/core/python/alix_taskgen/src/alix_taskgen`. Paths that start with
`/tmp/claude-…/scratchpad` or `SP/` point to the temporary analysis workspace, which no longer exists; the files they
named are the ones in this folder (for example `SP/maps/13` = `source_maps/13_regions_ppt_visual.md`).
