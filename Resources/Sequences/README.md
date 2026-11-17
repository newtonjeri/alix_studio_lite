# Sequences

The S2 sequences the lab sends, kept so none is lost (the user, 16 Nov). **Lite never loads from this folder.** The
sequence a project runs is its own `Resources/Projects/<project>/sequence/s2.json`, set by LOAD on the S2 page. To run
a version kept here, LOAD its two files on the S2 page: the left file into LEFT, the right file into RIGHT.

| Folder | What it is |
|---|---|
| `dual_arm_seq_15_nov_2026_v1.1/` | **Current** (lab, 15 Nov). Four versions, each pair with every Program's own pole and hand: `original`, `lid_open_checks_only`, `no_inspection`, `without_camera`. Its README says what each is. |
| `earlier/` | Earlier pairs exactly as sent or made, in the format of their day: their Program rows state no pole and hand, so ALIX Studio Demo refuses them, and Lite takes the pole and hand from its own records. `v4`, `v6`, `v7`, `v8` (10 Nov), `2026-11-12_lab_saved` (the lab's saves that v9 came from), `v9` (12 Nov), `v10_lid_open_checks_only` (16 Nov: the same rows as the current set's `lid_open_checks_only`). |

## Adding a sequence the lab sends

```
PYTHONPATH=src python3 tools/sequence_versions.py <name> --with-camera <folder> [--without-camera <folder>] --copy-to ~/Downloads
```

Each `<folder>` holds `left_workflow_sequence.json` and `right_workflow_sequence.json`. The tool writes
`Resources/Sequences/<name>/` with the versions, every file as Lite's SAVE writes it: each Program row states its own
pole and hand (`pole_mm`, `hand`), from the project's kept entries for each controller slot. Each pair is checked as
LOAD reads it and as S2 plans it, and read back after it is written. Nothing is sent to a robot.
