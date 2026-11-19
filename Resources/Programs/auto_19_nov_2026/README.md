# auto_19_nov_2026: Programs 61 to 65 made by the automatic optimisation, one set for each method

Made 2026-11-19 22:38 on the laptop from the files the lab sent on 19 November (base files v1.1 and the hand-tuned files of slots 61 to 65). **None of these files has run on the arms.** Lite never loads from this folder.

Three sets, one for each method of the Tune page. Each holds the ten Programs of the lab's sequence (five an arm) and the sequence files that run them. They are for uploading, running and comparing at the lab.

| Folder | Method (as the Tune page names it) | LEFT 61 to 65, s | RIGHT 61 to 65, s | Fastest joint, share of its speed limit | Highest acceleration, deg/s^2 | Highest jerk, rad/s^3 | Highest load on the UGV, N m | Furthest from the path it was made from, mm |
|---|---|---|---|---|---|---|---|---|
| `toppra/` | TOPP-RA | 127.1 | 88.3 | 0.93 | 612 | 422 | 19.6 | 5.8 |
| `ruckig_s/` | Ruckig on s | 146.8 | 102.4 | 0.97 | 611 | 458 | 18.2 | 6.3 |
| `custom/` | Jerk and UGV load | 129.4 | 87.0 | 0.99 | 612 | 456 | 20.3 | 7.0 |
| | the hand-tuned files the lab runs now | 146.4 | 125.8 | | | | | |
| | **the limit** | | | 1.00 | 612 (600 and the reading's 2 %) | 1500 | 52.9 | 10.0 |

## The limits every file is within

Each of the 30 files passed all of Lite's hard limits when it was made, and passed them again when the written file was read back: joint limits, joint speed, joint acceleration (600 deg/s^2), step size, the path (the contact pad within 10 mm of the path the file was made from), the first and the last line (those of the file it was made from), **joint jerk (1500 rad/s^3)** and **the load on the UGV (52.9 N m, half of what the UGV's weight holds, worked out at the pole height the Program runs at)**. These are the values the Tune page uses with nothing set on it. The load on the UGV is worked out from the model's link masses; it has never been measured.

## What each file was made from

A slot whose hand-tuned file follows another path than its base file is made from the hand-tuned file's path, the one the lab runs now; the others are made from the base file. Only the path is taken: the timing is the method's own.

| Arm | Slot | Made from | Why |
|---|---|---|---|
| LEFT | 61 | `61_L_P1_pole50_cleaningpose_48.2s_20pct_steady30s_plus20.txt` (hand-tuned file) | the hand-tuned file is up to 0.6 deg from the base file's path |
| LEFT | 62 | `2_L_P2_R05_PART1+R05_PART2_pole200_cleaningpose_13.9s_v1.1.txt` (base file) | the hand-tuned file follows the base file's path |
| LEFT | 63 | `63_L_P2_pole200_grip_10.5s_25pct_new_slowstart.txt` (hand-tuned file) | the hand-tuned file is up to 44.2 deg from the base file's path and ends 43.9 deg from its end |
| LEFT | 64 | `4_L_P3_R07_PART1+R07_PART2_pole200_cleaningpose_14.1s_v1.1.txt` (base file) | the hand-tuned file follows the base file's path |
| LEFT | 65 | `65_L_P3_pole200_grip_45.3s_smooth_noidle_faster.txt` (hand-tuned file) | the hand-tuned file is up to 90.1 deg from the base file's path and ends 172.9 deg from its end |
| RIGHT | 61 | `61_R_P1_pole200_cleaningpose_53.5s_20pct_steady40s.txt` (hand-tuned file) | the hand-tuned file is up to 1.3 deg from the base file's path |
| RIGHT | 62 | `62_R_P2_pole200_grip_13.0s_new_R04.txt` (hand-tuned file) | the hand-tuned file is up to 42.3 deg from the base file's path and ends 24.9 deg from its end |
| RIGHT | 63 | `3_R_P2_R05_pole200_cleaningpose_5.7s_v1.1.txt` (base file) | the hand-tuned file follows the base file's path |
| RIGHT | 64 | `64_1_R_P2_pole50_cleaningpose_10.4s_new_R06.txt` (hand-tuned file) | the hand-tuned file is up to 94.6 deg from the base file's path and ends 19.0 deg from its end |
| RIGHT | 65 | `65_R_P3_pole50_grip_39.8s_new_R07_R08P2_30pct_slowstart.txt` (hand-tuned file) | the hand-tuned file is up to 43.0 deg from the base file's path |

The right arm's 61 is made, named and stated at a 210 mm pole (the base and hand-tuned files' names say 200; the lab's sequence sets 210 mm before it).

## In each set's folder

- `left/`, `right/`: the five Programs of that arm. A file's name starts with its slot and states its pole and hand.
- `sequence/<version>/left_workflow_sequence.json`, `right_workflow_sequence.json`: the lab's sequence of 15 November (`Resources/Sequences/dual_arm_seq_15_nov_2026_v1.1`) in its four versions, with this set's Programs. The files are in the form Lite's SAVE writes and ALIX Studio Demo accepts: every Program row states the program's name as the controller lists it, its pole (`pole_mm`) and its hand (`hand`).
- `FILES.md`: the ten files with slot, pole, hand, seconds, the name the controller will list, and sha256.

`MANIFEST.json` here lists every file of the three sets with what it was made from, the limits asked for, each check's reading and its sha256.

## At the lab, for one set

1. Advanced Robot Controls: for each of the ten files choose the arm, the file and its Program ID (the number its name starts with), tick Replace, set the pole and the hand as its name and `FILES.md` say, Upload Only.
2. S2: LOAD the set's two sequence files (the left file into LEFT, the right file into RIGHT), in the version the lab runs.
3. SIMULATE, look at the 3D, then Hardware sim, then Real.
4. The next set: the same three steps with its folder. Uploading a set replaces the Programs in slots 61 to 65.

**Upload from Advanced Robot Controls.** That page sends the first 28 characters of a file's name, and the controller lists the program as `<slot>_<those characters>.txt` cut at 32 characters (so a listed name here ends in a full stop). That is the name the sequence files state, taken from how the lab's controllers listed the files sent that way on 13 and 15 November. Lite runs a Program by its slot, whatever its name. ALIX Studio Demo runs it only when the slot holds a program of exactly the stated name: after an upload from another page, which sends more of the name, LOAD the sequence in Lite and SAVE it again, and SAVE writes each name as the controller lists it.

Because the name is cut, a slot's listed name does not say which method its file is from: the cleaning pose Programs list under one name in all three sets. The day's `lite.log` line `Upload <file> to <ARM> program <slot>` does.

The first run of these files on the arms has a runbook: `RUNBOOK.md` in this folder.

## The sequence versions

| Version | As S2 plans it (the same for the three sets) |
|---|---|
| `sequence/original/` | 13 stages, 4 INSPECT; Open the lid: all its camera checks; Open the seat: all its camera checks; Close the lid: all its camera checks |
| `sequence/lid_open_checks_only/` | 8 stages, 0 INSPECT; Open the lid: verify_lid_lifted, verify_lid_open; Open the seat: no camera check; Close the lid: no camera check |
| `sequence/no_inspection/` | 8 stages, 0 INSPECT; Open the lid: no camera check; Open the seat: no camera check; Close the lid: no camera check |
| `sequence/without_camera/` | 2 stages, 0 INSPECT; no lid or seat task |

## Kept in two places

The same files are in ALIX's report `Resource/reports/2026-11-19_lite_auto_programs_61_65/programs/auto_19_nov_2026/`, with the harness that made them (`harness/make_lab_sets.py`; `--check` compares both places with the manifest), the lab's own files they were made from (`inputs/`) and the simulations.
