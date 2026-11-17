# dual_arm_seq_15_nov_2026_v1.1

Kept 2026-11-17 00:07 by tools/sequence_versions.py from `/home/newtonjeri/Downloads/dual_arm_seq_15_nov_2026_v1.1/arm_with_camera` and `/home/newtonjeri/Downloads/dual_arm_seq_15_nov_2026_v1.1/arm_without_camera`. Lite never loads from here: LOAD a version's two files on the S2 page (left into LEFT, right into RIGHT) to run it.

Every Program row states its own pole and hand (`pole_mm`, `hand`), as Lite's SAVE writes them: here from the project `Commode_C_17_10_2026`'s kept entries for each controller slot. A file saved again from the lab's Lite states what that Lite knows.

| Version | What it is | As S2 plans it |
|---|---|---|
| `original/` | the rows as the lab sent them | 13 stages, 4 INSPECT; Open the lid: all its camera checks; Open the seat: all its camera checks; Close the lid: all its camera checks |
| `lid_open_checks_only/` | no INSPECT; Open the lid keeps its mid-lift and end checks (the seat read from its end frames and the Close the seat link stay); every other lid or seat task asks no camera check | 8 stages, 0 INSPECT; Open the lid: verify_lid_lifted, verify_lid_open; Open the seat: no camera check; Close the lid: no camera check |
| `no_inspection/` | no INSPECT; no lid or seat task asks a camera check (Lite still refuses a lid or seat task with the camera unplugged; a seat that comes up with the lid is not noticed, a missed lift is not re-lifted) | 8 stages, 0 INSPECT; Open the lid: no camera check; Open the seat: no camera check; Close the lid: no camera check |
| `without_camera/` | the lab's pair for running without the camera: no INSPECT and no lid or seat task, as sent | 2 stages, 0 INSPECT; no lid or seat task |

Each Program's pole and hand as stated in the files:

| Arm | Program | Name | Pole (mm) | Hand |
|---|---|---|---|---|
| LEFT | 61 | `61_61_L_P1_pole50_cleaningpose_3` | 50 | [1000, 1000, 1000, 1000, 0, 1000] |
| LEFT | 62 | `62_62_L_P2_pole200_cleaningpose_` | 200 | [1000, 1000, 1000, 1000, 0, 1000] |
| LEFT | 63 | `63_63_L_P2_pole200_grip_10.5s_25` | 200 | [0, 0, 0, 0, 150, 1000] |
| LEFT | 64 | `64_64_L_P3_pole200_cleaningpose_` | 200 | [1000, 1000, 1000, 1000, 0, 1000] |
| LEFT | 65 | `65_65_L_P3_pole200_grip_80.8s_sm` | 200 | [0, 0, 0, 0, 150, 1000] |
| RIGHT | 61 | `61_61_R_P1_pole200_cleaningpose_` | 210 | [1000, 1000, 1000, 1000, 0, 1000] |
| RIGHT | 62 | `62_62_R_P2_pole200_grip_9.5s_25p` | 200 | [0, 0, 0, 0, 150, 1000] |
| RIGHT | 63 | `63_63_R_P2_pole200_cleaningpose_` | 200 | [1000, 1000, 1000, 1000, 0, 1000] |
| RIGHT | 64 | `64_64_1_R_P2_pole50_cleaningpose` | 50 | [1000, 1000, 1000, 1000, 0, 1000] |
| RIGHT | 65 | `65_65_R_P3_pole50_grip_28.1s_new` | 50 | [0, 0, 0, 0, 150, 1000] |
