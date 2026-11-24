# Lab Programs 61 to 67, 25 % (19 Nov 2026)

The 13 Program files the lab's two controllers held on 24 Nov 2026, copied unchanged from the owner's folder
`Base_files_25_pct_optimized files/25_percent_scale` (`left_25`, `right_25`; its older versions were left out).

Why they are here: a RealMan controller gives a stored program's slot, name, size and speed, never its points. A
computer that did not send a Program can only simulate and time it from a file of it. These files are in the
Trajectory Viewer's built-in sets (`assets/recordings/sets.yaml`, set `lab_25_pct_19_nov`), so every computer that has
Lite finds them by the name its controller lists and checks them by the stored size.

Each file makes exactly the bytes its slot held on 24 Nov (`controller_exports/<arm>/pole_and_hand.json` of the
project Commode_C_17_10_2026), and the right arm's Program 65 ends where the controller's feed shows the arm at its end.

| Arm | Slot | File | Lines | Seconds | Stored bytes |
|---|---|---|---|---|---|
| LEFT | 61 | 61_L_P1_pole50_cleaningpose_48.2s_20pct_steady30s_plus20.txt | 9636 | 48.2 | 555156 |
| LEFT | 62 | 62_L_P2_pole200_cleaningpose_22.6s_30pct.txt | 4513 | 22.6 | 260205 |
| LEFT | 63 | 63_L_P2_pole200_grip_10.5s_25pct_new_slowstart.txt | 2100 | 10.5 | 121219 |
| LEFT | 64 | 64_L_P3_pole200_cleaningpose_19.8s_25pct_slow10-15s.txt | 3963 | 19.8 | 231884 |
| LEFT | 65 | 65_L_P3_pole200_grip_45.3s_smooth_noidle_faster.txt | 9064 | 45.3 | 514605 |
| LEFT | 66 | 66_L_P3_pole20_grip_15.6s_25pct.txt | 3119 | 15.6 | 176105 |
| LEFT | 67 | 67_L_P3_pole200_allopen_12.5s_25pct.txt | 2497 | 12.5 | 144328 |
| RIGHT | 61 | 61_R_P1_pole200_cleaningpose_53.5s_20pct_steady40s.txt | 10694 | 53.5 | 616050 |
| RIGHT | 62 | 62_R_P2_pole200_grip_13.0s_new_R04.txt | 2600 | 13.0 | 140045 |
| RIGHT | 63 | 63_R_P2_pole200_cleaningpose_9.2s_30pct.txt | 1844 | 9.2 | 104017 |
| RIGHT | 64 | 64_1_R_P2_pole50_cleaningpose_10.4s_new_R06.txt | 2072 | 10.4 | 118251 |
| RIGHT | 65 | 65_R_P3_pole50_grip_39.8s_new_R07_R08P2_30pct_slowstart.txt | 7962 | 39.8 | 458059 |
| RIGHT | 66 | 66_R_P3_pole200_allopen_wait_73.1s_25pct.txt | 14636 | 73.2 | 854278 |

The pole and hand each runs with are not stated here: the project keeps them for the programs on its controllers.
A file changed and loaded again under the same name no longer matches its slot's size; Lite then says the two may differ.
Put the new file here in its place.
