DUAL ARM V6 - TIMING MATCHED + MERGED PACKAGE
================================================

Point timing model
------------------
Trajectory cadence used: 5 ms per point.
All timing-matched trajectories were stretched only by adding/interpolating
points along the same recorded joint path. No source trajectory was sped up.

Pole timing model
-----------------
Measured input: 10 mm -> 300 mm takes 4.0 s.
Average pole speed: 72.5 mm/s.
50 mm <-> 150 mm estimated time: 1.379 s.
For synchronization the files use 1.380 s.

Final motion plan
-----------------
Flows 1-7 and 12-14 are time matched.
Flows 8-11 remain alternating:
LEFT P8 -> RIGHT P8 -> LEFT P9 -> RIGHT P9.

Flow 15 / 16:
LEFT P13 and RIGHT P13 START TOGETHER.
RIGHT may finish earlier; no finish matching is applied.

Manual operation
----------------
After Flow 5:
LID OPEN MANUALLY.
Then both poles move to 150 mm before Flow 6.

Important merge detail
----------------------
RIGHT P4 and P5 are NOT merged because pole movement is required between them.

LEFT P3-P5 remains continuous:
P3 -> 1.380 s moving bridge -> P4 -> 1.380 s moving bridge -> P5.
Those two LEFT bridge intervals are intended to run while the RIGHT pole moves.

Large transition bridge
-----------------------
RIGHT P6 -> P7 has a large joint-pose gap.
A 1.600 s minimum-jerk joint-space bridge was inserted.
Its estimated peak joint-speed scale is approximately the same as the
previously hardware-tested smooth bridge.

Safety
------
The generated bridges are JOINT-SPACE bridges only.
They are NOT collision-checked against the toilet, cubicle walls, second robot,
end-effector, or right-arm camera.
Test BRIDGES_TEST_ONLY files first at reduced speed / with immediate stop ready,
especially:
  - L_P01_to_P02_BRIDGE_1.250s.txt
  - R_P06_to_P07_BRIDGE_1.600s.txt

JSON
----
JSON/ contains exactly two workflow JSON files:
  * LEFT_V6_FINAL.json
  * RIGHT_V6_FINAL.json

Both files contain the full arm sequence in one JSON. A top-level
manual_checkpoint field marks the required LID OPEN MANUALLY point after
Flow 5. IMPORTANT: the current runner does not automatically pause on this
metadata field; operator/manual checkpoint handling must still be respected.

CURRENT UI LIMITATION
---------------------
The existing v51 "RUN BOTH // PROGRAM RULES" waits for matching Program IDs
to finish as a pair. The optimized merge intentionally has different file
boundaries on LEFT and RIGHT (for example LEFT P1-P2 while RIGHT P1-P3).
Therefore v51 cannot execute the complete optimized overlapping timeline
correctly yet.

The per-arm JSON files are suitable for individual-arm testing.
DUAL_FLOW_TIMELINE_V6.json is the reference for the next dual-scheduler update.
Do NOT force these JSONs through current RUN BOTH Program Rules expecting the
optimized overlap.

Generated final trajectory runtimes
-----------------------------------
LEFT   L_P01P02.txt            P1 + P2            35.025 s
LEFT   L_P03P05.txt            P3 + P4 + P5       54.915 s
LEFT   L_P06P07.txt            P6 + P7            33.720 s
LEFT   L_P10P12.txt            P10 + P11 + P12    51.070 s
LEFT   M3_OPNST01.txt          P8                  2.275 s
LEFT   M3_OPNST02.txt          P9                  1.295 s
LEFT   R15_BUPFUL.txt          P13                48.705 s
RIGHT  M3_ASOPN01.txt          P8                  1.025 s
RIGHT  M3_ASOPN02.txt          P9                  2.605 s
RIGHT  R10_HNGR.txt            P5                 17.665 s
RIGHT  R2_LIDTOP.txt           P4                 18.755 s
RIGHT  R_P01P03.txt            P1 + P2 + P3       52.140 s
RIGHT  R_P06P07.txt            P6 + P7            33.720 s
RIGHT  R_P10P12.txt            P10 + P11 + P12    51.070 s
RIGHT  R_SPILLAGE.txt          P13                28.285 s


PROGRAM ID REMAP
----------------
All Program IDs in the final LEFT/RIGHT workflow JSON files were shifted by +70.
Examples: 1->71, 3->73, 4->74, 5->75, 6->76, 8->78, 9->79, 10->80, 13->83.
