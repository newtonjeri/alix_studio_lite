# Offset tests

`OFF_5P_TES.txt`: the right arm's five-point test, recorded at the lab and shared 25 Nov 2026.
- The index finger touches five red dots on a box with a grid sheet: the centre, then the four corners, then the centre
  again.
- Pole 150 mm; 8,099 lines, 40.5 s at 5 ms a line.
- The test: the UGV stands 20 mm back and 20 mm left of where it stood at the recording, so the moved Program must touch
  the same dots.

The hand tip in base_link at the recording (`motion/program_offset.py`, pole 150 mm), mm:

| Touch | Seconds | x | y | z |
|---|---|---|---|---|
| centre | 1.4-4.8 | 678.6 | -254.2 | 606.7 |
| far left | 6.4-8.8 | 772.0 | -131.3 | 610.7 |
| near left | 12.0-14.0 | 607.2 | -129.1 | 601.8 |
| near right | 17.8-20.5 | 594.7 | -377.7 | 600.3 |
| far right | 22.9-27.6 | 767.6 | -374.4 | 604.9 |
| centre | 30.8-38.4 | 681.8 | -253.4 | 604.7 |

Move it: `.venv/bin/python tools/program_offset.py --ui`, or
`.venv/bin/python tools/program_offset.py Resources/Programs/offset_tests/OFF_5P_TES.txt --side right --pole 150 --x -20 --y 20`.
