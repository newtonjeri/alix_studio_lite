# Runbook: Programs 61 to 65 made by the automatic optimisation, three sets, first run on the arms

Written 19 November 2026 on the laptop. Nothing below has run on the arms. Every number is from files or
from Lite's simulation on its stand-in arms; none is a hardware reading.

The three sets are in Lite's repository, `alix_studio_lite/Resources/Programs/auto_19_nov_2026/`, one folder
for each method of the Tune page:

| Folder | Method |
|---|---|
| `toppra/` | TOPP-RA |
| `ruckig_s/` | Ruckig on s |
| `custom/` | Jerk and UGV load |

Each holds ten Programs (`left/`, `right/`), the sequence files that run them (`sequence/<version>/`) and
`FILES.md`. The three are run the same way, one after the other, in the order the lab chooses.

## 1. Purpose

For each method: do its Programs 61 to 65 run to their end on both arms in the lab's sequence? And how do
the three compare with each other and with the hand-tuned files: the run's time, how the arms and the UGV
behave, how the pad cleans?

A result that changes the next step: a Program ended early by its controller, a controller code (4105 or
another), the UGV seen to rock or creep, the two arms closer than the simulation shows, or a surface the
pad no longer cleans.

## 2. Verified offline

On this laptop:

- **The 30 files**: each within every hard limit of Lite when it was made, and again when the written
  file was read back (joint limits, speed, 600 deg/s^2 acceleration, step, path, first and last line,
  jerk 1500 rad/s^3, load on the UGV 52.9 N m at the file's own pole height). The highest reading of each
  set's ten files:

| Set | Fastest joint, share of its speed limit | Acceleration, deg/s^2 | Jerk, rad/s^3 | Load on the UGV, N m (model) | Pad from the path it was made from, mm |
|---|---|---|---|---|---|
| `toppra/` | 0.93 | 612 | 422 | 19.6 | 5.8 |
| `ruckig_s/` | 0.97 | 611 | 458 | 18.2 | 6.3 |
| `custom/` | 0.99 | 612 | 456 | 20.3 | 7.0 |
| the limit | 1.00 | 612 (600 and the reading's 2 %) | 1500 | 52.9 | 10.0 |

- **Ends**: each file's first and last line are those of the file it was made from (0.0000 deg off), so
  every Program starts and ends where the lab's does now.
- **What each was made from**: the hand-tuned file's path where that file is not on its base file's path
  (left 61, 63, 65; right 61, 62, 64, 65), the base file for the others (left 62, 64; right 63). The
  right's 61 is made, named and stated at a 210 mm pole.
- **The sequence in Lite's S2 simulation** (stand-in arms, each set's own `sequence/original/` files
  LOADed, overall speed 60 %): done, no stop, with every set. Section 5 has the times.
- **The sequence files**: all 24 read by Lite's LOAD and planned by S2 as the lab's own files are, and
  all 24 accepted by ALIX Studio Demo's upload reader.
- **Lite's tests**: `results/tests.txt` of the report; no code has changed since.

Not checked offline, which is what the run is for:

- how the real controllers follow these lines, and what the joints draw;
- what the UGV does: its load here is worked out from the model's masses, never measured;
- the cleaning contact, with the pad up to 7.0 mm from the path it was made from and faster over the
  surface;
- whether the controllers' own early ends stay away;
- the two arms at the places of section 6: the simulation's hands are its model's;
- the names the controllers list after the upload: the sequence files state them by the rule read from
  the lab's logs of 13 and 15 November, not from a controller.

## 3. Setup

- **Computer**: the lab PC.
- **Code**: Lite's branch `lite_auto_optimization` with `Resources/Programs/auto_19_nov_2026/` in it.
  Running these files needs no code newer than `8ce14fdc`.
- **To put the lab's files back afterwards**: the hand-tuned files of slots 61 to 65 as the lab has them
  (they are also in ALIX's report folder, `inputs/hand_tuned/`).
- **Slots**: a set's ten files go into slots 61 to 65 with Replace. That overwrites what is there; step 8
  puts the hand-tuned Programs back.
- **Lid and seat timing**: planned, for these runs. The lid and seat on Ruckig on s have not run on the
  arms either (ALIX's report `2026-11-18_lite_auto_optimisation/RUNBOOK_lid_seat_ruckig.md`), and two new
  things in one run cannot be told apart. If that run has been done and accepted, leave
  `LITE_MECHANISM_TIMING` out of step 2.
- **Robot**: as for the lab's sequence runs: the UGV at the plans' 250 mm from commode C, lid and seat
  down, the wrist camera connected, both hands and poles answering.

## 4. Commands

From the folder that holds `alix_studio_lite/`.

1. The branch:

```bash
git -C alix_studio_lite switch lite_auto_optimization && git -C alix_studio_lite pull
```

2. Lite on the arms, the lid and seat on their planned timing:

```bash
LITE_MECHANISM_TIMING=planned alix_studio_lite/tools/run_lite.sh --robot realman
```

3. One set's ten files. In the page, Advanced Robot Controls; for each row choose the arm, the file of the
   set's `left/` or `right/` whose name starts with that Program ID, enter the Program ID, tick Replace,
   set the pole and the hand, Upload Only. The set's `FILES.md` lists the ten files.

| Arm | Program ID | Pole | Hand |
|---|---|---|---|
| LEFT | 61 | 50 mm | cleaning pose |
| LEFT | 62 | 200 mm | cleaning pose |
| LEFT | 63 | 200 mm | grip |
| LEFT | 64 | 200 mm | cleaning pose |
| LEFT | 65 | 200 mm | grip |
| RIGHT | 61 | 210 mm | cleaning pose |
| RIGHT | 62 | 200 mm | grip |
| RIGHT | 63 | 200 mm | cleaning pose |
| RIGHT | 64 (the file starts `64_1_`) | 50 mm | cleaning pose |
| RIGHT | 65 | 50 mm | grip |

   Cleaning pose is 1000, 1000, 1000, 1000, 0, 1000; grip is 0, 0, 0, 0, 150, 1000.

4. S2: LOAD the set's two sequence files, the left file into LEFT and the right file into RIGHT, from the
   version the lab runs: `sequence/original/` (the lab's rows of 15 November, with the INSPECT rows),
   `lid_open_checks_only/`, `no_inspection/` or `without_camera/`. SIMULATE. Then, in its 3D, look at the
   set's three places of section 6 before going on.

5. Run it in Hardware sim (nothing moves).

6. Tick the 20 ms feed box, then Real. (The feed slows the Programs by about 4.5 %; it is what shows how
   the joints followed.)

7. The next set: steps 3 to 6 with its folder.

8. To put the hand-tuned files back: step 3 with the lab's ten hand-tuned files and the same poles and
   hands, then LOAD the lab's own sequence
   (`Resources/Sequences/dual_arm_seq_15_nov_2026_v1.1/<version>/`) and SIMULATE.

## 5. What will happen

From the simulation (the `original` version), seconds from a Program's start to its end, and the whole
run:

| Arm | Program | Hand-tuned | `toppra/` | `ruckig_s/` | `custom/` |
|---|---|---|---|---|---|
| LEFT | 61 | 50.6 | 41.6 | 48.1 | 42.0 |
| LEFT | 62 | 23.8 | 17.6 | 21.4 | 17.4 |
| LEFT | 63 | 11.2 | 8.1 | 9.1 | 7.9 |
| LEFT | 64 | 20.9 | 18.6 | 22.3 | 18.2 |
| LEFT | 65 | 47.6 | 48.1 | 53.7 | 50.9 |
| RIGHT | 61 | 56.1 | 41.8 | 48.3 | 41.5 |
| RIGHT | 62 | 13.8 | 8.2 | 9.0 | 8.0 |
| RIGHT | 63 | 12.3 | 9.9 | 11.5 | 9.8 |
| RIGHT | 64 | 11.0 | 7.8 | 9.5 | 7.6 |
| RIGHT | 65 | 41.8 | 28.1 | 32.2 | 27.5 |
| **The whole run** | | **272.1** | **242.5** | **262.7** | **244.8** |

The lab's two passing runs of 13 November took 267 and 264 s with the files of that day.

Expected, though it will look different from the hand-tuned files:

- the arms start, turn and stop more gently, and go faster on the easy stretches: up to 0.93 to 0.99 of
  a joint's speed limit, by set;
- the left's 65 is as long as the hand-tuned one or longer (0.5 to 6 s, by set); everything else is
  shorter, except the left's 64 with `ruckig_s/`;
- the right arm ends its parts earlier than before and waits longer for the left.

## 6. What stops it

On its own:

- Lite refuses a Real run of a sequence not simulated as it is, with the files the slots hold now;
- a controller that ends a Program early: Lite says "program N ended after X s of the Y s its points take"
  and stops the run;
- a controller code, a silent hand or pole, the E-stop, a hot joint;
- in the lid and seat tasks, their own gates and camera checks, as before.

By hand, with STOP. Where the simulation shows the two arms closest while both run a Program, for each set
("at the worst timing": the closest over 22 timings, each arm early or late within the ranges the robot
shows):

| Set | About, into the run | LEFT | RIGHT | As simulated | At the worst timing |
|---|---|---|---|---|---|
| `toppra/` | 29 s | 61, its wrist (link 6), line 1,391 | 61, its index finger, line 1,153 | 28 mm apart | touching, the right 0.5 s behind |
| `toppra/` | 103 s | 62, its index fingertip, line 2,233 | 63, its palm, line 188 | over 50 mm | touching, the right 1.5 s ahead |
| `toppra/` | 157 s | 64, its thumb, line 538 | 65, its wrist connector, line 358 | over 50 mm | 21 mm |
| `ruckig_s/` | 30 s | 61, its wrist (link 6), line 1,620 | 61, its index finger, line 1,389 | 23 mm apart | touching, the right 0.5 s behind |
| `ruckig_s/` | 111 s | 62, its index fingertip, line 2,555 | 63, its palm, line 284 | 42 mm apart | touching, the right 1.2 s ahead |
| `ruckig_s/` | 169 s | 64, its index finger, line 1,781 | 65, its link 6, line 1,959 | over 50 mm | 25 mm |
| `custom/` | 29 s | 61, its wrist (link 6), line 1,380 | 61, its index finger, line 1,103 | 9 mm apart | touching, the right 0.6 s behind |
| `custom/` | 104 s | 62, its thumb, line 2,191 | 63, its palm, line 214 | over 50 mm | touching, the right 1.7 s ahead |
| `custom/` | 157 s | 64, its thumb, line 518 | 65, its link 7, line 343 | over 50 mm | 21 mm |

With the hand-tuned files the simulation shows other places: the two thumbs touching in the 61s at 62 s,
the left's 64 against the right's 65 at 182 s (31 mm, touching at the worst timing), the two 65s at 224 s
(18 mm). So the operator's eye has to be on new spots. The second place of each set, the left's 62 against
the right's 63, is new with these files: with the hand-tuned files that pair never comes within 50 mm.

**No gate covers the two arms against each other while they run Programs on the robot**: since 16 November
a Real run does no collision checking, and each arm's file is timed on its own. What there is, is the
simulation and STOP. If the look at step 4 shows the hands meeting in the left's 62 and the right's 63, do
not run that set as it is: the gate to add is a Wait for row before the right's 63 (it waits for the left's
62 to end), which costs about 8 to 11 s by set. That is a change to the lab's sequence, so it is a
decision, not something already made.

## 7. What to send back

For each set, the day's folder `Resources/Logs/<date>/`:

- `lite.log`: which files went into the slots (the `Upload <file> to <ARM> program <slot>` lines), each
  Program's start and end against the time its lines take, any early end, any code;
- `hardware.jsonl`: the stages, the start-together rows, the gates;
- the S2 feed of the run (20 ms): joint positions against the lines sent, joint currents and voltages.

And what was seen, set by set: whether the UGV moved or rocked and at which Program, how the pad cleaned
on each surface, and how close the arms came at the three places.

From these I will read, per Program and per set, next to the last runs with the hand-tuned files: whether
it ran to its end, how far the joints were behind their lines at the fast stretches and the turns, the
current peaks, and the run's time.
