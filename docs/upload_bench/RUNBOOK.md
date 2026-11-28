# Upload bench: runbook for the lab

## 1. Purpose

Answer, on a real controller:

- How fast does a Program file upload?
- Does a controller take an upload while it is running a Program: one file, then a batch?
- Does it take two uploads at once?
- What does an upload do to the run?

If uploads during a run are refused, or slow the run, Lite keeps loading Programs only between runs. If they are taken
cleanly, Lite can load the next Programs while one runs.

Nothing the controllers already hold may be damaged on the way (section 6 says how the bench makes sure).

## 2. Verified offline (on the development laptop, not the robot)

- **From the lab's own logs** (`docs/upload_bench/FROM_LOGS.md`, 179 uploads, 14 Oct to 13 Nov): every upload was
  stored and verified. The send alone runs at 46.3 KB/s plus 0.37 s per file (about 747 lines a second), the same on
  both arms (46.3 and 46.2 KB/s). The program-list reads add about 0.13 s before the send and 0.17 s after it. These
  are hardware numbers, and two other estimates agree: the uploads' total times give 47.1 KB/s, and ALIX's own upload
  tool measured 45.4 KB/s on 28 Aug.
- **The logs cannot answer the other questions.** No upload ever overlapped a run, any other motion, or another
  upload. Lite itself refuses an upload to an arm that is busy, so nobody has tried.
- **The bench's whole flow on ALIX's emulator** (`tests/test_upload_bench.py`, 7 tests, passing):
  - idle uploads, a baseline run, one upload during a run, a batch during a run, and two at once plus one to the
    other controller;
  - the mode that waits for a run started elsewhere;
  - the refusal to write over a slot that holds a lab program, at the start and again just before a send;
  - a lab program changed in the middle of the bench: the bench stops at its next check, with no further write and no
    delete;
  - a controller reading that stops answering: the bench stops sending and still deletes its own file;
  - the restore lookup: a file of the same name and size is found; a file of another size is not.
- **The upload call is Lite's own:** the same `rm_send_project` arguments as `rm_upload.py`, checked against the SDK's
  definition: store only (nothing runs), not the IO default program, stored speed 0. Every SDK call the bench makes
  is a 3rd-generation call (none of the 4th-generation-only calls that leave the SDK's lock held).
- **The lab's 13 Programs could be restored from the repository.** Left 61 to 67 and right 61 to 66 (as the
  controllers listed them on 24 Nov) each have a file of that name in `Resources/Programs/lab_25_pct_19_nov/`. For the
  six of them Lite uploaded on 12 and 13 Nov, the size the bench computes equals the size the controller reported
  then. The bench repeats this check against what the controllers hold at the start.

  The emulator stores whatever it is sent, at any time, so it proves the flow and the verdicts, not the controller's
  answers. Its timings are not a controller's.
- **The separate upload processes** start, load the real SDK, and name the controller they could not reach (tried
  against a closed port on the laptop, never against the arms).
- **Not checked offline:**
  - what a real controller answers to an upload during a run;
  - whether two uploads at once share its bandwidth;
  - whether the SDK library holds its lock for the whole send. If it does, a STOP sent from the uploading process
    would wait for the upload to end;
  - whether two sends to one controller at once are stored correctly. The bench checks each stored size and every
    other program after the test, and runs it last.

## 3. Setup

- **Machine:** the lab PC (Linux), with Lite's repository pulled after these files are committed:
  - `tools/upload_bench.py`
  - `tests/test_upload_bench.py`
  - `docs/upload_bench/`
  - `Resources/Logs/README.md`
- **Arm, session A:** left arm with no error and no program running. The pole, hand, lid and seat can be as they are:
  the arm does not move in session A.
- **Session B:** the lab runs, in Lite, a Program it already runs in the commode state it is in now. It should be
  20 s or longer, with its own pole and hand, which Lite stages as usual.
- **Lite during session A:** close it first (Ctrl+C in its terminal), so that the bench is the only client.
- **E-stops:** the red E-stops act on the controllers directly (IO3 is in the controller's own E-stop mode). Lite only
  reads them.

## 4. Commands

Run from Lite's folder on the lab PC.

**Step 1.** List what both controllers hold. This only reads.

```
python3 tools/upload_bench.py slots --side left
```

```
python3 tools/upload_bench.py slots --side right
```

**Step 2 (session A, the arm does not move).** The bench writes left slots 70 to 75 and right slot 71, only if they
are empty or hold its own UPBENCH files. If step 1 shows a lab program in any of them, the bench refuses and names the
slot. Then pick empty slots from step 1's list and change the numbers. Nothing is written before that check.

```
python3 tools/upload_bench.py live --side left --slots 70-75 --other-slots 71
```

Type `y` at "go on?", and `left` at each of the three run prompts.

**Step 3 (session B, the arm moves under Lite).** Start Lite as usual and connect the arms (`tools/run_lite.sh --robot
realman`). Then, in a second terminal:

```
python3 tools/upload_bench.py live --side left --slots 70-75 --external-run --tests during,batch --lines 1000,4000 --batch 2
```

Type `y` at "go on?". When the bench says "start a Program on the left arm in Lite now", run the chosen Program from
Lite. Do it twice: once for the single upload, once for the batch.

## 5. What will happen

**Session A, about 6 minutes:**

1. Connect, check the slots, read the arm's joints.
2. Read what each controller holds, before any write: every program's name, size and stored speed, the IO default
   program, the global points. Print how many of the lab's programs a repository file could restore, and name any it
   could not. Read this before typing `y` at "go on?".
3. Store a still Program in slot 70: 60 s of the arm's current joints, about 15 s to send.
4. **idle:** three files of 1,000, 4,000 and 11,000 lines, each sent three times (about 65 s).
5. **baseline:** the still Program runs, about 60 to 67 s.
6. **during:** the still Program runs again, and one 11,000-line upload starts 2 s into it.
7. **batch:** the still Program runs again, with three or four uploads back to back (as many as its measured length
   allows).
8. **parallel:** two 11,000-line files to the left at once and one to the right (15 to 30 s).
9. The bench deletes everything it stored and prints the summary. Its first lines say whether the controllers' data
   read the same throughout.

After step 3 and after every test, the bench reads both controllers again (under a second each time) and compares.

**Session B:** two runs of the lab's Program, started from Lite. The first gets one upload of 4,000 lines (about
5.5 s), the second two of them back to back.

**Expected, though it may look wrong:**

- "current c api version" prints at every connection, several times.
- During a still run the arm stands still for about a minute while the controller reports it running. The run prompt
  says the move to the first point is "up to 0.0 deg".
- The upload lines of a run print only after the run ends.
- A "note" that a lab Program's stored speed changed after it ran (session B), if the controller does that. A run takes
  a speed. The bench reports it and goes on: the program's name and size are compared as before.

## 6. What stops it

**The controllers' data.** RealMan's API gives no program's points back, so a damaged program can only be sent again
from a file. The bench:

- **Writes and deletes only its own slots.** It touches only `--slots` and `--other-slots`, and there only a slot that
  is empty or holds a bench file (listed as `<slot>_UPBENCH...`). This is checked at the start and again in the
  program-list read just before each send and each delete.
- **Sends store-only, as Lite does:** nothing runs, and the IO default program is not changed. It refuses to start if
  the IO default program is one of its slots.
- **Compares after every step.** It reads every program's name, size and stored speed, the IO default program and the
  global points before its first write, then after the still Program and after every test.
  - **Any change to anything not its own stops it:** no further send, and no delete either, so the controller is left
    as found for the lab to look at. The exit code is 1.
  - **A reading that fails twice, a second apart, also stops the sends.** The data is then not known to be unchanged.
    Its own files are still deleted, each slot read again first.
- **Runs the two-at-once test last,** the step the controller's behaviour is least known for.
- **On Ctrl+C,** waits for any send under way to end before deleting anything.

What a run leaves changed: the controller's global speed at 50 %, the same as after every run from Lite
(`rm_upload.py` sets it back; the controller cannot report it). In session B the bench runs nothing.

If the bench reports a change: do not run anything on that arm. Send the output back. Restore any damaged program from
the file the start of the bench named, with Advanced Robot Controls, to the same slot.

**Refused before anything is written:**

- a slot holding a program that is not the bench's;
- the IO default program in the bench's slots;
- a program already running;
- an arm error (session A);
- a controller that does not connect (the message names it).

**During session A's runs** (the control program's own `rm_upload.py` run):

- It asks for the side before each run.
- Ctrl+C slow-stops the arm.
- A run that has not ended after 150 s is slow-stopped.
- The global speed is put back to 50 % after every run.
- A run that does not complete ends the bench: the later tests are skipped.

**Ctrl+C at any time:** no new upload starts. An upload already sending finishes on its own, up to about 15 s; it
cannot be cut.

**Session B:** the bench sends no motion command and no STOP. Lite's own gates, its STOP and the red E-stops stop the
arm. The bench only stops waiting. If Lite reads the program list while a bench upload is sending, the controller may
not answer (it did not on 14 Oct). Lite then refuses that action: start it again once the bench's upload line has
printed.

**Not gated:**

- **A controller that pauses its Program when an upload arrives.** In session A the arm is still, so the bench waits
  out the 150 s and slow-stops; the record says "paused". In session B, Lite's run timeout ends it.
- **A controller that answers slowly during an upload.** Then STOP commands answer slowly too. The bench measures
  this (the watcher's reading times). The red E-stops do not depend on it.

## 7. What to send back

Commit `Resources/Logs/<today>/` as usual. It holds:

- the bench's folders `upload_bench_<time>_left/` (`summary.md`, `summary.json`, `results.jsonl`);
- for session B, Lite's own `lite.log` and `hardware.jsonl`.

Also send the terminal output of steps 1 to 3.

What will be read from them:

- `summary.md`: the answers in plain words, starting with whether the controllers' data stayed the same.
- `results.jsonl`, the `snapshot` records: what each controller held before the first write and at every check.
- `results.jsonl`, for each upload: its return code and refused line (`ret`, `err_line`) and its send time. Also how
  long the probe's write-lock call waited during the send, and how long the watcher's reading took.
- For each run: its play time, whether it paused, and any arm error.
- Lite's `hardware.jsonl` for session B: whether Lite saw anything unusual during the uploads.
