# Programs

Program files made on the laptop for the lab to upload, run and compare, kept with the repository so that a `git pull`
brings them to the lab's computer. **Lite never loads from this folder.** A Program reaches a controller only when it
is uploaded from the page (Advanced Robot Controls), and a sequence runs only when its two files are LOADed on the S2
page.

| Folder | What it is |
|---|---|
| `auto_19_nov_2026/` | Programs 61 to 65 of both arms made by the automatic optimisation, one set for each method (`toppra/`, `ruckig_s/`, `custom/`), each with the lab's sequence of 15 November stating that set's Programs. Its README says what each file was made from and the limits every file is within; `RUNBOOK.md` is for their first run on the arms. Not yet run on the arms. |

## The form of a set

```
<name>/
  README.md        what the set is, the limits, what each file was made from
  MANIFEST.json    every file: what it was made from, the limits asked for, each check's reading, sha256
  <method>/
    FILES.md       the ten files: Program ID, pole, hand, seconds, the name the controller will list
    left/  right/  the Program files; a name starts with its slot and states its pole and hand
    sequence/<version>/left_workflow_sequence.json, right_workflow_sequence.json
```

Every sequence file is in the form Lite's SAVE writes and ALIX Studio Demo accepts: each Program row states the
program's name as its controller lists it, its pole (`pole_mm`) and its hand (`hand`). The versions are those of
`Resources/Sequences/<the lab's set>/`.

## The same files are in ALIX

Each folder here has a copy, byte for byte, in ALIX's report folder of its day
(`alix_ws/src/alix/Resource/reports/<day>_.../programs/<name>/`), beside the harness that made it and the files it was
made from. The harness writes both places in one go and compares them with the manifest:

```
cd ~/alix_studio_lite_ws/alix_studio_lite
PYTHONPATH=src:<report>/harness /usr/bin/python3 <report>/harness/make_lab_sets.py --check
```

A file here is never edited by hand: it is made again by the harness, which replaces the folder in both places.
