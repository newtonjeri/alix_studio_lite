# Lite's logs

ALIX Studio Lite writes its logs here, one folder per day (`YYYY-MM-DD/`). They are committed with
the repository, so a run on the lab computer can be read on any copy after a pull (`.gitignore`
says how to keep them local instead). `run_lite --logs DIR` writes them somewhere else.

| File | What it holds |
|---|---|
| `activity.jsonl` | What the operator did: every change and every refusal, with the session and the outcome, and the page's own steps (view, stage, task, arm). PINs are never written. |
| `hardware.jsonl` | Every command to a controller: its arguments, its return code, the time it took and what it means (ALIX's RealMan error tables), and the lines ALIX's functions and the control program's `rm_upload.py` print while they run. Hardware alerts too. |
| `status.jsonl` | Twice a second while an arm is connected: each arm's joints, error codes and run mode, its pole, its hand, its E-stop input, and the jobs running. |
| `lite.log` | Activity and hardware as plain text: `tail -f Resources/Logs/$(date +%F)/lite.log` follows Lite live. |
| `threads-HHMMSS.txt` | Only when Lite did not close within 30 s of Ctrl+C: what each of its threads was doing as it quit (it is also printed in the terminal). `kill -USR1 <process>` prints the same in the terminal for a Lite that stops answering. |

Each line of a `.jsonl` file is one JSON record with `t`, the time in seconds since 1970.
