# HOW TO UPLOAD AND RUN A .txt plan file

A drag-taught `.txt` → a controller slot → the arm. Run from `/home/alai-labs/upload_channel/`.

## Once per terminal tab: set the SDK path

Set this once when you open a tab. Every command in that tab uses it, so you don't have to pass `--sdk`:

```bash
export ALIX_RM_SDK=/home/alai-labs/alix_ws/src/alix/external/butterfli-realman-alix/Python
```

To check that it is set: `echo $ALIX_RM_SDK`. A new tab starts without it, so run the `export` again there.
Passing `--sdk PATH` on a command still works and overrides the variable for that command.


This is a standalone uploader and runner, which uses only the RM API2 and skips the
certificate, the pole and the hand:

## Commands

### Upload only
```bash
   python3 rm_upload.py --sdk $ALIX_RM_SDK FILE.txt NN --side left  # upload + verify
```

### Run
```bash
   python3 rm_upload.py --sdk $ALIX_RM_SDK --run NN --side left   # run a slot
```

### Upload and Run
```bash
   python3 rm_upload.py --sdk $ALIX_RM_SDK FILE.txt NN --side left --run  
```

### List Occupied slots
```bash
   python3 rm_upload.py --sdk $ALIX_RM_SDK --list --side left
```
## Example session

```text
alai-labs@alailabs:~/upload_channel$ export ALIX_RM_SDK=/home/alai-labs/alix_ws/src/alix/external/butterfli-realman-alix/Python
alai-labs@alailabs:~/upload_channel$ python3 rm_upload.py --sdk $ALIX_RM_SDK plan_txt_files/FILE_01.1.txt 85 --side left 
current c api version:  1.1.6
upload FILE_01.1.txt: 12429 points, 712.8 KB -> slot 85 on 192.168.1.10
  slot 85 now: empty
  write it? [y/N] y
  sent: ret=0 err_line=-1
  VERIFIED: slot 85 holds '85_FILE_01.1.txt', 729857 bytes (sent 729857)
alai-labs@alailabs:~/upload_channel$ python3 rm_upload.py --sdk $ALIX_RM_SDK --run 85 --side left             # run a slot

```

## Notes

- The controller slots for taught tracks are 40–59. Without `--slot`, `certify` takes the first free one.
- If a command says it has no SDK path, the tab is missing the `export` above.