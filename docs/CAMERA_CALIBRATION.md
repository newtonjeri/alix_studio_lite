# Calibrating the right wrist camera

This measures where the D435 really sits on the right wrist. Lite's perception uses the URDF's design value, which has
never been measured. The method is RealMan's eye-in-hand calibration as ALIX implemented it
(ALIX `Resource/reports/2026-10-13_hand_eye_calibration`). Lite captures the session itself, without ROS, and solves it
with ALIX's own solver.

## What you need

- **The board.** Print `Resources/Calibration/board_10x7_30mm.svg` on A3 at 100 % ("actual size", never "fit to
  page"). It has 10 × 7 squares of 30 mm. Check the 100 mm scale bar with a ruler. Measure ten squares in a row with
  calipers and divide by ten: that is the measured square size. Glue the print flat to a rigid sheet.
  ALIX's own A3 board (12 × 9 squares) is too big here: from these poses the camera is about 0.5–0.7 m from the lid,
  and that board never fits whole in the image.
- **The cell.**
  - The commode's lid closed and its seat down.
  - The board flat on the lid, centred, its long side pointing away from the robot. Do not move it during the session.
  - The left arm at its rest.
  - Nobody inside the cell.
- **Lite.** Real Machine Mode, both arms connected, the wrist camera plugged in, and the project open (its room's
  walls are what each move is checked against).

## The session

Open **Advanced Robot Controls**, then go to **Camera calibration**.

1. Enter the **measured square** size, then press **Plan the views**. Nothing moves. Lite plans 15 views: 5 around
   each of rest, inspection pose 1 and inspection pose 4, each with a different wrist turn. Every planned view must
   meet these conditions:
   - the whole board is in the image, 0.35–0.85 m away and tilted 15–55° to it;
   - the arm stays at least 80 mm above the lid;
   - the move from the view before is clear of the room's walls, the commode and the left arm.
2. Press **Take the next view** 15 times, confirming each move. Before the first move the right pole goes to 250 mm.
   Each view goes like this:
   - the move is checked again from where the arms stand now;
   - the arm moves at 20 %;
   - once it is still, Lite takes two images 0.3 s apart and keeps the view only if the board is found in both and
     did not move between them.

   If the board is not found, that image is kept as `images/miss_NN.png` and you can take the view again.
3. Press **Back to rest**.
4. Press **Solve**. ALIX's solver runs five methods and then refines over every corner. The result is **accepted**
   when all three of these hold:
   - the board's position, seen through the arm from every view, scatters by at most 3 mm RMS;
   - the corners reproject within 1.5 px RMS;
   - the wrist turned more than 30° about each of its axes across the views.

The session is kept in `Resources/Calibration/hand_eye_<date>_<time>/`:
- `samples.yaml` and `images/`: the session in ALIX's own format;
- `plan.json`: the planned views;
- `hand_eye_result.yaml`: the result.

Commit the folder with the logs. ALIX can re-solve the same session at any time:
`python3 tools/perception/compute_hand_eye.py --data <folder> [--exclude N ...]`.

## Applying the result

Nothing is applied automatically. The result gives the camera's offset from the URDF's mount in two forms:
- millimetres and degrees in the wrist frame;
- the replacement `<origin>` for the camera mount.

Applying it is a change to the URDF in ALIX, which is then vendored into Lite. Lite's camera place references
(`Resources/Logs/place_reference.json`, taken with the camera as it was) must be taken again after that.
