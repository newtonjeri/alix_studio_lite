# Calibrating the right wrist camera

This measures where the D435 really sits on the right wrist. Lite's perception uses the URDF's design value, which has
never been measured. The method is RealMan's eye-in-hand calibration as ALIX implemented it
(ALIX `Resource/reports/2026-10-13_hand_eye_calibration`). Lite captures the session itself, without ROS, and solves it
with ALIX's own solver.

Open it from the Operator Journey: **Camera Calibration**, just below Data Management.

## What you need

- **The board.** Two options:
  - Print `Resources/Calibration/board_10x7_30mm.svg` (10 × 7 squares of 30 mm) on A3 at 100 % ("actual size",
    never "fit to page"). Check the 100 mm scale bar with a ruler, then glue the print flat to a rigid sheet.
  - Or use the lab's rigid calibration plate, and enter its squares along and across on the page. One count must be
    even and the other odd.

  Either way, measure ten squares in a row with calipers and divide by ten: that is the measured square size.
  ALIX's own A3 board (12 × 9 squares of 30 mm) is too big here: from these poses the camera is 0.5–0.7 m from the
  lid, and that board never fits whole in the image.
- **The cell.**
  - The commode's lid closed and its seat down.
  - The board flat on the lid, near its middle. It may be turned any way; the plan says how far from the middle it
    may lie (20 mm for the 300 × 210 mm board, 40 mm for a 240 × 180 mm plate). Do not move it during the session.
  - The left arm at its rest.
  - Nobody inside the cell.
- **Lite.** The project open (its room's walls are what each move is checked against), the right arm connected, and
  the wrist camera plugged in.

## The session

1. Enter the board's **squares along**, **squares across** and the **measured square**, then press **Plan the
   views**. Nothing moves. Lite plans 15 views: 5 around each of rest, inspection pose 1 and inspection pose 4, each
   with a different wrist turn. Every planned view must meet these conditions:
   - the whole board is in the image, turned any way and anywhere within the plan's distance of the lid's middle,
     0.35–0.85 m from the camera, and tilted 15–55° to the image;
   - the arm stays at least 80 mm above the lid;
   - the MoveJ from the view before is clear of the room's walls, the commode, the board lying on the lid and the
     left arm. A MoveJ moves straight in the joints, so the path checked is the path the controller runs.
2. **Check it offline.**
   - **Show the tour in 3D** plays every move. Nothing is sent to the robot.
   - In **Hardware sim**, **Run the calibration** sends the same MoveJs to the controllers in their SIM mode: the
     controllers run every move, the arm does not move, and nothing is captured.
3. In **Real Machine Mode**, press **Run the calibration**. Lite asks once, then does it all in one go:
   - the right pole goes to 250 mm;
   - the arm visits the 15 views at 40 %. Before each move, it is checked again from where both arms actually stand;
   - at each view, once the arm is still, Lite takes two images 0.3 s apart and keeps the view only if the board is
     found in both and did not move between them. A view whose board is not found is tried once more, then left out;
   - the arm goes back to rest, and the views are solved.

   **STOP** stops the run. Running again carries on from the next view; **Back to rest** brings the arm back first.
4. The result is **accepted** by ALIX's limits when all three of these hold:
   - the board's position, seen through the arm from every view, scatters by at most 3 mm RMS;
   - the corners reproject within 1.5 px RMS;
   - the wrist turned more than 30° about each of its axes across the views.

   **Solve again** re-solves the views already taken.

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
