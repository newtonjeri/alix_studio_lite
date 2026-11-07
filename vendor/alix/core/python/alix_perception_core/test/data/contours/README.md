# Depth frames for the contour reading (alix_perception_core.depth_contours)

The lab's own checks, 5-6 Nov 2026 (ALIX Studio Lite's Resources/Logs/<day>/checks): the D435's native depth in mm (`<stem>_depth.npz`, key `depth_mm`, 848 x 480) with the record's intrinsics, camera pose and context (`<stem>.json`, `what` says what the photo shows). The new robot's frame (6 Nov) is the one the fixture fit read lifted 55 deg with the lid closed.

- `20261106_215826_599_gap_check_lid_closed_uncertain`: lid closed, the new robot (camera pose 20 mm / 3.4 deg off; the fit read lifted 55 deg) (the lab: uncertain)
- `20261105_230601_739_gap_check_lid_closed_lid_closed`: lid closed, inspection pose 4 (the lab: lid_closed)
- `20261105_230104_379_gap_check_lid_closed_lid_closed`: lid resting on the seat after a closing push (~10 deg), a closing's end check (the lab: lid_closed)
- `20261105_230718_285_gap_check_lid_closed_lid_open`: lid open, seat down, inspection pose 4 at pole 50 (the lab: lid_open)
- `20261105_005754_226_gap_check_seat_open_seat_lifted`: lid open, the seat standing against it, pose 1 (the lab: seat_lifted)
- `20261105_033823_273_gap_check_lid_lifted_lid_lifted`: lid held lifted ~30 deg by the hand (the lab: lid_lifted)
- `20261105_034936_683_gap_check_lid_open_lid_open`: lid standing open, pose 1 (the lab: lid_open)
- `20261106_175505_250_gap_check_lid_closed_refused`: floor only, no commode in view (pole 10) (the lab: refused)

Isaac Sim renders (6 Nov, 1920 x 1080 ideal depth through the robot's own wrist camera prim, the synthetic washroom): `isaac_closed_pose4`, `isaac_open_pose4` (ALIX's `generated/datasets/commode_c_soil_v6_inspect`), and `isaac_lifted_<deg>deg` rendered with `ros/simulation/isaacsim/config/sdg/commode_c_lifted.yaml` (the lid held 12-50 deg from closed); `randomised.open_angle_deg` is the rendered angle (negative = raised).
