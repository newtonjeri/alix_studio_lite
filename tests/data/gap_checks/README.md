# The robot's gap checks, for the port's replay test

Copied from ALIX (`Resource/captures/<day>/checks/`, commit 842c0e59): the D435 checks the robot made on
10 and 12 October 2026 (`perception_node.py`'s gap_check, native depth, pose 4). Each `.json` is copied as it is;
each `_depth.npz` holds the record's `_depth.npy` (float32 metres, 480 x 848) as whole millimetres (`mm`, uint16,
0 where the depth was not finite): the depth came from the camera's z16 stream, so this is lossless
(`tests/test_gap_check.py` checks the values). The 12 October set is one whole lid_open run, in order.

| record | from |
|---|---|
| `20261012_093917_987_gap_check_lid_closed_lid_closed` | `2026-10-12/checks/20261012_093917_987_gap_check_lid_closed_lid_closed.json` (sha256 7ef0a513ba8267bf…), `_depth.npy` (sha256 a3922ad2a65c1fb2…) |
| `20261012_093924_332_gap_check_lid_lifted_lid_closed` | `2026-10-12/checks/20261012_093924_332_gap_check_lid_lifted_lid_closed.json` (sha256 f68c2cb819b33d6c…), `_depth.npy` (sha256 70cab64a72da8a0e…) |
| `20261012_093930_236_gap_check_lid_lifted_lid_closed` | `2026-10-12/checks/20261012_093930_236_gap_check_lid_lifted_lid_closed.json` (sha256 ba89e29cea695454…), `_depth.npy` (sha256 ee406fe6e22b9218…) |
| `20261012_093936_247_gap_check_lid_lifted_lid_closed` | `2026-10-12/checks/20261012_093936_247_gap_check_lid_lifted_lid_closed.json` (sha256 0c655ee1a82ec01a…), `_depth.npy` (sha256 49d903297d8491a3…) |
| `20261012_093942_104_gap_check_lid_lifted_lid_lifted` | `2026-10-12/checks/20261012_093942_104_gap_check_lid_lifted_lid_lifted.json` (sha256 0c719e97e095cbed…), `_depth.npy` (sha256 bce46172c53144d6…) |
| `20261012_093942_368_gap_check_seat_lifted_seat_lifted` | `2026-10-12/checks/20261012_093942_368_gap_check_seat_lifted_seat_lifted.json` (sha256 bfde70e0ea642848…), `_depth.npy` (sha256 24f9d75b6ea5a94c…) |
| `20261012_094001_199_gap_check_lid_open_lid_open` | `2026-10-12/checks/20261012_094001_199_gap_check_lid_open_lid_open.json` (sha256 074062a98af0c350…), `_depth.npy` (sha256 8947926ac7f7bd75…) |
| `20261012_094001_358_gap_check_seat_open_seat_closed` | `2026-10-12/checks/20261012_094001_358_gap_check_seat_open_seat_closed.json` (sha256 b47a7ae3c8fb9f2b…), `_depth.npy` (sha256 78109686d71fa261…) |
| `20261010_190855_049_gap_check_seat_closed_seat_closed` | `2026-10-10/checks/20261010_190855_049_gap_check_seat_closed_seat_closed.json` (sha256 91151411e75bedee…), `_depth.npy` (sha256 088bff8eae4c1674…) |
| `20261010_190903_863_gap_check_seat_lifted_seat_closed` | `2026-10-10/checks/20261010_190903_863_gap_check_seat_lifted_seat_closed.json` (sha256 1b054c6e6a7fca06…), `_depth.npy` (sha256 17d1a83d12ae0fc3…) |
| `20261010_190702_419_gap_check_seat_lifted_refused` | `2026-10-10/checks/20261010_190702_419_gap_check_seat_lifted_refused.json` (sha256 e03d741640024758…), `_depth.npy` (sha256 e37da44a2507c286…) |
