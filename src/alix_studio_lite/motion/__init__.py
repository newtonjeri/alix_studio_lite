"""Joint-space motion basics shared by both optimisers and the robot I/O.

- ``track``: read and write ``.txt`` tracks (one ``{"point":[7 integer millidegrees]}`` per line).
- ``timing``: the upload player's timing law, which turns line counts into robot seconds.
- ``limits``: the arm's limits (URDF positions and speed ratings, 600 deg/s^2, 10 deg per line).
- ``checks``: the hard limits L1-L6 and the checks C1-C4, judged on the integer bytes.
"""
