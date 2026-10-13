"""alix_dispatch — mode 1 of contract C16: the task config IS the program.

ROS 2-INDEPENDENT by ruling (Newton, 2026-08-19). Nothing here imports rclpy,
moveit or any ROS package: mode 1 walks a task config's `stage_sequence` in
order and issues the controller's own planning calls. It needs no MoveIt plan
and no recorded trajectory JSON.
"""
