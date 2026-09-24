"""Backends: where the joint-angle stream goes.

record  — virtual clock, keeps frames (Playground, dry runs, analysis)
websim  — PC Python -> web simulator over ws://127.0.0.1:8765
serial  — Arduino firmware (firmware/delta_servo)
ros2    — delta_robot driver (/delta/joint_command)
"""
