"""delta_pick_place_demo — the same DeltaRobot student code, running on ROS 2.

    ros2 launch delta_robot display.launch.py preset:=edu_servo
    ros2 run delta_robot delta_pick_place_demo --preset edu_servo --cycles 3

The library's ros2 backend streams /delta/joint_command straight to delta_driver
(the commander is not needed for this script).  Use the same preset as the launch file.
"""

from __future__ import annotations

import argparse
from typing import Optional, Sequence

from deltarobot import DeltaRobot
from deltarobot.robot import EmergencyStop, WorkspaceError


def pick_place(robot: DeltaRobot, cycles: int, height: float = 0.03) -> int:
    """Move `cycles` parts to the bins; returns the number of moves done (any backend)."""
    robot.home()
    parts = robot.parts()
    bins = robot.bins()
    if not parts or not bins:
        print("scene has no parts / bins")
        return 0
    for k in range(cycles):
        part = parts[k % len(parts)]
        box = bins[k % len(bins)]
        print("cycle %d: %s -> bin %s" % (k + 1, part["id"], box["id"]))
        robot.arch_to(part["x"], part["y"], part["top"], height=height)
        robot.tool_on()
        robot.wait(0.3)
        # carried part bottom ends just above the bin rim
        robot.arch_to(box["x"], box["y"], part["top"] + box["h"], height=height + box["h"])
        robot.tool_off()
        robot.wait(0.2)
    robot.home()
    return cycles


def run(preset: str, cycles: int, height: float) -> int:
    robot = DeltaRobot(preset, backend="ros2", scene="pick_place")
    try:
        done = pick_place(robot, cycles, height)
        print("done (%d cycles)" % done)
        return 0 if done else 1
    except EmergencyStop as e:
        print("emergency stop: %s" % e)
        return 2
    except WorkspaceError as e:
        print("workspace error: %s" % e)
        return 3
    finally:
        robot.close()


def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser(description="Pick-and-place loop over ROS 2 (delta_driver must be running)")
    ap.add_argument("--preset", default="edu_dynamixel")
    ap.add_argument("--cycles", type=int, default=3)
    ap.add_argument("--height", type=float, default=0.03, help="arch height [m]")
    # ros2 run may append --ros-args ...; ignore anything we do not know
    a, _unknown = ap.parse_known_args(argv)
    return run(a.preset, max(1, a.cycles), a.height)


if __name__ == "__main__":
    raise SystemExit(main())
