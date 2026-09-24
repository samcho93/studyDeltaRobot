"""Real robot (Arduino serial firmware), no RViz by default.

    ros2 launch delta_robot bringup.launch.py preset:=edu_servo port:=/dev/ttyACM0

Arguments: preset, design_file, use_rviz (false), web_bridge (false), hardware (serial), port, scene.
"""

from delta_robot.launch_util import make_launch


def generate_launch_description():
    return make_launch(hardware="serial", use_rviz="false")
