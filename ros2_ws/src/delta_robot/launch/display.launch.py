"""Mock robot + RViz (+ optional web bridge).

    ros2 launch delta_robot display.launch.py preset:=edu_servo web_bridge:=true

Arguments: preset, design_file, use_rviz (true), web_bridge (false), hardware (mock), port, scene.
The URDF is generated from the design at launch time (deltarobot.urdf.generate) and passed
to robot_state_publisher as robot_description.
"""

from delta_robot.launch_util import make_launch


def generate_launch_description():
    return make_launch(hardware="mock", use_rviz="true")
