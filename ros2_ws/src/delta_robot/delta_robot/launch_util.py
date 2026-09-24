"""Shared launch description for display.launch.py and bringup.launch.py."""

from __future__ import annotations

import os
from typing import Any, Dict, List

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from . import core


def _truthy(text: str) -> bool:
    return text.strip().lower() in ("1", "true", "yes", "on")


def _setup(context: LaunchContext, *args: Any, **kwargs: Any) -> List[Any]:
    cfg = {k: LaunchConfiguration(k).perform(context) for k in
           ("preset", "design_file", "use_rviz", "web_bridge", "hardware", "port", "scene")}
    from deltarobot import urdf as durdf
    design = core.load_design(cfg["preset"], cfg["design_file"])
    urdf_xml = durdf.generate(design)
    design_params: Dict[str, Any] = {"preset": cfg["preset"], "design_file": cfg["design_file"]}

    actions: List[Any] = [
        Node(package="robot_state_publisher", executable="robot_state_publisher", output="screen",
             parameters=[{"robot_description": ParameterValue(urdf_xml, value_type=str)}]),
        Node(package="delta_robot", executable="delta_driver", output="screen",
             parameters=[dict(design_params, hardware=cfg["hardware"], port=cfg["port"])]),
        Node(package="delta_robot", executable="delta_commander", output="screen",
             parameters=[design_params]),
    ]
    if _truthy(cfg["web_bridge"]):
        actions.append(Node(package="delta_robot", executable="delta_web_bridge", output="screen",
                            parameters=[dict(design_params, scene=cfg["scene"])]))
    if _truthy(cfg["use_rviz"]):
        rviz_cfg = os.path.join(get_package_share_directory("delta_robot"), "rviz", "delta.rviz")
        actions.append(Node(package="rviz2", executable="rviz2", output="log",
                            arguments=["-d", rviz_cfg]))
    return actions


def make_launch(hardware: str = "mock", use_rviz: str = "true") -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument("preset", default_value="edu_dynamixel",
                              description="catalog preset: edu_servo | edu_dynamixel | printer_stepper | industrial_picker"),
        DeclareLaunchArgument("design_file", default_value="",
                              description="design JSON exported from the web simulator (overrides preset)"),
        DeclareLaunchArgument("use_rviz", default_value=use_rviz),
        DeclareLaunchArgument("web_bridge", default_value="false",
                              description="start the WebSocket bridge for the web simulator (ws://127.0.0.1:8765)"),
        DeclareLaunchArgument("hardware", default_value=hardware, description="mock | serial"),
        DeclareLaunchArgument("port", default_value="/dev/ttyACM0", description="serial port (hardware:=serial)"),
        DeclareLaunchArgument("scene", default_value="pick_place",
                              description="scene sent to the web simulator: pick_place | conveyor | drawing | empty"),
        OpaqueFunction(function=_setup),
    ])
