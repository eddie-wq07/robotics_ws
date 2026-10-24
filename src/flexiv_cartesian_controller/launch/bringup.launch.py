"""Copyright 2026 Industrial Next. All rights reserved."""
import os
import resource
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def bringup(context):
    robot_sn = LaunchConfiguration("robot_sn").perform(context)
    if not robot_sn:
        raise ValueError("Set FLEXIV_ROBOT_SN or pass robot_sn:=<serial>")
    fake = LaunchConfiguration("use_fake_hardware").perform(context).lower() in ("true", "1")
    if not fake and os.geteuid() != 0 and resource.getrlimit(resource.RLIMIT_RTPRIO)[0] < 50:
        raise RuntimeError(
            "Real robot bringup requires an RT-priority allowance of 50. "
            "Check ulimit -r; open a fresh SSH/login session or a new tmux server."
        )
    package = Path(get_package_share_directory("flexiv_cartesian_controller"))
    vendor = Path(get_package_share_directory("flexiv_bringup"))
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(vendor / "launch/rizon.launch.py")),
            launch_arguments={
                "robot_sn": robot_sn,
                "rizon_type": LaunchConfiguration("rizon_type"),
                "use_fake_hardware": LaunchConfiguration("use_fake_hardware"),
                "start_rviz": LaunchConfiguration("start_rviz"),
                "robot_controller": "rizon_arm_controller",
                "description_file": LaunchConfiguration("description_file"),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(package / "launch/load_controller.launch.py")),
            launch_arguments={
                "robot_sn": robot_sn,
                "config_file": LaunchConfiguration("config_file"),
            }.items(),
        ),
    ]


def generate_launch_description():
    package = Path(get_package_share_directory("flexiv_cartesian_controller"))
    return LaunchDescription([
        DeclareLaunchArgument("robot_sn", default_value=os.environ.get("FLEXIV_ROBOT_SN", "")),
        DeclareLaunchArgument("rizon_type", default_value=os.environ.get("FLEXIV_ROBOT_TYPE", "Rizon4s")),
        DeclareLaunchArgument("use_fake_hardware", default_value="false"),
        DeclareLaunchArgument("start_rviz", default_value="false"),
        DeclareLaunchArgument("description_file", default_value=str(
            Path(get_package_share_directory("flexiv_description")) / "urdf/rizon.urdf.xacro")),
        DeclareLaunchArgument("config_file", default_value=str(package / "config/cartesian_controller.yaml")),
        OpaqueFunction(function=bringup),
    ])
