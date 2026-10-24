"""Copyright 2026 Industrial Next. All rights reserved."""
import os
import tempfile
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def load(context):
    robot_sn = LaunchConfiguration("robot_sn").perform(context)
    prefix = LaunchConfiguration("prefix").perform(context) or f"{robot_sn}_"
    manager = LaunchConfiguration("controller_manager").perform(context)
    if not robot_sn or not manager.startswith("/"):
        raise ValueError("robot_sn is required and controller_manager must be an absolute ROS name")
    config_file = LaunchConfiguration("config_file").perform(context)
    # A private parameter file supplies the exact prefix without changing shared config.
    params = {"/**/flexiv_cartesian_controller": {"ros__parameters": {
        "resource": f"{prefix}tcp", "joints": [f"{prefix}joint{i}" for i in range(1, 8)],
        "controller_manager": manager,
    }}}
    with tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", delete=False) as stream:
        yaml.safe_dump(params, stream)
        path = stream.name

    def cleanup(_context):
        Path(path).unlink(missing_ok=True)
        return []

    return [
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])),
        Node(package="controller_manager", executable="spawner", output="screen",
             arguments=["flexiv_cartesian_controller", "--inactive", "-c", manager,
                        "-t", "flexiv_cartesian_controller/FlexivCartesianController",
                        "-p", config_file, "-p", path]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot_sn", default_value=os.environ.get("FLEXIV_ROBOT_SN", "")),
        DeclareLaunchArgument("prefix", default_value=""),
        DeclareLaunchArgument("config_file", default_value=str(
            Path(get_package_share_directory("flexiv_cartesian_controller")) / "config/cartesian_controller.yaml")),
        DeclareLaunchArgument("controller_manager", default_value="/controller_manager"),
        OpaqueFunction(function=load),
    ])
