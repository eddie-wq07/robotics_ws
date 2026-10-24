from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("execution_mode", default_value="preview"),
        DeclareLaunchArgument("execution_backend", default_value="cartesian"),
        DeclareLaunchArgument("robot_sn", default_value="Rizon4s-000000"),
        DeclareLaunchArgument("controller_manager", default_value="/controller_manager"),
        DeclareLaunchArgument("jobs_root", default_value="/home/ubuntu/robotics_ws/assets/outputs"),
        DeclareLaunchArgument("require_heartbeat", default_value="true"),
        Node(package="pen_execution", executable="executor", output="screen", parameters=[{
            "execution_mode": LaunchConfiguration("execution_mode"),
            "execution_backend": LaunchConfiguration("execution_backend"),
            "robot_sn": LaunchConfiguration("robot_sn"),
            "controller_manager": LaunchConfiguration("controller_manager"),
            "jobs_root": LaunchConfiguration("jobs_root"),
            "require_heartbeat": ParameterValue(LaunchConfiguration("require_heartbeat"), value_type=bool),
        }]),
    ])
