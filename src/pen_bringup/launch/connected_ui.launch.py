"""Connect only the browser to an existing real executor; no driver or motion startup."""
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def start(context):
    root = LaunchConfiguration('workspace').perform(context)
    profile = LaunchConfiguration('profile').perform(context)
    if not profile:
        profile = str(Path(root) / 'src/pen_description/config/photo_board.yaml')
    return [Node(package='pen_bringup', executable='web_bridge', output='screen',
                 arguments=['--root', root, '--mode', 'real', '--profile', profile,
                            '--port', LaunchConfiguration('port').perform(context)])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('workspace', default_value=str(Path.home() / 'robotics_ws')),
        DeclareLaunchArgument('profile', default_value=''),
        DeclareLaunchArgument('port', default_value='8096'),
        OpaqueFunction(function=start),
    ])
