"""Application and live camera; attach to an arm driver without starting one."""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def start(context):
    root = LaunchConfiguration('workspace').perform(context)
    mode = LaunchConfiguration('execution_mode').perform(context)
    profile = LaunchConfiguration('profile').perform(context)
    if not profile and mode != 'real':
        profile = str(Path(root) / 'src/pen_description/config/fake.yaml')
    if mode == 'real' and not profile:
        raise RuntimeError('Real mode requires an explicit measured calibration profile')
    serial = LaunchConfiguration('robot_sn').perform(context)
    worker = str(Path(root) / '.venv-pen/bin/python')
    jobs_root = str(Path(root) / 'assets/outputs')
    web_args = ['--root', root, '--mode', mode, '--port', LaunchConfiguration('port').perform(context)]
    if profile:
        web_args += ['--profile', profile]
    return [
        Node(package='pen_bringup', executable='camera_feed', output='screen'),
        Node(package='pen_image_processing', executable='image_processing_node', output='screen',
             parameters=[{'worker_python': worker, 'jobs_root': jobs_root}]),
        Node(package='pen_toolpath', executable='path_generation_node', output='screen',
             parameters=[{'worker_python': worker, 'jobs_root': jobs_root}]),
        Node(package='pen_execution', executable='executor', output='screen', parameters=[{
            'execution_mode': mode, 'robot_sn': serial,
            'execution_backend': LaunchConfiguration('execution_backend').perform(context),
            'jobs_root': jobs_root,
            'commissioning_verified': LaunchConfiguration('commissioning_verified').perform(context).lower() == 'true',
        }]),
        Node(package='pen_bringup', executable='web_bridge', arguments=web_args, output='screen'),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('workspace', default_value=str(Path.home() / 'robotics_ws')),
        DeclareLaunchArgument('execution_mode', default_value='preview', choices=['preview','fake','real']),
        DeclareLaunchArgument('execution_backend', default_value='cartesian', choices=['cartesian','joint_simulation','joint']),
        DeclareLaunchArgument('robot_sn', default_value='Rizon4s-000000'),
        DeclareLaunchArgument('profile', default_value=''),
        DeclareLaunchArgument('port', default_value='8080'),
        DeclareLaunchArgument('commissioning_verified', default_value='false'),
        OpaqueFunction(function=start),
    ])
