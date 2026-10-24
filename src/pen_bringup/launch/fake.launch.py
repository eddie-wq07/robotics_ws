"""Isolated fake hardware with the full application. Never connects an SDK robot."""
from pathlib import Path
import os
import subprocess
import tempfile
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def start(context):
    if os.environ.get('ROS_DOMAIN_ID', '0') == '0':
        raise RuntimeError('Use an isolated ROS domain for fake hardware: export ROS_DOMAIN_ID=73')
    root = Path(LaunchConfiguration('workspace').perform(context))
    profile = root / 'src/pen_description/config/fake.yaml'
    tmp = tempfile.TemporaryDirectory(prefix='pen-fake-')
    urdf = Path(tmp.name) / 'cell.urdf'
    result = subprocess.run(['ros2','run','pen_description','render_description','--profile',str(profile),
                             '--mode','fake','--robot-sn','Rizon4s-000000','--ros2-control'],
                            capture_output=True, text=True, check=True, timeout=20)
    urdf.write_text(result.stdout)
    gpio = Path(tmp.name) / 'gpio.yaml'
    gpio.write_text('/**/gpio_controller:\n  ros__parameters:\n    robot_sn: Rizon4s-000000\n')
    vendor = Path(get_package_share_directory('flexiv_cartesian_controller'))
    app = Path(get_package_share_directory('pen_bringup'))
    return [
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=lambda _: tmp.cleanup() or [])])),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(vendor / 'launch/bringup.launch.py')),
            launch_arguments={'robot_sn':'Rizon4s-000000','use_fake_hardware':'true','rizon_type':'Rizon4s',
                              'start_rviz':'false','description_file':str(urdf)}.items()),
        Node(package='controller_manager', executable='spawner', output='screen',
             arguments=['gpio_controller','--controller-manager','/controller_manager','-p',str(gpio)]),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(app / 'launch/app.launch.py')),
            launch_arguments={'workspace':str(root),'execution_mode':'fake','robot_sn':'Rizon4s-000000',
                              'profile':str(profile),'port':LaunchConfiguration('port').perform(context)}.items()),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('workspace', default_value=str(Path.home() / 'robotics_ws')),
        DeclareLaunchArgument('port', default_value='8080'),
        OpaqueFunction(function=start),
    ])
