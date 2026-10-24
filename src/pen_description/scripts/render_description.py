#!/usr/bin/env python3
"""Render one validated pen cell without opening a robot connection."""
import argparse
import hashlib
from pathlib import Path
import sys

try:
    from pen_description.calibration import load_profile, xacro_mappings
except ModuleNotFoundError:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from pen_description.calibration import load_profile, xacro_mappings


def render(profile_path, mode='preview', robot_sn=None, rizon_type='Rizon4s', ros2_control=False):
    import xacro
    from ament_index_python.packages import get_package_share_directory
    profile = load_profile(profile_path, mode=mode, robot_sn=robot_sn)
    root = Path(__file__).resolve().parents[1]
    if not (root / 'urdf').is_dir():
        root = Path(get_package_share_directory('pen_description'))
    mappings = xacro_mappings(profile)
    kinematics = profile.get('robot_kinematics')
    if kinematics:
        path = Path(kinematics['path']).resolve()
        if (kinematics.get('robot_sn') != profile['robot_sn'] or
                hashlib.sha256(path.read_bytes()).hexdigest() != kinematics.get('sha256')):
            raise ValueError('Factory kinematics identity or file digest does not match the profile')
        mappings['kinematics_parameters_file'] = str(path)
    mappings.update(robot_sn=profile['robot_sn'], rizon_type=rizon_type,
                    use_fake_hardware=str(mode != 'real').lower(),
                    ros2_control=str(ros2_control).lower())
    return xacro.process_file(str(root / 'urdf/pen_cell.urdf.xacro'), mappings=mappings).toxml()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile', required=True)
    parser.add_argument('--mode', choices=['preview', 'fake', 'real'], default='preview')
    parser.add_argument('--robot-sn')
    parser.add_argument('--rizon-type', default='Rizon4s')
    parser.add_argument('--ros2-control', action='store_true')
    args = parser.parse_args()
    try:
        print(render(args.profile, args.mode, args.robot_sn, args.rizon_type, args.ros2_control))
    except (ValueError, OSError) as error:
        parser.exit(2, f'Cannot render pen cell: {error}\n')


if __name__ == '__main__':
    main()
