from pathlib import Path
import xml.etree.ElementTree as ET
import hashlib
import runpy

import pytest
import yaml

from pen_description.calibration import load_profile, xacro_mappings, relative, compose, transform

ROOT = Path(__file__).resolve().parents[1]


def test_factory_kinematics_are_pinned_to_robot_and_file(tmp_path):
    render = runpy.run_path(str(ROOT / 'scripts/render_description.py'))['render']
    from ament_index_python.packages import get_package_share_directory
    template = Path(get_package_share_directory('flexiv_description')) / 'config/Rizon4s/default_kinematics.yaml'
    kinematics = tmp_path / 'kinematics.yaml'
    kinematics.write_bytes(template.read_bytes())
    profile = yaml.safe_load((ROOT / 'config/fake.yaml').read_text())
    profile['robot_kinematics'] = {'path': str(kinematics), 'robot_sn': profile['robot_sn'],
                                  'sha256': hashlib.sha256(kinematics.read_bytes()).hexdigest()}
    path = tmp_path / 'profile.yaml'
    path.write_text(yaml.safe_dump(profile))
    assert ET.fromstring(render(path, mode='fake')).tag == 'robot'
    kinematics.write_text(kinematics.read_text() + '\n# changed\n')
    with pytest.raises(ValueError, match='kinematics identity or file digest'):
        render(path, mode='fake')
    profile['robot_kinematics']['sha256'] = hashlib.sha256(kinematics.read_bytes()).hexdigest()
    profile['robot_kinematics']['robot_sn'] = 'different-robot'
    path.write_text(yaml.safe_dump(profile))
    with pytest.raises(ValueError, match='kinematics identity or file digest'):
        render(path, mode='fake')


def test_missing_measurements_are_not_identity():
    with pytest.raises(ValueError, match='TBD'):
        load_profile(ROOT / 'config/calibration.template.yaml', mode='real')


def test_fake_cannot_be_used_as_real():
    with pytest.raises(ValueError, match='non-synthetic'):
        load_profile(ROOT / 'config/fake.yaml', mode='real')


@pytest.mark.parametrize('delay', [0.0, 0.15, -0.1, float('nan'), float('inf'), True, None])
def test_real_release_delay_allows_immediate_lift_but_rejects_invalid_values(tmp_path, delay):
    profile = yaml.safe_load((ROOT / 'config/fake.yaml').read_text())
    profile.update(calibrated=True, fake_only=False, synthetic=False)
    profile['visual_servo']['enabled'] = False
    profile['pen']['release_delay_s'] = delay
    filename = tmp_path / 'real.yaml'
    filename.write_text(yaml.safe_dump(profile))
    if delay in (0.0, 0.15) and not isinstance(delay, bool):
        assert load_profile(filename, mode='real')['pen']['release_delay_s'] == delay
    else:
        with pytest.raises(ValueError, match='pen.release_delay_s'):
            load_profile(filename, mode='real')


def test_jig_and_surface_share_transform():
    profile = load_profile(ROOT / 'config/fake.yaml', mode='fake')
    mappings = xacro_mappings(profile)
    jig = compose(transform(profile, 'T_world_base'), transform(profile, 'T_base_jig'))
    surface = compose(jig, relative(jig, transform(profile, 'T_world_surface')))
    assert surface[0] == pytest.approx(profile['T_world_surface']['xyz_m'])
    # Flat underside seats at 72.5 mm; the drawing face is at 75 mm.
    assert list(map(float, mappings['T_jig_surface_xyz'].split())) == pytest.approx([0, 0, 0.075])


def test_rotated_base_preserves_world_surface():
    profile = load_profile(ROOT / 'config/fake.yaml', mode='fake')
    profile['T_world_base'] = {'xyz_m': [0.1, 0.2, 0.3], 'quaternion_xyzw': [0, 0, 2**-0.5, 2**-0.5]}
    world_jig = compose(transform(profile, 'T_world_base'), transform(profile, 'T_base_jig'))
    world_surface = compose(world_jig, relative(world_jig, transform(profile, 'T_world_surface')))
    assert world_surface[0] == pytest.approx(profile['T_world_surface']['xyz_m'])
    assert world_surface[1] == pytest.approx(profile['T_world_surface']['quaternion_xyzw'])


def test_invalid_quaternion_rejected(tmp_path):
    profile = yaml.safe_load((ROOT / 'config/fake.yaml').read_text())
    profile['T_world_surface']['quaternion_xyzw'] = [0, 0, 0, 0]
    filename = tmp_path / 'invalid.yaml'
    filename.write_text(yaml.safe_dump(profile))
    with pytest.raises(ValueError, match='unit length'):
        load_profile(filename, mode='fake')


def test_rendered_cell_retains_hardware_and_fixed_fixture():
    import xacro
    profile = load_profile(ROOT / 'config/fake.yaml', mode='fake')
    mappings = xacro_mappings(profile)
    mappings.update(robot_sn='Rizon4s-000000', rizon_type='Rizon4s',
                    ros2_control='true', use_fake_hardware='true')
    doc = ET.fromstring(xacro.process_file(str(ROOT / 'urdf/pen_cell.urdf.xacro'), mappings=mappings).toxml())
    assert len(doc.findall('ros2_control')) == 1
    assert doc.find('ros2_control/hardware/plugin').text == 'mock_components/GenericSystem'
    assert doc.find("joint[@name='flange_to_pen_tip']/parent").get('link') == 'Rizon4s-000000_flange'
    assert doc.find("joint[@name='base_to_pen_jig']/parent").get('link') == 'Rizon4s-000000_base_link'
    assert doc.find("joint[@name='jig_to_pen_surface']").get('type') == 'fixed'
    assert doc.find("ros2_control/gpio[@name='Rizon4s-000000_tcp']") is not None
    assert len(doc.findall("ros2_control/joint")) == 7
    for mesh in doc.findall('.//mesh'):
        assert mesh.get('filename').startswith('package://')
    assert doc.find("link[@name='pen_holder']/visual/geometry/mesh").get('filename').endswith('/pen_holder_combined.stl')
    assert doc.find("link[@name='pen_body']/visual") is None


def test_direct_unconfigured_xacro_rejected():
    import xacro
    with pytest.raises(xacro.XacroException, match='valid calibration'):
        xacro.process_file(str(ROOT / 'urdf/pen_cell.urdf.xacro'))


def test_camera_requires_its_measured_mount(tmp_path):
    profile = yaml.safe_load((ROOT / 'config/fake.yaml').read_text())
    profile['camera']['enabled'] = True
    profile['camera']['T_flange_camera'] = {'xyz_m': None, 'quaternion_xyzw': None}
    filename = tmp_path / 'camera.yaml'
    filename.write_text(yaml.safe_dump(profile))
    with pytest.raises(ValueError, match='T_flange_camera'):
        load_profile(filename, mode='fake')


def test_optional_camera_composes_vendor_model():
    import xacro
    profile = load_profile(ROOT / 'config/fake.yaml', mode='fake')
    profile['camera'] = {'enabled': True, 'model': 'D405', 'T_flange_camera':
                         {'xyz_m': [0.05, 0, 0.02], 'quaternion_xyzw': [0, 0, 0, 1]}}
    mappings = xacro_mappings(profile)
    mappings.update(robot_sn='Rizon4s-000000', rizon_type='Rizon4s', use_fake_hardware='true')
    doc = ET.fromstring(xacro.process_file(str(ROOT / 'urdf/pen_cell.urdf.xacro'), mappings=mappings).toxml())
    assert doc.find("joint[@name='camera_joint']/parent").get('link') == 'Rizon4s-000000_flange'
    assert doc.find("link[@name='camera_link']") is not None


def test_generic_servo_is_attached_to_holder():
    import xacro
    profile = load_profile(ROOT / 'config/fake.yaml', mode='fake')
    mappings = xacro_mappings(profile)
    mappings.update(robot_sn='Rizon4s-000000', rizon_type='Rizon4s', use_fake_hardware='true')
    doc = ET.fromstring(xacro.process_file(str(ROOT / 'urdf/pen_cell.urdf.xacro'), mappings=mappings).toxml())
    assert doc.find("joint[@name='holder_to_visual_servo']/parent").get('link') == 'pen_holder'
    assert len(doc.findall("link[@name='pen_visual_servo']/visual")) == 4


def test_generic_servo_requires_synthetic_profile(tmp_path):
    profile = yaml.safe_load((ROOT / 'config/fake.yaml').read_text())
    profile.update(fake_only=False, synthetic=False)
    filename = tmp_path / 'generic-servo.yaml'
    filename.write_text(yaml.safe_dump(profile))
    with pytest.raises(ValueError, match='preview/fake-only'):
        load_profile(filename, mode='preview')


@pytest.mark.parametrize('radius', [float('nan'), float('inf'), None, True, 0.075])
def test_invalid_surface_radius_rejected(tmp_path, radius):
    profile = yaml.safe_load((ROOT / 'config/fake.yaml').read_text())
    profile['surface']['type'] = 'sphere'
    profile['surface']['radius_m'] = radius
    filename = tmp_path / 'surface.yaml'
    filename.write_text(yaml.safe_dump(profile))
    with pytest.raises(ValueError, match='radius_m'):
        load_profile(filename, mode='fake')


@pytest.mark.parametrize('serial', [None, '', 'TBD', 'Rizon4s-063112-extra'])
def test_robot_identity_required(tmp_path, serial):
    profile = yaml.safe_load((ROOT / 'config/fake.yaml').read_text())
    profile['robot_sn'] = serial
    filename = tmp_path / 'identity.yaml'
    filename.write_text(yaml.safe_dump(profile))
    with pytest.raises(ValueError, match='robot_sn'):
        load_profile(filename, mode='fake')


def test_plane_profile_cannot_disagree_with_mesh(tmp_path):
    profile = yaml.safe_load((ROOT / 'config/fake.yaml').read_text())
    profile['surface'] = {'type': 'plane', 'size_m': [0.2, 0.2, 0.005]}
    filename = tmp_path / 'plane.yaml'
    filename.write_text(yaml.safe_dump(profile))
    with pytest.raises(ValueError, match='canonical flat STL'):
        load_profile(filename, mode='fake')


@pytest.mark.parametrize('base', [
    {'xyz_m': [0.01, 0, 0], 'quaternion_xyzw': [0, 0, 0, 1]},
    {'xyz_m': [0, 0, 0], 'quaternion_xyzw': [0, 0, 2**-0.5, 2**-0.5]},
])
def test_real_control_cannot_use_shifted_rdk_base(tmp_path, base):
    profile = yaml.safe_load((ROOT / 'config/fake.yaml').read_text())
    profile.update(calibrated=True, fake_only=False, synthetic=False, T_world_base=base)
    filename = tmp_path / 'real.yaml'
    filename.write_text(yaml.safe_dump(profile))
    with pytest.raises(ValueError, match='world must equal the RDK base frame'):
        load_profile(filename, mode='real')


def test_real_plane_requires_explicit_dimensions(tmp_path):
    profile = yaml.safe_load((ROOT / 'config/fake.yaml').read_text())
    profile.update(calibrated=True, fake_only=False, synthetic=False, surface={'type': 'plane'})
    filename = tmp_path / 'real-plane.yaml'
    filename.write_text(yaml.safe_dump(profile))
    with pytest.raises(ValueError, match='surface.size_m'):
        load_profile(filename, mode='real')
