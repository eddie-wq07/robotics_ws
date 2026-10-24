"""The browser can show CAD registration without changing control calibration."""
import hashlib
from types import SimpleNamespace
from unittest.mock import patch

import pytest
import yaml

from pen_bringup.model import Model


URDF = '''<robot name="test">
  <link name="flange"/>
  <link name="pen_holder"><visual name="shell"><geometry>
    <mesh filename="package://pen_description/meshes/pen_holder.stl"/>
  </geometry><material><color rgba="0.2 0.3 0.4 1"/></material></visual></link>
  <joint name="mount" type="fixed"><parent link="flange"/><child link="pen_holder"/>
    <origin xyz="0 0 0.15" rpy="0 0 0"/></joint>
</robot>'''


@pytest.fixture
def model_root(tmp_path):
    config = tmp_path / 'src/pen_description/config'
    config.mkdir(parents=True)
    (config / 'fake.yaml').write_text(yaml.safe_dump({
        'synthetic': True, 'fake_only': True, 'robot_sn': 'Rizon4s-000000'}))
    (config / 'calibration.mocha.yaml').write_text(yaml.safe_dump({
        'T_flange_pen_tip': {'xyz_m': [.224709, .002292, -.001993]},
        'T_base_jig': {'xyz_m': [.508, 0, 0],
                       'quaternion_xyzw': [0, 0, 2**-.5, 2**-.5]}}))
    holder = tmp_path / 'assets/meshes/pen_holder.stl'
    flange = tmp_path / 'src/flexiv_description/meshes/Rizon4s/visual/link7.obj'
    for file in (holder, flange):
        file.parent.mkdir(parents=True, exist_ok=True)
        file.write_bytes(b'test mesh')
    checksum = hashlib.sha256(b'test mesh').hexdigest()
    (config / 'visualization.yaml').write_text(yaml.safe_dump({'joint_snapshot': {
        'robot_sn': 'Rizon4s-063112', 'positions_rad': [.1, .2, .3, .4, .5, .6, .7],
        'read_started_utc': '2026-10-24T08:40:44Z'}, 'holder_mount': {
        'xyz_m': [-.011, 0, .10379], 'quaternion_xyzw': [0, .70710678, 0, .70710678],
        'holder_mesh_sha256': checksum, 'flange_mesh_sha256': checksum}}))
    return tmp_path


def test_registration_is_separate_from_control_frames(model_root):
    with patch('pen_bringup.model.subprocess.run', return_value=SimpleNamespace(returncode=0, stdout=URDF)):
        model = Model(model_root).get()
    assert model['available']
    assert model['joints'][0]['xyz'] == [0, 0, .15]
    assert model['visualization']['holder_mount']['xyz_m'] == [-.011, 0, .10379]
    assert model['visualization']['jig_mount']['xyz_m'] == [.508, 0, 0]
    assert model['visualization']['joint_snapshot']['joints']['Rizon4s-000000_joint2'] == .2
    assert model['links'][1]['visuals'][0]['rgba'] == [.2, .3, .4, 1]


def test_changed_measurements_invalidate_browser_model(model_root):
    model = Model(model_root)
    with patch('pen_bringup.model.subprocess.run', return_value=SimpleNamespace(returncode=0, stdout=URDF)):
        first = model.get()
        path = model_root / 'src/pen_description/config/calibration.mocha.yaml'
        path.write_text(yaml.safe_dump({'T_flange_pen_tip': {'xyz_m': [.2, 0, 0]}}))
        second = model.get()
    assert first['model_revision'] != second['model_revision']
    assert first['profile_source_sha256'] == second['profile_source_sha256']
    assert 'pen_tip' not in second['visualization']['measurements']


def test_real_profile_keeps_its_own_mount(model_root):
    path = model_root / 'src/pen_description/config/fake.yaml'
    path.write_text(yaml.safe_dump({'synthetic': False, 'fake_only': False}))
    with patch('pen_bringup.model.subprocess.run', return_value=SimpleNamespace(returncode=0, stdout=URDF)):
        model = Model(model_root, mode='real').get()
    assert model['visualization']['holder_mount'] is None
    assert model['visualization']['jig_mount'] is None
    assert model['visualization']['joint_snapshot'] is None
    assert model['joints'][0]['xyz'] == [0, 0, .15]


def test_registration_rejects_changed_cad(model_root):
    (model_root / 'assets/meshes/pen_holder.stl').write_bytes(b'changed mesh')
    with patch('pen_bringup.model.subprocess.run', return_value=SimpleNamespace(returncode=0, stdout=URDF)):
        result = Model(model_root).get()
    assert not result['available']
    assert 'does not match' in result['message']


def test_embedded_pbr_asset_uses_asset_endpoint(model_root):
    urdf = URDF.replace('package://pen_description/meshes/pen_holder.stl',
                        'package://pen_description/assets/tylenol/tylenol.glb')
    with patch('pen_bringup.model.subprocess.run', return_value=SimpleNamespace(returncode=0, stdout=urdf)):
        model = Model(model_root).get()
    assert model['available']
    assert model['links'][1]['visuals'][0]['url'] == '/api/asset/tylenol/tylenol.glb'


def test_real_model_uses_recorded_home_for_offline_preview(model_root):
    path = model_root / 'src/pen_description/config/fake.yaml'
    path.write_text(yaml.safe_dump({'robot_sn': 'Rizon4s-063112',
        'home': {'joint_positions_rad': [.1]*7, 'recorded_utc': '2026-10-24'}}))
    with patch('pen_bringup.model.subprocess.run', return_value=SimpleNamespace(returncode=0, stdout=URDF)):
        model = Model(model_root, mode='real').get()
    assert model['available']
    snapshot = model['visualization']['joint_snapshot']
    assert snapshot['joints']['Rizon4s-063112_joint1'] == .1
    assert 'not live feedback' in snapshot['source']


@pytest.mark.parametrize('positions', [[0]*6, [0]*6+[float('nan')]])
def test_joint_snapshot_rejects_incomplete_or_nonfinite_angles(model_root, positions):
    path = model_root / 'src/pen_description/config/visualization.yaml'
    data = yaml.safe_load(path.read_text())
    data['joint_snapshot']['positions_rad'] = positions
    path.write_text(yaml.safe_dump(data))
    with patch('pen_bringup.model.subprocess.run', return_value=SimpleNamespace(returncode=0, stdout=URDF)):
        result = Model(model_root).get()
    assert not result['available']
    assert 'seven finite angles' in result['message']
