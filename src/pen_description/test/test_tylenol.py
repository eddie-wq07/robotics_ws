from pathlib import Path
import hashlib
import json

import pytest

from pen_description.calibration import compose, load_profile, transform


def test_tylenol_registration_and_execution_boundary():
    root = Path(__file__).resolve().parents[1]
    profile = load_profile(root / 'config/tylenol_sun.yaml', mode='preview')
    assert profile['selection'] == {'image': 'sun.png', 'images': ['sun.png'],
                                    'surface': 'cylinder', 'size_mm': 36., 'fill': False}
    assert profile['calibrated']
    assert not profile['fake_only']
    assert profile['execution']['commissioning_verified']
    assert profile['routes']['approach']['qualified']
    xyz, q = compose(compose(transform(profile, 'T_world_base'), transform(profile, 'T_base_jig')),
                     transform(profile, 'T_jig_surface'))
    assert xyz == pytest.approx(profile['T_world_surface']['xyz_m'])
    assert q == pytest.approx(profile['T_world_surface']['quaternion_xyzw'])
    assert load_profile(root / 'config/tylenol_sun.yaml', mode='real')['surface']['type'] == 'cylinder'
    assets = root / 'assets/tylenol'
    record = json.loads((assets / 'assembly.json').read_text())
    assert hashlib.sha256((assets / 'holder.stl').read_bytes()).hexdigest() == record['runtime_sha256']
    assert profile['workpiece'] == record['assembly']


def test_sun_and_moon_share_calibration_with_independent_drawing_settings():
    root=Path(__file__).resolve().parents[1]
    sun=load_profile(root/'config/tylenol_sun.yaml',mode='real')
    moon=load_profile(root/'config/tylenol_moon.yaml',mode='real')
    assert sun['selection']['images']==['sun.png']
    assert moon['selection']['image']=='moon.png'
    assert moon['selection']['images']==['moon.png']
    assert sun['drawing']['speed_m_s']==.015
    assert moon['drawing']['speed_m_s']==.004
    assert sun['pen']['stand_off_m']==.0019
    assert moon['pen']['stand_off_m']==.002
    assert moon['drawing']['local_speed_limits'] is True
    assert not sun['drawing'].get('local_speed_limits',False)
    for key in sun.keys()-{'profile_id','selection','drawing','pen'}:
        assert sun[key]==moon[key],key
