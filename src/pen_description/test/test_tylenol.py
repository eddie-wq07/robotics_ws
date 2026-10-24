from pathlib import Path
import hashlib
import json

import pytest

from pen_description.calibration import compose, load_profile, transform


def test_tylenol_registration_and_execution_boundary():
    root = Path(__file__).resolve().parents[1]
    profile = load_profile(root / 'config/tylenol.yaml', mode='preview')
    assert profile['selection'] == {'image': 'sun.png', 'surface': 'cylinder', 'size_mm': 36., 'fill': False}
    assert profile['calibrated']
    assert not profile['fake_only']
    assert profile['execution']['commissioning_verified']
    assert profile['routes']['approach']['qualified']
    xyz, q = compose(compose(transform(profile, 'T_world_base'), transform(profile, 'T_base_jig')),
                     transform(profile, 'T_jig_surface'))
    assert xyz == pytest.approx(profile['T_world_surface']['xyz_m'])
    assert q == pytest.approx(profile['T_world_surface']['quaternion_xyzw'])
    assert load_profile(root / 'config/tylenol.yaml', mode='real')['surface']['type'] == 'cylinder'
    assets = root / 'assets/tylenol'
    record = json.loads((assets / 'assembly.json').read_text())
    assert hashlib.sha256((assets / 'holder.stl').read_bytes()).hexdigest() == record['runtime_sha256']
    assert profile['workpiece'] == record['assembly']
