import numpy as np
import pytest

from pen_surface_geometry.api import make_surface, minimum_surface_segment_clearance
from pen_toolpath.compiler import compile_drawing, synthetic_profile


SPEC = {'type': 'cylinder', 'radius_m': .0291, 'height_m': .03, 'lying': True}


def test_horizontal_cylinder_mapping_and_chord_clearance():
    model = make_surface(SPEC)
    assert model.point(0, 0) == pytest.approx([0, 0, .0291])
    assert model.normal(0, 0) == pytest.approx([0, 0, 1])
    assert minimum_surface_segment_clearance(model, [0, .03, 0], [0, -.03, 0]) == pytest.approx(-.0291)
    assert minimum_surface_segment_clearance(model, [-1, 0, .031], [1, 0, .031]) == pytest.approx(.0019)
    with pytest.raises(ValueError, match='end'):
        model.fit(.02, (.002, -.02, .02))


def test_cylinder_plan_retains_surface_offset_and_release_schedule():
    profile = synthetic_profile('cylinder')
    profile['surface'] = SPEC
    drawing = {'kind': 'pen_drawing', 'schema_version': 1, 'units': 'm',
               'strokes': [{'id': 'sun', 'points_m': [[-.004, -.004], [0, .004], [.004, -.004]]}]}
    job, preview, _ = compile_drawing(drawing, surface=SPEC, profile=profile)
    points = np.asarray(preview['strokes'][0]['points_m'])
    assert np.linalg.norm(points[:, 1:], axis=1) == pytest.approx(.0291 + .0013)
    assert job['validation']['minimum_tool_surface_clearance_m'] > 0
    assert job['validation']['physical_execution_qualified'] is False
    assert job['samples'][-1]['extrude'] is False
    with pytest.raises(ValueError, match='end'):
        too_tall = {**drawing, 'strokes': [{'id': 'bad', 'points_m': [[0, -.02], [0, .02]]}]}
        compile_drawing(too_tall, surface=SPEC, profile=profile)


@pytest.mark.parametrize('field,value', [('radius_m', float('nan')), ('height_m', -1)])
def test_cylinder_dimensions_reject_invalid_values(field, value):
    with pytest.raises(ValueError, match='positive and finite'):
        make_surface({**SPEC, field: value})


def test_real_cylinder_requires_qualification_and_supports_drawing_and_air_check():
    profile = synthetic_profile('cylinder')
    identity = {'xyz_m': [0, 0, 0], 'quaternion_xyzw': [0, 0, 0, 1]}
    profile.update(surface=SPEC, calibrated=True, fake_only=False, synthetic=False,
                   qualified={'cartesian': True, 'rt': True},
                   qualification={key: True for key in ('rt', 'clearance', 'pen', 'tool')},
                   rdk_tool_name='pen', payload={'mass_kg': .3, 'com_m': [0, 0, .1],
                                                'inertia_kg_m2': [1e-5, 1e-5, 1e-5, 0, 0, 0]})
    for name in ('T_world_base', 'T_base_jig', 'T_flange_holder', 'T_holder_pen'):
        profile[name] = identity
    drawing = {'kind': 'pen_drawing', 'schema_version': 1, 'units': 'm',
               'strokes': [{'id': 'check', 'points_m': [[-.003, 0], [.003, 0]]}]}
    normal, _, _ = compile_drawing(drawing, surface=SPEC, profile=profile, execution_mode='real')
    assert not normal['air_run']
    assert any(row['extrude'] for row in normal['samples'])
    job, _, _ = compile_drawing(drawing, surface=SPEC, profile=profile,
                               execution_mode='real', settings={'air_offset_m': .01})
    assert job['air_run']
    assert job['motion_settings']['stand_off_m'] >= .01
    assert all(not row['extrude'] for row in job['samples'])
    assert all(not event['extrude'] for event in job['pen_events'])
    profile['calibrated'] = False
    with pytest.raises(ValueError, match='measured calibration'):
        compile_drawing(drawing, surface=SPEC, profile=profile, execution_mode='real')
