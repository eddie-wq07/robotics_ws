import json
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from pen_toolpath.compiler import compile_drawing, synthetic_profile
from pen_toolpath.robot_preview import Arm, build_preview


@pytest.fixture
def context():
    return json.loads((Path(__file__).parent/'fixtures/rizon4s_preview.json').read_text())


def test_urdf_jacobian_matches_finite_difference(context):
    arm = Arm(context)
    q = np.array([context['initial_positions'][name] for name in arm.names])
    pose, jac = arm.forward(q, True)
    for i in range(7):
        shifted = q.copy(); shifted[i] += 1e-7
        after = arm.forward(shifted)
        measured = np.r_[(after[:3, 3]-pose[:3, 3])/1e-7,
                         Rotation.from_matrix(after[:3, :3] @ pose[:3, :3].T).as_rotvec()/1e-7]
        assert np.allclose(measured, jac[:, i], atol=1e-6)


def test_preview_approaches_before_drawing_and_returns(context):
    profile = synthetic_profile('plane')
    profile['T_world_surface']['xyz_m'] = [.508, 0, .05967]
    profile['T_flange_pen_tip'] = context['flange_to_visual_tip']
    profile['T_flange_rdk_tcp'] = context['flange_to_visual_tip']
    drawing = {'kind':'pen_drawing', 'schema_version':1, 'units':'m',
               'strokes':[{'id':'line','points_m':[[-.004,0],[.004,0]]}]}
    job, _, _ = compile_drawing(drawing, surface='plane', profile=profile)
    preview = build_preview(job, context)
    frames = preview['frames']; arm = Arm(context)
    initial = np.array([context['initial_positions'][name] for name in arm.names])
    assert np.allclose(frames[0]['positions'], initial)
    assert frames[0]['phase']=='APPROACH' and frames[-1]['phase']=='RETURN'
    assert frames[0]['job_time_s'] is None
    assert np.linalg.norm(np.array(frames[-1]['tip_m'])-frames[0]['tip_m']) < .0002
    assert all(not f['extrude'] for f in frames if f['phase']!='DRAW')
    assert any(np.linalg.norm(np.array(f['positions'])-initial)>.1 for f in frames)
    assert preview['max_position_error_m'] < .0002
    assert preview['max_orientation_error_rad'] < .002
    for a,b in zip(frames,frames[1:]):
        qa,qb=np.array(a['positions']),np.array(b['positions'])
        assert np.all(qb>=arm.lower) and np.all(qb<=arm.upper)
        assert b['time_s']>a['time_s']
        assert np.all(np.abs(qb-qa)/(b['time_s']-a['time_s']) <= .5*arm.velocity+1e-7)


def test_unreachable_preview_is_rejected(context):
    arm = Arm(context)
    initial=np.array([context['initial_positions'][name] for name in arm.names])
    target=arm.forward(initial); target[:3,3]=[4,4,4]
    with pytest.raises(ValueError,match='cannot reach'):
        arm.solve(target,initial)
