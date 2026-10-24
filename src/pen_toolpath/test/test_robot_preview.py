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


def test_preview_rigid_alignment_preserves_kinematics(context):
    original=Arm(context)
    q=np.array([context['initial_positions'][name] for name in original.names])
    base={'xyz_m':[.001,-.002,.003],
          'quaternion_xyzw':Rotation.from_euler('z',.02).as_quat().tolist()}
    corrected=Arm({**context,'base_correction':base})
    before,jac=original.forward(q,True);after,aligned_jac=corrected.forward(q,True)
    rotation=Rotation.from_quat(base['quaternion_xyzw']).as_matrix()
    assert np.allclose(after[:3,3],rotation@before[:3,3]+base['xyz_m'])
    assert np.allclose(after[:3,:3],rotation@before[:3,:3])
    assert np.allclose(aligned_jac[:3],rotation@jac[:3])
    assert np.allclose(aligned_jac[3:],rotation@jac[3:])


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


def test_saved_joint_routes_include_home_connector_in_preview(context):
    profile = synthetic_profile('plane')
    profile['T_world_surface']['xyz_m'] = [.508, 0, .05967]
    profile['T_flange_pen_tip'] = context['flange_to_visual_tip']
    profile['T_flange_rdk_tcp'] = context['flange_to_visual_tip']
    drawing = {'kind':'pen_drawing', 'schema_version':1, 'units':'m',
               'strokes':[{'id':'line','points_m':[[-.004,0],[.004,0]]}]}
    job, _, _ = compile_drawing(drawing, surface='plane', profile=profile)
    baseline = build_preview(job, context)['frames']
    approach = [f for f in baseline[1:] if f['phase']=='APPROACH']
    park = [f for f in baseline if f['phase'] in ('RETRACT','RETURN')]
    final_draw = [f for f in baseline if f['job_time_s'] is not None][-1]
    start = baseline[0]['positions']
    home = list(start); home[0] += .05
    job['joint_routes'] = {
        'approach': {'start_positions':start, 'points':[
            {'time_s':f['time_s'], 'positions':f['positions']} for f in approach]},
        'park': {'start_positions':final_draw['positions'], 'points':[
            {'time_s':f['time_s']-final_draw['time_s'], 'positions':f['positions']} for f in park]},
        'home_connector': {'checked_joint_home':True, 'start_positions':home,
                           'points':[{'time_s':1., 'positions':start}]},
    }
    context['initial_positions'] = dict(zip(context['initial_positions'], home))
    frames = build_preview(job, context)['frames']
    assert frames[0]['positions'] == home
    assert frames[1]['positions'] == start
    assert frames[1]['time_s'] == 1.
    assert frames[2]['positions'] == approach[0]['positions']
    assert frames[-1]['positions'] == park[-1]['positions']
    job['joint_routes']['home_connector']['points'][0]['positions'] = home
    with pytest.raises(ValueError, match='does not end'):
        build_preview(job, context)

    # Only explicitly requested fake joint execution may reset from a different
    # finishing branch. Real previews retain their checked-start requirement.
    job['joint_routes']['home_connector']['points'][0]['positions'] = start
    other = list(home); other[0] += .2
    context['initial_positions'] = dict(zip(context['initial_positions'], other))
    with pytest.raises(ValueError, match='outside the saved joint route start'):
        build_preview(job, context)
    context['allow_simulation_joint_reset'] = True
    job['fake_only'] = False
    with pytest.raises(ValueError, match='outside the saved joint route start'):
        build_preview(job, context)
    job['fake_only'] = True
    frames = build_preview(job, context)['frames']
    assert frames[0]['positions'] == other
    assert any(np.allclose(f['positions'], start) for f in frames[1:])
    assert all(not f['extrude'] for f in frames if f['phase'] == 'APPROACH')


def test_batch_kinematics_matches_scalar(context):
    arm=Arm(context)
    rng=np.random.default_rng(42)
    positions=rng.uniform(arm.lower+.05,arm.upper-.05,size=(12,7))
    poses,jac=arm.forward_many(positions,True)
    for i,q in enumerate(positions):
        pose,expected=arm.forward(q,True)
        assert np.allclose(poses[i],pose,atol=1e-12)
        assert np.allclose(jac[i],expected,atol=1e-12)


def test_rejects_failed_jackolantern_posture(context):
    bad=[.31226138135,-.1794629379,-.2004850540,2.6955627432,
         -.3785110077,1.6944458100,-1.0549272447]
    arm=Arm({**context,'posture_reference':bad})
    with pytest.raises(ValueError,match='Unfavourable joint posture'):
        arm.solve(arm.forward(bad),np.array(bad))
    with pytest.raises(ValueError,match='Unfavourable joint posture'):
        arm.require_safe_postures(np.array([bad]))


def test_posture_biased_ik_still_reaches_target(context):
    seed=np.array([context['initial_positions'][n] for n in Arm(context).names])
    arm=Arm({**context,'posture_reference':seed.tolist()})
    desired=arm.forward(seed+np.array([.02,-.01,.01,0.,0.,0.,0.]))
    solved=arm.solve(desired,seed)
    error=arm.error(arm.forward(solved),desired)
    assert np.linalg.norm(error[:3])<2e-5
    assert np.linalg.norm(error[3:])<2e-4
    arm.require_safe_posture(solved)
