import copy
import hashlib
import json
from pathlib import Path
import numpy as np
import pytest
from scipy.interpolate import CubicHermiteSpline
from pen_toolpath.robot_preview import Arm
from pen_toolpath.compiler import synthetic_profile
from pen_toolpath.joint_trajectory import build_joint_trajectory


@pytest.fixture
def candidate(tmp_path):
    context=json.loads((Path(__file__).parent/'fixtures/rizon4s_preview.json').read_text())
    arm=Arm(context)
    path=tmp_path/'factory.yaml';path.write_text('factory model test fixture')
    profile=synthetic_profile('plane')
    profile['T_flange_pen_tip']=context['flange_to_visual_tip']
    profile['T_flange_rdk_tcp']=context['flange_to_visual_tip']
    profile['robot_kinematics']={'robot_sn':profile['robot_sn'],'path':str(path),
                               'sha256':hashlib.sha256(path.read_bytes()).hexdigest()}
    q=np.array([context['initial_positions'][n] for n in arm.names])
    frames=[]
    for i, delta in enumerate([0.,.002,.004,.004]):
        p=q.copy();p[0]+=delta
        frames.append({'positions':p.tolist(),'time_s':i*.01,'phase':'TRAVEL',
                       'extrude':False,'job_time_s':i*.01,'tip_m':arm.forward(p)[:3,3].tolist()})
    job={'profile':profile,'robot_sn':profile['robot_sn'],'pen':{'max_press_s':600.,'recycle_interval_s':15.}}
    motion={'frames':frames,'joint_names':arm.names}
    return job,context,motion


def test_retimed_cubic_is_bounded_and_starts_and_stops(candidate):
    job,context,motion=candidate
    result=build_joint_trajectory(job,context,motion)
    assert result['simulation_only'] is False
    assert result['time_scale']>1
    frames=result['frames'];t=np.array([f['time_s'] for f in frames])
    q=np.array([f['positions'] for f in frames]);v=np.array([f['velocities'] for f in frames])
    assert np.allclose(v[[0,-1]],0)
    spline=CubicHermiteSpline(t,q,v,axis=0);probe=np.linspace(0,t[-1],1001)
    assert np.max(np.abs(spline(probe,1)))<=.5
    assert np.max(np.abs(spline(probe,2)))<=1.001
    assert result['max_interpolation_deviation_m']<=.0005
    assert all(len(f['tcp_pose'])==7 for f in frames)


def test_factory_identity_and_no_display_correction(candidate):
    job,context,motion=candidate
    altered=copy.deepcopy(job);altered['profile']['preview_kinematic_correction']={'test':True}
    with pytest.raises(ValueError,match='Display-only'):build_joint_trajectory(altered,context,motion)
    Path(job['profile']['robot_kinematics']['path']).write_text('changed')
    with pytest.raises(ValueError,match='matching factory'):build_joint_trajectory(job,context,motion)


def test_retiming_cannot_silently_extend_extrusion(candidate):
    job,context,motion=candidate
    job['pen']['recycle_interval_s']=.001
    motion['frames'][0]['extrude']=True
    with pytest.raises(ValueError,match='pause interval'):build_joint_trajectory(job,context,motion)


def test_recycle_cadence_is_fifteen_seconds_without_added_hold():
    from pen_toolpath.joint_trajectory import align_recycle_intervals
    frames=[{'phase':phase,'time_s':stamp,'velocities':[v]*7}
            for phase,stamp,v in [('DRAW',0.,0.),('DRAW',7.,.1),('RELEASE',14.,0.),
                                  ('RELEASE',15.,0.),('DRAW',16.,0.),('DRAW',20.,0.),
                                  ('RELEASE',21.,0.),('LIFT',23.,0.)]]
    align_recycle_intervals(frames,15.)
    assert frames[2]['time_s']-frames[0]['time_s']==pytest.approx(15.)
    assert frames[4]['time_s']-frames[2]['time_s']==pytest.approx(2.)
    assert frames[1]['velocities'][0]<.1


def test_old_poorly_conditioned_trajectory_is_rejected(candidate):
    job,context,motion=candidate
    bad=[.31226138135,-.1794629379,-.2004850540,2.6955627432,
         -.3785110077,1.6944458100,-1.0549272447]
    for frame in motion['frames']:frame['positions']=bad
    with pytest.raises(ValueError,match='Unfavourable joint posture'):
        build_joint_trajectory(job,context,motion)
