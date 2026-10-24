import copy
import math
import numpy as np
import pytest

from pen_surface_geometry.api import minimum_segment_radius, make_surface
from pen_toolpath.compiler import compile_drawing, synthetic_profile, load_profile


def drawing(points=None):
    return {'kind':'pen_drawing','schema_version':1,'units':'m','strokes':[
        {'id':'one','points_m':points or [[-.006,0],[0,.006],[.006,0]]},
        {'id':'two','points_m':[[.007,-.004],[-.007,-.004]]}]}


@pytest.mark.parametrize('surface',['sphere','plane'])
def test_end_to_end_surface_and_release(surface):
    job,preview,report=compile_drawing(drawing(),surface=surface,execution_mode='fake')
    assert job['fake_only'] and not job['calibrated']
    assert job['samples'][0]['time_s']==0
    times=np.array([s['time_s'] for s in job['samples']])
    assert np.min(np.diff(times))>0
    assert np.max(np.diff(times))<=.010000001
    assert not job['samples'][0]['extrude'] and not job['samples'][-1]['extrude']
    release_time=None
    for i,sample in enumerate(job['samples']):
        assert not sample['extrude'] or sample['phase']=='DRAW'
        assert abs(np.linalg.norm(sample['pose'][3:])-1)<1e-8
        if sample['phase']=='RELEASE' and job['samples'][i-1]['phase']!='RELEASE':
            release_time=sample['time_s']
        if sample['phase']=='LIFT' and job['samples'][i-1]['phase']!='LIFT':
            assert sample['time_s']-release_time >= job['pen']['release_delay_s']-1e-8
            assert any(not e['extrude'] and e['time_s']==release_time for e in job['pen_events'])
    assert report['validation']['minimum_surface_clearance_m']>=.00005
    assert len(preview['strokes'])==2


def test_sphere_uses_curved_travel_and_drawing():
    job,preview,_=compile_drawing(drawing(),execution_mode='fake')
    model=make_surface('sphere')
    for stroke in preview['strokes']:
        points=np.asarray(stroke['points_m'])
        radii=np.linalg.norm(points-model.centre,axis=1)
        assert np.allclose(radii,.042)
    points=np.asarray(preview['travels'][0]['points_m'])
    assert np.allclose(np.linalg.norm(points-model.centre,axis=1),.047)
    assert np.ptp(points[:,2])>0
    assert all(s['extrude'] for s in job['samples'] if s['phase']=='DRAW')


def test_chord_oracle_detects_endpoints_that_cross_sphere():
    radius=.04
    a=[radius,0,0];b=[0,radius,0]
    assert minimum_segment_radius(a,b,[0,0,0])<radius


def test_two_mm_default_is_pen_tip_clearance_with_offset_tcp_and_rotated_surface():
    from scipy.spatial.transform import Rotation
    from pen_toolpath.compiler import transform
    profile=synthetic_profile('plane')
    profile['T_world_surface']={'xyz_m':[.508,0,.05967],
        'quaternion_xyzw':Rotation.from_euler('xy',[15,20],degrees=True).as_quat().tolist()}
    profile['T_flange_pen_tip']={'xyz_m':[.002,-.002,.2247],
        'quaternion_xyzw':[0,0,0,1]}
    profile['T_flange_rdk_tcp']={'xyz_m':[.005,.003,.230],
        'quaternion_xyzw':Rotation.from_euler('z',10,degrees=True).as_quat().tolist()}
    job,_,_=compile_drawing(drawing(),surface='plane',profile=profile)
    surface=transform(profile['T_world_surface'],'surface')
    tip_tcp=np.linalg.inv(transform(profile['T_flange_pen_tip'],'tip')) @ transform(profile['T_flange_rdk_tcp'],'tcp')
    for sample in job['samples']:
        pose=sample['pose'];tcp=np.eye(4)
        tcp[:3,:3]=Rotation.from_quat(pose[3:]).as_matrix();tcp[:3,3]=pose[:3]
        tip=(tcp @ np.linalg.inv(tip_tcp))[:3,3]
        gap=float((tip-surface[:3,3]) @ surface[:3,2])
        assert gap>=.002-1e-10
        if sample['phase']=='DRAW':assert gap==pytest.approx(.002,abs=1e-10)


@pytest.mark.parametrize('stand_off',[0,.0004,.001,.001999])
def test_sub_two_mm_stand_off_is_rejected(stand_off):
    with pytest.raises(ValueError,match='stand_off_m'):
        compile_drawing(drawing(),surface='plane',settings={'stand_off_m':stand_off})


def test_sub_two_mm_profile_stand_off_is_rejected():
    profile=synthetic_profile('plane')
    profile['pen']['stand_off_m']=.0004
    with pytest.raises(ValueError,match='stand_off_m'):
        compile_drawing(drawing(),surface='plane',profile=profile)


@pytest.mark.parametrize('change',[
    {'speed_m_s':0}, {'speed_m_s':float('nan')}, {'sample_rate_hz':1},
    {'lift_m':-.1}, {'stand_off_m':.0001,'clearance_margin_m':.0002}])
def test_invalid_settings_refused(change):
    with pytest.raises(ValueError):compile_drawing(drawing(),settings=change)


def test_wrong_radius_and_outside_patch_refused():
    with pytest.raises(ValueError,match='80 mm'):compile_drawing(drawing(),surface={'type':'sphere','radius_m':.075})
    with pytest.raises(ValueError):compile_drawing(drawing([[0,0],[.08,0]]))


def test_missing_or_fake_calibration_never_becomes_real():
    with pytest.raises(ValueError,match='TBD'):compile_drawing(drawing(),execution_mode='real')
    with pytest.raises(ValueError,match='measured'):compile_drawing(drawing(),profile=synthetic_profile(),execution_mode='real')
    bad=synthetic_profile();bad['T_world_surface']['xyz_m']=None
    with pytest.raises(ValueError,match='T_world_surface'):compile_drawing(drawing(),profile=bad)


def test_profile_rotation_applies_to_points_and_orientation():
    base=synthetic_profile('plane')
    rotated=copy.deepcopy(base)
    rotated['T_world_surface']['quaternion_xyzw']=[0,0,math.sqrt(.5),math.sqrt(.5)]
    a,_,_=compile_drawing(drawing(),surface='plane',profile=base)
    b,_,_=compile_drawing(drawing(),surface='plane',profile=rotated)
    for sa,sb in zip(a['samples'],b['samples']):
        x,y,z=sa['pose'][:3]
        assert np.allclose(sb['pose'][:3],[.45-y,x-.45,z])


def test_maximum_press_is_enforced():
    profile=synthetic_profile();profile['pen']['max_press_s']=.1
    with pytest.raises(ValueError,match='maximum continuous'):compile_drawing(drawing(),profile=profile)


def test_explicit_drawing_handoff_never_reextracts_image(tmp_path, monkeypatch):
    import json
    from pen_toolpath import worker
    source=tmp_path/'drawing.json';source.write_text(json.dumps(drawing()))
    def forbidden(*args,**kwargs):
        raise AssertionError('stage two re-extracted the image')
    monkeypatch.setattr(worker,'extract_drawing',forbidden)
    result=worker.prepare({'drawing_path':str(source),'image_path':'not-an-image.png','surface':'plane'},tmp_path/'output')
    assert result['success']


def test_profile_surface_radius_mismatch_refused():
    profile=synthetic_profile();profile['surface']['radius_m']=.075
    with pytest.raises(ValueError,match='radius does not match'):
        compile_drawing(drawing(),profile=profile)


def test_real_profile_requires_complete_tool_and_description():
    profile=synthetic_profile()
    profile.update(calibrated=True,fake_only=False,synthetic=False)
    profile['qualified']={'cartesian':True,'rt':True}
    with pytest.raises(ValueError,match='T_world_base'):
        compile_drawing(drawing(),profile=profile,execution_mode='real')


def test_air_run_is_precompiled_offset_with_all_extrusion_off():
    normal,_,_=compile_drawing(drawing(),surface='plane')
    air,_,_=compile_drawing(drawing(),surface='plane',settings={'air_offset_m':.005})
    assert air['air_run'] and air['air_offset_m']==.005
    assert not any(s['extrude'] for s in air['samples'])
    assert not any(e['extrude'] for e in air['pen_events'])
    # Plane offset preserves horizontal trajectory and raises the full route.
    for a,b in zip(normal['samples'],air['samples']):
        assert abs(b['pose'][2]-a['pose'][2]-.005)<1e-9


@pytest.mark.parametrize('world_base',[
    {'xyz_m':[.1,0,0],'quaternion_xyzw':[0,0,0,1]},
    {'xyz_m':[0,0,0],'quaternion_xyzw':[0,0,math.sqrt(.5),math.sqrt(.5)]},
])
def test_real_command_frame_must_coincide_with_rdk_base(world_base):
    profile=synthetic_profile()
    profile.update(calibrated=True,fake_only=False,synthetic=False)
    profile['qualified']={'cartesian':True,'rt':True}
    profile['T_world_base']=world_base
    with pytest.raises(ValueError,match='T_world_base must be identity'):
        compile_drawing(drawing(),profile=profile,execution_mode='real')
