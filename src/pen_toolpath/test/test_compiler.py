import copy
import math
import numpy as np
import pytest

from pen_surface_geometry.api import minimum_segment_radius, make_surface
from pen_toolpath.compiler import compile_drawing, synthetic_profile, load_profile, transform


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
        assert np.allclose(radii,.0413)
    points=np.asarray(preview['travels'][0]['points_m'])
    assert np.allclose(np.linalg.norm(points-model.centre,axis=1),.0463)
    assert np.ptp(points[:,2])>0
    assert all(s['extrude'] for s in job['samples'] if s['phase']=='DRAW')


def test_chord_oracle_detects_endpoints_that_cross_sphere():
    radius=.04
    a=[radius,0,0];b=[0,radius,0]
    assert minimum_segment_radius(a,b,[0,0,0])<radius


def test_travel_speed_is_separate_from_extrusion_speed():
    slow,_,_=compile_drawing(drawing(),surface='plane',settings={'speed_m_s':.001})
    fast,_,_=compile_drawing(drawing(),surface='plane',settings={'speed_m_s':.001,'travel_speed_m_s':.0025})
    assert fast['duration_s']<slow['duration_s']
    assert fast['validation']['max_draw_speed_m_s']<=.001*1.02
    assert .001<fast['validation']['max_travel_speed_m_s']<=.0025*1.02
    assert fast['motion_settings']['stand_off_m']==.0013
    assert sum(s['extrude'] for s in fast['samples'])==sum(s['extrude'] for s in slow['samples'])
    with pytest.raises(ValueError,match='travel_speed_m_s'):
        compile_drawing(drawing(),surface='plane',settings={'travel_speed_m_s':2.5})


@pytest.mark.parametrize('sample_rate',[600,900,1000])
def test_profile_requests_trajectory_sample_rate(sample_rate):
    profile=synthetic_profile('plane');profile['drawing']={'sample_rate_hz':sample_rate}
    job,_,_=compile_drawing(drawing(),surface='plane',profile=profile)
    assert job['sample_rate_hz']==sample_rate
    assert np.diff([r['time_s'] for r in job['samples']]).max()<=1/sample_rate+1e-9


def test_setup_speed_doubles_only_before_first_extrusion():
    slow,_,_=compile_drawing(drawing(),surface='plane',settings={'approach_speed_scale':1})
    fast,_,_=compile_drawing(drawing(),surface='plane',settings={'approach_speed_scale':2})
    first=next(i for i,row in enumerate(slow['samples']) if row['extrude'])
    assert fast['samples'][first]['time_s']==pytest.approx(slow['samples'][first]['time_s']/2)
    assert [r['pose'] for r in slow['samples']]==[r['pose'] for r in fast['samples']]
    assert np.diff([r['time_s'] for r in fast['samples'][first:]])==pytest.approx(
        np.diff([r['time_s'] for r in slow['samples'][first:]]))
    assert np.diff([e['time_s'] for e in fast['pen_events'][1:]])==pytest.approx(
        np.diff([e['time_s'] for e in slow['pen_events'][1:]]))
    assert fast['motion_settings']['approach_speed_scale']==2
    assert fast['validation']['max_draw_speed_m_s']==pytest.approx(slow['validation']['max_draw_speed_m_s'])
    with pytest.raises(ValueError,match='approach_speed_scale'):
        compile_drawing(drawing(),surface='plane',settings={'approach_speed_scale':3})


def test_motion_multipliers_preserve_pen_delays():
    profile=synthetic_profile('plane')
    profile['pen'].update(press_delay_s=1.,release_delay_s=1.5)
    slow,_,_=compile_drawing(drawing(),surface='plane',profile=profile)
    fast,_,_=compile_drawing(drawing(),surface='plane',profile=profile,
                            settings={'draw_speed_scale':1.5,'non_extruding_speed_scale':3.})
    def moving_time(job,on):
        return sum(b['time_s']-a['time_s'] for a,b in zip(job['samples'],job['samples'][1:])
                   if b['extrude']==on and np.linalg.norm(np.array(a['pose'])-b['pose'])>1e-10)
    assert moving_time(fast,True)==pytest.approx(moving_time(slow,True)/1.5,abs=1e-7)
    assert moving_time(fast,False)==pytest.approx(moving_time(slow,False)/3,abs=1e-7)
    assert fast['motion_settings']['approach_speed_scale']==3.
    def stationary_time(job,on):
        return sum(b['time_s']-a['time_s'] for a,b in zip(job['samples'],job['samples'][1:])
                   if a['extrude']==b['extrude']==on and np.linalg.norm(np.array(a['pose'])-b['pose'])<1e-10)
    assert stationary_time(fast,True)==pytest.approx(stationary_time(slow,True),abs=.03)
    assert stationary_time(fast,False)==pytest.approx(stationary_time(slow,False),abs=.03)
    assert fast['pen']==slow['pen']


def test_half_second_lead_then_prompt_motion_without_slow_stroke_ramp():
    profile=synthetic_profile('plane')
    profile['pen'].update(press_delay_s=.5,recycle_interval_s=15.,recycle_pause_s=2.)
    shape=drawing([[-.02,0],[.02,0]])
    job,_,_=compile_drawing(shape,surface='plane',profile=profile,
                            settings={'continuous_strokes':True,'speed_m_s':.003})
    rows=job['samples'];i=next(i for i,r in enumerate(rows) if r['extrude'])
    start=rows[i]
    moved=next(r for r in rows[i:] if r['extrude'] and np.linalg.norm(np.array(r['pose'][:3])-start['pose'][:3])>=.0001)
    assert .5 < moved['time_s']-start['time_s'] < .7
    for i,row in enumerate(rows[:-1]):
        if i and rows[i-1]['extrude'] and not row['extrude']:
            next_on=next((r for r in rows[i+1:] if r['extrude']),None)
            if next_on and rows[i+1]['phase']=='RELEASE' and next_on['pose']==row['pose']:
                assert next_on['time_s']-row['time_s']==pytest.approx(2.,abs=1e-7)


@pytest.mark.parametrize('continuous',[False,True])
@pytest.mark.parametrize('end_dwell',[.3,.75,1.])
@pytest.mark.parametrize('release_delay',[0.,1.,1.5])
def test_endpoint_extrusion_dwell_precedes_release_and_respects_recycle_limit(continuous,end_dwell,release_delay):
    profile=synthetic_profile('plane')
    profile['pen'].update(press_delay_s=.5,end_extrusion_dwell_s=end_dwell,release_delay_s=release_delay,
                          recycle_interval_s=15.,recycle_pause_s=2.)
    shape={'kind':'pen_drawing','schema_version':1,'units':'m','strokes':[
        {'id':'line','points_m':[[-.0195,0],[.0195,0]]}]}
    job,_,_=compile_drawing(shape,surface='plane',profile=profile,
                            settings={'continuous_strokes':continuous,'speed_m_s':.003})
    rows=job['samples'];end=max(i for i,r in enumerate(rows) if r['extrude'])
    start=end
    while start>0 and rows[start-1]['extrude'] and rows[start-1]['pose']==rows[end]['pose']:
        start-=1
    assert rows[end]['time_s']-rows[start]['time_s']==pytest.approx(end_dwell,abs=1e-8)
    assert rows[end+1]['phase']=='RELEASE' and not rows[end+1]['extrude']
    lift=next(r for r in rows[end+1:] if r['phase']=='LIFT')
    assert lift['time_s']-rows[end+1]['time_s']>=release_delay-1e-8
    if release_delay==0:
        assert lift['time_s']-rows[end+1]['time_s']<=.02+1e-8
    assert job['validation']['pen_recycle_count']>=1
    on=None
    for event in job['pen_events']:
        if event['extrude']:on=event['time_s']
        elif on is not None:
            assert event['time_s']-on<=15.+1e-8
            on=None


@pytest.mark.parametrize('continuous',[False,True])
@pytest.mark.parametrize('start_dwell,end_dwell',[(.75,.75),(.8,.5)])
def test_lifted_stroke_leads_are_independent_of_recycle_lead(continuous,start_dwell,end_dwell):
    profile=synthetic_profile('plane')
    profile['pen'].update(press_delay_s=start_dwell,recycle_press_delay_s=.5,
                          end_extrusion_dwell_s=end_dwell,release_delay_s=0.)
    source={'kind':'pen_drawing','schema_version':1,'units':'m','strokes':[
        {'id':'line','points_m':[[-.025,0],[.025,0]]}]}
    job,_,_=compile_drawing(source,surface='plane',profile=profile,
                            settings={'continuous_strokes':continuous,'speed_m_s':.002})
    rows=job['samples']
    starts=[i for i,row in enumerate(rows) if row['extrude'] and not rows[i-1]['extrude']]
    assert len(starts)>1
    assert rows[starts[0]-1]['phase']=='LOWER'
    for n,start in enumerate(starts):
        moved=next(row for row in rows[start+1:] if row['pose']!=rows[start]['pose'])
        assert moved['time_s']-rows[start]['time_s']==pytest.approx(start_dwell if n==0 else .5,abs=.02)
        if n:
            off=start-1
            while off>0 and not rows[off-1]['extrude']:off-=1
            assert rows[start]['time_s']-rows[off]['time_s']==pytest.approx(2.,abs=1e-8)
            assert all(row['phase']!='LIFT' for row in rows[off:start])
    end=max(i for i,row in enumerate(rows) if row['extrude'])
    hold=end
    while rows[hold-1]['extrude'] and rows[hold-1]['pose']==rows[end]['pose']:hold-=1
    assert rows[end]['time_s']-rows[hold]['time_s']==pytest.approx(end_dwell,abs=1e-8)
    assert not rows[end+1]['extrude'] and rows[end+2]['phase']=='LIFT'


def test_joint_to_cartesian_connector_is_in_checked_samples():
    profile=synthetic_profile('sphere');profile['pen']['stand_off_m']=.002
    entry={'xyz_m':[.45,0,.31],'quaternion_xyzw':[1,0,0,0]}
    start={'xyz_m':[.45,0,.30],'quaternion_xyzw':[1,0,0,0]}
    profile['routes']={'cartesian_entry_pose':entry,'cartesian_entry_only':True}
    job,_,_=compile_drawing(drawing(),surface='sphere',profile=profile,
                            settings={'approach_start_pose':start,'non_extruding_speed_scale':3})
    assert job['samples'][0]['pose']==pytest.approx(start['xyz_m']+start['quaternion_xyzw'])
    assert any(np.allclose(row['pose'],entry['xyz_m']+entry['quaternion_xyzw']) for row in job['samples'])
    assert all(not row['extrude'] for row in job['samples'][:next(i for i,r in enumerate(job['samples']) if r['extrude'])])
    assert job['validation']['minimum_surface_clearance_m']>.00005
    start['xyz_m'][2]=.25
    with pytest.raises(ValueError,match='clearance'):
        compile_drawing(drawing(),surface='sphere',profile=profile,settings={'approach_start_pose':start})


def test_plane_uses_rectangle_and_checks_rotated_extent():
    model=make_surface('plane')
    assert model.fit(.058,(.045,-.035,.035))['max_length_error']==0
    for extent in [(.051,-.035,.035),(.045,-.041,.035)]:
        with pytest.raises(ValueError,match='rectangular tile'):
            model.fit(.07,extent)
    rotated=make_surface({'type':'plane','marker':{'rotation_deg':45}})
    with pytest.raises(ValueError,match='rectangular tile'):
        rotated.fit(.058,(.045,-.035,.035))


def test_saved_route_entry_wraps_drawing_without_extrusion():
    profile=synthetic_profile('plane')
    original,_,_=compile_drawing(drawing(),surface='plane',profile=profile)
    entry={'xyz_m':[.45,0,.265], 'quaternion_xyzw':original['samples'][0]['pose'][3:]}
    profile['routes']={'cartesian_entry_pose':entry}
    job,path,_=compile_drawing(drawing(),surface='plane',profile=profile)
    assert np.allclose(job['samples'][0]['pose'],entry['xyz_m']+entry['quaternion_xyzw'])
    assert np.allclose(job['samples'][-1]['pose'],job['samples'][0]['pose'])
    assert not job['samples'][0]['extrude'] and not job['samples'][-1]['extrude']
    assert min(s['pose'][2] for s in job['samples'])>=.2513-1e-10
    assert len(path['strokes'])==2 and len(path['travels'])==3
    assert sum(s['extrude'] for s in job['samples'])==sum(s['extrude'] for s in original['samples'])
    profile['routes']['cartesian_entry_pose']['xyz_m'][2]=.251
    with pytest.raises(ValueError,match='above the travel clearance'):
        compile_drawing(drawing(),surface='plane',profile=profile)


def test_saved_entry_allows_small_measured_plate_tilt():
    from scipy.spatial.transform import Rotation
    profile=synthetic_profile('plane')
    baseline,_,_=compile_drawing(drawing(),surface='plane',profile=profile)
    profile['routes']={'cartesian_entry_pose':{'xyz_m':[.45,0,.265],
        'quaternion_xyzw':baseline['samples'][0]['pose'][3:]}}
    profile['T_world_surface']['quaternion_xyzw']=Rotation.from_euler('xy',[.8,.2],degrees=True).as_quat().tolist()
    job,_,_=compile_drawing(drawing(),surface='plane',profile=profile)
    assert job['validation']['minimum_surface_clearance_m']>=.0013-1e-9
    assert job['validation']['minimum_tool_surface_clearance_m']>0
    profile['T_world_surface']['quaternion_xyzw']=Rotation.from_euler('x',5,degrees=True).as_quat().tolist()
    with pytest.raises(ValueError,match='above the travel clearance'):
        compile_drawing(drawing(),surface='plane',profile=profile)


def test_measured_sphere_entry_adds_only_a_safe_prefix():
    profile=synthetic_profile('sphere')
    original,_,_=compile_drawing(drawing(),surface='sphere',profile=profile)
    entry={'xyz_m':[.45,0,.30],'quaternion_xyzw':[1,0,0,0]}
    profile['routes']={'cartesian_entry_pose':entry,'cartesian_entry_only':True}
    job,_,_=compile_drawing(drawing(),surface='sphere',profile=profile)
    assert job['samples'][0]['pose']==entry['xyz_m']+entry['quaternion_xyzw']
    assert job['samples'][-1]['pose']==original['samples'][-1]['pose']
    assert job['validation']['minimum_surface_clearance_m']>=.0013-1e-7
    assert job['validation']['minimum_tool_surface_clearance_m']>=.00005
    first=next(i for i,r in enumerate(job['samples']) if r['phase']=='DRAW')
    assert all(not r['extrude'] for r in job['samples'][:first])
    profile['routes']['cartesian_entry_pose']['xyz_m'][2]=.29
    with pytest.raises(ValueError,match='above the travel clearance'):
        compile_drawing(drawing(),surface='sphere',profile=profile)


def test_surface_profiles_keep_independent_motion_settings():
    plane=synthetic_profile('plane');sphere=synthetic_profile('sphere')
    plane['drawing']={'speed_m_s':.012,'travel_speed_m_s':.03,'approach_speed_scale':2}
    plane['pen']['stand_off_m']=.0013
    sphere['drawing']={'speed_m_s':.006,'travel_speed_m_s':.015,'approach_speed_scale':1}
    sphere['pen']['stand_off_m']=.003
    sphere['routes']={'cartesian_entry_pose':{'xyz_m':[.45,0,.2965],'quaternion_xyzw':[1,0,0,0]},'cartesian_entry_only':True}
    flat,_,_=compile_drawing(drawing(),surface='plane',profile=plane)
    ball,_,_=compile_drawing(drawing(),surface='sphere',profile=sphere)
    assert flat['motion_settings']=={'draw_speed_m_s':.012,'travel_speed_m_s':.03,'approach_speed_scale':2.,'stand_off_m':.0013,
                                     'pen_orientation':'world_vertical_with_clearance_tilt','maximum_surface_normal_angle_deg':25.,'non_extruding_speed_scale':1.}
    assert ball['motion_settings']=={'draw_speed_m_s':.006,'travel_speed_m_s':.015,'approach_speed_scale':1.,'stand_off_m':.003,
                                     'pen_orientation':'world_vertical_with_clearance_tilt','maximum_surface_normal_angle_deg':25.,'non_extruding_speed_scale':1.}
    assert ball['validation']['minimum_surface_clearance_m']>=.003-1e-7


def test_sphere_can_return_to_measured_entry_below_raised_travel_height():
    profile=synthetic_profile('sphere');profile['pen']['stand_off_m']=.003
    entry={'xyz_m':[.45,0,.2965],'quaternion_xyzw':[1,0,0,0]}
    profile['routes']={'cartesian_entry_pose':entry}
    job,_,_=compile_drawing(drawing(),surface='sphere',profile=profile)
    assert job['samples'][-1]['pose']==pytest.approx(entry['xyz_m']+entry['quaternion_xyzw'])
    assert not job['samples'][-1]['extrude']
    assert job['validation']['minimum_surface_clearance_m']>=.003-1e-7


def test_sphere_cartesian_park_retracts_and_finishes_pen_off():
    profile=synthetic_profile('sphere');profile['pen']['stand_off_m']=.003
    park={'xyz_m':[.30,0,.35],'quaternion_xyzw':[1,0,0,0]}
    profile['routes']={'cartesian_entry_pose':{'xyz_m':[.45,0,.2965],'quaternion_xyzw':[1,0,0,0]},
                       'cartesian_entry_only':True,'cartesian_park_pose':park}
    job,_,_=compile_drawing(drawing(),surface='sphere',profile=profile)
    assert job['samples'][-1]['pose']==pytest.approx(park['xyz_m']+park['quaternion_xyzw'])
    last_draw=max(i for i,r in enumerate(job['samples']) if r['extrude'])
    assert all(not r['extrude'] for r in job['samples'][last_draw+1:])
    assert job['validation']['minimum_surface_clearance_m']>=.003-1e-7
    assert job['validation']['minimum_tool_surface_clearance_m']>=.00005


def test_1p3_mm_default_is_pen_tip_clearance_with_offset_tcp_and_rotated_surface():
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
        assert gap>=.0013-1e-10
        if sample['phase']=='DRAW':assert gap==pytest.approx(.0013,abs=1e-10)


@pytest.mark.parametrize('stand_off',[0,.0004,.001,.001299])
def test_sub_1p3_mm_stand_off_is_rejected(stand_off):
    with pytest.raises(ValueError,match='stand_off_m'):
        compile_drawing(drawing(),surface='plane',settings={'stand_off_m':stand_off})


def test_sub_1p3_mm_profile_stand_off_is_rejected():
    profile=synthetic_profile('plane')
    profile['pen']['stand_off_m']=.0004
    with pytest.raises(ValueError,match='stand_off_m'):
        compile_drawing(drawing(),surface='plane',profile=profile)


@pytest.mark.parametrize('stand_off',[0,.001249])
def test_sphere_rejects_below_1p25_mm_stand_off(stand_off):
    with pytest.raises(ValueError,match='stand_off_m'):
        compile_drawing(drawing(),surface='sphere',settings={'stand_off_m':stand_off})


@pytest.mark.parametrize('change',[
    {'speed_m_s':0}, {'speed_m_s':float('nan')}, {'sample_rate_hz':1},
    {'lift_m':-.1}, {'stand_off_m':.0001,'clearance_margin_m':.0002}])
def test_invalid_settings_refused(change):
    with pytest.raises(ValueError):compile_drawing(drawing(),settings=change)


def test_wrong_radius_refused_but_old_sphere_patch_limit_removed():
    with pytest.raises(ValueError,match='80 mm'):compile_drawing(drawing(),surface={'type':'sphere','radius_m':.075})
    job,_,_=compile_drawing(drawing([[0,0],[.08,0]]))
    assert job['validation']['maximum_drawing_surface_normal_angle_deg']<=25.000001
    assert job['validation']['minimum_tool_surface_clearance_m']>=.00005


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


@pytest.mark.parametrize('stand_off',[.00125,.002])
def test_downward_pen_keeps_fixed_orientation_and_tip_clearance_on_sphere(stand_off):
    from scipy.spatial.transform import Rotation
    profile=synthetic_profile('sphere')
    profile['pen']['stand_off_m']=stand_off
    # Nontrivial TCP offset must be compensated about the actual pen tip.
    profile['T_flange_rdk_tcp']={'xyz_m':[.004,.003,.02],
                               'quaternion_xyzw':Rotation.from_euler('z',20,degrees=True).as_quat().tolist()}
    job,_,_=compile_drawing(drawing(),surface='sphere',profile=profile)
    tcp_to_tip=np.linalg.inv(transform(profile['T_flange_rdk_tcp'],'TCP'))@transform(profile['T_flange_pen_tip'],'tip')
    centre=make_surface('sphere').centre+np.asarray(profile['T_world_surface']['xyz_m'])
    rotations=[]
    for sample in job['samples']:
        if not sample['extrude']:continue
        pose=transform({'xyz_m':sample['pose'][:3],'quaternion_xyzw':sample['pose'][3:]},'sample')@tcp_to_tip
        assert pose[:3,2]==pytest.approx([0,0,-1],abs=1e-10)
        assert np.linalg.norm(pose[:3,3]-centre)==pytest.approx(.04+stand_off,abs=1e-9)
        rotations.append(pose[:3,:3])
    assert all(np.allclose(r,rotations[0]) for r in rotations)


def test_large_holder_uses_only_clearance_required_tilt():
    profile=synthetic_profile('sphere')
    profile['pen']['stand_off_m']=.002
    profile['tool_envelope']={'radius_m':.075925842,'length_m':.302634839}
    job,_,_=compile_drawing(drawing([[-.018,0],[0,.018],[.018,0]]),surface='sphere',profile=profile)
    tilt=job['validation']['maximum_drawing_pen_tilt_deg']
    assert 0 < tilt < 10
    assert job['validation']['minimum_tool_surface_clearance_m']>=.00005


def test_wider_sphere_patch_uses_normal_limit_instead_of_vertical_limit():
    profile=synthetic_profile('sphere')
    profile['pen']['stand_off_m']=.002
    profile['tool_envelope']={'radius_m':.075925842,'length_m':.302634839}
    job,_,_=compile_drawing(drawing([[-.026,0],[.026,0]]),surface='sphere',profile=profile)
    assert job['validation']['maximum_drawing_pen_tilt_deg']>10
    assert job['validation']['maximum_drawing_surface_normal_angle_deg']<=25.000001
    assert job['validation']['minimum_tool_surface_clearance_m']>=.00005


def test_twenty_five_degree_normal_limit_applies_even_with_small_holder():
    profile=synthetic_profile('sphere');profile['pen']['stand_off_m']=.002
    job,_,_=compile_drawing(drawing([[-.026,0],[.026,0]]),surface='sphere',profile=profile)
    assert job['validation']['maximum_drawing_pen_tilt_deg']>10.
    assert 24. < job['validation']['maximum_drawing_surface_normal_angle_deg']<=25.000001


@pytest.mark.parametrize('limit', [15., 35.])
def test_profile_controls_surface_normal_angle_limit(limit):
    profile=synthetic_profile('sphere')
    profile['pen']['stand_off_m']=.002
    profile['drawing']={'max_surface_normal_angle_deg':limit}
    job,_,_=compile_drawing(drawing([[-.026,0],[.026,0]]),surface='sphere',profile=profile)
    assert job['motion_settings']['maximum_surface_normal_angle_deg']==limit
    assert limit-1. < job['validation']['maximum_drawing_surface_normal_angle_deg']<=limit+1e-6
    assert job['validation']['minimum_tool_surface_clearance_m']>=.00005


@pytest.mark.parametrize('limit', [-1, 90, float('nan'), float('inf'), True, None, 'invalid'])
def test_invalid_profile_surface_normal_angle_rejected(limit):
    profile=synthetic_profile('sphere')
    profile['drawing']={'max_surface_normal_angle_deg':limit}
    with pytest.raises(ValueError,match='max_surface_normal_angle_deg'):
        compile_drawing(drawing(),surface='sphere',profile=profile)


def test_maximum_press_is_enforced():
    profile=synthetic_profile();profile['pen']['max_press_s']=.1
    with pytest.raises(ValueError,match='maximum continuous'):compile_drawing(drawing(),profile=profile)


@pytest.mark.parametrize('points', [[[-.03,0],[.03,0]],
    [[-.03,0],[-.01,0],[.01,0],[.03,0]]])
@pytest.mark.parametrize('continuous',[False,True])
def test_long_stroke_pauses_stationary_for_two_seconds_then_resumes(points,continuous):
    source={'kind':'pen_drawing','schema_version':1,'units':'m',
            'strokes':[{'id':'long','points_m':points}]}
    job,path,_=compile_drawing(source,surface='plane',settings={'speed_m_s':.002,'continuous_strokes':continuous})
    events=job['pen_events'];runs=[];start=None
    for event in events:
        if event['extrude']:start=event['time_s']
        elif start is not None:runs.append((start,event['time_s']));start=None
    assert len(runs)>1
    assert job['pen']['recycle_interval_s']==15
    assert all(end-start<=15+1e-8 for start,end in runs)
    assert job['validation']['pen_recycle_count']==len(runs)-1
    for (_,off),(on,_) in zip(runs,runs[1:]):
        rows=[r for r in job['samples'] if off<=r['time_s']<=on]
        assert set(r['phase'] for r in rows)=={'RELEASE','DRAW'}
        assert np.allclose([r['pose'] for r in rows],rows[0]['pose'],atol=1e-12,rtol=0)
        assert on-off==pytest.approx(2.,abs=1e-8)
        assert all(not r['extrude'] for r in rows[:-1])
    assert path['lift_count']==1
    deposited=np.array(path['strokes'][0]['points_m'])
    assert np.ptp(deposited[:,2])<1e-10
    surface=make_surface('plane')
    assert deposited[0]==pytest.approx(surface.point(-.03,0,.0013))
    assert deposited[-1]==pytest.approx(surface.point(.03,0,.0013))


def test_short_strokes_do_not_add_recycle_cycles():
    job,_,_=compile_drawing(drawing(),surface='plane')
    assert job['validation']['pen_recycle_count']==0
    assert sum(e['extrude'] for e in job['pen_events'])==2


def test_waits_one_and_a_half_seconds_before_lifting():
    profile=synthetic_profile('plane')
    profile['pen']['release_delay_s']=1.5
    job,_,_=compile_drawing(drawing(),surface='plane',profile=profile)
    release=None
    for index,row in enumerate(job['samples']):
        if row['phase']=='RELEASE' and job['samples'][index-1]['phase']!='RELEASE':
            release=row
        if row['phase']=='LIFT' and job['samples'][index-1]['phase']!='LIFT':
            assert release is not None
            assert row['time_s']-release['time_s']>=1.5-1e-8
            waiting=[r for r in job['samples'] if release['time_s']<=r['time_s']<row['time_s']]
            assert all(not r['extrude'] for r in waiting)
            assert np.allclose([r['pose'] for r in waiting],release['pose'],atol=1e-12,rtol=0)


def test_explicit_drawing_handoff_never_reextracts_image(tmp_path, monkeypatch):
    import json
    from pen_toolpath import worker
    source=tmp_path/'drawing.json';source.write_text(json.dumps(drawing()))
    def forbidden(*args,**kwargs):
        raise AssertionError('stage two re-extracted the image')
    monkeypatch.setattr(worker,'extract_drawing',forbidden)
    result=worker.prepare({'drawing_path':str(source),'image_path':'not-an-image.png','surface':'plane'},tmp_path/'output')
    assert result['success']


def test_surface_preview_survives_failed_robot_path_without_executable_job(tmp_path, monkeypatch):
    import json
    from pen_toolpath import worker, robot_preview
    source=tmp_path/'drawing.json';source.write_text(json.dumps(drawing()))
    def fail_motion(*args,**kwargs):
        raise ValueError('Unsafe posture')
    monkeypatch.setattr(robot_preview,'build_preview',fail_motion)
    output=tmp_path/'output'
    with pytest.raises(ValueError,match='Unsafe posture'):
        worker.prepare({'drawing_path':str(source),'surface':'plane','robot_preview':{'enabled':True}},output)
    assert json.loads((output/'toolpath.json').read_text())['strokes']
    assert not (output/'job.json').exists()


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


@pytest.mark.parametrize('sample_rate',[100,1000])
def test_continuous_stroke_does_not_stop_at_polyline_vertices(sample_rate):
    shape={'kind':'pen_drawing','schema_version':1,'units':'m','strokes':[
        {'id':'one','points_m':[[-.008,0],[-.004,.001],[0,0],[.004,.001],[.008,0]]}]}
    job,_,_=compile_drawing(shape,surface='plane',settings={'continuous_strokes':True,'speed_m_s':.003,
                                                         'sample_rate_hz':sample_rate})
    rows=[s for s in job['samples'] if s['phase']=='DRAW']
    q=np.array([r['pose'][:3] for r in rows]);t=np.array([r['time_s'] for r in rows])
    speed=np.linalg.norm(np.diff(q,axis=0)/np.diff(t)[:,None],axis=1)
    # Ignore initial press dwell and the intended endpoint speed ramps.
    moving=np.flatnonzero(speed>.0005)
    interior=speed[moving[0]+10:moving[-1]-10]
    assert np.min(interior)>.0001
    assert job['validation']['max_draw_speed_m_s']<=.003*1.02
