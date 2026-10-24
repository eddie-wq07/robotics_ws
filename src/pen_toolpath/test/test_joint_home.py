import copy
import math
import numpy as np
import pytest
from pen_toolpath.joint_home import build_joint_home
from pen_toolpath.robot_preview import Arm


def inputs():
    ident={'xyz_m':[0,0,0],'quaternion_xyzw':[0,0,0,1]}
    names=[f'robot_joint{i}' for i in range(1,8)];chain=[]
    parent='base'
    for i,name in enumerate(names):
        child=f'link{i}';chain.append({'name':name,'parent':parent,'child':child,'type':'revolute',
            'xyz':[0,0,.05],'rpy':[0,0,0],'axis':[0,0,1],'limit':{'lower':-math.pi,'upper':math.pi,'velocity':1}});parent=child
    chain.append({'name':'flange','parent':parent,'child':'robot_flange','type':'fixed','xyz':[0,0,0],'rpy':[math.pi,0,0]})
    source={'robot_sn':'robot','profile':{'surface':{'type':'plane'},'T_world_surface':ident,'T_flange_pen_tip':ident,'T_flange_rdk_tcp':ident,
        'home':{'controller':'rizon_arm_controller','joint_names':names,'joint_positions_rad':[.5,0,0,0,0,0,0]},
        'tool_envelope':{'radius_m':.01,'length_m':.1}}}
    state={'joints':dict.fromkeys(names,0.),'pose':[0,0,.35,1,0,0,0]}
    return source,state,{'joints':chain}


def test_joint_home_uses_recorded_joint_target_and_stays_pen_off():
    source,state,model=inputs();before=copy.deepcopy(source)
    home=build_joint_home(source,state,model)
    route=home['joint_home_route']
    assert home['purpose']=='joint_home' and source==before
    assert route['start_positions']==[0]*7
    assert route['points'][-1]['positions']==pytest.approx([.5,0,0,0,0,0,0])
    assert all(not s['extrude'] for s in home['samples'])
    assert route['minimum_tool_plane_clearance_m']>.3
    assert max(abs(v) for p in route['points'] for v in p['velocities'])<=.45
    previous_duration=math.ceil(1.1*max(5.,1.875*.5/.15,math.sqrt(5.78*.5/.3))*100)/100
    assert route['points'][-1]['time_s']==pytest.approx(previous_duration/3)
    assert route['speed_scale']==3
    assert route['points'][-1]['time_s']<=120


def test_three_times_home_speed_still_checks_robot_velocity_limits():
    source,state,model=inputs()
    model['joints'][0]['limit']['velocity']=.1
    with pytest.raises(ValueError,match='velocity limit'):
        build_joint_home(source,state,model)


def test_joint_home_rejects_limit_and_clearance_violations():
    source,state,model=inputs();source['profile']['home']['joint_positions_rad'][0]=4
    with pytest.raises(ValueError,match='joint limits'):build_joint_home(source,state,model)
    source,state,model=inputs();source['profile']['T_world_surface']={'xyz_m':[0,0,.349],'quaternion_xyzw':[0,0,0,1]}
    with pytest.raises(ValueError,match='too close'):build_joint_home(source,state,model)


def test_home_can_retreat_from_drawing_height_but_cannot_dip(monkeypatch):
    source,state,model=inputs()
    state['pose']=[0,0,.0013,1,0,0,0]
    def rising(self,q):
        pose=np.diag([1.,-1.,-1.,1.]);pose[2,3]=.0013+q[0]
        return pose
    monkeypatch.setattr(Arm,'forward',rising)
    home=build_joint_home(source,state,model,allow_departure=True)
    assert home['joint_home_route']['departure_floor_m']==pytest.approx(.0013)
    def dipping(self,q):
        pose=rising(self,q);pose[2,3]-=.8*math.sin(math.pi*q[0])
        return pose
    monkeypatch.setattr(Arm,'forward',dipping)
    with pytest.raises(ValueError,match='closer'):
        build_joint_home(source,state,model,allow_departure=True)
    state['pose'][2]=0
    with pytest.raises(ValueError,match='below the drawing clearance'):
        build_joint_home(source,state,model,allow_departure=True)


def test_home_checks_sphere_side_not_only_horizontal_plane():
    source,state,model=inputs()
    source['profile']['surface']={'type':'sphere','radius_m':.04}
    source['profile']['T_world_surface']['xyz_m']=[.03,0,.35]
    with pytest.raises(ValueError,match='too close'):
        build_joint_home(source,state,model)


def test_home_checks_cylinder_body_not_only_horizontal_plane():
    source,state,model=inputs()
    source['profile']['surface']={'type':'cylinder','radius_m':.06,'height_m':.08,
                                  'lying':True,'frame_origin':'cylinder_center'}
    source['profile']['T_world_surface']['xyz_m']=[0,0,.3]
    with pytest.raises(ValueError,match='too close'):
        build_joint_home(source,state,model)
