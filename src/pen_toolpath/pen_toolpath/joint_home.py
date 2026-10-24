"""Build and check a bounded joint-controller return to the taught Home target."""
import argparse
from copy import deepcopy
import json
import math
from pathlib import Path

import numpy as np

from .compiler import transform
from .robot_preview import Arm
from pen_surface_geometry.api import minimum_segment_radius


def build_joint_home(source, state, model, *, allow_departure=False):
    profile=source['profile'];home=profile.get('home',{})
    arm=Arm({'joints':model['joints'],'flange_link':source['robot_sn']+'_flange',
             'flange_to_visual_tip':profile['T_flange_pen_tip']})
    if home.get('controller')!='rizon_arm_controller' or home.get('joint_names')!=arm.names:
        raise ValueError('Recorded Home joint names do not match this robot')
    start=np.asarray([state['joints'][name] for name in arm.names],float)
    end=np.asarray(home.get('joint_positions_rad'),float)
    if end.shape!=(7,) or not np.isfinite(end).all():raise ValueError('Invalid recorded Home joint pose')
    if np.any(np.minimum(start,end)<arm.lower) or np.any(np.maximum(start,end)>arm.upper):
        raise ValueError('Home route exceeds joint limits')
    # Same stop-to-stop quintic route and geometric samples, retimed to 3x.
    # Relative to the old 0.15 rad/s / 0.3 rad/s^2 bounds, derivatives scale
    # by 3 and 9 respectively. Check against each robot joint's velocity limit.
    delta=end-start;largest=float(np.abs(delta).max())
    original_duration=1.1*max(5.,1.875*largest/.15,math.sqrt(5.78*largest/.3))
    count=math.ceil(original_duration*100);duration=count/100/3.
    if np.any(1.875*np.abs(delta)/duration>arm.velocity):
        raise ValueError('3x Home speed exceeds a robot joint velocity limit')
    if duration>120:raise ValueError('Home route exceeds the 120-second bound')
    surface=transform(profile['T_world_surface'],'surface')
    normal=surface[:3,2];origin=surface[:3,3]
    measured=transform({'xyz_m':state['pose'][:3],'quaternion_xyzw':state['pose'][3:]},'current TCP')
    tip_tcp=np.linalg.inv(transform(profile['T_flange_pen_tip'],'tip'))@transform(profile['T_flange_rdk_tcp'],'TCP')
    measured_tip=measured@np.linalg.inv(tip_tcp)
    correction=measured_tip[:3,3]-arm.forward(start)[:3,3]
    radius=profile['tool_envelope']['radius_m'];length=profile['tool_envelope']['length_m']
    minimum=math.inf;rows=[];departure_floor=None;clear_of_surface=False
    for i in range(count+1):
        u=i/count;f=10*u**3-15*u**4+6*u**5
        df=(30*u*u-60*u**3+30*u**4)/duration
        ddf=(60*u-180*u*u+120*u**3)/(duration*duration)
        q=start+f*delta;tip=arm.forward(q);position=tip[:3,3]+correction;axis=-tip[:3,2]
        near=position+radius*axis;far=position+(length-radius)*axis
        if profile['surface']['type']=='sphere':
            gap=minimum_segment_radius(near,far,origin)-float(profile['surface']['radius_m'])-radius
        else:
            gap=min((near-origin)@normal,(far-origin)@normal)-radius
        minimum=min(minimum,float(gap))
        if i==0 and allow_departure and gap<.005:
            stand_off=max(.0013,float(profile.get('pen',{}).get('stand_off_m',.0013)))
            if gap<stand_off-.0001:
                raise ValueError('Tool is below the drawing clearance; lift it clear before homing')
            departure_floor=float(gap)
        if departure_floor is not None and not clear_of_surface:
            if gap<departure_floor-1e-7:
                raise ValueError('Joint Home retreat would move closer to the surface')
            clear_of_surface=gap>=.005
        elif gap<.005:
            raise ValueError('Direct joint Home route is too close to the surface; lift the tool clear before homing')
        if i and (i%10==0 or i==count):
            rows.append({'time_s':duration*i/count,'positions':q.tolist(),'velocities':(df*delta).tolist(),'accelerations':(ddf*delta).tolist()})
    if departure_floor is not None and not clear_of_surface:
        raise ValueError('Joint Home retreat does not reach the required clearance')
    result=deepcopy(source)
    result.update(purpose='joint_home',duration_s=.01,
        samples=[{'time_s':t,'pose':state['pose'],'phase':'HOLD','extrude':False} for t in [0.,.01]],
        pen_events=[{'time_s':0.,'extrude':False}],
        joint_home_route={'checked_joint_home':True,'start_positions':start.tolist(),'points':rows,
                          'speed_scale':3.,
                          'minimum_tool_plane_clearance_m':minimum,
                          'departure_floor_m':departure_floor,
                          'validation':'Joint limits and sampled tool capsule versus registered surface; complete arm collision check not performed.'})
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--request',type=Path,required=True);parser.add_argument('--out',type=Path,required=True)
    args=parser.parse_args();request=json.loads(args.request.read_text())
    result=build_joint_home(request['source'],request['state'],request['model'],allow_departure=True)
    args.out.write_text(json.dumps(result,separators=(',',':'))+'\n')


if __name__=='__main__':main()
