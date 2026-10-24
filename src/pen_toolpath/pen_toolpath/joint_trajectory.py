"""Generate a checked joint-controller artifact from factory-model IK samples."""
import copy
import hashlib
from pathlib import Path
import numpy as np
import toppra as ta
from scipy.interpolate import CubicHermiteSpline, PchipInterpolator
from scipy.spatial.transform import Rotation
from .robot_preview import Arm, transform


class TimingLimitError(ValueError):
    def __init__(self, duration_s):
        super().__init__('Retimed extrusion exceeds the continuous-stroke pause interval')
        self.duration_s=duration_s


def align_recycle_intervals(frames, interval):
    """Stretch short periodic draw spans to the requested cadence, without holds."""
    i=0
    while i<len(frames):
        if frames[i]['phase']!='DRAW':
            i+=1;continue
        end=i+1
        while end<len(frames) and frames[end]['phase']=='DRAW':end+=1
        resume=end
        while resume<len(frames) and frames[resume]['phase']=='RELEASE':resume+=1
        if resume<len(frames) and frames[resume]['phase']=='DRAW':
            start_time=frames[i]['time_s'];duration=frames[end]['time_s']-start_time
            if duration>interval+1e-6:raise TimingLimitError(duration)
            scale=interval/duration;extra=interval-duration
            if max(abs(x) for k in (i,end) for x in frames[k]['velocities'])>1e-8:
                raise ValueError('Recycle must start and end at rest')
            for k in range(i,end+1):
                frames[k]['time_s']=start_time+(frames[k]['time_s']-start_time)*scale
                frames[k]['velocities']=(np.asarray(frames[k]['velocities'])/scale).tolist()
            for k in range(end+1,len(frames)):frames[k]['time_s']+=extra
        i=end


def build_joint_trajectory(job, context, motion):
    profile = job['profile']
    factory = profile.get('robot_kinematics', {})
    if (not factory or factory.get('robot_sn') != job['robot_sn'] or
            hashlib.sha256(Path(factory['path']).read_bytes()).hexdigest() != factory.get('sha256')):
        raise ValueError('Joint execution requires matching factory kinematics')
    if profile.get('preview_kinematic_correction') or context.get('base_correction'):
        raise ValueError('Display-only kinematic correction cannot drive physical joints')
    arm = Arm(context)
    frames = copy.deepcopy(motion['frames'])
    q = np.array([f['positions'] for f in frames], float)
    minimum_posture_sigma,maximum_posture_condition = arm.require_safe_postures(q)
    old_t = np.array([f['time_s'] for f in frames], float)
    # Tiny duplicate IK intervals otherwise produce artificial acceleration spikes.
    dt = np.maximum(np.diff(old_t), .01)
    original_duration = old_t[-1]
    velocity_limit = np.minimum(.5*arm.velocity, .5)
    acceleration_limit = 1.
    parameter = np.r_[0., np.cumsum(dt)]
    slopes = PchipInterpolator(parameter, q, axis=0).derivative()(parameter)
    slopes[0] = 0.; slopes[-1] = 0.
    geometric = CubicHermiteSpline(parameter, q, slopes, axis=0)

    class TimedPath(ta.interpolator.AbstractGeometricPath):
        @property
        def dof(self): return 8
        @property
        def path_interval(self): return np.array([0.,parameter[-1]])
        def __call__(self, x, order=0):
            x = np.asarray(x)
            clock = x if order == 0 else np.ones_like(x) if order == 1 else np.zeros_like(x)
            return np.concatenate([geometric(x, order), clock[...,None]], axis=-1)

    # The eighth coordinate is the original clock: its speed limit prevents
    # retiming from shortening commanded dwells or speeding up the source path.
    grid = np.r_[np.array([parameter[:-1]+u*dt for u in (0.,.25,.5,.75)]).T.ravel(), parameter[-1]]
    algorithm = ta.algorithm.TOPPRA([
        ta.constraint.JointVelocityConstraint([*(.9*velocity_limit),1.]),
        ta.constraint.JointAccelerationConstraint([*([.8*acceleration_limit]*7),1000.]),
    ], TimedPath(), gridpoints=grid, solver_wrapper='seidel')
    _, speeds, _ = algorithm.compute_parameterization(0.,0.)
    if speeds is None or np.any(speeds[:-1]+speeds[1:]<=0):
        raise ValueError('Unable to retime joint path within dynamic limits')
    grid_time = np.r_[0.,np.cumsum(2*np.diff(grid)/(speeds[:-1]+speeds[1:]))]
    # Retain the solver grid so the ROS cubic follows the bounded timing law
    # through acceleration changes, rather than skipping those control points.
    source_frames = frames
    original_parameter = parameter
    expanded = []
    for position in grid:
        i = min(int(np.searchsorted(original_parameter, position, side='right')-1), len(source_frames)-1)
        frame = copy.deepcopy(source_frames[i])
        if i+1<len(source_frames) and frame['job_time_s'] is not None and source_frames[i+1]['job_time_s'] is not None:
            u=(position-original_parameter[i])/(original_parameter[i+1]-original_parameter[i])
            frame['job_time_s']=(1-u)*frame['job_time_s']+u*source_frames[i+1]['job_time_s']
        expanded.append(frame)
    frames=expanded
    q=geometric(grid)
    t=grid_time
    v=geometric(grid,1)*speeds[:,None]
    original_dt=np.diff(grid)
    spline = CubicHermiteSpline(t,q,v,axis=0)
    c3,c2,c1,_ = spline.c
    h = np.diff(t)[:,None]
    with np.errstate(divide='ignore',invalid='ignore'):
        vertex=np.where(c3!=0,-c2/(3*c3),0.)
    vertex=np.clip(vertex,0.,h)
    peak_v=np.maximum.reduce([np.abs(c1),np.abs(3*c3*h*h+2*c2*h+c1),
                              np.abs(3*c3*vertex*vertex+2*c2*vertex+c1)])
    peak_a=np.maximum(np.abs(2*c2),np.abs(6*c3*h+2*c2))
    # Check exact extrema of the cubic actually sent to ROS, including each
    # side of acceleration discontinuities, then add a small numeric margin.
    final_scale=max(1.,float(np.max(peak_v/velocity_limit)),
                    float(np.sqrt(np.max(peak_a/acceleration_limit))))*1.01
    t*=final_scale;v/=final_scale
    # Zero-motion dwells keep the exact requested duration (not a scaled pause).
    stationary=(np.max(np.abs(np.diff(q,axis=0)),axis=1)<1e-13) & (np.max(np.abs(v[:-1]),axis=1)<1e-13) & (np.max(np.abs(v[1:]),axis=1)<1e-13)
    dt=np.diff(t)
    dt[stationary]=original_dt[stationary]
    t=np.r_[0.,np.cumsum(dt)]
    spline=CubicHermiteSpline(t,q,v,axis=0)
    a,b,c,_=spline.c
    h=np.diff(t)[:,None]
    with np.errstate(divide='ignore',invalid='ignore'):
        vertex=np.clip(np.where(a!=0,-b/(3*a),0.),0.,h)
    final_v=np.maximum.reduce([np.abs(c),np.abs(3*a*h*h+2*b*h+c),np.abs(3*a*vertex*vertex+2*b*vertex+c)])
    final_a=np.maximum(np.abs(2*b),np.abs(6*a*h+2*b))
    if np.any(final_v>velocity_limit+1e-6) or np.any(final_a>acceleration_limit+1e-6):
        raise ValueError('Final joint schedule exceeds dynamic limits')
    probe=t[:-1,None]+np.diff(t)[:,None]*np.array([0.,.25,.5,.75,1.])
    scale=t[-1]/original_duration
    if t[-1]>3600:raise ValueError('Joint trajectory exceeds one hour after dynamic limits')
    samples = spline(probe)
    if np.any(samples < arm.lower) or np.any(samples > arm.upper):
        raise ValueError('Joint interpolation exceeds joint limits')
    # Check the same interior samples for geometry and posture, in bounded
    # batches so large paths do not require per-waypoint Python kinematics.
    fractions=np.array([.25,.5,.75])
    interior=spline(t[:-1,None]+np.diff(t)[:,None]*fractions).reshape(-1,7)
    sigma,condition=arm.require_safe_postures(interior)
    minimum_posture_sigma=min(minimum_posture_sigma,sigma)
    maximum_posture_condition=max(maximum_posture_condition,condition)
    tip_poses=np.concatenate([arm.forward_many(q[i:i+4096]) for i in range(0,len(q),4096)])
    actual=np.concatenate([arm.forward_many(interior[i:i+4096])[:,:3,3]
                           for i in range(0,len(interior),4096)]).reshape(-1,3,3)
    expected=(tip_poses[:-1,None,:3,3]*(1-fractions[None,:,None])+
              tip_poses[1:,None,:3,3]*fractions[None,:,None])
    drawing=np.array([a['job_time_s'] is not None and b['job_time_s'] is not None
                      for a,b in zip(frames,frames[1:])])
    max_deviation=float(np.linalg.norm(actual[drawing]-expected[drawing],axis=2).max()) if drawing.any() else 0.
    if max_deviation > .0005:
        raise ValueError('Joint interpolation deviates more than 0.5 mm from the checked tip path')
    tip_to_tcp = np.linalg.inv(np.linalg.inv(transform(profile['T_flange_rdk_tcp'])) @
                               transform(profile['T_flange_pen_tip']))
    for i, frame in enumerate(frames):
        tcp = tip_poses[i] @ tip_to_tcp
        frame.update(time_s=float(t[i]), positions=q[i].tolist(), tip_m=tip_poses[i,:3,3].tolist(), velocities=v[i].tolist(),
                     tcp_pose=[*tcp[:3,3].tolist(), *Rotation.from_matrix(tcp[:3,:3]).as_quat().tolist()])
    draw_at=None;max_draw_duration=0.
    for frame in frames:
        if frame['phase']=='DRAW' and draw_at is None:draw_at=frame['time_s']
        if frame['phase']!='DRAW' and draw_at is not None:
            max_draw_duration=max(max_draw_duration,frame['time_s']-draw_at);draw_at=None
    if max_draw_duration>job['pen'].get('recycle_interval_s',15.)+1e-6:
        raise TimingLimitError(max_draw_duration)
    pressed_at = None
    for frame in frames:
        if frame['extrude'] and pressed_at is None: pressed_at = frame['time_s']
        if not frame['extrude'] and pressed_at is not None:
            cap = min(job['pen']['max_press_s'],job['pen'].get('recycle_interval_s',15.))
            if frame['time_s']-pressed_at > cap+.025:
                raise TimingLimitError(frame['time_s']-pressed_at)
            pressed_at = None
    align_recycle_intervals(frames,job['pen'].get('recycle_interval_s',15.))
    max_draw_duration=max(max_draw_duration,job['pen'].get('recycle_interval_s',15.)) if any(frames[i]['phase']=='RELEASE' and frames[i+1]['phase']=='DRAW' for i in range(len(frames)-1)) else max_draw_duration
    return {**motion, 'kind':'pen_joint_trajectory', 'simulation_only':False,
            'factory_kinematics_sha256':factory['sha256'], 'frames':frames,
            'duration_s':frames[-1]['time_s'], 'time_scale':frames[-1]['time_s']/original_duration,
            'max_interpolation_deviation_m':max_deviation, 'max_continuous_draw_s':max_draw_duration,
            'posture_checked':True, 'minimum_posture_sigma':minimum_posture_sigma,
            'maximum_posture_condition':maximum_posture_condition,
            'velocity_limit_rad_s':velocity_limit.tolist(), 'acceleration_limit_rad_s2':acceleration_limit}
