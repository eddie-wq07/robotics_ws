"""Compile drawing geometry into an immutable sampled TCP/pen schedule.

Sampling and time laws live here, outside the ROS controller's update loop.
"""
from copy import deepcopy
from hashlib import sha256
import json
import math
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation, Slerp
from scipy.interpolate import CubicSpline
import yaml

from pen_surface_geometry.api import make_surface, surface_spec, minimum_segment_radius
from .clearance import MIN_PEN_CLEARANCE_M, MIN_SPHERE_PEN_CLEARANCE_M


MAX_SAMPLES = 500_000


def digest(data):
    return sha256(json.dumps(data, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()).hexdigest()


def synthetic_profile(surface='sphere'):
    """Explicit fake-only registration; never inferred as a physical calibration."""
    ident = {'xyz_m': [0, 0, 0], 'quaternion_xyzw': [0, 0, 0, 1]}
    return {'calibrated': False, 'fake_only': True, 'robot_sn': 'Rizon4s-000000',
            'world_frame': 'world', 'surface': {'type': surface, 'radius_m': 0.04},
            'T_world_surface': {'xyz_m': [0.45, 0, 0.25], 'quaternion_xyzw': [0, 0, 0, 1]},
            'T_flange_pen_tip': deepcopy(ident), 'T_flange_rdk_tcp': deepcopy(ident),
            'pen': {'release_delay_s': 0.2, 'press_delay_s': 0.1, 'max_press_s': 120.0},
            'qualified': {'cartesian': False, 'rt': False}, 'synthetic': True,
            'tool_envelope': {'radius_m': 0.005, 'length_m': 0.15}}


def transform(value, name):
    if not isinstance(value, dict):
        raise ValueError(f'{name} is TBD; calibration required')
    try:
        xyz = np.asarray(value['xyz_m'], float)
        q = np.asarray(value['quaternion_xyzw'], float)
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError(f'{name} is TBD; measured xyz_m and quaternion_xyzw required') from error
    if xyz.shape != (3,) or q.shape != (4,) or not np.isfinite(xyz).all() or not np.isfinite(q).all() or abs(float(q @ q) - 1) > 1e-3:
        raise ValueError(f'{name} requires finite position and unit quaternion')
    result = np.eye(4)
    result[:3, :3] = Rotation.from_quat(q).as_matrix(); result[:3, 3] = xyz
    return result


def load_profile(path_or_data, mode, kind):
    if mode not in ('preview', 'fake', 'real'):
        raise ValueError('execution_mode must be preview, fake or real')
    if path_or_data is None:
        if mode == 'real':
            raise ValueError('physical calibration is TBD; profile_path is required')
        return synthetic_profile(kind)
    profile = deepcopy(path_or_data) if isinstance(path_or_data, dict) else yaml.safe_load(Path(path_or_data).read_text())
    if not isinstance(profile, dict):
        raise ValueError('profile must be a mapping')
    if mode == 'real':
        if profile.get('calibrated') is not True or profile.get('fake_only', True) or profile.get('synthetic', False):
            raise ValueError('real execution requires measured calibration; synthetic/TBD profile refused')
        qualified = profile.get('qualified', {})
        if qualified.get('cartesian') is not True or qualified.get('rt') is not True:
            raise ValueError('physical Cartesian and real-time qualification are TBD')
    if mode == 'real':
        # The current hardware streams these coordinates directly to the RDK,
        # whose Cartesian frame is the robot base. No command/feedback frame
        # adapter exists between the world-labelled ROS API and that interface.
        world_base = transform(profile.get('T_world_base'), 'T_world_base')
        if not np.allclose(world_base, np.eye(4), rtol=0, atol=1e-9):
            raise ValueError('real execution requires world to coincide with the RDK base; T_world_base must be identity')
        for name in ('T_world_base', 'T_base_jig', 'T_flange_holder', 'T_holder_pen',
                     'T_world_surface', 'T_flange_pen_tip', 'T_flange_rdk_tcp'):
            transform(profile.get(name), name)
        tool_name = profile.get('rdk_tool_name')
        if not isinstance(tool_name, str) or not tool_name.strip() or tool_name == 'TBD':
            raise ValueError('rdk_tool_name is TBD')
        payload = profile.get('payload', {})
        _number(payload, 'mass_kg', None, 0.00001, 20)
        for field, count in [('com_m', 3), ('inertia_kg_m2', 6)]:
            values = np.asarray(payload.get(field), float)
            if values.shape != (count,) or not np.isfinite(values).all():
                raise ValueError(f'payload.{field} is TBD or invalid')
        if any(profile.get('qualification', {}).get(key) is not True for key in ('rt', 'clearance', 'pen', 'tool')):
            raise ValueError('physical clearance, timing, pen and tool qualification are TBD')
        if profile.get('camera', {}).get('enabled') is True:
            transform(profile['camera'].get('T_flange_camera'), 'T_flange_camera')
    if profile.get('world_frame', 'world') != 'world':
        raise ValueError('controller requires world frame')
    if profile.get('surface', {}).get('type', kind) != kind:
        if mode != 'real' and profile.get('fake_only') is True:
            profile['surface'] = {**profile.get('surface', {}), 'type': kind}
        else:
            raise ValueError('selected surface does not match calibration profile')
    return profile


def _number(settings, key, default, low, high):
    try:
        value = float(settings.get(key, default))
    except (TypeError, ValueError) as error:
        raise ValueError(f'{key} is TBD; measured setting required') from error
    if not math.isfinite(value) or not low <= value <= high:
        raise ValueError(f'{key} must be finite within [{low}, {high}]')
    return value


def compile_drawing(drawing, *, surface='sphere', profile=None, execution_mode='preview', settings=None):
    if drawing.get('kind') != 'pen_drawing' or drawing.get('schema_version') != 1 or drawing.get('units') != 'm':
        raise ValueError('expected version-1 pen_drawing in metres')
    settings = deepcopy(settings or {})
    spec = surface_spec(surface); model = make_surface(spec)
    profile = load_profile(profile, execution_mode, spec['type'])
    calibrated_surface = profile.get('surface', {})
    if spec['type'] == 'sphere':
        radius = calibrated_surface.get('radius_m', 0.04)
        if not isinstance(radius, (float, int)) or not math.isfinite(radius) or abs(radius - spec['radius_m']) > 1e-9:
            raise ValueError('surface radius does not match the calibration profile')
    elif execution_mode == 'real' or 'size_m' in calibrated_surface:
        dimensions = np.asarray(calibrated_surface.get('size_m'), float)
        if dimensions.shape != (3,) or not np.isfinite(dimensions).all() or not np.allclose(dimensions, spec['size_m'], rtol=0, atol=1e-9):
            raise ValueError('plane dimensions are TBD or do not match the calibration profile')
    world_surface = transform(profile.get('T_world_surface'), 'T_world_surface')
    flange_tip = transform(profile.get('T_flange_pen_tip'), 'T_flange_pen_tip')
    flange_tcp = transform(profile.get('T_flange_rdk_tcp'), 'T_flange_rdk_tcp')
    tip_tcp = np.linalg.inv(flange_tip) @ flange_tcp
    # Hold the nozzle vertically downward in world coordinates. Keep one yaw
    # aligned with the drawing's centre frame instead of following every local
    # sphere normal, which swings the long pen/holder sideways.
    centre_right = model.frame(np.array([0.]), np.array([0.]))[0][0]
    world_right = world_surface[:3, :3] @ centre_right
    world_right[2] = 0.
    if np.linalg.norm(world_right)<1e-8:
        raise ValueError('Drawing heading cannot define a vertical pen orientation')
    world_right /= np.linalg.norm(world_right)
    down = np.array([0., 0., -1.])
    local_up = world_surface[:3, :3].T @ -down
    envelope = profile.get('tool_envelope', {})
    body_radius = _number(envelope, 'radius_m', None, 0.0001, 0.2)
    body_length = _number(envelope, 'length_m', None, 0.001, 0.5)
    if body_length < 2 * body_radius:
        raise ValueError('tool envelope length must be at least twice its radius')
    pen = profile.get('pen', {})
    release_delay = _number(pen, 'release_delay_s', None, 0., 10)
    press_delay = _number(pen, 'press_delay_s', None, 0, 10)
    recycle_press_delay = _number(pen, 'recycle_press_delay_s', press_delay, 0, 10)
    end_dwell = _number(pen, 'end_extrusion_dwell_s', 0., 0., 10.)
    max_press = _number(pen, 'max_press_s', None, 0.1, 600)
    requested_recycle_interval = _number(pen, 'recycle_interval_s', 15., 1., 600.)
    recycle_interval = requested_recycle_interval
    recycle_interval = min(recycle_interval, _number(settings, 'recycle_planning_interval_s', recycle_interval, 1., 600.))
    continuous_strokes = settings.get('continuous_strokes', False)
    if type(continuous_strokes) is not bool:raise ValueError('continuous_strokes must be boolean')
    recycle_pause = _number(pen, 'recycle_pause_s', 2., 0.02, 10.)
    if recycle_pause < release_delay:
        raise ValueError('Recycle pause must allow the full pen release delay')
    speed = _number(settings, 'speed_m_s', profile.get('drawing', {}).get('speed_m_s', 0.01), 0.0001, 0.05)
    travel_speed = _number(settings, 'travel_speed_m_s', profile.get('drawing', {}).get('travel_speed_m_s', speed), 0.0001, 0.05)
    draw_scale = _number(settings, 'draw_speed_scale', profile.get('drawing', {}).get('draw_speed_scale', 1.), 1., 3.)
    travel_scale = _number(settings, 'non_extruding_speed_scale', profile.get('drawing', {}).get('non_extruding_speed_scale', 1.), 1., 3.)
    approach_scale = _number(settings, 'approach_speed_scale', profile.get('drawing', {}).get('approach_speed_scale', 1.), 1., 2.)
    acceleration = _number(settings, 'acceleration_m_s2', 0.1, 0.001, 0.5)
    angular_speed = _number(settings, 'angular_speed_rad_s', 0.25, 0.01, 0.5)
    angular_acceleration = _number(settings, 'angular_acceleration_rad_s2', 1, 0.01, 2)
    rate = _number(settings, 'sample_rate_hz', profile.get('drawing', {}).get('sample_rate_hz',100), 50, 1000)
    if recycle_pause <= 1/rate:
        raise ValueError('Recycle pause must exceed one sample interval')
    if max(press_delay,recycle_press_delay)+end_dwell+2/rate >= recycle_interval:
        raise ValueError('Press delay leaves no drawing time before the recycle interval')
    lift = _number(settings, 'lift_m', 0.005, 0.001, 0.02)
    stand_off = _number(settings, 'stand_off_m', pen.get('stand_off_m', MIN_PEN_CLEARANCE_M),
                        MIN_SPHERE_PEN_CLEARANCE_M if model.kind == 'sphere' else MIN_PEN_CLEARANCE_M, 0.005)
    air_offset = _number(settings, 'air_offset_m', 0.0, 0.0, 0.02)
    air_run = air_offset > 0.0
    stand_off += air_offset
    clearance = _number(settings, 'clearance_margin_m', 0.00005, 0.000001, 0.002)
    if stand_off <= clearance:
        raise ValueError('stand_off_m must exceed the surface clearance margin')
    raw_strokes = drawing.get('strokes', [])
    if not raw_strokes or len(raw_strokes) > 2000:
        raise ValueError('drawing must contain 1..2000 strokes')
    strokes = []
    for item in raw_strokes:
        points = np.asarray(item.get('points_m'), float)
        if points.ndim != 2 or points.shape[1] != 2 or len(points) < 2 or not np.isfinite(points).all():
            raise ValueError('each drawing stroke needs at least two finite 2D points')
        # Remove consecutive duplicates before deriving durations.
        points = points[np.r_[True, np.linalg.norm(np.diff(points, axis=0), axis=1) > 1e-10]]
        if len(points) >= 2:
            strokes.append((item, points))
    if not strokes:
        raise ValueError('drawing contains only zero-length strokes')
    all_points = np.vstack([points for _, points in strokes])
    reach = float(np.linalg.norm(all_points, axis=1).max())
    fit = model.fit(reach, (float(np.abs(all_points[:, 0]).max()), float(all_points[:, 1].min()), float(all_points[:, 1].max())))
    samples, events, local_tips, local_normals, preview_strokes, preview_travels = [], [], [], [], [], []
    time_s = 0.0
    previous_q = None

    def pose(st, height):
        tip = np.asarray(model.point(st[0], st[1], height), float)
        outward = np.asarray(model.normal(st[0], st[1]), float)
        # The first capsule endpoint lies one radius behind the tip. Find the
        # largest permissible deviation from the outward surface normal, then
        # choose the closest axis to world vertical inside that cone. The final
        # full capsule/path checks below still validate the resulting poses.
        margin = clearance + 1e-7
        if spec['type']=='sphere':
            rho=float(np.linalg.norm(tip-model.centre))
            cosine=((model.R+body_radius+margin)**2-rho*rho-body_radius**2)/(2*rho*body_radius)
        else:
            cosine=1-(tip[2]-model.origin[2]-margin)/body_radius
        allowed=min(math.radians(25.),math.acos(float(np.clip(cosine,-1.,1.))))
        angle=math.acos(float(np.clip(local_up@outward,-1.,1.)))
        # Start the small clearance tilt gradually before the cone boundary.
        # This C2 smooth upper bound avoids a velocity kink at max(0, excess).
        excess=angle-allowed
        band=math.radians(5.)
        if excess<=-band:
            tilt=0.
        elif excess>=band:
            tilt=excess
        else:
            u=(excess+band)/(2*band)
            tilt=2*band*(u**3-.5*u**4)
        tilt=min(angle,tilt)
        if tilt<1e-12:
            axis=local_up
        else:
            toward_up=local_up-(local_up@outward)*outward
            if np.linalg.norm(toward_up)<1e-8:
                toward_up=world_surface[:3,:3].T@world_right
                toward_up-=outward*(toward_up@outward)
            toward_up/=np.linalg.norm(toward_up)
            deviation=angle-tilt
            axis=math.cos(deviation)*outward+math.sin(deviation)*toward_up
        z=-(world_surface[:3,:3]@axis)
        x=world_right-z*(world_right@z)
        if np.linalg.norm(x)<1e-8:
            x=-down-z*((-down)@z)
        x/=np.linalg.norm(x)
        world_rotation=np.column_stack([x,np.cross(z,x),z])
        local = np.eye(4); local[:3, :3] = world_surface[:3,:3].T@world_rotation; local[:3, 3] = tip
        tcp = world_surface @ local @ tip_tcp
        return tcp, tip

    def append(st, height, phase, extrude, *, event=False):
        nonlocal previous_q
        extrude = bool(extrude and not air_run)
        tcp, tip = pose(st, height)
        q = Rotation.from_matrix(tcp[:3, :3]).as_quat()
        if previous_q is not None and q @ previous_q < 0:
            q = -q
        previous_q = q
        sample = {'time_s': round(time_s, 9), 'pose': [*tcp[:3, 3].tolist(), *q.tolist()],
                  'phase': phase, 'extrude': bool(extrude)}
        if samples and sample['time_s'] <= samples[-1]['time_s']:
            raise ValueError('non-increasing sample time')
        if event or not samples or samples[-1]['extrude'] != sample['extrude']:
            events.append({'time_s': sample['time_s'], 'extrude': bool(extrude)})
        samples.append(sample); local_tips.append(tip)
        # Capsule clearance follows the actual pen axis, not the surface normal.
        local_normals.append(-(world_surface[:3,:3].T @ tcp[:3,:3] @ tip_tcp[:3,:3].T)[:,2])
        if len(samples) > MAX_SAMPLES:
            raise ValueError('job exceeds maximum sample count; reduce drawing complexity')

    def dwell(st, height, duration, phase, extrude, event=False):
        nonlocal time_s
        count = max(1, math.ceil(duration * rate))
        for index in range(count):
            time_s += duration / count
            append(st, height, phase, extrude, event=event and index == 0)

    def motion_timing(a, b, h0, h1, extrude):
        # Rest-to-rest quintic in drawing coordinates, evaluated on the analytic
        # surface at every sample. Never linearly shortcut sphere XYZ endpoints.
        ta, pa = pose(a, h0); tb, pb = pose(b, h1)
        length = max(float(np.linalg.norm(pb - pa)), float(np.linalg.norm(b - a)), abs(h1-h0))
        angle = float(Rotation.from_matrix(ta[:3, :3].T @ tb[:3, :3]).magnitude())
        motion_speed = speed if extrude else travel_speed
        # A small nozzle tilt moves the offset TCP too. Bound derivatives along
        # the actual compensated TCP curve, including clearance-induced tilt.
        parameters=np.linspace(0.,1.,33)
        poses=[pose(a+(b-a)*u,h0+(h1-h0)*u)[0] for u in parameters]
        xyz=np.array([p[:3,3] for p in poses])
        tangent=np.gradient(xyz,parameters,axis=0)
        curve=np.gradient(tangent,parameters,axis=0)
        first=float(np.linalg.norm(tangent,axis=1).max())
        second=float(np.linalg.norm(curve,axis=1).max())
        rotations=Rotation.from_matrix(np.array([p[:3,:3] for p in poses]))
        angular_first=float(((rotations[:-1].inv()*rotations[1:]).magnitude()/np.diff(parameters)).max())
        duration = max(1 / rate, 1.875 * length / motion_speed, math.sqrt(5.78 * length / acceleration),
                       1.875 * angle / angular_speed, math.sqrt(5.78 * angle / angular_acceleration),
                       1.875*first/motion_speed,math.sqrt((5.78*first+1.875**2*second)/acceleration),
                       1.875*angular_first/angular_speed)
        duration *= 1.1  # Budget curvature and finite sampling beyond endpoint estimates.
        duration /= draw_scale if extrude else travel_scale
        return duration, length, angle

    def move(a, b, h0, h1, phase, extrude):
        nonlocal time_s
        duration, length, angle = motion_timing(a, b, h0, h1, extrude)
        count = max(1, math.ceil(duration * rate), math.ceil(length / 0.0002), math.ceil(angle / 0.01))
        start = time_s
        for index in range(1, count+1):
            u = index / count; blend = 10*u**3 - 15*u**4 + 6*u**5
            st = a + (b-a) * blend; height = h0 + (h1-h0) * blend
            time_s = start + duration * u
            append(st, height, phase, extrude)

    first = strokes[0][1][0]
    append(first, stand_off+lift, 'HOLD', False, event=True)
    recycle_count = 0
    for i, (item, points) in enumerate(strokes):
        if i:
            previous = strokes[i-1][1][-1]
            travel_start = len(samples)
            move(previous, points[0], stand_off+lift, stand_off+lift, 'TRAVEL', False)
            preview_travels.append({'after': strokes[i-1][0]['id'], 'before': item['id'],
                                    'points_m': [p.tolist() for p in local_tips[travel_start:]],
                                    'lift_mm': (stand_off+lift)*1000})
        move(points[0], points[0], stand_off+lift, stand_off, 'LOWER', False)
        draw_start = len(samples)
        # The first DRAW sample is the explicit press event and optional lead.
        dwell(points[0], stand_off, max(press_delay, 1/rate), 'DRAW', True, event=True)
        press_at = samples[draw_start]['time_s']

        def recycle(st):
            nonlocal time_s, press_at, recycle_count
            press_duration=max(recycle_press_delay,1/rate)
            press_step=press_duration/math.ceil(press_duration*rate)
            # Explicit off sample makes off-to-on timing exactly the pause,
            # independent of the press lead's sampling interval.
            time_s += 1/rate
            append(st,stand_off,'RELEASE',False,event=True)
            dwell(st,stand_off,recycle_pause-press_step,'RELEASE',False)
            restart = len(samples)
            dwell(st, stand_off, press_duration, 'DRAW', True, event=True)
            press_at = samples[restart]['time_s']
            recycle_count += 1

        if continuous_strokes:
            from .continuous_stroke import rounded_stroke, StrokeTiming
            arc,curve=rounded_stroke(points)
            # Interpolating the sampled curve linearly adds a velocity corner
            # at every sample. Use one C2 curve; all resulting poses still pass
            # the sampled speed, acceleration, orientation and clearance checks.
            stroke_curve=CubicSpline(arc,curve,axis=0,bc_type='natural')
            xyz=np.array([pose(st,stand_off)[0][:3,3] for st in curve])
            # Geometry is tangent through corners. Reserve acceleration for
            # both the speed ramp and the curvature of the surface path.
            tangents=np.gradient(xyz,arc,axis=0)
            tangent_scale=max(1.,float(np.linalg.norm(tangents,axis=1).max()))
            curvature=float(np.linalg.norm(np.gradient(tangents,arc,axis=0),axis=1).max())
            curve_rotations=Rotation.from_matrix(np.array([pose(st,stand_off)[0][:3,:3] for st in curve]))
            angular_scale=float(((curve_rotations[:-1].inv()*curve_rotations[1:]).magnitude()/np.diff(arc)).max())
            effective_speed=min(speed/tangent_scale, math.sqrt(.35*acceleration/max(curvature,1e-9)),
                                angular_speed/max(angular_scale,1e-9))
            def stroke_timing(length):
                return StrokeTiming(length,effective_speed/1.1,
                                    .35*acceleration/(tangent_scale*1.1**2))
            def continuous_duration(length):
                return stroke_timing(length).duration/draw_scale
            distance=0.
            while distance < arc[-1]-1e-12:
                remaining=recycle_interval-(time_s-press_at)-1/rate-1e-7
                if remaining < .05:
                    st=stroke_curve(distance)
                    recycle(st);continue
                end=arc[-1]
                # If this would finish the stroke, leave room for the final
                # extrusion-on dwell before the same 15-second press deadline.
                final_duration=continuous_duration(end-distance)
                if final_duration<=remaining and final_duration+end_dwell>remaining:
                    remaining-=end_dwell
                    if remaining<.05:
                        st=stroke_curve(distance)
                        recycle(st);continue
                if continuous_duration(end-distance)>remaining:
                    low,high=distance,end
                    for _ in range(40):
                        mid=(low+high)/2
                        if continuous_duration(mid-distance)<=remaining:low=mid
                        else:high=mid
                    end=low
                if end-distance < 1e-10:
                    st=stroke_curve(distance)
                    recycle(st);continue
                duration=continuous_duration(end-distance)
                timing=stroke_timing(end-distance)
                count=max(2,math.ceil(duration*rate));start=time_s
                for k in range(1,count+1):
                    u=k/count
                    at=distance+timing.distance(duration*u*draw_scale)
                    st=curve[-1].copy() if at>=arc[-1]-1e-12 else stroke_curve(at)
                    time_s=start+duration*u;append(st,stand_off,'DRAW',True)
                distance=end
                if distance < arc[-1]-1e-12:recycle(st)
        else:
            for segment_index,(a, b) in enumerate(zip(points[:-1], points[1:])):
                a = a.copy()
                while True:
                    # Reserve one sample for the explicit release event. Splitting
                    # a quintic segment gives each pause a zero-velocity endpoint.
                    remaining = recycle_interval-(time_s-press_at)-1/rate-1e-7
                    duration = motion_timing(a,b,stand_off,stand_off,True)[0]
                    if segment_index==len(points)-2 and duration<=remaining and duration+end_dwell>remaining:
                        remaining-=end_dwell
                    if duration <= remaining:
                        move(a,b,stand_off,stand_off,'DRAW',True)
                        break
                    if remaining < .05:
                        recycle(a)
                        continue
                    low, high = 0., 1.
                    for _ in range(40):
                        fraction = (low+high)/2
                        middle = a+(b-a)*fraction
                        if motion_timing(a,middle,stand_off,stand_off,True)[0] <= remaining:
                            low = fraction
                        else:
                            high = fraction
                    if low < 1e-10:
                        recycle(a)
                        continue
                    middle = a+(b-a)*low
                    move(a,middle,stand_off,stand_off,'DRAW',True)
                    recycle(middle)
                    a = middle
        if end_dwell>0:
            dwell(points[-1],stand_off,end_dwell,'DRAW',True)
        if not air_run and time_s - press_at > max_press:
            raise ValueError(f'stroke {item["id"]} exceeds calibrated maximum continuous press duration')
        draw_end = len(samples)
        dwell(points[-1], stand_off, release_delay + 1/rate, 'RELEASE', False, event=True)
        # Full release delay elapses BEFORE the first lifting sample.
        move(points[-1], points[-1], stand_off, stand_off+lift, 'LIFT', False)
        preview_strokes.append({'id': item['id'], 'layer': 1, 'level': item.get('kind', 'outline'),
                                'kind': item.get('kind', 'outline'), 'closed': item.get('closed', False),
                                'points_m': [local_tips[k].tolist() for k in range(draw_start,draw_end) if samples[k]['phase']=='DRAW'],
                                'st_mm': (points*1000).tolist(), 'lift_mm': stand_off*1000,
                                'normals': model.normal(points[:,0],points[:,1]).tolist()})
    dwell(strokes[-1][1][-1], stand_off+lift, 0.1, 'HOLD', False)
    entry_spec = profile.get('routes', {}).get('cartesian_entry_pose')
    if entry_spec is not None:
        entry = transform(entry_spec, 'routes.cartesian_entry_pose')
        tip_entry = np.linalg.inv(world_surface) @ entry @ np.linalg.inv(tip_tcp)
        entry_only=profile['routes'].get('cartesian_entry_only',False)
        if type(entry_only) is not bool:
            raise ValueError('cartesian_entry_only must be boolean')
        if spec['type']=='sphere':
            entry_clearance=stand_off
            if np.linalg.norm(tip_entry[:3,3]-model.centre)-model.R < entry_clearance:
                raise ValueError('Saved sphere entry must be above the travel clearance')
        elif (tip_entry[2, 3] < stand_off+lift or
              float(-tip_entry[2, 2]) < math.cos(math.radians(2))):
            raise ValueError('Saved route entry must be tip-down above the travel clearance')

        def transition(a, b, start):
            a, b = np.asarray(a), np.asarray(b)
            distance = float(np.linalg.norm(b[:3]-a[:3]))
            rotations = Rotation.from_quat([a[3:], b[3:]])
            angle = float((rotations[0].inv()*rotations[1]).magnitude())
            duration = 1.1*max(1/rate, 1.875*distance/travel_speed,
                math.sqrt(5.78*distance/acceleration), 1.875*angle/angular_speed,
                math.sqrt(5.78*angle/angular_acceleration))/travel_scale
            count = max(1, math.ceil(duration*rate))
            duration = count/rate
            rotation = Slerp([0, 1], rotations)
            rows, tips, normals = [], [], []
            for i in range(count+1):
                u = i/count; blend = 10*u**3-15*u**4+6*u**5
                xyz = a[:3]+(b[:3]-a[:3])*blend
                q = rotation([blend]).as_quat()[0]
                tcp = np.eye(4); tcp[:3, :3] = Rotation.from_quat(q).as_matrix(); tcp[:3, 3] = xyz
                local = np.linalg.inv(world_surface) @ tcp @ np.linalg.inv(tip_tcp)
                rows.append({'time_s': round(start+i/rate, 9), 'pose': [*xyz.tolist(), *q.tolist()],
                             'phase': 'TRAVEL', 'extrude': False})
                tips.append(local[:3, 3]); normals.append(-local[:3, 2])
            return rows, tips, normals, duration

        entry_pose = [*entry[:3, 3], *Rotation.from_matrix(entry[:3, :3]).as_quat()]
        prefix, prefix_tips, prefix_normals, delay = transition(entry_pose, samples[0]['pose'], 0)
        if settings.get('approach_start_pose') is not None:
            approach_start=transform(settings['approach_start_pose'],'approach_start_pose')
            start_pose=[*approach_start[:3,3],*Rotation.from_matrix(approach_start[:3,:3]).as_quat()]
            connector,connector_tips,connector_normals,connector_time=transition(start_pose,entry_pose,0)
            prefix=connector[:-1]+[{**row,'time_s':round(row['time_s']+connector_time,9)} for row in prefix]
            prefix_tips=connector_tips[:-1]+prefix_tips
            prefix_normals=connector_normals[:-1]+prefix_normals
            delay+=connector_time
        prefix[0]['phase'] = 'HOLD'
        samples = prefix[:-1] + [{**row, 'time_s': round(row['time_s']+delay, 9)} for row in samples]
        events = [{'time_s': 0., 'extrude': False}] + [{**event, 'time_s': round(event['time_s']+delay, 9)} for event in events]
        local_tips = prefix_tips[:-1] + local_tips
        local_normals = prefix_normals[:-1] + local_normals
        transitions=[(prefix_tips,'route-entry',strokes[0][0]['id'])]
        park_spec=profile['routes'].get('cartesian_park_pose')
        if park_spec is not None:
            park=transform(park_spec,'routes.cartesian_park_pose')
            park_pose=[*park[:3,3],*Rotation.from_matrix(park[:3,:3]).as_quat()]
            normal=world_surface[:3,2]
            raised=np.array(samples[-1]['pose']);raised[:3]+=.05*normal
            above=np.array(park_pose);above[:3]+=normal*max(0.,float((raised[:3]-above[:3])@normal))
            exit_tips=[]
            for destination in [raised,above,park_pose]:
                segment,segment_tips,segment_normals,_=transition(samples[-1]['pose'],destination,samples[-1]['time_s'])
                samples+=segment[1:];local_tips+=segment_tips[1:];local_normals+=segment_normals[1:]
                exit_tips+=segment_tips
            transitions.append((exit_tips,strokes[-1][0]['id'],'route-exit'))
        elif not entry_only:
            suffix, suffix_tips, suffix_normals, _ = transition(samples[-1]['pose'], entry_pose, samples[-1]['time_s'])
            samples += suffix[1:]; local_tips += suffix_tips[1:]; local_normals += suffix_normals[1:]
            transitions.append((suffix_tips,strokes[-1][0]['id'],'route-exit'))
        for pts, before, after in transitions:
            preview_travels.append({'after': before, 'before': after,
                                   'points_m': [p.tolist() for p in pts], 'lift_mm': (stand_off+lift)*1000})
        if len(samples) > MAX_SAMPLES:
            raise ValueError('job including route entry exceeds maximum sample count')
    first_draw_index = next(i for i, row in enumerate(samples) if row['phase']=='DRAW')
    setup_end = samples[first_draw_index]['time_s']
    setup_saving = setup_end*(1-1/approach_scale)
    for row in samples:
        stamp=row['time_s']
        row['time_s']=round(stamp/approach_scale if stamp<setup_end else stamp-setup_saving,9)
    for event in events:
        stamp=event['time_s']
        event['time_s']=round(stamp/approach_scale if stamp<setup_end else stamp-setup_saving,9)
    pressed_at = None
    for event in events:
        if event['extrude']:
            if pressed_at is None:
                pressed_at = event['time_s']
        elif pressed_at is not None:
            if event['time_s'] - pressed_at > min(max_press,recycle_interval)+1e-8:
                raise ValueError('scheduled extrusion exceeds maximum continuous press duration')
            pressed_at = None
    tips = np.asarray(local_tips)
    minimum = math.inf
    for a, b in zip(tips[:-1], tips[1:]):
        gap = (minimum_segment_radius(a, b, model.centre) - model.R) if spec['type']=='sphere' else min(a[2],b[2])-model.origin[2]
        minimum = min(minimum, gap)
    if minimum < clearance - 1e-10:
        raise ValueError('sampled trajectory chord violates the surface clearance budget')
    # Conservative capsule aligned behind the nozzle. The measured radius must
    # enclose the complete pen/holder assembly for this check to be meaningful.
    body_minimum = math.inf
    for tip, normal in zip(local_tips, local_normals):
        a = tip + body_radius * normal
        b = tip + (body_length - body_radius) * normal
        body_gap = ((minimum_segment_radius(a, b, model.centre) - model.R - body_radius)
                    if spec['type'] == 'sphere' else min(a[2], b[2]) - model.origin[2] - body_radius)
        body_minimum = min(body_minimum, body_gap)
    if body_minimum < clearance - 1e-10:
        raise ValueError('tool envelope intersects the surface clearance margin')
    times = np.array([s['time_s'] for s in samples])
    positions = np.array([s['pose'][:3] for s in samples])
    dt = np.diff(times)
    velocity = np.diff(positions, axis=0) / dt[:, None]
    measured_speed = float(np.linalg.norm(velocity, axis=1).max())
    extruding = np.array([s['extrude'] for s in samples], dtype=bool)
    world_axes=np.asarray(local_normals)@world_surface[:3,:3].T
    draw_tilt=np.degrees(np.arccos(np.clip(world_axes[extruding,2],-1.,1.)))
    surface_normals=model.height_and_normal(tips[extruding])[1] if extruding.any() else np.empty((0,3))
    deviation=np.degrees(np.arccos(np.clip(np.sum(np.asarray(local_normals)[extruding]*surface_normals,axis=1),-1.,1.)))
    if len(deviation) and deviation.max()>25.+1e-6:
        raise ValueError('Pen orientation exceeds 25 degrees from the local surface normal')
    draw_segments = extruding[:-1] & extruding[1:]
    segment_speeds = np.linalg.norm(velocity, axis=1)
    draw_speed = float(segment_speeds[draw_segments].max()) if draw_segments.any() else 0.
    non_draw_speed = float(segment_speeds[~draw_segments].max()) if (~draw_segments).any() else 0.
    sampled_acceleration = np.diff(velocity, axis=0) / ((dt[1:] + dt[:-1]) / 2)[:, None]
    measured_acceleration = float(np.linalg.norm(sampled_acceleration, axis=1).max()) if len(sampled_acceleration) else 0.0
    rotations = Rotation.from_quat([s['pose'][3:] for s in samples])
    measured_angular_speed = float(((rotations[:-1].inv() * rotations[1:]).magnitude() / dt).max())
    setup_segments = np.arange(len(dt)) < first_draw_index
    segment_scale = np.where(setup_segments, approach_scale, 1.)
    segment_scale *= np.where(draw_segments, draw_scale, travel_scale)
    speed_limits = np.where(draw_segments, speed, travel_speed)*segment_scale
    acceleration_limits = acceleration*np.maximum(segment_scale[:-1],segment_scale[1:])**2
    angular_speeds = (rotations[:-1].inv()*rotations[1:]).magnitude()/dt
    if (np.any(segment_speeds > speed_limits*1.02) or
            np.any(np.linalg.norm(sampled_acceleration,axis=1)>acceleration_limits*1.15) or
            np.any(angular_speeds>angular_speed*segment_scale*1.02)):
        raise ValueError('sampled TCP velocity/acceleration exceeds motion limits; reduce speed or revise tool offset')
    job = {'schema_version': 1, 'version': 1, 'kind': 'pen_job', 'frame': 'world', 'units': 'm',
           'pose_format': 'x,y,z,qx,qy,qz,qw', 'air_run': air_run, 'air_offset_m': air_offset, 'fake_only': bool(profile.get('fake_only', True)),
           'calibrated': bool(profile.get('calibrated', False)), 'robot_sn': profile.get('robot_sn'),
           'profile_sha256': digest(profile), 'drawing_sha256': digest(drawing), 'settings_sha256': digest(settings),
           'surface': spec, 'qualified': profile.get('qualified', {}),
           'qualification': profile.get('qualification', {}), 'joint_routes': profile.get('routes', {}), 'duration_s': samples[-1]['time_s'],
           'sample_rate_hz': rate, 'samples': samples, 'pen_events': events,
           'motion_settings': {'draw_speed_m_s': speed*draw_scale, 'travel_speed_m_s': travel_speed*travel_scale,
                               'pen_orientation': 'world_vertical_with_clearance_tilt',
                               'maximum_surface_normal_angle_deg': 25.,
                               'approach_speed_scale': approach_scale*travel_scale,
                               'non_extruding_speed_scale':travel_scale,
                               'stand_off_m': stand_off},
           'pen': {'release_delay_s': release_delay, 'press_delay_s': press_delay,
                   'recycle_press_delay_s':recycle_press_delay,
                   'end_extrusion_dwell_s':end_dwell,
                   'max_press_s': max_press, 'recycle_interval_s': requested_recycle_interval,
                   'recycle_pause_s': recycle_pause},
           'validation': {'minimum_surface_clearance_m': minimum, 'required_surface_clearance_m': clearance,
                          'maximum_drawing_pen_tilt_deg':float(draw_tilt.max()) if len(draw_tilt) else 0.,
                          'maximum_drawing_surface_normal_angle_deg':float(deviation.max()) if len(deviation) else 0.,
                          'minimum_tool_surface_clearance_m': body_minimum,
                          'max_speed_m_s': measured_speed, 'max_acceleration_m_s2': measured_acceleration,
                          'max_draw_speed_m_s': draw_speed, 'max_travel_speed_m_s': non_draw_speed,
                          'max_angular_speed_rad_s': measured_angular_speed,
                          'pen_recycle_count': recycle_count,
                          'tool_envelope_checked': True, 'fixture_collision_checked': False,
                          'full_arm_collision_checked': False,
                          'physical_execution_qualified': bool(profile.get('qualified', {}).get('cartesian') and profile.get('qualified', {}).get('rt')),
                          'qualification_source': 'profile'},
           'profile': profile}
    e_s,e_t,n = model.frame(np.zeros(1),np.zeros(1))
    preview = {'kind':'tactile_toolpath','schema_version':1,'surface':model.describe(),'fit':fit,'layers':1,
               'strand_width_mm':1.0,'layer_height_mm':stand_off*2000,'strokes':preview_strokes,'travels':preview_travels,
               'draw_length_m':float(sum(np.linalg.norm(np.diff(np.asarray(s['points_m']),axis=0),axis=1).sum() for s in preview_strokes)),
               'lift_count':len(strokes),'legibility':{'warnings':[]},
               'marker_frame':{'point':model.point(0.,0.).tolist(),'normal':n[0].tolist(),'right':e_s[0].tolist(),'up':e_t[0].tolist()}}
    report = {'status':'PREVIEW_ONLY' if job['fake_only'] else 'COMPILED', 'sample_count':len(samples),
              'stroke_count':len(strokes),'duration_s':job['duration_s'],'fake_only':job['fake_only'],
              'calibrated':job['calibrated'],'validation':job['validation'],
              'notes':['Synthetic preview/fake profile is not a physical calibration'] if job['fake_only'] else []}
    return job, preview, report
