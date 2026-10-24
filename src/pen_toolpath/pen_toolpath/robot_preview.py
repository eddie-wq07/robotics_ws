"""Offline URDF joint preview. Never imports ROS, connects hardware or commands it.

Uses the original pen preview's seeded, bounded Jacobian IK approach with the
current description and CAD tip. It establishes kinematics, not collision safety.
"""
import math
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


def transform(value):
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_quat(value['quaternion_xyzw']).as_matrix()
    matrix[:3, 3] = value['xyz_m']
    return matrix


def sample_pose(sample):
    return transform({'xyz_m': sample['pose'][:3], 'quaternion_xyzw': sample['pose'][3:]})


class Arm:
    def __init__(self, context):
        by_child = {j['child']: j for j in context['joints']}
        chain, child = [], context['flange_link']
        while child in by_child:
            joint = by_child[child]
            chain.append(joint)
            child = joint['parent']
            if len(chain) > len(by_child):
                raise ValueError('Robot description contains a joint cycle')
        self.chain = []
        self.names, lower, upper, velocity = [], [], [], []
        for joint in reversed(chain):
            origin = np.eye(4)
            origin[:3, :3] = Rotation.from_euler('xyz', joint['rpy']).as_matrix()
            origin[:3, 3] = joint['xyz']
            index, axis = None, None
            if joint['type'] != 'fixed':
                if joint['type'] != 'revolute':
                    raise ValueError('Preview supports bounded revolute arm joints')
                index = len(self.names)
                self.names.append(joint['name'])
                limit = joint['limit']
                lower.append(limit['lower']); upper.append(limit['upper']); velocity.append(limit['velocity'])
                axis = np.asarray(joint['axis'], float)
                axis /= np.linalg.norm(axis)
            self.chain.append((origin, axis, index))
        if len(self.names) != 7:
            raise ValueError('Robot preview requires seven arm joints')
        self.lower, self.upper, self.velocity = map(np.asarray, (lower, upper, velocity))
        self.offset = transform(context['flange_to_visual_tip'])
        self.base = transform(context['base_correction']) if context.get('base_correction') else np.eye(4)
        self.position_tolerance = context.get('ik_position_tolerance_m', 2e-5)
        self.angle_tolerance = context.get('ik_angle_tolerance_rad', 2e-4)
        self.posture_reference = (np.asarray(context['posture_reference'], float)
                                  if context.get('posture_reference') is not None else None)

    def posture_metrics(self, q):
        jac = self.forward(q, True)[1] * np.array([1.,1.,1.,.15,.15,.15])[:,None]
        values = np.linalg.svd(jac, compute_uv=False)
        return float(values[-1]), float(values[0]/max(values[-1], 1e-12))

    def require_safe_posture(self, q):
        minimum, condition = self.posture_metrics(q)
        if minimum < .015 or condition > 60.:
            raise ValueError(f'Unfavourable joint posture: condition={condition:.1f}, '
                             f'minimum scaled singular value={minimum:.5f}; revise the approach or placement')
        return minimum, condition

    def forward_many(self, positions, jacobian=False):
        positions=np.asarray(positions,float)
        if positions.ndim!=2 or positions.shape[1]!=7 or not np.isfinite(positions).all():
            raise ValueError('Batch kinematics needs finite seven-joint positions')
        current=np.broadcast_to(self.base,(len(positions),4,4)).copy()
        axes,origins=[],[]
        for origin,axis,index in self.chain:
            current=current@origin
            if index is not None:
                axes.append(current[:,:3,:3]@axis)
                origins.append(current[:,:3,3].copy())
                rotation=np.broadcast_to(np.eye(4),current.shape).copy()
                rotation[:,:3,:3]=Rotation.from_rotvec(positions[:,index,None]*axis).as_matrix()
                current=current@rotation
        tip=current@self.offset
        if not jacobian:return tip
        axes=np.stack(axes,axis=1);origins=np.stack(origins,axis=1)
        linear=np.cross(axes,tip[:,None,:3,3]-origins)
        return tip,np.concatenate([linear,axes],axis=2).transpose(0,2,1)

    def require_safe_postures(self, positions):
        minimum=float('inf');condition=0.
        for start in range(0,len(positions),4096):
            batch=np.asarray(positions[start:start+4096])
            jac=self.forward_many(batch,True)[1]*np.array([1.,1.,1.,.15,.15,.15])[None,:,None]
            values=np.linalg.svd(jac,compute_uv=False)
            ratios=values[:,0]/np.maximum(values[:,-1],1e-12)
            bad=np.flatnonzero((values[:,-1]<.015)|(ratios>60.))
            if len(bad):self.require_safe_posture(batch[bad[0]])
            minimum=min(minimum,float(values[:,-1].min()))
            condition=max(condition,float(ratios.max()))
        return minimum,condition

    def forward(self, q, jacobian=False):
        current, axes, origins = self.base.copy(), [], []
        for origin, axis, index in self.chain:
            current = current @ origin
            if index is not None:
                axes.append(current[:3, :3] @ axis)
                origins.append(current[:3, 3].copy())
                rotation = np.eye(4)
                rotation[:3, :3] = Rotation.from_rotvec(axis * q[index]).as_matrix()
                current = current @ rotation
        tip = current @ self.offset
        if not jacobian:
            return tip
        jac = np.column_stack([np.r_[np.cross(a, tip[:3, 3]-p), a] for a, p in zip(axes, origins)])
        return tip, jac

    @staticmethod
    def error(actual, desired):
        return np.r_[desired[:3, 3]-actual[:3, 3],
                     Rotation.from_matrix(desired[:3, :3] @ actual[:3, :3].T).as_rotvec()]

    def solve(self, desired, seed):
        q = seed.copy()
        weights = np.array([1, 1, 1, .15, .15, .15])
        for _ in range(60):
            actual, jac = self.forward(q, True)
            error = self.error(actual, desired)
            if np.linalg.norm(error[:3]) < self.position_tolerance and np.linalg.norm(error[3:]) < self.angle_tolerance:
                if self.posture_reference is not None:
                    self.require_safe_posture(q)
                return q
            jac = weights[:, None] * jac
            inverse = jac.T @ np.linalg.solve(jac @ jac.T + 1e-8*np.eye(6), np.eye(6))
            step = inverse @ (weights*error)
            if self.posture_reference is not None:
                # Keep the redundant motion near the initial arm posture while
                # the task-space step continues to meet the requested TCP pose.
                step += (np.eye(7)-inverse@jac) @ (.02*(self.posture_reference-q))
            step *= min(1, .15 / max(np.abs(step).max(), 1e-12))
            q = np.clip(q+step, self.lower+1e-7, self.upper-1e-7)
        def residual(value):
            return np.r_[weights*self.error(self.forward(value), desired), 1e-5*(value-seed)]
        solved = least_squares(residual, q, bounds=(self.lower+1e-7, self.upper-1e-7),
                               max_nfev=150, ftol=1e-9, xtol=1e-9, gtol=1e-9).x
        error = self.error(self.forward(solved), desired)
        if np.linalg.norm(error[:3]) > 10*self.position_tolerance or np.linalg.norm(error[3:]) > 10*self.angle_tolerance:
            raise ValueError('Robot preview cannot reach a requested pen pose within joint limits')
        if self.posture_reference is not None:
            self.require_safe_posture(solved)
        return solved


def build_preview(job, context, progress=None):
    if progress:
        progress('approach')
    if job['profile'].get('robot_kinematics'):
        context={**context,'ik_position_tolerance_m':1e-7,'ik_angle_tolerance_rad':1e-6}
    if context.get('joint_execution'):
        context={**context,'posture_reference':[context['initial_positions'][name]
                 for name in Arm(context).names]}
    correction=job['profile'].get('preview_kinematic_correction')
    if correction:
        context={**context,'base_correction':correction}
    arm = Arm(context)
    seed = np.array([context['initial_positions'][name] for name in arm.names], float)
    if not np.isfinite(seed).all() or np.any(seed < arm.lower) or np.any(seed > arm.upper):
        raise ValueError('Robot preview initial joint state is invalid')
    q = seed.copy()
    initial = arm.forward(q)
    tcp_to_tip = np.linalg.inv(transform(job['profile']['T_flange_rdk_tcp'])) @ transform(job['profile']['T_flange_pen_tip'])
    target_poses = [sample_pose(sample) @ tcp_to_tip for sample in job['samples']]
    normal = transform(job['profile']['T_world_surface'])[:3, 2]
    travel_scale=job.get('motion_settings',{}).get('non_extruding_speed_scale',1.)
    first, last = target_poses[0], target_poses[-1]
    frames, elapsed, max_error, max_angle = [], 0., 0., 0.

    def emit(desired, dt, phase, job_time=None, extrude=False):
        nonlocal q, elapsed, max_error, max_angle
        try:
            solved = arm.solve(desired, q)
        except ValueError as error:
            raise ValueError(f'{phase} at drawing time {job_time}: {error}') from error
        delta = np.abs(solved-q)
        if delta.max() > .25:
            raise ValueError('Robot preview rejected a discontinuous joint solution')
        # Slow the preview if a joint needs longer than the Cartesian schedule.
        elapsed += max(dt, float(np.max(delta / (.5*arm.velocity))))
        q = solved
        actual = arm.forward(q)
        error = arm.error(actual, desired)
        max_error = max(max_error, float(np.linalg.norm(error[:3])))
        max_angle = max(max_angle, float(np.linalg.norm(error[3:])))
        frames.append({'time_s': elapsed, 'positions': q.tolist(), 'phase': phase,
                       'job_time_s': job_time, 'extrude': bool(extrude),
                       'tip_m': actual[:3, 3].tolist()})

    frames.append({'time_s': 0., 'positions': seed.tolist(), 'phase': 'APPROACH',
                   'job_time_s': None, 'extrude': False, 'tip_m': initial[:3, 3].tolist()})

    def joint_route(route, phase, speed_scale=1.):
        nonlocal q, elapsed
        start = np.asarray(route['start_positions'], float)
        if start.shape != (7,) or not np.isfinite(start).all() or np.max(np.abs(q-start)) > .03:
            raise ValueError('Robot preview is outside the saved joint route start')
        previous = 0.
        for row in route['points']:
            point = np.asarray(row['positions'], float)
            stamp = float(row['time_s'])
            if (point.shape != (7,) or not np.isfinite(point).all() or
                    np.any(point < arm.lower) or np.any(point > arm.upper) or
                    not math.isfinite(stamp) or stamp <= previous):
                raise ValueError('Invalid saved joint route in robot preview')
            if np.any(np.abs(point-q)/((stamp-previous)/speed_scale)>arm.velocity*1.001):
                raise ValueError('Scaled approach/return route exceeds a robot joint velocity limit')
            q = point
            elapsed += (stamp-previous)/speed_scale
            previous = stamp
            frames.append({'time_s': elapsed, 'positions': q.tolist(), 'phase': phase,
                           'job_time_s': None, 'extrude': False,
                           'tip_m': arm.forward(q)[:3, 3].tolist()})

    def move(destination, phase):
        start = arm.forward(q)
        distance = float(np.linalg.norm(destination[:3, 3]-start[:3, 3]))
        turn = Rotation.from_matrix(start[:3, :3].T @ destination[:3, :3]).as_rotvec()
        duration = max(.08, 1.875*distance/.04, math.sqrt(5.78*distance/.15),
                       1.875*np.linalg.norm(turn)/.4)/travel_scale
        count = max(2, math.ceil(duration*25))
        for i in range(1, count+1):
            u = i/count; blend = 10*u**3-15*u**4+6*u**5
            pose = np.eye(4)
            pose[:3, 3] = start[:3, 3]+blend*(destination[:3, 3]-start[:3, 3])
            pose[:3, :3] = start[:3, :3] @ Rotation.from_rotvec(turn*blend).as_matrix()
            emit(pose, duration/count, phase)

    routes = job.get('joint_routes', {})
    approach_scale=job.get('motion_settings',{}).get('approach_speed_scale',1.)
    if routes.get('approach'):
        start = np.asarray(routes['approach']['start_positions'], float)
        end = np.asarray(routes['approach']['points'][-1]['positions'],float)
        if np.max(np.abs(q-end)) > .03:
            if np.max(np.abs(q-start)) > .03:
                connector = routes.get('home_connector')
                at_connector = connector and np.max(np.abs(q-np.asarray(connector['start_positions']))) <= .03
                if (job.get('fake_only') and context.get('allow_simulation_joint_reset') and
                        not at_connector):
                    # A fake run may finish on another IK branch. Return its joints
                    # smoothly to the saved approach start; never use this for hardware.
                    origin = q.copy()
                    duration = max(2., float(np.max(1.875*np.abs(start-origin)/(.5*arm.velocity))))
                    count = math.ceil(duration*25)
                    points = []
                    for i in range(1, count+1):
                        u = i/count
                        blend = 10*u**3-15*u**4+6*u**5
                        points.append({'time_s': duration*u,
                                       'positions': (origin+blend*(start-origin)).tolist()})
                    joint_route({'start_positions': origin.tolist(), 'points': points}, 'APPROACH')
                else:
                    if not connector or not connector.get('checked_joint_home'):
                        raise ValueError('A checked Home connector is required for this starting pose')
                    if not np.allclose(connector['points'][-1]['positions'], start, atol=1e-6, rtol=0):
                        raise ValueError('Home connector does not end at the saved approach start')
                    joint_route(connector, 'APPROACH',approach_scale)
            joint_route(routes['approach'], 'APPROACH',approach_scale)
        if arm.posture_reference is not None:
            arm.posture_reference=q.copy()
        if job['profile'].get('robot_kinematics'):
            # A raised joint-controller check can start above the saved hover.
            # Include that connecting motion instead of a one-sample jump.
            move(first, 'APPROACH')
    else:
        height = max(float(initial[:3, 3] @ normal), float(first[:3, 3] @ normal)+.05)
        raised = initial.copy(); raised[:3, 3] += normal*(height-float(initial[:3, 3] @ normal))
        above_start = first.copy(); above_start[:3, 3] += normal*(height-float(first[:3, 3] @ normal))
        move(raised, 'APPROACH'); move(above_start, 'APPROACH'); move(first, 'APPROACH')
    previous_time = 0.
    total = len(job['samples'])
    if progress:
        progress('trajectory', 0, total)
    for index, (sample, pose) in enumerate(zip(job['samples'], target_poses)):
        dt = sample['time_s']-previous_time
        emit(pose, max(dt, .000001), sample['phase'], sample['time_s'], sample['extrude'])
        previous_time = sample['time_s']
        if progress and (index % 128 == 0 or index+1 == total):
            progress('trajectory', index+1, total)
    if progress:
        progress('return')
    if routes.get('park'):
        joint_route(routes['park'], 'RETURN',travel_scale)
    elif not routes.get('cartesian_park_pose'):
        retract = last.copy(); retract[:3, 3] += .05*normal
        move(retract, 'RETRACT')
        return_above = initial.copy()
        return_above[:3, 3] += normal*max(0., float(retract[:3, 3] @ normal)-float(initial[:3, 3] @ normal))
        move(return_above, 'RETURN'); move(initial, 'RETURN')
    return {'schema_version': 1, 'kind': 'pen_robot_preview', 'simulation_only': True,
            'joint_names': arm.names, 'duration_s': elapsed, 'frames': frames,
            'initial_source': context.get('initial_source'), 'model_revision': context.get('model_revision'),
            'max_position_error_m': max_error, 'max_orientation_error_rad': max_angle,
            'joint_limits_checked': True, 'collision_checked': False}
