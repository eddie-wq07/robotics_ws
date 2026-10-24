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

    def forward(self, q, jacobian=False):
        current, axes, origins = np.eye(4), [], []
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
            if np.linalg.norm(error[:3]) < 2e-5 and np.linalg.norm(error[3:]) < 2e-4:
                return q
            jac = weights[:, None] * jac
            step = jac.T @ np.linalg.solve(jac @ jac.T + 1e-8*np.eye(6), weights*error)
            step *= min(1, .15 / max(np.abs(step).max(), 1e-12))
            q = np.clip(q+step, self.lower+1e-7, self.upper-1e-7)
        def residual(value):
            return np.r_[weights*self.error(self.forward(value), desired), 1e-5*(value-seed)]
        solved = least_squares(residual, q, bounds=(self.lower+1e-7, self.upper-1e-7),
                               max_nfev=150, ftol=1e-9, xtol=1e-9, gtol=1e-9).x
        error = self.error(self.forward(solved), desired)
        if np.linalg.norm(error[:3]) > .0002 or np.linalg.norm(error[3:]) > .002:
            raise ValueError('Robot preview cannot reach a requested pen pose within joint limits')
        return solved


def build_preview(job, context):
    arm = Arm(context)
    seed = np.array([context['initial_positions'][name] for name in arm.names], float)
    if not np.isfinite(seed).all() or np.any(seed < arm.lower) or np.any(seed > arm.upper):
        raise ValueError('Robot preview initial joint state is invalid')
    q = seed.copy()
    initial = arm.forward(q)
    tcp_to_tip = np.linalg.inv(transform(job['profile']['T_flange_rdk_tcp'])) @ transform(job['profile']['T_flange_pen_tip'])
    target_poses = [sample_pose(sample) @ tcp_to_tip for sample in job['samples']]
    normal = transform(job['profile']['T_world_surface'])[:3, 2]
    first, last = target_poses[0], target_poses[-1]
    frames, elapsed, max_error, max_angle = [], 0., 0., 0.

    def emit(desired, dt, phase, job_time=None, extrude=False):
        nonlocal q, elapsed, max_error, max_angle
        solved = arm.solve(desired, q)
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

    def move(destination, phase):
        start = arm.forward(q)
        distance = float(np.linalg.norm(destination[:3, 3]-start[:3, 3]))
        turn = Rotation.from_matrix(start[:3, :3].T @ destination[:3, :3]).as_rotvec()
        duration = max(.08, 1.875*distance/.04, math.sqrt(5.78*distance/.15),
                       1.875*np.linalg.norm(turn)/.4)
        count = max(2, math.ceil(duration*25))
        for i in range(1, count+1):
            u = i/count; blend = 10*u**3-15*u**4+6*u**5
            pose = np.eye(4)
            pose[:3, 3] = start[:3, 3]+blend*(destination[:3, 3]-start[:3, 3])
            pose[:3, :3] = start[:3, :3] @ Rotation.from_rotvec(turn*blend).as_matrix()
            emit(pose, duration/count, phase)

    height = max(float(initial[:3, 3] @ normal), float(first[:3, 3] @ normal)+.05)
    raised = initial.copy(); raised[:3, 3] += normal*(height-float(initial[:3, 3] @ normal))
    above_start = first.copy(); above_start[:3, 3] += normal*(height-float(first[:3, 3] @ normal))
    move(raised, 'APPROACH'); move(above_start, 'APPROACH'); move(first, 'APPROACH')
    previous_time = 0.
    for sample, pose in zip(job['samples'], target_poses):
        dt = sample['time_s']-previous_time
        emit(pose, max(dt, .000001), sample['phase'], sample['time_s'], sample['extrude'])
        previous_time = sample['time_s']
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
