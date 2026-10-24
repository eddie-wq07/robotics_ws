"""Strict profile loading for model rendering and application preflight."""
from copy import deepcopy
import math
from pathlib import Path
import re
import yaml

TRANSFORMS = ('T_world_surface', 'T_world_base', 'T_base_jig', 'T_flange_holder',
              'T_holder_pen', 'T_flange_pen_tip', 'T_flange_rdk_tcp')


def _numbers(value, count, name):
    if (not isinstance(value, (list, tuple)) or len(value) != count or
            any(isinstance(x, bool) or not isinstance(x, (int, float)) or not math.isfinite(x) for x in value)):
        raise ValueError(f'{name}: TBD or invalid; supply {count} finite measured values')
    return [float(x) for x in value]


def transform(profile, name):
    value = profile.get(name)
    if not isinstance(value, dict):
        raise ValueError(f'{name}: TBD transform')
    xyz = _numbers(value.get('xyz_m'), 3, name + '.xyz_m')
    q = _numbers(value.get('quaternion_xyzw'), 4, name + '.quaternion_xyzw')
    norm2 = sum(v*v for v in q)
    if abs(norm2 - 1.0) > 0.002:
        raise ValueError(f'{name}: quaternion must be unit length')
    return xyz, [v / math.sqrt(norm2) for v in q]


def quaternion_rpy(q):
    x, y, z, w = q
    return [math.atan2(2*(w*x + y*z), 1-2*(x*x+y*y)),
            math.asin(max(-1.0, min(1.0, 2*(w*y-z*x)))),
            math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))]


def multiply_quaternion(a, b):
    x, y, z, w = a
    X, Y, Z, W = b
    return [w*X+x*W+y*Z-z*Y, w*Y-x*Z+y*W+z*X,
            w*Z+x*Y-y*X+z*W, w*W-x*X-y*Y-z*Z]


def rotate(q, v):
    return multiply_quaternion(multiply_quaternion(q, [*v, 0]), [-q[0], -q[1], -q[2], q[3]])[:3]


def compose(a, b):
    av, aq = a
    bv, bq = b
    rotated = rotate(aq, bv)
    return [av[i]+rotated[i] for i in range(3)], multiply_quaternion(aq, bq)


def relative(parent_world, child_world):
    p, q = parent_world
    inverse_q = [-q[0], -q[1], -q[2], q[3]]
    return compose((rotate(inverse_q, [-v for v in p]), inverse_q), child_world)


def load_profile(path, *, mode='preview', robot_sn=None):
    if mode not in ('preview', 'fake', 'real'):
        raise ValueError('mode must be preview, fake, or real')
    profile = yaml.safe_load(Path(path).read_text())
    if not isinstance(profile, dict) or type(profile.get('schema_version')) is not int or profile.get('schema_version') != 1:
        raise ValueError('Calibration schema_version must be 1')
    serial = profile.get('robot_sn')
    if not isinstance(serial, str) or re.fullmatch(r'Rizon[A-Za-z0-9]*-\d{6}', serial) is None:
        raise ValueError('robot_sn must be an explicit Rizon serial, for example Rizon4s-000000')
    for flag in ('calibrated', 'fake_only', 'synthetic'):
        if type(profile.get(flag)) is not bool:
            raise ValueError(f'{flag} must be explicitly true or false')
    if profile.get('world_frame') != 'world':
        raise ValueError('world_frame must match the configured controller: world')
    for name in TRANSFORMS:
        transform(profile, name)
    camera = profile.get('camera', {})
    if not isinstance(camera, dict):
        raise ValueError('camera must be a mapping with an explicit enabled flag')
    if 'enabled' in camera and type(camera['enabled']) is not bool:
        raise ValueError('camera.enabled must be explicitly true or false')
    if camera.get('enabled') is True:
        if camera.get('model') != 'D405':
            raise ValueError('The optional camera model is D405')
        transform(camera, 'T_flange_camera')
    if robot_sn is not None and profile.get('robot_sn') != robot_sn:
        raise ValueError('Calibration robot_sn does not match requested robot')
    surface = profile.get('surface', {})
    if not isinstance(surface, dict) or surface.get('type') not in ('sphere', 'plane'):
        raise ValueError('surface.type must be sphere or plane')
    if surface['type'] == 'sphere':
        radius = surface.get('radius_m')
        if (isinstance(radius, bool) or not isinstance(radius, (int, float)) or
                not math.isfinite(radius) or abs(radius - 0.04) > 1e-9):
            raise ValueError('This cell uses an 80 mm diameter sphere: radius_m must be finite and 0.040')
    elif mode == 'real' or 'size_m' in surface:
        size = _numbers(surface.get('size_m'), 3, 'surface.size_m')
        if any(abs(a-b) > 1e-8 for a, b in zip(size, [0.08, 0.10, 0.0025])):
            raise ValueError('surface.size_m differs from the canonical flat STL: 0.08 x 0.10 x 0.0025 m')
    if mode == 'fake' and not (profile.get('fake_only') is True and profile.get('synthetic') is True):
        raise ValueError('Fake mode requires an explicitly synthetic fake-only profile')
    if mode == 'real':
        if profile.get('calibrated') is not True or profile.get('fake_only') is not False or profile.get('synthetic') is not False:
            raise ValueError('Real mode requires calibrated, non-synthetic, non-fake profile')
        # The controller's "world" target values pass directly to the RDK, whose
        # Cartesian reference is the robot base. No runtime TF conversion exists.
        base_xyz, base_q = transform(profile, 'T_world_base')
        if (any(abs(v) > 1e-9 for v in base_xyz) or
                any(abs(v) > 1e-9 for v in base_q[:3]) or abs(abs(base_q[3]) - 1) > 1e-9):
            raise ValueError('Real control requires T_world_base identity: world must equal the RDK base frame')
        if not isinstance(profile.get('rdk_tool_name'), str) or not profile['rdk_tool_name'] or profile['rdk_tool_name'] == 'TBD':
            raise ValueError('rdk_tool_name: TBD')
        payload = profile.get('payload', {})
        _numbers(payload.get('com_m'), 3, 'payload.com_m')
        _numbers(payload.get('inertia_kg_m2'), 6, 'payload.inertia_kg_m2')
        for section, fields in [('payload', ['mass_kg']), ('pen', ['press_delay_s', 'max_press_s']),
                                ('tool_envelope', ['radius_m', 'length_m'])]:
            for field in fields:
                value = profile.get(section, {}).get(field)
                if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value) or value <= 0:
                    raise ValueError(f'{section}.{field}: TBD or non-positive measured value')
        release_delay = profile.get('pen', {}).get('release_delay_s')
        if (isinstance(release_delay, bool) or not isinstance(release_delay, (float, int)) or
                not math.isfinite(release_delay) or release_delay < 0):
            raise ValueError('pen.release_delay_s: supply a finite nonnegative delay')
    servo = profile.get('visual_servo', {})
    if not isinstance(servo, dict) or type(servo.get('enabled', False)) is not bool:
        raise ValueError('visual_servo.enabled must be explicitly true or false')
    if servo.get('enabled'):
        if mode == 'real' or not (profile.get('fake_only') and profile.get('synthetic')):
            raise ValueError('Generic servo is preview/fake-only')
        transform(servo, 'T_holder_servo')
    return deepcopy(profile)


def xacro_mappings(profile):
    result = {}
    for name in TRANSFORMS:
        xyz, q = transform(profile, name)
        result[name + '_xyz'] = ' '.join(map(str, xyz))
        result[name + '_rpy'] = ' '.join(map(str, quaternion_rpy(q)))
    world_jig = compose(transform(profile, 'T_world_base'), transform(profile, 'T_base_jig'))
    xyz, q = relative(world_jig, transform(profile, 'T_world_surface'))
    result['T_jig_surface_xyz'] = ' '.join(map(str, xyz))
    result['T_jig_surface_rpy'] = ' '.join(map(str, quaternion_rpy(q)))
    result['surface_type'] = profile['surface']['type']
    result['profile_id'] = str(profile.get('profile_id', 'unnamed'))
    result['profile_calibrated'] = str(profile.get('calibrated') is True).lower()
    result['profile_fake_only'] = str(profile.get('fake_only') is True).lower()
    result['camera_enabled'] = str(profile.get('camera', {}).get('enabled') is True).lower()
    result['visual_servo_enabled'] = str(profile.get('visual_servo', {}).get('enabled') is True).lower()
    if profile.get('visual_servo', {}).get('enabled') is True:
        xyz, q = transform(profile['visual_servo'], 'T_holder_servo')
        result['visual_servo_xyz'] = ' '.join(map(str, xyz))
        result['visual_servo_rpy'] = ' '.join(map(str, quaternion_rpy(q)))
    if profile.get('camera', {}).get('enabled') is True:
        xyz, q = transform(profile['camera'], 'T_flange_camera')
        result['camera_xyz'] = ' '.join(map(str, xyz))
        result['camera_rpy'] = ' '.join(map(str, quaternion_rpy(q)))
    return result
