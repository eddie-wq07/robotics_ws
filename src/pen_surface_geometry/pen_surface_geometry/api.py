"""Validated surface factories and geometric clearance primitives."""
from copy import deepcopy
import math
import numpy as np
from .surface import load_surface


def surface_spec(value='sphere'):
    spec = deepcopy(value) if isinstance(value, dict) else {'type': value}
    kind = spec.get('type')
    if kind == 'sphere':
        radius = float(spec.get('radius_m', 0.04))
        if not math.isfinite(radius) or abs(radius - 0.04) > 1e-9:
            raise ValueError('this installation requires an 80 mm diameter sphere (radius_m=0.040)')
        defaults = {'type': 'sphere', 'radius_m': radius, 'raise_m': -radius,
                    'position_m': [0, 0], 'marker': {'azimuth_deg': 0, 'elevation_deg': 90}}
    elif kind == 'plane':
        defaults = {'type': 'plane', 'size_m': [0.08, 0.10, 0.0025], 'raise_m': -0.0025,
                    'position_m': [0, 0], 'marker': {'rotation_deg': 0}}
    elif kind == 'cylinder':
        radius = float(spec.get('radius_m', 0.029))
        height = float(spec.get('height_m', 0.03))
        if not all(math.isfinite(v) and v > 0 for v in (radius, height)):
            raise ValueError('cylinder radius_m and height_m must be positive and finite')
        if spec.get('lying', True) is not True:
            raise ValueError('the pen pipeline supports horizontal cylinders only')
        defaults = {'type': 'cylinder', 'radius_m': radius, 'height_m': height,
                    'lying': True, 'axis_azimuth_deg': 0, 'raise_m': -radius,
                    'position_m': [0, 0], 'marker': {'along_m': 0, 'azimuth_deg': 0}}
    else:
        raise ValueError('surface must be plane, sphere or cylinder')
    result = defaults | spec
    if kind == 'plane':
        size = np.asarray(result['size_m'], float)
        if size.shape != (3,) or not np.isfinite(size).all() or (size <= 0).any():
            raise ValueError('plane size_m requires three positive finite dimensions')
        result['raise_m'] = spec.get('raise_m', -float(size[2]))
    position = np.asarray(result['position_m'], float)
    if position.shape != (2,) or not np.isfinite(position).all() or not math.isfinite(float(result['raise_m'])):
        raise ValueError('invalid surface placement')
    return result


def minimum_surface_segment_clearance(model, a, b):
    """Exact chord clearance to the plane, sphere or infinite cylinder."""
    if model.kind == 'plane':
        return min(a[2], b[2]) - model.origin[2]
    if model.kind == 'cylinder':
        a, b = np.asarray(a) - model.centre, np.asarray(b) - model.centre
        a = a - (a @ model.axis) * model.axis
        b = b - (b @ model.axis) * model.axis
        return minimum_segment_radius(a, b, np.zeros(3)) - model.R
    return minimum_segment_radius(a, b, model.centre) - model.R


def make_surface(value='sphere'):
    return load_surface(surface_spec(value))


def minimum_segment_radius(a, b, centre):
    """Exact distance from sphere centre to a straight interpolation chord."""
    a, b, centre = np.asarray(a, float), np.asarray(b, float), np.asarray(centre, float)
    delta = b - a
    squared = float(delta @ delta)
    fraction = 0.0 if squared == 0 else float(np.clip((centre - a) @ delta / squared, 0, 1))
    return float(np.linalg.norm(a + fraction * delta - centre))
