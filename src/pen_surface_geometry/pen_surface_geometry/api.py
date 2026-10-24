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
    else:
        raise ValueError('surface must be plane or sphere')
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


def make_surface(value='sphere'):
    return load_surface(surface_spec(value))


def minimum_segment_radius(a, b, centre):
    """Exact distance from sphere centre to a straight interpolation chord."""
    a, b, centre = np.asarray(a, float), np.asarray(b, float), np.asarray(centre, float)
    delta = b - a
    squared = float(delta @ delta)
    fraction = 0.0 if squared == 0 else float(np.clip((centre - a) @ delta / squared, 0, 1))
    return float(np.linalg.norm(a + fraction * delta - centre))
