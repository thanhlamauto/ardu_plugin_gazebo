"""Geometric clearance along a constant-direction stopping segment.

This checks observed points only; it is not a guarantee about unobserved space
or about the vehicle's actual braking capability.
"""
import numpy as np


def stopping_segment_clearance(position, velocity, points, acceleration, delay):
    position = np.asarray(position, dtype=float)
    velocity = np.asarray(velocity, dtype=float)
    points = np.asarray(points, dtype=float).reshape(-1, 3)
    if acceleration <= 0 or delay < 0:
        raise ValueError('positive braking acceleration and nonnegative delay required')
    speed = float(np.linalg.norm(velocity))
    stop = speed*delay + speed*speed/(2*acceleration)
    if not len(points):
        return float('inf'), None, stop
    offsets = points-position
    direction = velocity/speed if speed > 1e-9 else np.zeros(3)
    along = np.clip(offsets @ direction, 0, stop)
    distances = np.linalg.norm(offsets-along[:, None]*direction, axis=1)
    index = int(np.argmin(distances))
    return float(distances[index]), points[index], stop


def validate_stopping_states(states, points, geometry, acceleration, delay,
                             base_clearance, uncertainty_m=0.0,
                             sample_spacing_m=0.1):
    """Check the straight stopping segment at every predicted state.

    Cloud clearance is exact for point samples. Known SDF solids are sampled
    along each segment with a conservative half-spacing distance bound.
    This is a model-relative guard, not a certified real-vehicle brake.
    """
    states = np.asarray(states, dtype=float)
    cloud = np.asarray(points, dtype=float).reshape(-1, 3)
    required = base_clearance + uncertainty_m
    if (states.ndim != 2 or states.shape[1] < 6 or not len(states)
            or not np.isfinite(states).all() or not np.isfinite(cloud).all()
            or not np.isfinite([acceleration, delay, base_clearance,
                                uncertainty_m, sample_spacing_m]).all()
            or acceleration <= 0 or delay < 0 or base_clearance < 0
            or uncertainty_m < 0 or sample_spacing_m <= 0):
        return {'valid': False, 'reason': 'invalid_stopping_geometry',
                'min_clearance_m': None, 'required_clearance_m': required}
    if not len(cloud) and geometry is None:
        return {'valid': False, 'reason': 'unknown_stopping_geometry',
                'min_clearance_m': None, 'required_clearance_m': required}
    minimum = float('inf')
    failing_step = None
    for index, state in enumerate(states):
        p, v = state[:3], state[3:6]
        cloud_clearance, _, length = stopping_segment_clearance(
            p, v, cloud, acceleration, delay)
        clearance = cloud_clearance
        if geometry is not None:
            count = max(1, int(np.ceil(length / sample_spacing_m)))
            speed = float(np.linalg.norm(v))
            endpoint = p + v / speed * length if speed > 1e-9 else p
            samples = p + np.linspace(0, 1, count + 1)[:, None] * (endpoint-p)
            map_clearance = float(np.min(geometry.clearance(samples))) - length / (2*count)
            clearance = min(clearance, map_clearance)
        minimum = min(minimum, clearance)
        if clearance <= required and failing_step is None:
            failing_step = index
    return {'valid': failing_step is None,
            'reason': 'clear_stopping_segments' if failing_step is None else 'stopping_clearance',
            'min_clearance_m': minimum, 'required_clearance_m': required,
            'first_failing_step': failing_step, 'checked_states': len(states),
            'map_checked': geometry is not None}
