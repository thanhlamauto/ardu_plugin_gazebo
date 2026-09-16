"""Swept clearance against observed cloud only, not an unseen-space certificate."""
import numpy as np

def validate_trajectory(trajectory, points, radius):
    path = np.asarray(trajectory, dtype=float)
    cloud = np.asarray(points, dtype=float).reshape(-1, 3)
    if (path.ndim != 2 or path.shape[1] != 3 or len(path) < 2
            or not np.isfinite(path).all() or not np.isfinite(cloud).all()
            or not np.isfinite(radius) or radius <= 0):
        return {'valid': False, 'reason': 'nonfinite_or_invalid_geometry', 'min_clearance_m': None}
    if not len(cloud):
        return {'valid': False, 'reason': 'empty_observed_cloud', 'min_clearance_m': None}
    minimum = float('inf')
    for a, b in zip(path[:-1], path[1:]):
        ab = b-a
        t = np.clip((cloud-a) @ ab / max(float(ab@ab), 1e-12), 0, 1)
        minimum = min(minimum, float(np.linalg.norm(cloud-a-t[:, None]*ab, axis=1).min()))
    return {'valid': minimum > radius, 'reason': 'clear_observed_cloud' if minimum > radius else 'swept_collision',
            'min_clearance_m': minimum, 'radius_m': float(radius)}
