"""Metric monocular depth geometry; camera optical -> sensor_suite FLU."""
import numpy as np


def depth_to_points(depth, horizontal_fov=1.3962634, stride=8,
                    min_depth=0.2, max_depth=25.0):
    depth = np.asarray(depth)
    if depth.ndim != 2 or stride < 1 or not 0 < horizontal_fov < np.pi:
        raise ValueError('invalid depth image, stride or FOV')
    h, w = depth.shape
    f = w / (2 * np.tan(horizontal_fov / 2))
    v, u = np.mgrid[0:h:stride, 0:w:stride]
    z = depth[::stride, ::stride]
    valid = np.isfinite(z) & (z >= min_depth) & (z <= max_depth)
    # Gazebo camera looks +X, image right is -Y and down is -Z.
    # Camera translation relative to sensor_suite_link is +0.09 m along X.
    z, u, v = z[valid], u[valid], v[valid]
    points = np.stack((z + 0.09, -(u - (w-1)/2)*z/f,
                       -(v - (h-1)/2)*z/f), axis=-1)
    return np.asarray(points, dtype='<f4')


def depth_metrics(prediction, reference, min_depth=0.2, max_depth=25.):
    p, g = np.asarray(prediction), np.asarray(reference)
    mask = np.isfinite(p) & np.isfinite(g) & (p > 0) & (g >= min_depth) & (g < max_depth)
    if not mask.any():
        return {'valid_pixels': 0}
    p, g = p[mask], g[mask]
    return dict(valid_pixels=int(mask.sum()), abs_rel=float(np.mean(abs(p-g)/g)),
                rmse_m=float(np.sqrt(np.mean((p-g)**2))),
                delta1=float(np.mean(np.maximum(p/g, g/p) < 1.25)))
