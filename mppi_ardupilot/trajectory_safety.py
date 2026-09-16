"""One trajectory-safety predicate shared by MPPI selection and final gate."""
import math
import numpy as np


def _known_segment_clearance(torch, starts, ends, geometry, spacing):
    lengths = torch.linalg.vector_norm(ends-starts, dim=-1)
    counts = torch.ceil(lengths/spacing).clamp_min(1).to(torch.long)
    maximum = max(1, int(counts.max().item()))
    index = torch.arange(maximum+1, dtype=starts.dtype, device=starts.device)
    fractions = (index.view(*([1]*counts.ndim), -1)
                 / counts.unsqueeze(-1)).clamp_max(1.0)
    samples = starts.unsqueeze(-2) + fractions.unsqueeze(-1)*(ends-starts).unsqueeze(-2)
    clearance = geometry.torch_clearance(samples).min(dim=-1).values
    return clearance - lengths/(2*counts.to(starts.dtype))


def _cloud_segment_clearance(torch, starts, ends, cloud):
    ab = ends-starts
    cloud_view = cloud.view(*([1]*(starts.dim()-1)), -1, 3)
    offsets = cloud_view-starts.unsqueeze(-2)
    denom = (ab*ab).sum(-1).clamp_min(1e-12).unsqueeze(-1)
    projection = (offsets*ab.unsqueeze(-2)).sum(-1)
    fraction = projection.div(denom).clamp(0, 1)
    distance2 = (offsets.square().sum(-1)-2*fraction*projection
                 + fraction.square()*denom).min(-1).values
    return distance2.clamp_min(0).sqrt()


def _relevant_cloud(torch, starts, ends, cloud, radius):
    """Discard points outside the union AABB; they cannot violate a segment."""
    lower=torch.minimum(starts.amin(dim=tuple(range(starts.ndim-1))),
                        ends.amin(dim=tuple(range(ends.ndim-1))))-radius
    upper=torch.maximum(starts.amax(dim=tuple(range(starts.ndim-1))),
                        ends.amax(dim=tuple(range(ends.ndim-1))))+radius
    return cloud[((cloud>=lower)&(cloud<=upper)).all(-1)]


def evaluate_trajectory_safety_batch(states, points, geometry, *,
                                     collision_radius, acceleration, delay,
                                     stopping_clearance, uncertainty=0.0,
                                     sample_spacing=0.1, chunk_size=256,
                                     torch_module=None, detailed=False,
                                     cloud_map_tolerance=0.1):
    """Evaluate swept collision and straight-stop feasibility for K trajectories.

    ``states`` is ``[K,T,NX]`` and must include the measured initial state at
    index zero. The same function is used for sampled trajectories and the
    final nominal, preventing two definitions of safe.
    """
    if torch_module is None:
        import torch as torch_module
    torch = torch_module
    if not torch.is_tensor(states):
        states = torch.as_tensor(states, dtype=torch.double)
    if states.ndim == 2:
        states = states.unsqueeze(0)
    if states.ndim != 3 or states.shape[-1] < 6 or states.shape[1] < 2:
        raise ValueError('states must have shape [K,T,NX], T>=2, NX>=6')
    values = [collision_radius, acceleration, delay, stopping_clearance,
              uncertainty, sample_spacing, cloud_map_tolerance]
    if (not np.isfinite(values).all() or collision_radius <= 0 or acceleration <= 0
            or delay < 0 or stopping_clearance < 0 or uncertainty < 0
            or sample_spacing <= 0 or cloud_map_tolerance < 0):
        raise ValueError('invalid trajectory safety parameters')
    cloud_np = np.asarray(points, dtype=float).reshape(-1, 3)
    cloud_present = bool(len(cloud_np))
    map_cloud_expansion = 0.0
    if geometry is not None and cloud_present:
        residual = np.asarray(geometry.clearance(cloud_np), dtype=float)
        explained = residual <= cloud_map_tolerance
        if np.any(explained):
            map_cloud_expansion = float(np.max(residual[explained]))
        cloud_np = cloud_np[~explained]
    cloud = torch.as_tensor(cloud_np,
                            dtype=states.dtype, device=states.device)
    finite = torch.isfinite(states).all(dim=(-1, -2))
    outputs = {name: [] for name in ('cloud_min', 'map_min', 'stop_min',
                                      'stop_cloud_min',
                                      'map_safe', 'stop_map_safe')}
    for begin in range(0, len(states), chunk_size):
        batch = states[begin:begin+chunk_size]
        p, v = batch[..., :3], batch[..., 3:6]
        starts, ends = p[:, :-1], p[:, 1:]
        collision_cloud = (cloud if detailed and len(states)==1 else
            _relevant_cloud(torch, starts, ends, cloud, collision_radius)) if len(cloud) else cloud
        if len(collision_cloud):
            cloud_collision = _cloud_segment_clearance(
                torch, starts, ends, collision_cloud).min(-1).values
        else:
            cloud_collision = torch.full((len(batch),),
                                         torch.inf if cloud_present else -torch.inf,
                                         dtype=states.dtype, device=states.device)
        if geometry is None:
            map_collision = torch.full_like(cloud_collision, torch.inf)
            map_safe = torch.ones_like(cloud_collision, dtype=torch.bool)
        else:
            segment_safe = geometry.torch_segments_safe(
                starts, ends, collision_radius+map_cloud_expansion)
            map_safe = segment_safe.all(-1)
            if detailed and len(states) == 1:
                map_collision = _known_segment_clearance(
                    torch, starts, ends, geometry, sample_spacing).min(-1).values
            else:
                map_collision = torch.where(
                    map_safe, torch.full_like(cloud_collision, torch.inf),
                    torch.zeros_like(cloud_collision))

        speed = torch.linalg.vector_norm(v, dim=-1)
        stop_length = speed*delay + speed.square()/(2*acceleration)
        direction = v/speed.clamp_min(1e-9).unsqueeze(-1)
        stop_ends = p+direction*stop_length.unsqueeze(-1)
        stop_cloud = (cloud if detailed and len(states)==1 else
            _relevant_cloud(torch, p, stop_ends, cloud,
                            stopping_clearance+uncertainty)) if len(cloud) else cloud
        if len(stop_cloud):
            cloud_stop = _cloud_segment_clearance(torch, p, stop_ends, stop_cloud)
        else:
            cloud_stop = torch.full_like(speed, torch.inf)
        if geometry is None:
            map_stop = torch.full_like(speed, torch.inf)
            map_stop_safe = torch.ones_like(speed, dtype=torch.bool)
        else:
            map_stop_safe = geometry.torch_segments_safe(
                p, stop_ends, stopping_clearance+uncertainty+map_cloud_expansion)
            if detailed and len(states) == 1:
                map_stop = _known_segment_clearance(
                    torch, p, stop_ends, geometry, sample_spacing)
            else:
                map_stop = torch.where(
                    map_stop_safe, torch.full_like(speed, torch.inf),
                    torch.zeros_like(speed))
        stop_min = torch.minimum(cloud_stop, map_stop).min(-1).values
        outputs['cloud_min'].append(cloud_collision)
        outputs['map_min'].append(map_collision)
        outputs['stop_min'].append(stop_min)
        outputs['stop_cloud_min'].append(cloud_stop.min(-1).values)
        outputs['map_safe'].append(map_safe)
        outputs['stop_map_safe'].append(map_stop_safe.all(-1))
    result = {key: torch.cat(value) for key, value in outputs.items()}
    required_stop = stopping_clearance+uncertainty
    result['cloud_safe'] = result['cloud_min'] > collision_radius
    if not cloud_present:
        result['cloud_safe'] &= False
    result['stopping_cloud_safe'] = result.pop('stop_cloud_min') > required_stop
    result['stopping_safe'] = result['stopping_cloud_safe'] & result.pop('stop_map_safe')
    result['safe'] = (finite & result['cloud_safe'] & result['map_safe']
                      & result['stopping_safe'])
    result['required_stop_clearance'] = required_stop
    result['cloud_points_not_covered_by_map'] = int(len(cloud_np))
    result['map_expansion_for_cloud_m'] = map_cloud_expansion
    return result


def single_safety_diagnostics(result):
    """Convert the K=1 tensor result into JSON-safe final-gate diagnostics."""
    if len(result['safe']) != 1:
        raise ValueError('single diagnostics requires exactly one trajectory')
    cloud_ok = bool(result['cloud_safe'][0].item())
    map_ok = bool(result['map_safe'][0].item())
    stop_ok = bool(result['stopping_safe'][0].item())
    reason = ('clear_trajectory' if cloud_ok and map_ok and stop_ok else
              'swept_collision' if not cloud_ok else
              'known_map_collision' if not map_ok else
              'predicted_stopping_clearance')
    def finite(value):
        value=float(value)
        return value if math.isfinite(value) else None
    return {
        'valid': bool(result['safe'][0].item()), 'reason': reason,
        'min_clearance_m': finite(result['cloud_min'][0].item()),
        'known_map': {'valid': map_ok, 'reason': 'known_sdf_geometry',
                      'min_clearance_lower_bound_m': finite(result['map_min'][0].item()),
                      'cloud_expansion_m': float(result['map_expansion_for_cloud_m'])},
        'stopping': {'valid': stop_ok,
                     'reason': 'clear_stopping_segments' if stop_ok else 'stopping_clearance',
                     'min_clearance_m': finite(result['stop_min'][0].item()),
                     'required_clearance_m': float(result['required_stop_clearance'])},
        'cloud_points_not_covered_by_map': result['cloud_points_not_covered_by_map'],
    }
