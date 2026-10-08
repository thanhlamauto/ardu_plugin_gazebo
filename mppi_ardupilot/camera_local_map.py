"""Short-lived occupied voxel evidence from monocular RGB depth and camera poses.

No SDF, range-sensor or ground-truth obstacle input. Unknown space is not
represented as free; this occupied-only adapter alone cannot certify coverage.
"""
import numpy as np


class CameraLocalMap:
    def __init__(self, voxel_size=.2, lifetime=3., min_hits=2, altitude=3., half_band=.75,
                 fusion_radius=0., publish_age=None):
        if voxel_size<=0 or lifetime<=0 or min_hits<1 or half_band<=0 or fusion_radius<0:
            raise ValueError('invalid local map configuration')
        if publish_age is not None and not 0<publish_age<=lifetime:
            raise ValueError('publish_age must be in (0, lifetime]')
        self.voxel_size=voxel_size;self.lifetime=lifetime;self.min_hits=min_hits
        self.altitude=altitude;self.half_band=half_band;self.fusion_radius=fusion_radius
        self.publish_age=publish_age if publish_age is not None else lifetime
        self.voxels={};self.last_stamp=None

    def update(self, sensor_points, position, rotation, stamp):
        points=np.asarray(sensor_points).reshape(-1,3)
        # sensor_suite mount + camera offset already included by backprojection.
        world=(points+np.array([.08,0,.16]))@np.asarray(rotation).T+position
        return self.update_world(world,stamp)

    def update_world(self, world_points, stamp):
        if self.last_stamp is not None and stamp<=self.last_stamp:
            if stamp<self.last_stamp:self.voxels.clear()
            else:return self.points(stamp)
        self.last_stamp=stamp
        world=np.asarray(world_points).reshape(-1,3)
        world=world[np.isfinite(world).all(axis=1) & (abs(world[:,2]-self.altitude)<=self.half_band)]
        self.voxels={k:v for k,v in self.voxels.items() if stamp-v[0]<=self.lifetime}
        if len(world):
            keys=np.unique(np.floor(world/self.voxel_size).astype(np.int64),axis=0)
            observed=set()
            for cell in map(tuple,keys):
                key=cell
                if self.fusion_radius and cell not in self.voxels:
                    center=(np.asarray(cell)+.5)*self.voxel_size
                    candidates=[(float(np.linalg.norm(center-(np.asarray(old)+.5)*self.voxel_size)),old)
                                for old in self.voxels if old not in observed]
                    if candidates:
                        distance,nearest=min(candidates)
                        if distance<=self.fusion_radius:key=nearest
                if key in observed:continue  # One hit per frame, even for duplicate features.
                previous=self.voxels.get(key)
                count=previous[1]+1 if previous else 1
                self.voxels[key]=(stamp,count)
                observed.add(key)
        return self.points(stamp)

    def points(self, stamp):
        keys=[k for k,(seen,hits) in self.voxels.items()
              if stamp-seen<=self.publish_age and hits>=self.min_hits]
        return np.asarray([(np.array(k)+.5)*self.voxel_size for k in keys],dtype='<f4').reshape(-1,3)
