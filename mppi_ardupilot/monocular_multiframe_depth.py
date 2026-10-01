"""Persistent monocular landmarks with inverse-depth uncertainty.

Uncertainty is conditional on supplied metric camera poses and a configured
pose-error floor. It is not a calibrated safety covariance. No unseen ray is
declared free here; mapping and planning are deliberately separate.
"""
from dataclasses import dataclass, field

import numpy as np

from .monocular_triangulation import CAMERA_OFFSET_FLU, OPTICAL_TO_FLU


@dataclass
class Observation:
    pixel: np.ndarray
    center: np.ndarray
    ray: np.ndarray
    body_rotation: np.ndarray


@dataclass
class LandmarkEstimate:
    feature_id: int
    point_enu: np.ndarray
    inverse_depth_1_m: float
    inverse_depth_sigma_1_m: float
    depth_sigma_m: float
    observations: int
    baseline_m: float
    reprojection_rmse_px: float


@dataclass
class _Track:
    feature_id: int
    pixel: np.ndarray
    observations: list[Observation] = field(default_factory=list)
    estimate: LandmarkEstimate | None = None


def camera_observation(pixel, body_position, body_rotation, intrinsics):
    """Turn one image measurement and metric body pose into a world ray."""
    k = np.asarray(intrinsics, float)
    p = np.asarray(body_position, float).reshape(3)
    r = np.asarray(body_rotation, float).reshape(3, 3)
    uv = np.asarray(pixel, float).reshape(2)
    center = p + r @ CAMERA_OFFSET_FLU
    ray = r @ OPTICAL_TO_FLU @ np.linalg.solve(k, np.r_[uv, 1.])
    ray /= np.linalg.norm(ray)
    return Observation(uv.copy(), center, ray, r.copy())


def estimate_inverse_depth(observations, intrinsics, *, feature_id=0,
                           pixel_sigma=.7, pose_sigma_m=.15,
                           min_parallax_deg=1., max_reprojection_px=1.5,
                           max_relative_sigma=.25, max_depth_sigma_m=.75,
                           min_range=.3, max_range=20.):
    """Fit a world point to 3+ rays; return None until depth is observable.

    The covariance uses image noise plus a conservative pose-error floor.
    Correlated frames must be thinned before calling this function.
    """
    if len(observations) < 3:
        return None
    centers = np.array([o.center for o in observations], float)
    rays = np.array([o.ray for o in observations], float)
    baseline = float(max(np.linalg.norm(centers-centers[0], axis=1)))
    if baseline <= 0 or not np.isfinite(centers).all() or not np.isfinite(rays).all():
        return None
    cosine = np.clip(rays @ rays[0], -1., 1.)
    parallax = float(np.degrees(np.arccos(cosine.min())))
    if parallax < min_parallax_deg:
        return None
    projectors = np.eye(3)[None, :, :] - rays[:, :, None]*rays[:, None, :]
    normal = projectors.sum(axis=0)
    if np.linalg.cond(normal) > 1e7:
        return None
    point = np.linalg.solve(normal, np.einsum('nij,nj->i', projectors, centers))
    distances = np.linalg.norm(point-centers, axis=1)
    along = np.einsum('ni,ni->n', point-centers, rays)
    if np.any(along <= 0) or np.any(distances < min_range) or np.any(distances > max_range):
        return None
    k = np.asarray(intrinsics, float)
    errors = []
    for obs in observations:
        optical = OPTICAL_TO_FLU.T @ obs.body_rotation.T @ (point-obs.center)
        if optical[2] <= 0:
            return None
        projected = (k @ optical)[:2] / optical[2]
        errors.append(float(np.linalg.norm(projected-obs.pixel)))
    if max(errors) > max_reprojection_px:
        return None
    depth = float(along[0])
    focal = float((k[0, 0]+k[1, 1])/2)
    sigma_angle = pixel_sigma/focal
    geometric_cov = (sigma_angle * max(distances))**2 * np.linalg.inv(normal)
    geometric_sigma = float(np.sqrt(max(0., rays[0] @ geometric_cov @ rays[0])))
    depth_sigma = float(np.hypot(geometric_sigma, pose_sigma_m*depth/baseline))
    if depth_sigma/depth > max_relative_sigma or depth_sigma > max_depth_sigma_m:
        return None
    return LandmarkEstimate(int(feature_id), point.astype('<f4'), 1./depth,
                            depth_sigma/depth**2, depth_sigma, len(observations),
                            baseline, float(np.sqrt(np.mean(np.square(errors)))))

class MultiFrameLandmarkTracker:
    def __init__(self, intrinsics, *, max_features=500, min_observations=3,
                 min_pose_step_m=.12, max_observations=16,
                 pixel_sigma=.7, pose_sigma_m=.15,
                 max_relative_sigma=.25, max_depth_sigma_m=.75):
        self.k = np.asarray(intrinsics, float)
        if (self.k.shape != (3, 3) or not np.isfinite(self.k).all() or
                self.k[0,0] <= 0 or self.k[1,1] <= 0 or
                max_features < 1 or min_observations < 3 or
                max_observations < min_observations or min_pose_step_m <= 0 or
                pixel_sigma <= 0 or pose_sigma_m < 0 or
                not 0 < max_relative_sigma < 1 or max_depth_sigma_m <= 0):
            raise ValueError('invalid multi-frame tracker configuration')
        self.max_features = int(max_features)
        self.min_observations = int(min_observations)
        self.min_pose_step_m = float(min_pose_step_m)
        self.max_observations = int(max_observations)
        self.pixel_sigma = float(pixel_sigma)
        self.pose_sigma_m = float(pose_sigma_m)
        self.max_relative_sigma = float(max_relative_sigma)
        self.max_depth_sigma_m = float(max_depth_sigma_m)
        self.tracks: dict[int, _Track] = {}
        self.next_id = 0
        self.previous_gray = None

    def observe(self, rgb, body_position, body_rotation):
        import cv2
        cv2.setNumThreads(1)
        gray = cv2.cvtColor(np.asarray(rgb), cv2.COLOR_RGB2GRAY)
        p = np.asarray(body_position, float)
        r = np.asarray(body_rotation, float)
        if self.previous_gray is not None and gray.shape != self.previous_gray.shape:
            self.tracks.clear()
            self.previous_gray = None
        matched = 0
        new_geometry = 0
        if self.previous_gray is not None and self.tracks:
            ids = list(self.tracks)
            old_pixels = np.array([self.tracks[i].pixel for i in ids], np.float32).reshape(-1, 1, 2)
            next_pixels, status, _ = cv2.calcOpticalFlowPyrLK(
                self.previous_gray, gray, old_pixels, None, winSize=(21, 21), maxLevel=3)
            if next_pixels is not None:
                back, reverse, _ = cv2.calcOpticalFlowPyrLK(
                    gray, self.previous_gray, next_pixels, None, winSize=(21, 21), maxLevel=3)
                good = (status[:, 0].astype(bool) & reverse[:, 0].astype(bool) &
                        (np.linalg.norm(back[:, 0]-old_pixels[:, 0], axis=1) < .5)) if back is not None else np.zeros(len(ids), bool)
                for index, feature_id in enumerate(ids):
                    if not good[index]:
                        del self.tracks[feature_id]
                        continue
                    track = self.tracks[feature_id]
                    track.pixel = next_pixels[index, 0].astype(float)
                    matched += 1
                    obs = camera_observation(track.pixel, p, r, self.k)
                    if np.linalg.norm(obs.center-track.observations[-1].center) >= self.min_pose_step_m:
                        track.observations.append(obs)
                        if len(track.observations) > self.max_observations:
                            # Retain the anchor; a sliding window can erase the
                            # parallax earned before the vehicle comes to rest.
                            track.observations.pop(1)
                        new_geometry += 1
                        if len(track.observations) >= self.min_observations:
                            track.estimate = estimate_inverse_depth(
                                track.observations, self.k, feature_id=feature_id,
                                pixel_sigma=self.pixel_sigma, pose_sigma_m=self.pose_sigma_m,
                                max_relative_sigma=self.max_relative_sigma,
                                max_depth_sigma_m=self.max_depth_sigma_m)
            else:
                self.tracks.clear()
        mask = np.full(gray.shape, 255, np.uint8)
        for track in self.tracks.values():
            cv2.circle(mask, tuple(np.rint(track.pixel).astype(int)), 6, 0, -1)
        capacity = self.max_features-len(self.tracks)
        if capacity > 0:
            corners = cv2.goodFeaturesToTrack(gray, maxCorners=capacity,
                                               qualityLevel=.004, minDistance=7, mask=mask)
            if corners is not None:
                for pixel in corners[:, 0]:
                    obs = camera_observation(pixel, p, r, self.k)
                    self.tracks[self.next_id] = _Track(self.next_id, pixel.astype(float), [obs])
                    self.next_id += 1
        self.previous_gray = gray
        landmarks = [track.estimate for track in self.tracks.values() if track.estimate is not None]
        diagnostics = dict(active_tracks=len(self.tracks), matched_tracks=matched,
                           geometry_observations_added=new_geometry,
                           confident_landmarks=len(landmarks),
                           median_depth_sigma_m=float(np.median([e.depth_sigma_m for e in landmarks])) if landmarks else None)
        return landmarks, diagnostics
