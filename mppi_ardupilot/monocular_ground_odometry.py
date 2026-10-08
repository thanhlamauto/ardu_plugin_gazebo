"""RGB ground-feature translation with IMU attitude and a declared height prior.

Ground features are discovered in the image; their world coordinates are
created from the initial local frame and tracked, never read from an SDF.
The estimator intentionally returns no pose when evidence is insufficient.
"""
import numpy as np

from .monocular_triangulation import CAMERA_OFFSET_FLU, OPTICAL_TO_FLU


class GroundVisualOdometry:
    def __init__(self, intrinsics, height_m=3.0, min_tracks=8, marker_size_m=None):
        self.k = np.asarray(intrinsics, float)
        self.k_inv = np.linalg.inv(self.k)
        self.height_m = float(height_m)
        self.min_tracks = int(min_tracks)
        self.marker_size_m = marker_size_m
        if not np.isfinite(self.height_m) or self.height_m <= 0 or self.min_tracks < 4:
            raise ValueError('invalid height or track floor')
        self.xy = np.zeros(2)
        self.velocity = np.zeros(2)
        self.gray = None
        self.pixels = np.empty((0, 2), np.float32)
        self.ground = np.empty((0, 2), float)
        self.previous_time = None
        self.failed = False
        self.height_samples = []

    def _marker_height(self, mask, rotation):
        """Estimate camera height from image quadrilaterals of known square size."""
        import cv2
        if self.marker_size_m is None:
            return None
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        heights = []
        for contour in contours:
            if cv2.contourArea(contour) < 35:
                continue
            corners = cv2.approxPolyDP(contour, .04 * cv2.arcLength(contour, True), True)
            if len(corners) != 4:
                continue
            pixels = corners[:, 0].astype(float)
            radial = pixels - pixels.mean(axis=0)
            pixels += .25 * radial / np.maximum(np.linalg.norm(radial, axis=1, keepdims=True), 1)
            rays = self._rays(pixels, rotation)
            if np.any(rays[:, 2] >= -.08):
                continue
            unit_ground = -rays[:, :2] / rays[:, 2, None]
            side = np.linalg.norm(np.roll(unit_ground, -1, axis=0) - unit_ground, axis=1)
            heights.extend((self.marker_size_m / side[side > 1e-5]).tolist())
        if len(heights) < 12:
            return None
        camera_height = float(np.median(heights))
        body_height = camera_height - (rotation @ CAMERA_OFFSET_FLU)[2] + .006
        return body_height if 2.5 <= body_height <= 4.0 else None

    def _rays(self, pixels, rotation):
        optical = np.column_stack((pixels, np.ones(len(pixels)))) @ self.k_inv.T
        return optical @ (rotation @ OPTICAL_TO_FLU).T

    def _ground(self, pixels, rotation, xy):
        offset = rotation @ CAMERA_OFFSET_FLU
        center = np.array([xy[0] + offset[0], xy[1] + offset[1], self.height_m + offset[2]])
        rays = self._rays(pixels, rotation)
        valid = rays[:, 2] < -.08
        scale = np.zeros(len(rays))
        scale[valid] = -center[2] / rays[valid, 2]
        valid &= (scale > .3) & (scale < 30)
        return center[None, :2] + scale[:, None] * rays[:, :2], valid

    def _project_ground(self, ground, rotation, xy):
        offset = rotation @ CAMERA_OFFSET_FLU
        center = np.array([xy[0] + offset[0], xy[1] + offset[1], self.height_m + offset[2]])
        optical = (np.column_stack((ground, np.zeros(len(ground)))) - center) @ (
            rotation @ OPTICAL_TO_FLU)
        pixel = optical @ self.k.T
        return (pixel[:, :2] / np.maximum(pixel[:, 2, None], 1e-9)).astype(np.float32)

    def observe(self, rgb, rotation, time_s):
        if self.failed:
            return None, dict(reason='visual-odom-lost')
        import cv2
        cv2.setNumThreads(1)
        image = np.asarray(rgb)
        gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
        # Cyan runway markings are image evidence, never supplied as world XY.
        r, g, b = [image[:, :, i].astype(np.int16) for i in range(3)]
        mask = ((g > r + 25) & (b > r + 25) & (g > 65) & (b > 65)).astype(np.uint8) * 255
        mask[:image.shape[0] // 2] = 0
        rotation = np.asarray(rotation, float)
        if rotation.shape != (3, 3) or not np.isfinite(rotation).all():
            return None, dict(reason='invalid-imu-orientation')
        measured_height = self._marker_height(mask, rotation)
        if measured_height is not None and len(self.height_samples) < 20:
            self.height_samples.append(measured_height)
            self.height_m = float(np.median(self.height_samples))
        estimated = self.gray is None
        tracks = np.empty((0, 2), np.float32)
        world = np.empty((0, 2), float)
        residual = float('nan')
        if self.gray is not None and len(self.pixels):
            old = self.pixels.reshape(-1, 1, 2)
            dt = float(time_s - self.previous_time)
            if not 0 < dt <= .5:
                self.failed = True
                return None, dict(reason='invalid-visual-timestamp', delta_s=dt)
            predicted_xy = self.xy + self.velocity * dt
            initial = self._project_ground(self.ground, rotation, predicted_xy)
            nxt, status, _ = cv2.calcOpticalFlowPyrLK(self.gray, gray, old,
                                                       initial.reshape(-1, 1, 2),
                                                       winSize=(25, 25), maxLevel=3,
                                                       flags=cv2.OPTFLOW_USE_INITIAL_FLOW)
            if nxt is not None:
                back, reverse, _ = cv2.calcOpticalFlowPyrLK(gray, self.gray, nxt, None,
                                                              winSize=(25, 25), maxLevel=3)
                if back is not None:
                    good = status[:, 0].astype(bool) & reverse[:, 0].astype(bool)
                    good &= np.linalg.norm(back[:, 0] - old[:, 0], axis=1) < .8
                    tracks = nxt[good, 0]
                    world = self.ground[good]
                    ix = np.clip(np.rint(tracks[:, 0]).astype(int), 0, gray.shape[1] - 1)
                    iy = np.clip(np.rint(tracks[:, 1]).astype(int), 0, gray.shape[0] - 1)
                    visible = mask[iy, ix] > 0
                    tracks, world = tracks[visible], world[visible]
            if len(tracks) >= self.min_tracks:
                rays = self._rays(tracks, rotation)
                offset = rotation @ CAMERA_OFFSET_FLU
                height = self.height_m + offset[2]
                valid = rays[:, 2] < -.08
                candidates = world[valid] + height * rays[valid, :2] / rays[valid, 2, None] - offset[:2]
                if len(candidates) >= self.min_tracks:
                    candidate = np.median(candidates, axis=0)
                    errors = np.linalg.norm(candidates - candidate, axis=1)
                    inlier = errors < max(.35, 2.5 * np.median(errors))
                    if inlier.sum() >= self.min_tracks:
                        candidate = np.median(candidates[inlier], axis=0)
                        residual = float(np.median(errors[inlier]))
                        if np.linalg.norm(candidate - self.xy) <= 2.0 and residual <= .5:
                            velocity = (candidate - self.xy) / dt
                            if np.linalg.norm(velocity - self.velocity) <= 2.0 + 8.0 * dt:
                                self.xy = candidate
                                self.velocity = velocity
                                estimated = True
        # Replenish in the current image, but only publish after a valid update.
        corners = cv2.goodFeaturesToTrack(gray, maxCorners=600, qualityLevel=.004,
                                          minDistance=5, mask=mask)
        pixels = corners[:, 0] if corners is not None else np.empty((0, 2), np.float32)
        points, valid = self._ground(pixels, rotation, self.xy)
        self.gray = gray
        self.pixels = pixels[valid].astype(np.float32)
        self.ground = points[valid]
        if self.previous_time is None and len(self.pixels) < self.min_tracks:
            self.gray = None
            return None, dict(reason='insufficient-ground-markers', markers=len(self.pixels))
        if not estimated:
            self.failed = True
            return None, dict(reason='insufficient-ground-motion', tracks=len(tracks),
                              markers=len(self.pixels), residual_m=residual)
        self.previous_time = float(time_s)
        return np.array([self.xy[0], self.xy[1], self.height_m]), dict(
            reason='visual-ground-odom', tracks=len(tracks), markers=len(self.pixels),
            residual_m=residual, height_from_marker_m=measured_height,
            height_calibration_samples=len(self.height_samples))
