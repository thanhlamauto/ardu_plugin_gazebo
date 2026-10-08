import numpy as np

from mppi_ardupilot.monocular_ground_odometry import GroundVisualOdometry
from mppi_ardupilot.monocular_triangulation import CAMERA_OFFSET_FLU, OPTICAL_TO_FLU


def image_at(x):
    import cv2
    k = np.array([[381., 0, 319.5], [0, 381., 179.5], [0, 0, 1.]])
    image = np.full((360, 640, 3), 25, np.uint8)
    center = np.array([x, 0., 3.]) + CAMERA_OFFSET_FLU
    for xx in np.arange(7.5, 18., 1.0):
        for yy in np.arange(-5., 5.1, .7):
            optical = OPTICAL_TO_FLU.T @ (np.array([xx, yy, 0.]) - center)
            pixel = k @ optical
            u, v = np.rint(pixel[:2] / pixel[2]).astype(int)
            if 7 < u < 632 and 188 < v < 352:
                cv2.rectangle(image, (u - 2, v - 2), (u + 2, v + 2), (20, 190, 210), -1)
    return image, k


def test_ground_feature_motion_recovers_metric_translation():
    first, k = image_at(0.)
    estimator = GroundVisualOdometry(k, height_m=3.)
    pose, diag = estimator.observe(first, np.eye(3), 0.)
    assert pose is not None and diag['markers'] >= 8
    second, _ = image_at(.2)
    pose, diag = estimator.observe(second, np.eye(3), .1)
    assert pose is not None, diag
    assert abs(pose[0] - .2) < .07
    assert abs(pose[1]) < .07


def test_ground_motion_loss_does_not_publish_pose():
    first, k = image_at(0.)
    estimator = GroundVisualOdometry(k)
    assert estimator.observe(first, np.eye(3), 0.)[0] is not None
    blank = np.full_like(first, 25)
    assert estimator.observe(blank, np.eye(3), .1)[0] is None
    assert estimator.observe(first, np.eye(3), .2)[0] is None


def test_known_ground_squares_recover_height_scale():
    import cv2
    k = np.array([[381., 0, 319.5], [0, 381., 179.5], [0, 0, 1.]])
    height = 3.2
    image = np.full((360, 640, 3), 25, np.uint8)
    center = np.array([0., 0., height]) + CAMERA_OFFSET_FLU
    for xx in np.arange(6., 12., 1.5):
        for yy in np.arange(-4., 4., 1.5):
            ground = np.array([[xx + dx, yy + dy, 0.] for dx, dy in
                               ((-.21, -.21), (.21, -.21), (.21, .21), (-.21, .21))])
            optical = (ground - center) @ OPTICAL_TO_FLU
            pixels = optical @ k.T
            pixels = np.rint(pixels[:, :2] / pixels[:, 2, None]).astype(np.int32)
            cv2.fillConvexPoly(image, pixels, (20, 190, 210))
    estimator = GroundVisualOdometry(k, height_m=3., marker_size_m=.42)
    pose, diag = estimator.observe(image, np.eye(3), 0.)
    assert pose is not None, diag
    assert abs(pose[2] - height) < .25


def test_late_occlusion_cannot_change_calibrated_height():
    first, k = image_at(0.)
    estimator = GroundVisualOdometry(k, marker_size_m=.42)
    estimator._marker_height = lambda mask, rotation: 3.2
    for frame in range(20):
        pose, diag = estimator.observe(first, np.eye(3), frame * .1)
        assert pose is not None, diag
    estimator._marker_height = lambda mask, rotation: 3.8
    pose, diag = estimator.observe(first, np.eye(3), 2.1)
    assert pose is not None, diag
    assert abs(pose[2] - 3.2) < 1e-6


def test_impossible_visual_velocity_reversal_is_rejected():
    first, k = image_at(0.)
    estimator = GroundVisualOdometry(k)
    assert estimator.observe(first, np.eye(3), 0.)[0] is not None
    for frame, x in enumerate((.2, .4), 1):
        image, _ = image_at(x)
        pose, diag = estimator.observe(image, np.eye(3), frame * .1)
        assert pose is not None, diag
    reversed_image, _ = image_at(-.7)
    assert estimator.observe(reversed_image, np.eye(3), .3)[0] is None
