#!/usr/bin/env python3
"""Check the simulated camera/IMU geometry and measured timestamp alignment."""
import argparse
import csv
import json
from pathlib import Path
import re
import sys
import xml.etree.ElementTree as ET

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from mppi_ardupilot.monocular_triangulation import CAMERA_OFFSET_FLU


def pose(element):
    text = element.findtext('pose')
    return np.array([float(x) for x in text.split()]) if text else np.zeros(6)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trial', type=Path)
    args = parser.parse_args()
    trial = args.trial
    model = ET.parse(ROOT/'models/iris_with_monocular_camera/model.sdf').getroot().find('model')
    link = model.find("link[@name='sensor_suite_link']")
    camera = link.find("sensor[@name='rgb_camera']")
    imu = link.find("sensor[@name='imu_viz']")
    link_pose, camera_pose, imu_pose = pose(link), pose(camera), pose(imu)
    cam_base = link_pose[:3]+camera_pose[:3]
    cam_imu = camera_pose[:3]-imu_pose[:3]
    width = int(camera.findtext('camera/image/width'))
    height = int(camera.findtext('camera/image/height'))
    hfov = float(camera.findtext('camera/horizontal_fov'))
    focal = width/(2*np.tan(hfov/2))
    log = (trial/'openvins.log').read_text()
    marker = re.search(r'CALIB width=(\d+) height=(\d+) hfov=([\d.]+) fx=([\d.]+) '
                       r'imu_to_camera_flu=([\d.,-]+) camera_to_imu_rotation_flu_optical=([\d.,-]+)', log)
    with (trial/'openvins.csv').open(newline='') as handle:
        rows = list(csv.DictReader(handle))
    offsets = [abs(float(row['imu_alignment_ms'])) for row in rows
               if row.get('imu_alignment_ms')]
    gyro_noises=imu.findall('imu/angular_velocity/*/noise')
    accel_noises=imu.findall('imu/linear_acceleration/*/noise')
    rate=float(imu.findtext('update_rate'))
    profile=re.search(r'IMU_PROFILE gaussian_sdf_100hz sigma_w=([\deE+.-]+) '
                      r'sigma_a=([\deE+.-]+) sigma_wb=([\deE+.-]+) '
                      r'sigma_ab=([\deE+.-]+)',log)
    noise_matches=(len(gyro_noises)==len(accel_noises)==3 and rate==100 and
                   all(n.get('type')=='gaussian' and abs(float(n.findtext('stddev'))-.0016968)<1e-8
                       for n in gyro_noises) and
                   all(n.get('type')=='gaussian' and abs(float(n.findtext('stddev'))-.02)<1e-8
                       for n in accel_noises) and profile is not None and
                   abs(float(profile[1])-.0016968/np.sqrt(rate))<1e-8 and
                   abs(float(profile[2])-.02/np.sqrt(rate))<1e-8)
    checks = dict(
        sensor_frames_match=camera.findtext('gz_frame_id')==imu.findtext('gz_frame_id')=='sensor_suite_link',
        imu_orientation_reference_enu=imu.findtext('imu/orientation_reference_frame/localization')=='ENU',
        gaussian_imu_noise_matches_bridge=bool(noise_matches),
        gaussian_imu_profile_logged='IMU_PROFILE gaussian_sdf_100hz' in log,
        mount_rotations_zero=bool(np.allclose(np.r_[link_pose[3:],camera_pose[3:],imu_pose[3:]],0)),
        mapping_offset_matches_sdf=bool(np.allclose(CAMERA_OFFSET_FLU,cam_base,atol=1e-6)),
        bridge_calibration_logged=marker is not None,
        image_imu_alignment_p95_under_10ms=bool(offsets and np.percentile(offsets,95)<=10),
        image_imu_alignment_max_under_20ms=bool(offsets and max(offsets)<=20),
    )
    if marker:
        bridge_cam_imu=np.array([float(x) for x in marker[5].split(',')])
        bridge_rotation=np.array([float(x) for x in marker[6].split(',')])
        checks['bridge_intrinsics_match_sdf']=bool(int(marker[1])==width and int(marker[2])==height and
            abs(float(marker[3])-hfov)<1e-6 and abs(float(marker[4])-focal)<.01)
        checks['bridge_translation_matches_sdf']=bool(np.allclose(bridge_cam_imu,cam_imu,atol=1e-6))
        checks['bridge_rotation_matches_optical_convention']=bool(np.allclose(
            bridge_rotation,[0,0,1,-1,0,0,0,-1,0],atol=1e-6))
    report=dict(checks=checks,passed=all(checks.values()),width=width,height=height,hfov_rad=hfov,
        focal_px=round(float(focal),3),camera_in_body_flu_m=cam_base.tolist(),
        camera_in_imu_flu_m=cam_imu.tolist(),images_with_alignment=len(offsets),
        imu_alignment_abs_ms=dict(p50=round(float(np.percentile(offsets,50)),3),
                                  p95=round(float(np.percentile(offsets,95)),3),
                                  max=round(max(offsets),3)) if offsets else None,
        note='SDF geometry and bridge stamp alignment only; not an offline camera/IMU noise or time-offset calibration.')
    (trial/'calibration_audit.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))
    if not report['passed']:
        raise SystemExit(1)


if __name__=='__main__':
    main()
