#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
CONDA_ENV="${ARDUPILOT_RVIZ_CONDA_ENV:-ardupilot-rviz}"
CONDA_PREFIX_PATH="$(conda run -n "$CONDA_ENV" sh -c 'printf %s "$CONDA_PREFIX"')"

export GZ_PARTITION="${GZ_PARTITION:-ardupilot_sensor_suite}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"

bridge_args=(
    '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
    '/sensor_suite/rgb@sensor_msgs/msg/Image[gz.msgs.Image'
    '/sensor_suite/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'
    '/sensor_suite/depth@sensor_msgs/msg/Image[gz.msgs.Image'
    '/sensor_suite/depth/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked'
    '/sensor_suite/lidar@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'
    '/sensor_suite/lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked'
    '/sensor_suite/imu@sensor_msgs/msg/Imu[gz.msgs.IMU'
    '/sensor_suite/magnetic_field@sensor_msgs/msg/MagneticField[gz.msgs.Magnetometer'
    '/sensor_suite/air_pressure@sensor_msgs/msg/FluidPressure[gz.msgs.FluidPressure'
    '/sensor_suite/navsat@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat'
)

child_pids=()
cleanup() {
    for pid in "${child_pids[@]:-}"; do
        kill "$pid" 2>/dev/null || true
    done
    wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

conda run -n "$CONDA_ENV" ros2 run ros_gz_bridge parameter_bridge "${bridge_args[@]}" &
child_pids+=("$!")

conda run -n "$CONDA_ENV" ros2 run tf2_ros static_transform_publisher \
    --x 0 --y 0 --z 0 --yaw 0 --pitch 0 --roll 0 \
    --frame-id map --child-frame-id sensor_suite_link &
child_pids+=("$!")

"$CONDA_PREFIX_PATH/bin/python" "$REPO_DIR/scripts/sensor_status_markers.py" &
child_pids+=("$!")

sleep 2
conda run -n "$CONDA_ENV" rviz2 -d "$REPO_DIR/config/sensor_suite.rviz"
