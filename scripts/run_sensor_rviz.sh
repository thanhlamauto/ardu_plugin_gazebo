#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
CONDA_ENV="${ARDUPILOT_RVIZ_CONDA_ENV:-ardupilot-rviz}"
CONDA_BIN="${ARDUPILOT_RVIZ_CONDA_EXE:-}"

if [[ -z "$CONDA_BIN" && -x /opt/miniconda3/bin/conda && \
      -d "/opt/miniconda3/envs/$CONDA_ENV" ]]; then
    CONDA_BIN=/opt/miniconda3/bin/conda
fi

if [[ -z "$CONDA_BIN" ]]; then
    CONDA_BIN="$(command -v conda || true)"
fi

if [[ -z "$CONDA_BIN" ]]; then
    echo "Không tìm thấy conda. Hãy cài Miniconda hoặc đặt ARDUPILOT_RVIZ_CONDA_EXE." >&2
    exit 1
fi

CONDA_PREFIX_PATH="$("$CONDA_BIN" run -n "$CONDA_ENV" sh -c 'printf %s "$CONDA_PREFIX"')"

export GZ_PARTITION="${GZ_PARTITION:-ardupilot_sensor_suite}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"

if [[ ! -x "$CONDA_PREFIX_PATH/bin/python" ||
      ! -x "$CONDA_PREFIX_PATH/bin/ros2" ||
      ! -x "$CONDA_PREFIX_PATH/bin/rviz2" ]]; then
    echo "Conda env '$CONDA_ENV' thiếu python/ros2/rviz2: $CONDA_PREFIX_PATH" >&2
    exit 1
fi

export AMENT_PREFIX_PATH="$CONDA_PREFIX_PATH${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"
export PATH="$CONDA_PREFIX_PATH/bin:$PATH"

bridge_args=(
    '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
    '/iris/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry'
    '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'
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
kill_tree() {
    local parent_pid="$1"
    local child_pid
    while IFS= read -r child_pid; do
        [[ -n "$child_pid" ]] || continue
        kill_tree "$child_pid"
    done < <(pgrep -P "$parent_pid" 2>/dev/null || true)
    kill -TERM "$parent_pid" 2>/dev/null || true
}

cleanup() {
    for pid in "${child_pids[@]:-}"; do
        kill_tree "$pid"
    done
    wait 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 130' INT TERM

"$CONDA_PREFIX_PATH/bin/ros2" run ros_gz_bridge parameter_bridge "${bridge_args[@]}" &
child_pids+=("$!")

"$CONDA_PREFIX_PATH/bin/ros2" run robot_state_publisher \
    robot_state_publisher "$REPO_DIR/config/iris_sensor_suite.urdf" &
child_pids+=("$!")

"$CONDA_PREFIX_PATH/bin/python" "$REPO_DIR/scripts/sensor_status_markers.py" &
child_pids+=("$!")

sleep 2
"$CONDA_PREFIX_PATH/bin/rviz2" -d "$REPO_DIR/config/sensor_suite.rviz"
