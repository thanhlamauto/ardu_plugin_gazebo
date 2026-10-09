#!/usr/bin/env bash
# Replay a saved RGB frame at 10 Hz through the TensorRT ROS depth node.
set -euo pipefail
repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
rgb="${UAV_RGB_FILE:-$repo_dir/docs/mentor_depth_evidence/cpp/sample_rgb_000220.rgb}"
image="${UAV_GPU_DOCKER_IMAGE:-uav-monocular:humble-gpu}"
frames="${UAV_REPLAY_FRAMES:-100}"
if [[ ! -r "$rgb" || ! "$frames" =~ ^[1-9][0-9]*$ ]] || (( frames > 1000 )); then
  echo 'Set UAV_RGB_FILE to a readable 640x360 RGB file and UAV_REPLAY_FRAMES to 1..1000.' >&2
  exit 1
fi
exec docker run --rm --runtime runc --network host --ipc host \
  --read-only --tmpfs /tmp:rw,nosuid,nodev --tmpfs /root/.ros:rw,nosuid,nodev \
  --cap-drop ALL --security-opt no-new-privileges \
  --env "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}" \
  --mount "type=bind,src=$(realpath "$rgb"),dst=/input.rgb,readonly" \
  "$image" ros2 run uav_navigation_ros rgb_depth_ros_replay /input.rgb "$frames"
