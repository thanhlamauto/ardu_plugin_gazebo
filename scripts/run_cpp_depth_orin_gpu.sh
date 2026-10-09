#!/usr/bin/env bash
# Start the C++ ROS depth node with TensorRT on Orin. No planner or autopilot.
set -euo pipefail
mode="${1:-check}"
case "$mode" in check|run) ;; *) echo 'usage: bash scripts/run_cpp_depth_orin_gpu.sh [check|run]' >&2; exit 2;; esac
if [[ "$(uname -m)" != aarch64 || ! -r /etc/nv_tegra_release || ! -e /dev/nvhost-gpu ]]; then
  echo 'Run this on a Jetson Orin host with GPU device available.' >&2; exit 1
fi
engine="${UAV_DEPTH_ENGINE:-}"
if [[ -z "$engine" || "$engine" != /* || ! -r "$engine" ]]; then
  echo 'Set UAV_DEPTH_ENGINE to an absolute path to the TensorRT .plan file.' >&2; exit 1
fi
image="${UAV_GPU_DOCKER_IMAGE:-uav-monocular:humble-gpu}"
if ! docker info --format '{{.ServerVersion}}' >/dev/null; then
  echo "Cannot access Docker daemon as $(id -un); check Docker context and permissions." >&2
  exit 1
fi
if ! arch="$(docker image inspect "$image" --format '{{.Os}}/{{.Architecture}}')"; then
  echo "Docker image unavailable: $image (context: $(docker context show); user: $(id -un))." >&2
  echo 'Build it with: docker build --platform linux/arm64 -f docker/Dockerfile.humble-gpu -t uav-monocular:humble-gpu .' >&2
  exit 1
fi
if [[ "$arch" != linux/arm64 ]]; then
  echo "Expected linux/arm64 image, found $arch: $image" >&2; exit 1
fi
if ! docker info --format '{{json .Runtimes}}' | grep -q nvidia; then
  echo 'Docker NVIDIA runtime is unavailable.' >&2; exit 1
fi
echo "GPU image: $image; engine: $engine"
[[ "$mode" == check ]] && exit 0
exec docker run --rm --name uav-monocular-depth-gpu --runtime nvidia \
  --network host --ipc host --read-only \
  --tmpfs /tmp:rw,nosuid,nodev --tmpfs /root/.ros:rw,nosuid,nodev \
  --cap-drop ALL --security-opt no-new-privileges \
  --env NVIDIA_VISIBLE_DEVICES=all \
  --env NVIDIA_DRIVER_CAPABILITIES=compute,utility \
  --env "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}" \
  --mount "type=bind,src=$(realpath "$engine"),dst=/models/depth_fp16.plan,readonly" \
  "$image" /ws/install/lib/uav_navigation_ros/monocular_depth_node --ros-args \
    -p model_path:=/models/depth_fp16.plan -p backend:=tensorrt \
    -p image_topic:=/sensor_suite/rgb \
    -p cloud_topic:=/perception/obstacles_camera \
    -p diagnostics_topic:=/perception/monocular_depth/diagnostics \
    -p max_image_age_s:=0.25 -p max_inference_ms:=100.0 \
    -p max_processing_ms:=100.0 -p point_stride:=8
