#!/usr/bin/env bash
# Replay two saved Gazebo RGB/pose snapshots through C++ TensorRT depth and MPPI.
# Test only: no adapter or autopilot container is started.
set -euo pipefail

if [[ "$(uname -m)" != aarch64 || ! -e /dev/nvhost-gpu ]]; then
  echo 'Run on Jetson Orin with the GPU device available.' >&2
  exit 1
fi
repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
samples="${UAV_SHADOW_SAMPLES:-$repo/artifacts/orin_depth_mppi_samples}"
config="${UAV_SHADOW_CONFIG:-$repo/uav_navigation_bringup/config/monocular.yaml}"
engine="${UAV_DEPTH_ENGINE:-$HOME/uav_deploy/depth_fp16.plan}"
image="${UAV_GPU_DOCKER_IMAGE:-uav-monocular:humble-gpu-shadow}"
frames="${UAV_SHADOW_FRAMES:-80}"
domain="${ROS_DOMAIN_ID:-74}"
if [[ ! "$frames" =~ ^[1-9][0-9]*$ ]] || (( frames > 1000 )); then
  echo 'UAV_SHADOW_FRAMES must be 1..1000.' >&2
  exit 1
fi
for path in "$samples/planner_start.rgb" "$samples/poses.f32" \
            "$config" "$engine"; do
  if [[ ! -r "$path" ]]; then
    echo "Missing readable input: $path" >&2
    exit 1
  fi
done
depth_name="uav-mppi-shadow-depth-$$"
planner_name="uav-mppi-shadow-planner-$$"
cleanup() {
  docker stop "$depth_name" "$planner_name" >/dev/null 2>&1 || true
}
trap cleanup EXIT

docker run -d --rm --name "$depth_name" --runtime nvidia \
  --network host --ipc host --read-only \
  --tmpfs /tmp:rw,nosuid,nodev --tmpfs /root/.ros:rw,nosuid,nodev \
  --cap-drop ALL --security-opt no-new-privileges \
  --env NVIDIA_VISIBLE_DEVICES=all --env NVIDIA_DRIVER_CAPABILITIES=compute,utility \
  --env "ROS_DOMAIN_ID=$domain" \
  --mount "type=bind,src=$engine,dst=/models/depth_fp16.plan,readonly" \
  "$image" /ws/install/lib/uav_navigation_ros/monocular_depth_node --ros-args \
  -p model_path:=/models/depth_fp16.plan -p backend:=tensorrt \
  -p image_topic:=/sensor_suite/rgb -p cloud_topic:=/perception/obstacles_camera \
  -p diagnostics_topic:=/perception/monocular_depth/diagnostics \
  -p max_image_age_s:=0.25 -p max_inference_ms:=100.0 \
  -p max_processing_ms:=100.0 -p point_stride:=8 >/dev/null

docker run -d --rm --name "$planner_name" --runtime runc \
  --network host --ipc host --read-only \
  --tmpfs /tmp:rw,nosuid,nodev --tmpfs /root/.ros:rw,nosuid,nodev \
  --cap-drop ALL --security-opt no-new-privileges \
  --env "ROS_DOMAIN_ID=$domain" \
  --mount "type=bind,src=$config,dst=/params/monocular.yaml,readonly" \
  "$image" /ws/install/lib/uav_navigation_ros/local_navigation_node --ros-args \
  --params-file /params/monocular.yaml -p use_sim_time:=false \
  -p visualization.enabled:=false >/dev/null

sleep 3
docker ps --format '{{.Names}}' | grep -Fqx "$depth_name"
docker ps --format '{{.Names}}' | grep -Fqx "$planner_name"
for scene in 0 1; do
  docker run --rm --runtime runc --network host --ipc host --read-only \
    --tmpfs /tmp:rw,nosuid,nodev --tmpfs /root/.ros:rw,nosuid,nodev \
    --cap-drop ALL --security-opt no-new-privileges \
    --env "ROS_DOMAIN_ID=$domain" \
    --mount "type=bind,src=$samples,dst=/samples,readonly" \
    "$image" /ws/install/lib/uav_navigation_ros/rgb_depth_mppi_shadow \
    /samples/planner_start.rgb /samples/poses.f32 "$scene" "$frames" \
    | tee "$samples/shadow_scene_${scene}.log"
done
