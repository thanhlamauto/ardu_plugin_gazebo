#!/usr/bin/env bash
# Run the ARM64 ROS 2 Humble C++ perception image on Jetson Orin Nano.
set -euo pipefail

usage() {
  cat <<'EOF'
Usage: UAV_DEPTH_ONNX=/absolute/path/depth.onnx bash scripts/run_cpp_depth_orin.sh [check|probe|run]

  check  Verify Jetson release, Docker image architecture and model mount (default)
  probe  Run one CPU inference on a synthetic RGB frame inside the container
  run    Subscribe to /sensor_suite/rgb and publish the depth obstacle cloud

Optional: UAV_DOCKER_IMAGE (default uav-monocular:humble), ROS_DOMAIN_ID (default 0)
EOF
}

mode="${1:-check}"
case "$mode" in
  check|probe|run) ;;
  -h|--help) usage; exit 0 ;;
  *) usage >&2; exit 2 ;;
esac

if [[ "$(uname -m)" != aarch64 ]]; then
  echo "This runner must be used on the ARM64 Jetson host." >&2
  exit 1
fi
if [[ ! -r /etc/nv_tegra_release ]]; then
  echo "Missing /etc/nv_tegra_release; this does not look like a Jetson host." >&2
  exit 1
fi
release_line="$(head -n 1 /etc/nv_tegra_release)"
if [[ "$release_line" != *"R36 (release)"* ]]; then
  echo "Expected Jetson Linux R36 (JetPack 6); found: $release_line" >&2
  exit 1
fi
echo "Jetson host: $release_line"

image="${UAV_DOCKER_IMAGE:-uav-monocular:humble}"
model="${UAV_DEPTH_ONNX:-}"
if [[ -z "$model" || "$model" != /* || ! -r "$model" || ! -f "$model" ]]; then
  echo "Set UAV_DEPTH_ONNX to a readable absolute path to the exported ONNX model." >&2
  exit 1
fi
if ! command -v docker >/dev/null 2>&1 || ! docker info >/dev/null 2>&1; then
  echo "Docker daemon is unavailable. Check Docker installation and socket access." >&2
  exit 1
fi
arch="$(docker image inspect "$image" --format '{{.Os}}/{{.Architecture}}' 2>/dev/null || true)"
if [[ "$arch" != linux/arm64 ]]; then
  echo "Expected image $image with linux/arm64 architecture; found: ${arch:-missing}." >&2
  exit 1
fi
echo "Docker image: $image ($arch)"
echo "ONNX model: $model"

if [[ "$mode" == check ]]; then
  exit 0
fi

docker_args=(
  run --rm --name uav-monocular-depth
  --runtime runc
  --network host --ipc host
  --read-only --tmpfs /tmp:rw,nosuid,nodev --tmpfs /root/.ros:rw,nosuid,nodev
  --cap-drop ALL --security-opt no-new-privileges
  --mount "type=bind,src=$model,dst=/models/depth.onnx,readonly"
  --env "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}"
)

if [[ "$mode" == probe ]]; then
  exec docker "${docker_args[@]}" "$image" bash -lc \
    'dd if=/dev/zero of=/tmp/black.rgb bs=691200 count=1 status=none && ros2 run uav_navigation_ros depth_anything_onnx_probe /models/depth.onnx /tmp/black.rgb /tmp/depth.f32 1'
fi

echo "Starting perception only (OpenCV DNN CPU; planner and autopilot adapter off)."
exec docker "${docker_args[@]}" "$image" \
  ros2 launch uav_navigation_ros monocular_runtime.launch.xml \
    model_file:=/models/depth.onnx params_file:=/opt/uav/monocular.yaml \
    depth_backend:=cpu enable_local_navigation:=false
