#!/usr/bin/env bash
# Run the standalone C++ RGB-to-depth TensorRT probe in a JetPack container.
set -euo pipefail

if [[ "$(uname -m)" != aarch64 || ! -r /etc/nv_tegra_release ]]; then
  echo "Run this on a Jetson ARM64 host." >&2
  exit 1
fi
repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
engine="${UAV_TRT_ENGINE:-}"
rgb="${UAV_RGB_FILE:-$repo_dir/docs/mentor_depth_evidence/cpp/sample_rgb_000220.rgb}"
if [[ -z "$engine" || "$engine" != /* || ! -r "$engine" ]]; then
  echo "Set UAV_TRT_ENGINE to an absolute path to a readable TensorRT .plan." >&2
  exit 1
fi
if [[ ! -r "$rgb" ]]; then
  echo "Set UAV_RGB_FILE to a readable 640x360 packed RGB frame." >&2
  exit 1
fi
engine="$(realpath "$engine")"
rgb="$(realpath "$rgb")"
image="${UAV_JETPACK_IMAGE:-nvcr.io/nvidia/l4t-jetpack:r36.4.0}"
artifact_dir="${UAV_TRT_ARTIFACT_DIR:-$repo_dir/artifacts/orin_cpp_tensorrt}"
mkdir -p "$artifact_dir"
artifact_dir="$(realpath "$artifact_dir")"
repeats="${UAV_TRT_REPEATS:-30}"
if ! [[ "$repeats" =~ ^[1-9][0-9]*$ ]] || (( repeats > 1000 )); then
  echo "UAV_TRT_REPEATS must be 1..1000." >&2
  exit 1
fi
if ! docker info --format '{{json .Runtimes}}' | grep -q 'nvidia'; then
  echo "Docker NVIDIA runtime is unavailable." >&2
  exit 1
fi
if [[ ! -e /dev/nvhost-gpu ]]; then
  echo "Jetson GPU device /dev/nvhost-gpu is unavailable." >&2
  exit 1
fi
gpu_gid="$(stat -c %g /dev/nvhost-gpu)"

docker run --rm --runtime nvidia --network none --read-only \
  --tmpfs /tmp:rw,nosuid,nodev \
  --user "$(id -u):$(id -g)" --group-add "$gpu_gid" \
  --cap-drop ALL --security-opt no-new-privileges \
  --env NVIDIA_VISIBLE_DEVICES=all \
  --env NVIDIA_DRIVER_CAPABILITIES=compute,utility \
  --mount "type=bind,src=$repo_dir,dst=/repo,readonly" \
  --mount "type=bind,src=$engine,dst=/engine.plan,readonly" \
  --mount "type=bind,src=$rgb,dst=/input.rgb,readonly" \
  --mount "type=bind,src=$artifact_dir,dst=/artifacts" \
  --env "UAV_TRT_REPEATS=$repeats" \
  "$image" bash -lc '
    set -euo pipefail
    g++ -O2 -std=c++17 \
      -I/repo/uav_navigation_core/include -I/usr/local/cuda/include \
      /repo/tools/depth_tensorrt_probe.cpp \
      /repo/uav_navigation_core/src/metric_depth.cpp \
      -o /artifacts/depth_tensorrt_probe \
      -L/usr/local/cuda/lib64 -lnvinfer -lcudart
    /artifacts/depth_tensorrt_probe /engine.plan /input.rgb \
      /artifacts/depth.f32 "$UAV_TRT_REPEATS" | tee /artifacts/benchmark.log
  '
