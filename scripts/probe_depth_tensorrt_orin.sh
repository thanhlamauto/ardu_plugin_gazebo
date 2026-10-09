#!/usr/bin/env bash
# Build and benchmark the pinned ONNX model with TensorRT on the Orin GPU.
# This probes model compatibility; it does not start the ROS depth node.
set -euo pipefail

if [[ "$(uname -m)" != aarch64 || ! -r /etc/nv_tegra_release ]]; then
  echo "Run this on the Jetson Orin ARM64 host." >&2
  exit 1
fi
if ! grep -q 'R36 (release)' /etc/nv_tegra_release; then
  echo "Expected Jetson Linux R36 (JetPack 6)." >&2
  exit 1
fi
model="${UAV_DEPTH_ONNX:-}"
if [[ -z "$model" || "$model" != /* || ! -f "$model" || ! -r "$model" ]]; then
  echo "Set UAV_DEPTH_ONNX to the readable absolute path of depth.onnx." >&2
  exit 1
fi
model="$(realpath "$model")"
model_dir="$(dirname "$model")"
model_name="$(basename "$model")"
if ! command -v docker >/dev/null 2>&1 || ! docker info >/dev/null 2>&1; then
  echo "Docker daemon is unavailable." >&2
  exit 1
fi
if ! docker info --format '{{json .Runtimes}}' | grep -q 'nvidia'; then
  echo "NVIDIA Container Runtime is not configured in Docker." >&2
  exit 1
fi

image="${UAV_JETPACK_IMAGE:-nvcr.io/nvidia/l4t-jetpack:r36.4.0}"
artifact_dir="${UAV_TRT_ARTIFACT_DIR:-$PWD/artifacts/orin_tensorrt}"
mkdir -p "$artifact_dir"
artifact_dir="$(cd "$artifact_dir" && pwd -P)"
echo "Host: $(head -n 1 /etc/nv_tegra_release)"
echo "Container: $image"
echo "ONNX: $model"
echo "TensorRT engine/logs: $artifact_dir"

docker run --rm --runtime nvidia --network none \
  --env NVIDIA_VISIBLE_DEVICES=all \
  --env NVIDIA_DRIVER_CAPABILITIES=compute,utility \
  --mount "type=bind,src=$model_dir,dst=/models,readonly" \
  --mount "type=bind,src=$artifact_dir,dst=/artifacts" \
  --env "UAV_MODEL_BASENAME=$model_name" \
  "$image" bash -lc '
    set -euo pipefail
    trtexec=/usr/src/tensorrt/bin/trtexec
    test -x "$trtexec"
    "$trtexec" --onnx="/models/$UAV_MODEL_BASENAME" \
      --saveEngine=/artifacts/depth_fp16.plan --fp16 --skipInference \
      2>&1 | tee /artifacts/build.log
    "$trtexec" --loadEngine=/artifacts/depth_fp16.plan --duration=10 \
      2>&1 | tee /artifacts/benchmark.log
  '
