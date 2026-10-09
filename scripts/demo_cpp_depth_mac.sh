#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
python_bin="${UAV_DEMO_PYTHON:-/opt/miniconda3/bin/python}"
exec "$python_bin" "$repo_root/scripts/demo_cpp_depth_mac.py" "$@"
