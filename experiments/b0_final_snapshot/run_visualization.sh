#!/usr/bin/env bash
set -eo pipefail
B0_CLOUD_PROJECT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ ! -x "$B0_CLOUD_PROJECT/build/hector_cloud_node" || ! -x "$B0_CLOUD_PROJECT/build/carto_rotation_deskew" ]]; then
  bash "$B0_CLOUD_PROJECT/build.sh"
fi
B0_CLOUD_OUTPUT="/home/lc/slam_11_13/results/b0_hector_gravity_cloud_$(date +%Y%m%d_%H%M%S)"
exec bash "$B0_CLOUD_PROJECT/run_b0.sh" --output "$B0_CLOUD_OUTPUT" --rviz --hold --duration 30 "$@"
