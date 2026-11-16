#!/usr/bin/env bash
set -eo pipefail
FIXED_PRIOR_DIR=/home/lc/ZHUXING_V2WS-master/test/hector_fixed_prior
if [[ "${1:-}" == --help || "${1:-}" == -h ]]; then
  echo 'Usage: bash run_hector_ekf_desk_pre_time_fixed_localization.sh [NEW_OUTPUT_DIRECTORY] [--no-rviz] [--no-hold]'
  echo 'Fixed-map Hector final XYZ/voxel adapter; EKF pre=true desk=true time=true; cached business FFT; playback 1x.'
  exit 0
fi
if [[ $# -gt 0 && "$1" != --* ]]; then
  FIXED_PRIOR_OUTPUT="$1"
  shift
else
  FIXED_PRIOR_OUTPUT="/home/lc/slam_11_13/results/hector_ekf_desk_pre_time_fixed_$(TZ=Asia/Shanghai date +%Y%m%d_%H%M%S)_$$"
fi
if [[ -e "$FIXED_PRIOR_OUTPUT" ]]; then
  echo "Output already exists: $FIXED_PRIOR_OUTPUT" >&2
  exit 2
fi
exec bash "$FIXED_PRIOR_DIR/run.sh" --output "$FIXED_PRIOR_OUTPUT" --initializer business_fft "$@"
