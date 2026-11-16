#!/usr/bin/env bash
set -eo pipefail
ALIGNED_SCRIPTS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ALIGNED_BAG_ROOT="$(cd "$ALIGNED_SCRIPTS/.." && pwd)"
if [[ "${1:-}" == --help || "${1:-}" == -h ]]; then
  echo 'Usage: bash run_amcl_ekf_time_aligned_localization.sh [NEW_OUTPUT_DIRECTORY] [--no-hold] [--no-rviz] [--rviz-config FILE]'
  echo 'Replay once with the original wheel/IMU EKF, common-time motion-compensated scans, amcl and RViz.'
  echo 'Frame-header compensation only; no pose low-pass and no per-beam timing assumption. Results are saved before holding RViz.'
  exit 0
fi
if [[ $# -gt 0 && "$1" != --* ]]; then
  ALIGNED_OUTPUT="$1"
  shift
else
  ALIGNED_OUTPUT="$ALIGNED_BAG_ROOT/results/amcl_ekf_time_aligned_$(TZ=Asia/Shanghai date +%Y%m%d_%H%M%S)_$$"
fi
if [[ -e "$ALIGNED_OUTPUT" ]]; then
  echo "Result directory already exists: $ALIGNED_OUTPUT" >&2
  exit 2
fi
bash "$ALIGNED_SCRIPTS/stop_time_aligned_localization.sh" amcl
export ROS_DOMAIN_ID=102
exec bash /home/lc/ZHUXING_V2WS-master/test/hector_prior_ws/scripts/run_time_aligned_replay.sh \
  --algorithm amcl --expected-domain 102 --output "$ALIGNED_OUTPUT" --ekf --rviz-config "$ALIGNED_SCRIPTS/amcl_ekf_time_aligned.rviz" "$@"
