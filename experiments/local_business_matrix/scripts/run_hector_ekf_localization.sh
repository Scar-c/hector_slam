#!/usr/bin/env bash
set -eo pipefail
HECTOR_SCRIPTS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HECTOR_BAG_ROOT="$(cd "$HECTOR_SCRIPTS/.." && pwd)"
if [[ "${1:-}" == --help || "${1:-}" == -h ]]; then
  cat <<'USAGE'
Usage: bash run_hector_ekf_localization.sh [NEW_OUTPUT_DIRECTORY] [--no-hold] [--no-rviz] [--no-global-localization] [--rviz-config FILE]
Start fixed-map Hector + the ORIGINAL wheel/IMU EKF, static extrinsics and RViz.
Default: no initial pose, global search once, replay wheel+scan+IMU at 1x, save map/trajectory/MCAP/plot,
then keep RViz and Hector open. Ctrl+C stops this session. No AMCL or extra map-correction low-pass filter is started.
RViz changes saved with Ctrl+S persist in scripts/hector_ekf.rviz.
USAGE
  exit 0
fi
if [[ $# -gt 0 && "$1" != --* ]]; then
  HECTOR_OUTPUT="$1"
  shift
else
  HECTOR_OUTPUT="$HECTOR_BAG_ROOT/results/hector_ekf_replay_$(TZ=Asia/Shanghai date +%Y%m%d_%H%M%S)_$$"
fi
if [[ -e "$HECTOR_OUTPUT" ]]; then
  echo "Result directory already exists: $HECTOR_OUTPUT" >&2
  exit 2
fi
bash "$HECTOR_SCRIPTS/stop_hector_localization.sh"
export ROS_DOMAIN_ID=98
exec bash /home/lc/ZHUXING_V2WS-master/test/hector_prior_ws/scripts/run_prior_replay.sh \
  --output "$HECTOR_OUTPUT" --ekf --rviz-config "$HECTOR_SCRIPTS/hector_ekf.rviz" "$@"
