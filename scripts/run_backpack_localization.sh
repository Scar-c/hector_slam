#!/usr/bin/env bash
set -eo pipefail
HECTOR_REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
HECTOR_WS="${HECTOR_WS:-$(cd "$HECTOR_REPO/../.." && pwd -P)}"
source /opt/ros/jazzy/setup.bash
source "$HECTOR_WS/install_online/local_setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-230}"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-LOCALHOST}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-$HECTOR_WS/log_localization_runtime}"
HECTOR_PRIOR="${HECTOR_PRIOR:-$HECTOR_WS/data/prior_hector_backend/map.yaml}"
HECTOR_INITIALIZER_RESULT="${HECTOR_INITIALIZER_RESULT:-$HECTOR_WS/data/global_initialization/chosen_seed.json}"
[[ -r "$HECTOR_INITIALIZER_RESULT" ]] || { echo "Global initialization result is required: $HECTOR_INITIALIZER_RESULT" >&2; exit 1; }
exec ros2 launch hector_cloud_frontend backpack_localization.launch.py map:="$HECTOR_PRIOR" initial_pose_file:="$HECTOR_INITIALIZER_RESULT" "$@"
