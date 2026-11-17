#!/usr/bin/env bash
set -eo pipefail
HECTOR_REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
HECTOR_WS="${HECTOR_WS:-$(cd "$HECTOR_REPO/../.." && pwd -P)}"
source /opt/ros/jazzy/setup.bash
source "$HECTOR_WS/install_online/local_setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-228}"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-LOCALHOST}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-$HECTOR_WS/log_online_runtime}"
exec ros2 launch hector_backend online_backpack.launch.py use_sim_time:=true "$@"
