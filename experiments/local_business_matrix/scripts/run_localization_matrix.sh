#!/usr/bin/env bash
set -e
TASK_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
source /home/lc/ZHUXING_V2WS-master/install/local_setup.bash
source "$TASK_DIR/install/local_setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-97}"
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_STATIC_PEERS=''
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
unset ROS_LOCALHOST_ONLY
unset CYCLONEDDS_URI
exec python3 "$TASK_DIR/scripts/run_localization_matrix.py" "$@"
