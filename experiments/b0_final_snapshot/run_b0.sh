#!/usr/bin/env bash
set -eo pipefail
B0_PROJECT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/jazzy/setup.bash
source /home/lc/ZHUXING_V2WS-master/install/local_setup.bash
source /home/lc/ZHUXING_V2WS-master/test/hector_prior_ws/install/local_setup.bash
export ROS_DOMAIN_ID=221
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_STATIC_PEERS=''
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
unset ROS_LOCALHOST_ONLY CYCLONEDDS_URI
exec python3 "$B0_PROJECT/run_b0.py" "$@"
