#!/usr/bin/env bash
set -eo pipefail
FIXED_PRIOR_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ ! -x "$FIXED_PRIOR_DIR/build/hector_cloud_node" ]]; then
  bash "$FIXED_PRIOR_DIR/build.sh"
fi
source /opt/ros/jazzy/setup.bash
source /home/lc/ZHUXING_V2WS-master/install/local_setup.bash
source /home/lc/ZHUXING_V2WS-master/test/hector_prior_ws/install/local_setup.bash
export ROS_DOMAIN_ID=229
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_STATIC_PEERS=''
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
unset ROS_LOCALHOST_ONLY
unset CYCLONEDDS_URI
exec python3 "$FIXED_PRIOR_DIR/run_fixed_prior.py" --profile hector_ekf_desk_pre_time --expected-domain 229 "$@"
