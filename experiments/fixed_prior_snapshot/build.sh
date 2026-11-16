#!/usr/bin/env bash
set -eo pipefail
FIXED_PRIOR_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/jazzy/setup.bash
source /home/lc/ZHUXING_V2WS-master/install/local_setup.bash
source /home/lc/ZHUXING_V2WS-master/test/hector_prior_ws/install/local_setup.bash
cmake -S "$FIXED_PRIOR_DIR" -B "$FIXED_PRIOR_DIR/build" -DCMAKE_BUILD_TYPE=Release
cmake --build "$FIXED_PRIOR_DIR/build" -j2
