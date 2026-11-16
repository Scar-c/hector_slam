#!/usr/bin/env bash
set -eo pipefail
B0_CLOUD_PROJECT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/jazzy/setup.bash
source /home/lc/ZHUXING_V2WS-master/install/local_setup.bash
source /home/lc/ZHUXING_V2WS-master/test/hector_prior_ws/install/local_setup.bash
cmake -S "$B0_CLOUD_PROJECT" -B "$B0_CLOUD_PROJECT/build" -DCMAKE_BUILD_TYPE=Release
cmake --build "$B0_CLOUD_PROJECT/build" -j2
