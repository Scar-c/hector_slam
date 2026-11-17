#!/usr/bin/env bash
set -eo pipefail
HECTOR_REPOSITORY="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
HECTOR_WORKSPACE="$(cd "$HECTOR_REPOSITORY/../.." && pwd -P)"
HECTOR_SKIP_DEPS=false
HECTOR_DRY_RUN=false
while [[ $# -gt 0 ]]; do
 case "$1" in
  --workspace) HECTOR_WORKSPACE="$2"; shift 2;;
  --skip-system-deps) HECTOR_SKIP_DEPS=true; shift;;
  --dry-run) HECTOR_DRY_RUN=true; shift;;
  --help|-h) echo 'Usage: bash install.sh [--workspace PATH] [--skip-system-deps] [--dry-run]'; exit 0;;
  *) echo "Unknown argument: $1" >&2; exit 2;;
 esac
done
HECTOR_DEPS=(build-essential cmake pkg-config qtbase5-dev libfftw3-dev libopencv-dev libyaml-cpp-dev python3-yaml libeigen3-dev libceres-dev libboost-iostreams-dev libabsl-dev libprotobuf-dev protobuf-compiler libgoogle-glog-dev libgflags-dev liblua5.3-dev libcairo2-dev python3-colcon-common-extensions ros-jazzy-ament-cmake ros-jazzy-ament-cmake-gtest ros-jazzy-rosidl-default-generators ros-jazzy-tf2-geometry-msgs ros-jazzy-robot-localization ros-jazzy-robot-state-publisher ros-jazzy-rviz2 ros-jazzy-pluginlib ros-jazzy-eigen3-cmake-module ros-jazzy-rmw-cyclonedds-cpp ros-jazzy-rosbag2-transport ros-jazzy-rosbag2-storage-mcap)
HECTOR_MISSING=()
for dependency in "${HECTOR_DEPS[@]}"; do
 if ! dpkg-query -W -f='${Status}' "$dependency" 2>/dev/null | grep -q 'install ok installed'; then HECTOR_MISSING+=("$dependency"); fi
done
printf 'Workspace: %s\nSources: %s\n' "$HECTOR_WORKSPACE" "$HECTOR_REPOSITORY"
printf 'Missing system packages: %s\n' "${HECTOR_MISSING[*]:-none}"
if $HECTOR_DRY_RUN; then
 echo 'Will cache missing .deb files under 3rd/debs, install them, and build Hector online packages plus full-map initializers in a clean ROS Jazzy environment.'
 exit 0
fi
mkdir -p "$HECTOR_WORKSPACE" "$HECTOR_REPOSITORY/3rd/debs"
if [[ ! -e "$HECTOR_WORKSPACE/3rd" ]]; then ln -s "$HECTOR_REPOSITORY/3rd" "$HECTOR_WORKSPACE/3rd"; fi
if ! $HECTOR_SKIP_DEPS && [[ ${#HECTOR_MISSING[@]} -gt 0 ]]; then
 if [[ $EUID -eq 0 ]]; then HECTOR_SUDO=(); else HECTOR_SUDO=(sudo); fi
 "${HECTOR_SUDO[@]}" apt-get update
 (cd "$HECTOR_REPOSITORY/3rd/debs" && apt-get download "${HECTOR_MISSING[@]}")
 "${HECTOR_SUDO[@]}" apt-get install -y "${HECTOR_MISSING[@]}"
fi
[[ -r /opt/ros/jazzy/setup.bash ]] || { echo 'Install ROS 2 Jazzy before building Hector.' >&2; exit 1; }
# Explicitly remove overlays: no installed Cartographer / ZX library is used.
env -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH -u COLCON_PREFIX_PATH -u LD_LIBRARY_PATH -u PYTHONPATH -u ROS_PACKAGE_PATH -u PKG_CONFIG_PATH \
 bash --noprofile --norc -c '
 set -eo pipefail
 source /opt/ros/jazzy/setup.bash
 export CMAKE_BUILD_PARALLEL_LEVEL=3
 export MAKEFLAGS=-j3
 colcon --log-base "$1/log_online" build --base-paths "$2" --build-base "$1/build_online" --install-base "$1/install_online" --executor sequential --packages-up-to hector_backend_rviz hector_backend hector_cloud_frontend hector_slam --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
 ' hector-build "$HECTOR_WORKSPACE" "$HECTOR_REPOSITORY"
# Build scan-only global initializers against this workspace, never a business overlay.
env -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH -u COLCON_PREFIX_PATH -u LD_LIBRARY_PATH -u PYTHONPATH -u ROS_PACKAGE_PATH -u PKG_CONFIG_PATH \
 bash --noprofile --norc -c '
 set -eo pipefail
 source /opt/ros/jazzy/setup.bash
 source "$1/install_online/local_setup.bash"
 cmake -S "$2/tools/global_initialization" -B "$1/build_global_initialization" -DCMAKE_BUILD_TYPE=Release
 cmake --build "$1/build_global_initialization" -j3
 ' hector-global-build "$HECTOR_WORKSPACE" "$HECTOR_REPOSITORY"
printf 'Build complete. Run: source %s/install_online/setup.bash\n' "$HECTOR_WORKSPACE"
# Keep workspace entry points reproducible after a fresh clone.
{
 printf '#!/usr/bin/env bash\nset -eo pipefail\n'
 printf 'exec bash %q --workspace %q "$@"\n' "$HECTOR_REPOSITORY/scripts/install.sh" "$HECTOR_WORKSPACE"
} > "$HECTOR_WORKSPACE/install.sh"
{
 printf '#!/usr/bin/env bash\nset -eo pipefail\n'
 printf 'export HECTOR_WS=%q\n' "$HECTOR_WORKSPACE"
 printf 'exec bash %q "$@"\n' "$HECTOR_REPOSITORY/scripts/run_online_backpack.sh"
} > "$HECTOR_WORKSPACE/run_online.sh"
chmod +x "$HECTOR_WORKSPACE/install.sh" "$HECTOR_WORKSPACE/run_online.sh"
{
 printf '#!/usr/bin/env bash\nset -eo pipefail\n'
 printf 'export HECTOR_WS=%q\n' "$HECTOR_WORKSPACE"
 printf 'exec bash %q "$@"\n' "$HECTOR_REPOSITORY/scripts/run_backpack_localization.sh"
} > "$HECTOR_WORKSPACE/run_relocalization.sh"
chmod +x "$HECTOR_WORKSPACE/run_relocalization.sh"
