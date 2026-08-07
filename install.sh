#!/bin/bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
INSTALL_DIR="${BXI_SLAM_INSTALL_DIR:-/opt/bxi/bxi_rc_slam/install}"

if [ "$(id -u)" -ne 0 ]; then
    echo "install.sh must run as root" >&2
    exit 1
fi

# ROS setup scripts probe optional variables and are not nounset-safe.
set +u
source /opt/ros/humble/setup.bash
if [ -f /opt/bxi/bxi_ros2_pkg/setup.bash ]; then
    source /opt/bxi/bxi_ros2_pkg/setup.bash
fi
set -u

cd "$ROOT_DIR"

# Source archives created on Windows may not retain Unix executable bits.
# Restore them before running the converter contract check below.
chmod 755 "$ROOT_DIR/pcd2pgm_headless" \
          "$ROOT_DIR/start.sh" \
          "$ROOT_DIR/stop.sh" \
          "$ROOT_DIR/install.sh"

# Keep the bundled converter and map-pipeline config on the same operation-ID
# contract. This fails installation immediately instead of failing when the
# user finishes a mapping session (for example, legacy "closing" vs "close").
while IFS='=' read -r key operation; do
    case "$key" in
        pipeline.step.*.operation)
            "$ROOT_DIR/pcd2pgm_headless" \
                --params map "$operation" >/dev/null
            ;;
    esac
done < "$ROOT_DIR/scans_nav2_map.cfg"

# livox_ros_driver2 keeps ROS 1/2 manifests and launch trees side by side.
# Materialize the ROS 2 names expected by colcon so a source tarball can be
# installed directly without running the driver's interactive build helper.
cp -f src/livox_ros_driver2/package_ROS2.xml \
    src/livox_ros_driver2/package.xml
rm -rf src/livox_ros_driver2/launch
cp -a src/livox_ros_driver2/launch_ROS2 \
      src/livox_ros_driver2/launch

rm -rf build log "$INSTALL_DIR"

cmake_args=(
    -DCMAKE_BUILD_TYPE=Release
    -DBUILD_TESTING=OFF
    -DROS_EDITION=ROS2
    -DHUMBLE_ROS=humble
)

# Robots may be deployed on an isolated LAN. Allow the installer to reuse a
# verified small_gicp source cache instead of making the build depend on
# GitHub availability. FetchContent still handles online developer builds.
if [[ -n "${SMALL_GICP_SOURCE_DIR:-}" ]]; then
    if [[ ! -f "$SMALL_GICP_SOURCE_DIR/CMakeLists.txt" ]]; then
        echo "invalid SMALL_GICP_SOURCE_DIR: $SMALL_GICP_SOURCE_DIR" >&2
        exit 1
    fi
    cmake_args+=("-DFETCHCONTENT_SOURCE_DIR_SMALL_GICP=$SMALL_GICP_SOURCE_DIR")
fi

colcon build \
    --merge-install \
    --install-base "$INSTALL_DIR" \
    --cmake-args "${cmake_args[@]}"

chmod 755 "$ROOT_DIR/start.sh" "$ROOT_DIR/stop.sh" "$ROOT_DIR/install.sh"
echo "bxi_rc_slam installed at $INSTALL_DIR"
