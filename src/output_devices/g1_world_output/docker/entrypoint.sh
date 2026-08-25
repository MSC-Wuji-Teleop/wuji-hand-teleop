#!/bin/bash
set -e

WS="$HOME/ros2_ws"

source /opt/ros/humble/setup.bash

if [ ! -d "$WS/src/g1_world_output" ] || [ -z "$(ls -A "$WS/src/g1_world_output" 2>/dev/null)" ]; then
    echo ""
    echo "[ERROR] $WS/src/g1_world_output is empty or not mounted"
    echo "  Configure the g1_world_output volume in docker-compose.yml:"
    echo "    - ../src/output_devices/g1_world_output:/home/wuji/ros2_ws/src/g1_world_output:rw"
    echo ""
    exec "$@"
fi

if [ ! -f "$WS/install/setup.bash" ]; then
    echo "[INFO] First startup, building g1_world_output..."
    cd "$WS"
    # g1_wuji2_description must be built too -- config_loader.py resolves the
    # URDF via get_package_share_directory('g1_wuji2_description'), which
    # only exists once this package is installed.
    colcon build --symlink-install --packages-select g1_world_output g1_wuji2_description \
        --cmake-args -DCMAKE_BUILD_TYPE=Release
    echo "[INFO] Build complete"
fi

source "$WS/install/setup.bash"

exec "$@"
