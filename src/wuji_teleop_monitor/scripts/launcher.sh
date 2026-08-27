#!/bin/bash
# Wuji Teleop Monitor — desktop launcher (single entry)
#
# Usage:
#   bash launcher.sh                # launch the Monitor UI
#   bash launcher.sh teleop         # same, explicit

set -euo pipefail

# Source ROS2 + workspace before invoking the entry point.
source /opt/ros/humble/setup.bash
if [ -f "$HOME/ros2_ws/install/setup.bash" ]; then
    source "$HOME/ros2_ws/install/setup.bash"
fi

APP="${1:-teleop}"

case "$APP" in
    teleop|monitor) exec ros2 run wuji_teleop_monitor monitor ;;
    *) echo "Unknown app: $APP (only 'teleop' remains)" >&2; exit 1 ;;
esac
