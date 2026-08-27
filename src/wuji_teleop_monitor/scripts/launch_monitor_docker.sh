#!/bin/bash
# Back-compat shim — kept so the README and any existing teleop-monitor.desktop
# shortcut keep working. The real implementation moved to launch_ui_docker.sh
# when the launcher gained a UI-name argument.
exec "$(dirname "${BASH_SOURCE[0]}")/launch_ui_docker.sh" monitor
