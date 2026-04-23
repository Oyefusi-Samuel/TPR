#!/bin/bash
# =============================================================
# Entrypoint — sources ROS2 + workspace, then runs CMD
# =============================================================
set -e

source /opt/ros/jazzy/setup.bash

# Source the colcon workspace if it has been built
if [ -f "${WORKSPACE}/install/setup.bash" ]; then
    source "${WORKSPACE}/install/setup.bash"
fi

exec "$@"
