#!/bin/bash
# =============================================================
# Entrypoint — sources ROS2 + workspace, then runs CMD
# =============================================================
set -e

source /opt/ros/jazzy/setup.bash

[ -f "/ros2_ws/src/install/setup.bash" ] && \
    source /ros2_ws/src/install/setup.bash

exec "$@"
