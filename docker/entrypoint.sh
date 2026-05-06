#!/bin/bash
set -e

source /opt/ros/jazzy/setup.bash

[ -f "/ros2_ws/src/install/setup.bash" ] && \
    source /ros2_ws/src/install/setup.bash

exec "$@"
