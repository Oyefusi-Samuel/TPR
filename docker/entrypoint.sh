#!/bin/bash
set -e

source /opt/ros/jazzy/setup.bash

[ -f "/ros2_ws/src/install/setup.bash" ] && \
    source /ros2_ws/src/install/setup.bash

[ -f "/ros2_ws/src/ar4_ros_driver/install/setup.bash" ] && \
    source /ros2_ws/src/ar4_ros_driver/install/setup.bash

[ -f "/ros2_ws/src/goal_manager_pkg/install/setup.bash" ] && \
    source /ros2_ws/src/goal_manager_pkg/install/setup.bash

exec "$@"
