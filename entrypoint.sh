#!/bin/bash
# =============================================================
# Entrypoint — sources ROS2 + all workspaces, then runs CMD
# =============================================================
set -e

source /opt/ros/jazzy/setup.bash

# TPR root (a_star_smooth_planner)
[ -f "/ros2_ws/src/install/setup.bash" ] && \
    source /ros2_ws/src/install/setup.bash

# ar4_ros_driver
[ -f "/ros2_ws/src/ar4_ros_driver/install/setup.bash" ] && \
    source /ros2_ws/src/ar4_ros_driver/install/setup.bash

# goal_manager_pkg
[ -f "/ros2_ws/src/goal_manager_pkg/install/setup.bash" ] && \
    source /ros2_ws/src/goal_manager_pkg/install/setup.bash

exec "$@"
