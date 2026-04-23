#!/usr/bin/env bash
# =============================================================
# run.sh — convenience wrapper for the ROS2 Jazzy container
# Usage:
#   ./run.sh              — open an interactive shell
#   ./run.sh build        — build the Docker image
#   ./run.sh colcon       — build the ROS2 workspace inside container
#   ./run.sh rviz         — launch RViz2
#   ./run.sh rqt          — launch rqt
#   ./run.sh exec <cmd>   — run an arbitrary command
# =============================================================
set -e

IMAGE="ros2_jazzy_desktop:latest"
CONTAINER="ros2_jazzy"

# Allow X11 connections from the container (Linux only)
xhost +local:docker 2>/dev/null || true

case "${1:-shell}" in

  build)
    echo ">>> Building Docker image..."
    docker compose build
    ;;

  colcon)
    echo ">>> Building ROS2 workspace with colcon..."
    docker compose run --rm ros2 bash -c \
      "source /opt/ros/jazzy/setup.bash && \
       colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo"
    ;;

  rviz)
    echo ">>> Launching RViz2..."
    docker compose run --rm ros2 bash -c \
      "source /opt/ros/jazzy/setup.bash && \
       source /ros2_ws/install/setup.bash && \
       rviz2"
    ;;

  rqt)
    echo ">>> Launching rqt..."
    docker compose run --rm ros2 bash -c \
      "source /opt/ros/jazzy/setup.bash && \
       source /ros2_ws/install/setup.bash && \
       rqt"
    ;;

  exec)
    shift
    echo ">>> Running: $*"
    docker compose run --rm ros2 bash -c "$*"
    ;;

  shell|*)
    echo ">>> Opening interactive shell in container..."
    docker compose run --rm ros2 bash
    ;;

esac
