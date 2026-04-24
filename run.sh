#!/usr/bin/env bash
# =============================================================
# run.sh — convenience wrapper for the ROS2 Jazzy container
# Usage:
#   ./run.sh              — open an interactive shell
#   ./run.sh build        — build the Docker image
#   ./run.sh colcon       — build the ROS2 workspace inside container
#   ./run.sh rviz         — launch RViz2
#   ./run.sh rqt          — launch rqt
#   ./run.sh launch <pkg> <file> [args] — run a launch file
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
    echo ">>> Building ROS2 workspace..."
    docker compose run --rm ros2 bash -c \
      "source /opt/ros/jazzy/setup.bash && \
       cd /ros2_ws/src && \
       colcon build --symlink-install && \
       source /ros2_ws/src/install/setup.bash && \
       echo '--- Build complete! ---'"
    ;;

  rviz)
    echo ">>> Launching RViz2..."
    docker compose run --rm ros2 bash -c \
      "source /opt/ros/jazzy/setup.bash && \
       source /ros2_ws/src/install/setup.bash && \
       rviz2"
    ;;

  rqt)
    echo ">>> Launching rqt..."
    docker compose run --rm ros2 bash -c \
      "source /opt/ros/jazzy/setup.bash && \
       source /ros2_ws/src/install/setup.bash && \
       rqt"
    ;;

  launch)
    PKG="${2:?Usage: ./run.sh launch <package> <launch_file> [args]}"
    FILE="${3:?Usage: ./run.sh launch <package> <launch_file> [args]}"
    shift 3
    echo ">>> Launching $PKG $FILE $*..."
    docker compose run --rm ros2 bash -c \
      "source /opt/ros/jazzy/setup.bash && \
       source /ros2_ws/src/install/setup.bash && \
       ros2 launch $PKG $FILE $*"
    ;;

  exec)
    shift
    echo ">>> Running: $*"
    docker compose run --rm ros2 bash -c \
      "source /opt/ros/jazzy/setup.bash && \
       source /ros2_ws/src/install/setup.bash && \
       $*"
    ;;

  shell|*)
    echo ">>> Opening interactive shell in container..."
    docker compose run --rm ros2 bash -c \
	"source /opt/ros/jazzy/setup.bash && \
	source /ros2_ws/src/install/setup.bash && \
	ros2 launch goal_manager_pkg tpr_full.launch.py"
    ;;

esac
