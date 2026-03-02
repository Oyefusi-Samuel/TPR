# Jackal AR4-MK3 Integration for Trash Picking Robot in ROS 2 Jazzy

Mobile manipulation platform integrating a Clearpath Jackal UGV with an Annin Robotics AR4-MK3 arm.

## Installation
Assume a desktop install of ROS2 Jazzy according to docs.ros.org 
1. **Install dependencies:**
   ```bash
   sudo apt update && sudo apt install -y ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-moveit ros-jazzy-xacro ros-jazzy-joint-state-publisher-gui
   ```
2. **Build Workspace**:
   ```bash
   cd jackal_ar4_ws/
   source /opt/ros/jazzy/setup.bash
   colcon build --symlink-install
   source install/setup.bash
   ```

## Bringup
```bash
ros2 launch jackal_ar4_description jackal_ar4.launch
```
- **Navigation**: Use the "2D Nav Goal" tool in RViz or run:
  `ros2 run jackal_ar4_goals send_nav_goal <x> <y> <theta>`

## Docker Support
1. **Allow GUI (run once per session)**: `xhost +local:docker`
2. **Build and Run**:
   ```bash
   docker compose up --build
   ```
3. **Interactive Shell From Another Terminal**:
   ```bash
   docker exec -it jackal_ar4_container bash
   ```

For shutdown and cleanup instructions, see [REMOVAL.md](./REMOVAL.md).
