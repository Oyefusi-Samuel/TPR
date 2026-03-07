# Removal & Cleanup Guide

Instructions/commands for safely removing the Jackal AR4 integration components and resetting the workspace to a clean state.

## Docker Cleanup

### Stop and Shutdown
To stop the running containers without deleting them:
```bash
docker compose down
```

### Full Removal
To remove containers, networks, and the built images:
```bash
docker compose down --rmi all
```

## Nuke the Workspace
To completely remove all build artifacts and return the workspace to its original source state:
**NOTE** Don't remove /src. 

```bash
cd ~/jackal_ar4_ws
rm -rf build/ install/ log/
```
*Note: This is useful if you encounter persistent CMake cache issues or want a "fresh" build from scratch.*

## Full Project Removal
To remove the entire project from your host machine:

1. **Remove Workspace**:
   ```bash
   rm -rf <host location>/jackal_ar4_ws
   ```
2. **Uninstall ROS Dependencies**:
   ```bash
   sudo apt remove ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-moveit ros-jazzy-xacro ros-jazzy-joint-state-publisher-gui
   sudo apt autoremove
   ```
