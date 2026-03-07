# Jackal + AR4 Integration Launch & Description

This directory contains the launch and configuration files for the integrated Jackal UGV and AR4-MK3 robotic arm platform.

## Implementation Details (Through Phase 4)

### 1. Unified Robot Description (URDF/Xacro)
The robot description is managed in the `jackal_ar4_description` package.
- **`urdf/ar4_arm.urdf.xacro`**: A modified version of the AR4 description wrapped in a `xacro:macro`. It supports a `prefix` parameter to ensure all links and joints are unique, preventing conflicts with the Jackal's `base_link`.
- **`urdf/jackal_ar4.urdf.xacro`**: The master Xacro file that:
    - Includes the standard Clearpath `j100.urdf.xacro`.
    - Includes the modified `ar4_arm.urdf.xacro`.
    - Defines a physical **`ar4_mount_plate`** (0.2m x 0.2m x 0.01m).
    - Mounts the plate to the Jackal's `default_mount` frame.
    - Attaches the AR4 `base_link` to the mount plate.

### 2. Launch Configurations
Two launch methods are provided for flexibility:
- **`jackal_ar4.launch` (XML)**: The primary launch file. It uses a robust `$(command 'xacro ...')` tag to pass the processed URDF to the `robot_state_publisher`. This method avoids common YAML parsing errors in ROS 2 Jazzy.
- **`display.launch.py` (Python)**: A standard ROS 2 Python launch script using `urdf_launch` for modularity.

### 3. RViz Visualization
- **`config/rviz_config.rviz`**: Pre-configured for this platform.
    - **Fixed Frame**: `base_link`.
    - **Displays**: RobotModel (via `/robot_description`), TF tree, and Grid.
- **Joint Control**: Both launch files include `joint_state_publisher_gui`, allowing manual manipulation of the Jackal's wheels and all 6 axes of the AR4 arm.

## Usage

To launch the visualization:

```bash
cd ~/jackal_ar4_ws
source install/setup.bash
ros2 launch jackal_ar4_description jackal_ar4.launch
```

## Troubleshooting Note
During Phase 4, a specific issue was identified where colons in URDF comments (e.g., `<!-- Joint: ... -->`) caused the ROS 2 YAML parameter parser to crash. All comments in the integrated URDF have been sanitized to use spaces instead of colons.
