# Jackal AR4 Simulation Workspace

A ROS 2 Jazzy simulation of the **Clearpath Jackal J100** mobile robot equipped with an **AR4 6-DOF robotic arm** and gripper, running in **Gazebo Harmonic** with full **MoveIt 2** and **Nav2** integration.

![Robot Preview](docs/preview.png)

---

## Table of Contents

- [Overview](#overview)
- [System Requirements](#system-requirements)
- [Repository Structure](#repository-structure)
- [Installation](#installation)
  - [1. Install ROS 2 Jazzy](#1-install-ros-2-jazzy)
  - [2. Install Gazebo Harmonic](#2-install-gazebo-harmonic)
  - [3. Clone the Repository](#3-clone-the-repository)
  - [4. Install Dependencies with rosdep](#4-install-dependencies-with-rosdep)
  - [5. Build the Workspace](#5-build-the-workspace)
- [Running the Simulation](#running-the-simulation)
  - [RViz Only (No Gazebo)](#rviz-only-no-gazebo)
  - [Full Gazebo Simulation](#full-gazebo-simulation)
  - [Choosing a World](#choosing-a-world)
- [Available Worlds](#available-worlds)
- [Verifying Everything Works](#verifying-everything-works)
- [Sending Goals](#sending-goals)
  - [Navigation Goal (Nav2)](#navigation-goal-nav2)
  - [Arm Goal (MoveIt)](#arm-goal-moveit)
- [Architecture Overview](#architecture-overview)
- [Package Breakdown](#package-breakdown)
- [Troubleshooting](#troubleshooting)
- [Contributing](#contributing)
- [License](#license)

---

## Overview

This workspace provides a full simulation stack for a Jackal + AR4 mobile manipulator:

- **Gazebo Harmonic** — physics simulation with differential drive, collision, and sensors
- **MoveIt 2** — motion planning and arm trajectory execution via OMPL
- **Nav2** — autonomous navigation with global/local costmaps and MPPI controller
- **ros2_control** — hardware abstraction for arm and gripper joints
- **RViz2** — visualisation with full MoveIt motion planning panel

---

## System Requirements

| Requirement | Version |
|---|---|
| Ubuntu | 24.04 (Noble) |
| ROS 2 | Jazzy |
| Gazebo | Harmonic |
| Python | 3.12+ |
| RAM | 8 GB minimum (16 GB recommended) |
| GPU | Optional but recommended for Gazebo rendering |

---

## Repository Structure

```
jackal_ar4_ws/
├── src/
│   ├── ar4/                          # AR4 arm URDF, meshes, MoveIt config
│   │   ├── ar4_description/
│   │   └── ar4_moveit_config/
│   ├── clearpath_common/             # Clearpath platform description (J100 base)
│   │   ├── clearpath_platform_description/
│   │   └── clearpath_control/
│   ├── jackal_ar4_description/       # Combined robot URDF, launch files
│   │   ├── urdf/
│   │   │   ├── jackal_ar4.urdf.xacro
│   │   │   ├── jackal_ar4.ros2_control.xacro
│   │   │   └── ar4_arm.urdf.xacro
│   │   └── launch/
│   │       ├── jackal_ar4.launch     # RViz-only launch (no Gazebo)
│   │       └── gazebo.launch.py      # Full Gazebo simulation launch
│   ├── jackal_ar4_moveit_config/     # MoveIt SRDF, kinematics, controllers
│   │   └── config/
│   │       ├── jackal_ar4.srdf
│   │       ├── ros2_controllers.yaml
│   │       ├── moveit_controllers.yaml
│   │       └── kinematics.yaml
│   ├── jackal_ar4_navigation/        # Nav2 config, maps, navigation launch
│   │   ├── config/nav2_params.yaml
│   │   ├── maps/
│   │   └── launch/navigation.launch.py
│   ├── jackal_ar4_goals/             # Python scripts for sending arm/nav goals
│   └── worlds/                       # Gazebo SDF world files
│       ├── empty.sdf
│       ├── empty_room.sdf
│       ├── room_with_walls.sdf
│       ├── room_with_walls_star.sdf
│       ├── turtlebot_arena.sdf
│       └── tpr.sdf
├── Dockerfile
├── docker-compose.yml
└── README.md
```

---

## Installation

### 1. Install ROS 2 Jazzy

Follow the official installation guide:
```bash
# Set locale
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# Add ROS 2 apt repository
sudo apt install -y software-properties-common curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Jazzy
sudo apt update
sudo apt install -y ros-jazzy-desktop
```

### 2. Install Gazebo Harmonic

```bash
sudo apt install -y ros-jazzy-ros-gz
```

### 3. Clone the Repository

```bash
mkdir -p ~/jackal_ar4_ws/src
cd ~/jackal_ar4_ws
git clone https://github.com/Oyefusi-Samuel/TPR.git src
```

### 4. Install Dependencies with rosdep

This is the single most important step — it installs all ROS and system dependencies automatically by scanning every `package.xml` in the workspace.

```bash
# Initialise rosdep (first time only)
sudo rosdep init
rosdep update

# Source ROS 2
source /opt/ros/jazzy/setup.bash

# Install all dependencies
cd ~/jackal_ar4_ws
rosdep install --from-paths src --ignore-src -r -y
```

> **Note:** `rosdep` handles everything including `joint_trajectory_controller`, `ros2_control`, `moveit`, `nav2`, `topic_tools`, and all Gazebo bridge packages. You should not need to install anything manually.

### 5. Build the Workspace

```bash
cd ~/jackal_ar4_ws
colcon build --symlink-install
source install/setup.bash
```

> **Tip:** Add `source ~/jackal_ar4_ws/install/setup.bash` to your `~/.bashrc` so you don't need to source it every terminal session.

```bash
echo "source ~/jackal_ar4_ws/install/setup.bash" >> ~/.bashrc
```

---

## Running the Simulation

### RViz Only (No Gazebo)

Use this to verify the robot model, MoveIt planning, and Nav2 are configured correctly before launching Gazebo. This uses `mock_components` hardware — joints respond instantly with no physics.

```bash
ros2 launch jackal_ar4_description jackal_ar4.launch
```

### Full Gazebo Simulation

```bash
ros2 launch jackal_ar4_description gazebo.launch.py
```

This launches Gazebo Harmonic, spawns the robot, brings up all controllers, Nav2, MoveIt, and RViz in one command. Wait approximately 7–10 seconds for all nodes to initialise.

### Choosing a World

Pass the world name (without `.sdf`) as an argument:

```bash
ros2 launch jackal_ar4_description gazebo.launch.py world:=tpr
ros2 launch jackal_ar4_description gazebo.launch.py world:=room_with_walls
ros2 launch jackal_ar4_description gazebo.launch.py world:=turtlebot_arena
```

To also suppress RViz (headless mode):

```bash
ros2 launch jackal_ar4_description gazebo.launch.py world:=tpr launch_rviz:=false
```

---

## Available Worlds

| World Name | Description |
|---|---|
| `empty` | Flat infinite ground plane (default) |
| `empty_room` | Enclosed empty room |
| `room_with_walls` | Room with wall obstacles |
| `room_with_walls_star` | Star-shaped room layout |
| `turtlebot_arena` | Standard TurtleBot benchmark arena |
| `tpr` | Custom TPR project environment |

To add your own world, drop the `.sdf` file into `src/worlds/` and rebuild.

---

## Verifying Everything Works

After launching, open a second terminal and run:

```bash
source ~/jackal_ar4_ws/install/setup.bash

# Check all three controllers are active
ros2 control list_controllers
```

Expected output:
```
joint_state_broadcaster  active
arm_controller           active
ar_gripper_controller    active
```

```bash
# Confirm joint states are being published
ros2 topic echo /joint_states --once

# Confirm Gazebo odometry is flowing
ros2 topic echo /odom --once

# Confirm TF tree is complete
ros2 run tf2_tools view_frames
```

---

## Sending Goals

### Navigation Goal (Nav2)

In RViz, use the **2D Goal Pose** button in the toolbar, then click anywhere on the map. The Jackal will plan and drive to that position using the MPPI controller.

Alternatively from the terminal:

```bash
ros2 run jackal_ar4_goals send_nav_goal.py
```

### Arm Goal (MoveIt)

In RViz, use the **Motion Planning** panel:
1. Drag the interactive marker to a target pose
2. Click **Plan**
3. Click **Execute**

The arm will move in both RViz and Gazebo simultaneously.

Alternatively from the terminal:

```bash
ros2 run jackal_ar4_goals send_arm_goal.py
```

---

## Architecture Overview

```
                    ┌─────────────────────────────────────────┐
                    │              Gazebo Harmonic             │
                    │  ┌──────────────┐  ┌──────────────────┐ │
                    │  │  diff_drive  │  │ JointStatePublish│ │
                    │  │   plugin     │  │ er (wheels)      │ │
                    │  └──────┬───────┘  └────────┬─────────┘ │
                    └─────────┼────────────────────┼───────────┘
                              │ /tf (odom→base)    │ /wheel_joint_states
                    ┌─────────▼────────────────────▼───────────┐
                    │           ros_gz_bridge                   │
                    └─────────┬────────────────────┬───────────┘
                              │                    │ topic_tools relay
                    ┌─────────▼──────────┐  ┌──────▼──────────────┐
                    │  robot_state_pub   │  │    /joint_states     │
                    │  (URDF → TF tree)  │  │  (arm + wheels)      │
                    └─────────┬──────────┘  └──────┬──────────────┘
                              │                    │
              ┌───────────────▼──────┐  ┌──────────▼──────────────┐
              │       Nav2           │  │   ros2_control_node      │
              │  (navigation stack)  │  │   arm_controller         │
              │  /cmd_vel → Gazebo   │  │   ar_gripper_controller  │
              └──────────────────────┘  └──────────┬──────────────┘
                                                   │
                                        ┌──────────▼──────────────┐
                                        │       MoveIt 2           │
                                        │   move_group + OMPL      │
                                        └─────────────────────────┘
```

---

## Package Breakdown

| Package | Purpose |
|---|---|
| `jackal_ar4_description` | Combined robot URDF and all launch files |
| `jackal_ar4_moveit_config` | MoveIt SRDF, OMPL config, controller mappings |
| `jackal_ar4_navigation` | Nav2 parameters, map, navigation launch |
| `jackal_ar4_goals` | Python scripts to send arm and navigation goals |
| `jackal_ar4_worlds` | Gazebo SDF world files |
| `ar4_description` | AR4 arm URDF macros and STL meshes |
| `clearpath_platform_description` | Jackal J100 URDF macros and meshes |

---

## Troubleshooting

**Controllers fail to load (`joint_trajectory_controller not found`)**
```bash
sudo apt install -y ros-jazzy-joint-trajectory-controller
```

**`ros2 control` command not found**
```bash
sudo apt install -y ros-jazzy-ros2controlcli
```

**Arm moves in RViz but not in Gazebo**

Ensure all three controllers are active:
```bash
ros2 control list_controllers
```
If `arm_controller` or `ar_gripper_controller` is missing, the spawner likely timed out. Relaunch.

**Robot bouncing in RViz**

This is a TF timestamp conflict. Ensure you are using the latest `gazebo.launch.py` — the `map_to_odom` static publisher must **not** have `use_sim_time=True`.

**`map` frame not found / Nav2 costmap timeout**

The `map_to_odom` static publisher must be running. Check:
```bash
ros2 node list | grep static_transform
```

**Meshes missing in Gazebo (grey boxes)**

The `GZ_SIM_RESOURCE_PATH` is not set correctly. The launch file sets this automatically — ensure you are using `gazebo.launch.py` and not launching `gz sim` manually.

**`rosdep install` fails on a package**

```bash
# Update rosdep database first
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

**Build fails with symlink install error on `jackal_ar4_worlds`**

Ensure your world `.sdf` files are directly inside `src/worlds/` with no subdirectory, and that `CMakeLists.txt` is also present in `src/worlds/`.

---

## Contributing

Contributions are welcome! Please follow these steps:

### Getting Started

1. Fork the repository on GitHub
2. Create a feature branch:
   ```bash
   git checkout -b feature/your-feature-name
   ```
3. Make your changes
4. Test your changes by running the full simulation:
   ```bash
   colcon build --symlink-install
   source install/setup.bash
   ros2 launch jackal_ar4_description gazebo.launch.py
   ```
5. Commit with a clear message:
   ```bash
   git commit -m "feat: add description of what you changed"
   ```
6. Push and open a Pull Request against the `main` branch

### Contribution Guidelines

- **URDF/XACRO changes** — always verify with `check_urdf` and test in both `jackal_ar4.launch` (RViz) and `gazebo.launch.py` before submitting
- **New worlds** — drop `.sdf` files into `src/worlds/` and add an entry to the Available Worlds table in this README
- **New controllers** — add to `ros2_controllers.yaml` and `moveit_controllers.yaml`, and update the spawner in `gazebo.launch.py`
- **Navigation tuning** — update `nav2_params.yaml` and document what was changed and why
- **Python scripts** — follow PEP 8, add a docstring, and place them in `jackal_ar4_goals/`
- **Dependencies** — if you add a new ROS package dependency, add it to the appropriate `package.xml`. Do not add manual `apt install` instructions — use `rosdep` so it stays reproducible

### Code Style

- Python: PEP 8
- C++: follow `ament_clang_format` (`.clang-format` in `src/ar4/`)
- URDF/XACRO: 2-space indentation, descriptive link and joint names
- Commit messages: use [Conventional Commits](https://www.conventionalcommits.org/) format (`feat:`, `fix:`, `docs:`, `refactor:`)

### Reporting Issues

Open a GitHub Issue with:
- Your Ubuntu and ROS 2 version
- The full launch command you ran
- The relevant terminal error output
- Output of `ros2 control list_controllers` and `ros2 run tf2_tools view_frames` if it's a TF or controller issue

---

## License

This project is licensed under the MIT License. See [LICENSE](LICENSE) for details.

Third-party packages included in this workspace retain their original licenses:
- `ar4_description` / `ar4_moveit_config` — see `src/ar4/ar4_description/LICENSE`
- `clearpath_common` — see `src/clearpath_common/LICENSE`
