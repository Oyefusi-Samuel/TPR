# Jackal AR4 Simulation Workspace

A ROS 2 Jazzy simulation of the **Clearpath Jackal J100** mobile robot equipped with an **AR4 6-DOF robotic arm**, a **360° 2D LiDAR**, and a custom **A\* Smooth Planner** for autonomous navigation. Runs in **Gazebo Harmonic** with **SLAM Toolbox** mapping, **AMCL** localisation, **MoveIt 2** arm control, and **Nav2**.

After building, one line launch with: 
`ros2 launch goal_manager_pkg tpr_full.launch.py`

---

## Table of Contents

- [Overview](#overview)
- [System Requirements](#system-requirements)
- [Repository Structure](#repository-structure)
- [Installation](#installation)
- [Quick Start](#quick-start)
- [Running the Simulation](#running-the-simulation)
- [Available Worlds](#available-worlds)
- [Architecture Overview](#architecture-overview)
- [Package Breakdown](#package-breakdown)
- [Troubleshooting](#troubleshooting)
- [Contributing](#contributing)
- [License](#license)

---

## Overview

This workspace provides a complete mobile manipulation and autonomous navigation simulation:

| Component | Technology |
|---|---|
| Physics simulation | Gazebo Harmonic |
| Mobile base | Clearpath Jackal J100 (differential drive) |
| Manipulator | Annin Robotics AR4 6-DOF arm + gripper |
| Sensor | 360° GPU LiDAR → `/lidar/scan` |
| Mapping | SLAM Toolbox (online async) |
| Localisation | AMCL |
| Navigation | A\* Smooth Planner + Pure Pursuit controller |
| Arm planning | MoveIt 2 + OMPL |
| Hardware interface | ros2_control (gz_ros2_control + mock_components) |

---

## System Requirements

| Requirement | Version |
|---|---|
| Ubuntu | 24.04 (Noble) |
| ROS 2 | Jazzy |
| Gazebo | Harmonic |
| Python | 3.12+ |
| RAM | 8 GB minimum, 16 GB recommended |
| GPU | Optional but recommended |

---

## Repository Structure

```
jackal_ar4_ws/src/
├── jackal_ar4_description/        ← Main robot package
│   ├── urdf/
│   │   ├── jackal_ar4.urdf.xacro          # Robot URDF + LiDAR + Gazebo plugins
│   │   ├── jackal_ar4.ros2_control.xacro  # Hardware interface config
│   │   └── ar4_arm.urdf.xacro             # AR4 arm macro
│   └── launch/
│       ├── gazebo.launch.py      # Full simulation (Gazebo + controllers + Nav2 + RViz)
│       └── jackal_ar4.launch     # RViz-only (no Gazebo, for arm testing)
│
├── jackal_ar4_navigation/         ← Navigation stack
│   ├── config/
│   │   ├── nav2_params.yaml               # Nav2 costmap + controller params
│   │   ├── slam_toolbox_mapping_params.yaml
│   │   ├── mapping.rviz                   # RViz for SLAM mapping
│   │   └── navigation.rviz                # RViz for A* navigation
│   ├── maps/
│   │   ├── tpr_map.yaml                   # Saved map (metadata)
│   │   └── tpr_map.pgm                    # Saved map (image)
│   ├── launch/
│   │   ├── mapping.launch.py              # SLAM-only launch
│   │   └── astar_navigation.launch.py     # A* navigation launch
│   └── scripts/
│       └── twist_unstamper.py             # Converts TwistStamped → Twist
│
├── jackal_ar4_moveit_config/      ← MoveIt 2 configuration
├── jackal_ar4_worlds/             ← Gazebo SDF worlds
├── jackal_ar4_goals/              ← Goal-sending scripts
├── a_star_smooth_planner/         ← A* planner + smoother + pure pursuit
├── ar4/                           ← AR4 arm description + MoveIt
└── clearpath_common/              ← Jackal J100 base description
```

---

## Installation

### 1. Install ROS 2 Jazzy + Gazebo Harmonic

```bash
# ROS 2 Jazzy
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
sudo apt install -y software-properties-common curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update && sudo apt install -y ros-jazzy-desktop

# Gazebo + navigation packages
sudo apt install -y ros-jazzy-ros-gz
sudo apt install -y ros-jazzy-slam-toolbox
sudo apt install -y ros-jazzy-nav2-bringup
sudo apt install -y ros-jazzy-topic-tools
sudo apt install -y ros-jazzy-teleop-twist-keyboard
```

### 2. Clone and Build

```bash
mkdir -p ~/tpr/jackal_ar4_ws/src
cd ~/tpr/jackal_ar4_ws
git clone https://github.com/Oyefusi-Samuel/TPR.git src/

source /opt/ros/jazzy/setup.bash
sudo rosdep init      # first time only
rosdep update
rosdep install --from-paths src --ignore-src -r -y

colcon build --symlink-install
source install/setup.bash
```

Add to `~/.bashrc`:
```bash
echo "source ~/tpr/jackal_ar4_ws/install/setup.bash" >> ~/.bashrc
```

---

## Quick Start

```bash
# T1 — Launch simulation
ros2 launch jackal_ar4_description gazebo.launch.py world:=tpr

# T2 — Drive with keyboard
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel

# T3 — Map the world (while sim is running)
ros2 launch jackal_ar4_navigation mapping.launch.py
# Drive around to build the map, then save:
ros2 run nav2_map_server map_saver_cli -f ~/tpr/jackal_ar4_ws/src/jackal_ar4_navigation/maps/tpr_map
cd ~/tpr/jackal_ar4_ws && colcon build --symlink-install && source install/setup.bash

# T3 — Navigate with A* (after mapping, with sim running)
ros2 launch jackal_ar4_navigation astar_navigation.launch.py
# Use '2D Pose Estimate' then '2D Goal Pose' in RViz
```

---

## Running the Simulation

### Step 1 — Full Gazebo Simulation

```bash
ros2 launch jackal_ar4_description gazebo.launch.py world:=tpr
```

Wait ~15–20 s for all nodes to start. Starts: Gazebo, robot, LiDAR, bridges, arm controllers, Nav2, MoveIt, RViz.

```bash
# Other world options
ros2 launch jackal_ar4_description gazebo.launch.py world:=room_with_walls
ros2 launch jackal_ar4_description gazebo.launch.py world:=tpr launch_rviz:=false
```

### Step 2 — Teleop

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel
```

Press `x` several times to reduce speed before driving. Use `i`/`,` to move forward/backward, `j`/`l` to turn.

### Step 3 — SLAM Mapping

Run **after** the simulation is fully started:

```bash
ros2 launch jackal_ar4_navigation mapping.launch.py
```

RViz opens showing the map being built live. Drive around to cover the entire environment. When done:

```bash
# Save the map
ros2 run nav2_map_server map_saver_cli \
  -f ~/tpr/jackal_ar4_ws/src/jackal_ar4_navigation/maps/tpr_map

# Verify the saved yaml has a relative image path
cat ~/tpr/jackal_ar4_ws/src/jackal_ar4_navigation/maps/tpr_map.yaml
# Should say:  image: tpr_map.pgm   (NOT an absolute path)

# Rebuild to install the map
cd ~/tpr/jackal_ar4_ws && colcon build --symlink-install && source install/setup.bash
```

> **Critical:** If the saved map looks completely blank (all white with no walls), SLAM didn't capture wall data. This happens when the robot barely moves or the LiDAR data wasn't flowing yet. Remap: kill the mapping launch, relaunch it after the sim is fully up, drive slowly around all walls, then save again.

### Step 4 — A\* Navigation

Run **after** mapping is done and the simulation is running:

```bash
ros2 launch jackal_ar4_navigation astar_navigation.launch.py
```

RViz opens with: saved map (white=free, black=walls), LiDAR scan (red), AMCL particles, costmap, A\* path (orange), smoothed path (green).

**To navigate:**
1. Click **2D Pose Estimate** → click on the map where the robot actually is → drag to set heading
2. Wait for AMCL particles to converge around the robot (a few seconds)
3. Click **2D Goal Pose** → click anywhere on white (free) space
4. The robot plans and drives autonomously

### Step 5 — MoveIt Arm Control

In the RViz window launched by `gazebo.launch.py`:
1. Open the **Motion Planning** panel
2. Drag the interactive end-effector marker to a target pose
3. Click **Plan** then **Execute**

The arm moves in both RViz and Gazebo simultaneously.

---

## Available Worlds

| World | Description |
|---|---|
| `empty` | Flat ground plane |
| `empty_room` | Enclosed empty room |
| `room_with_walls` | Room with obstacles |
| `room_with_walls_star` | Star-shaped room |
| `turtlebot_arena` | Standard benchmark arena |
| `tpr` | Custom TPR environment |

---

## Architecture Overview

```
Gazebo Harmonic
  ├── diff_drive plugin     → /tf (odom→base_link), /odom
  ├── JointStatePublisher   → /wheel_joint_states
  └── gpu_lidar sensor      → /lidar/scan
         │
         ▼
  ros_gz_bridge             → ROS topics: /clock /tf /cmd_vel /odom /lidar/scan
         │
         ├── topic_tools relay  (/wheel_joint_states → /joint_states)
         ├── topic_tools relay  (/cmd_vel_smoothed → /cmd_vel)
         │
         ├── robot_state_publisher  → TF tree (URDF joints)
         │
         ├── ros2_control_node
         │     └── arm_controller + ar_gripper_controller
         │           └── MoveIt 2 move_group
         │
         └── Nav2 stack
               ├── velocity_smoother  (/cmd_vel → /cmd_vel_smoothed)
               └── controller_server  → /cmd_vel

Navigation (A* mode):
  /lidar/scan ──► SLAM Toolbox ──► tpr_map.yaml (offline)
  /lidar/scan ──► AMCL ──────────► /amcl_pose
  /lidar/scan ──► Costmap ────────► /costmap
  /goal_pose  ──► A* Planner ─────► /a_star/path
                  ↓
               A* Smoother ──────► /a_star/path/smooth
                  ↓
               Pure Pursuit ──────► /cmd_vel_stamped
                  ↓
               twist_unstamper ───► /cmd_vel ──► Gazebo diff_drive
```

---

## Package Breakdown

| Package | Role |
|---|---|
| `jackal_ar4_description` | Robot URDF, LiDAR, Gazebo plugins, all launch files |
| `jackal_ar4_navigation` | Nav2 params, SLAM params, mapping/navigation launches, RViz configs |
| `jackal_ar4_moveit_config` | MoveIt SRDF, kinematics, OMPL config, controller mappings |
| `jackal_ar4_goals` | Python scripts for sending arm and nav goals programmatically |
| `jackal_ar4_worlds` | Gazebo SDF worlds (tpr, room_with_walls, etc.) |
| `a_star_smooth_planner` | A\* planner (Python), path smoother (C++), pure pursuit (C++) |
| `ar4_description` | AR4 arm URDF macros + STL meshes |
| `clearpath_platform_description` | Jackal J100 base URDF macros + meshes |

---

## Troubleshooting

### Simulation

| Problem | Fix |
|---|---|
| Controllers not active | Relaunch — spawners start at t=10–11s and may race on slow machines |
| Teleop not driving robot | Verify with `ros2 topic echo /cmd_vel --once`. The cmd_vel_relay must be running |
| Robot bouncing in RViz | `map_to_odom` static publisher must NOT have `use_sim_time=True` — already fixed |
| Meshes missing in Gazebo | Only use `gazebo.launch.py` — it sets `GZ_SIM_RESOURCE_PATH` automatically |

### LiDAR

| Problem | Fix |
|---|---|
| `/lidar/scan` not publishing | Check bridge: `ros2 topic hz /lidar/scan` |
| No LiDAR in RViz | Add → LaserScan → `/lidar/scan`, Fixed Frame = `lidar` or `map` |

### Mapping

| Problem | Fix |
|---|---|
| SLAM exits immediately | `use_lifecycle_manager: false` must be in `slam_toolbox_mapping_params.yaml` |
| Map blank after saving | Remap — the robot didn't move enough or LiDAR wasn't flowing. Drive all walls |
| Map not visible in RViz | Map display must use `Durability Policy: Transient Local` — `mapping.rviz` already has this |

### A\* Navigation

| Problem | Fix |
|---|---|
| `Failed to change state for node: map_server` | Race condition — lifecycle_manager now has 3s delay. If still fails, increase delay |
| `No map received!` | Either map is blank (remap) or obstacle_layer scan topic is wrong (`/lidar/scan` not `/scan`) |
| Map shows with no walls | Map was saved blank — remap the TPR world |
| AMCL won't converge | Use 2D Pose Estimate to give the robot its initial position on the map |
| GLSL shader error in RViz | Use `Binary representation: true` + `Color Scheme: map` — already set in `navigation.rviz` |

### Build

```bash
# Dependencies missing
sudo apt install -y ros-jazzy-slam-toolbox ros-jazzy-nav2-bringup ros-jazzy-topic-tools
rosdep install --from-paths src --ignore-src -r -y

# Always rebuild after changing config files
cd ~/tpr/jackal_ar4_ws && colcon build --symlink-install && source install/setup.bash
```

---

## Contributing

1. Fork → branch → change → test → PR
2. Test with: `ros2 launch jackal_ar4_description gazebo.launch.py`
3. Commit style: [Conventional Commits](https://www.conventionalcommits.org/) (`feat:`, `fix:`, `docs:`)
4. New worlds → add `.sdf` to `jackal_ar4_worlds/worlds/`
5. New dependencies → add to `package.xml`, never raw `apt install`

---

## License

MIT License — see [LICENSE](LICENSE).

Third-party components retain their original licenses:
- `ar4_description` / `ar4_moveit_config` → `src/ar4/ar4_description/LICENSE`
- `clearpath_common` → `src/clearpath_common/LICENSE`
- `a_star_smooth_planner` → `src/a_star_smooth_planner/LICENSE`
