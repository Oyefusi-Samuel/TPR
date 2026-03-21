# Arm / Gripper → Gazebo Bridge Implementation

## Problem

The AR4 arm and gripper had no physical connection to the Gazebo simulation.
Commands sent from MoveIt/RViz were accepted by a `mock_components/GenericSystem`
hardware plugin that simply echoed commands back as states — no Gazebo joint
physics were ever driven. Arm trajectories executed visually in RViz but the
Gazebo model did not move.

Root causes:
1. `jackal_ar4.ros2_control.xacro` always used `mock_components/GenericSystem`,
   even when `is_sim:=true`.
2. `urdf/jackal_ar4.urdf.xacro` was missing the `gz_ros2_control` Gazebo plugin
   and the `gazebo_controllers` xacro arg needed to activate it.
3. `gazebo.launch.py` launched a standalone `ros2_control_node` which won the
   controller_manager race against the Gazebo-embedded one, leaving Gazebo's
   joint actuators unclaimed.

---

## Changes

### 1. `src/jackal_ar4_description/urdf/jackal_ar4.ros2_control.xacro`

**What changed:** Gated the hardware plugin on `is_sim`.

| Before | After |
|--------|-------|
| Always `mock_components/GenericSystem` | `gz_ros2_control/GazeboSimSystem` when `is_sim:=true`; `mock_components/GenericSystem` otherwise |

`gz_ros2_control/GazeboSimSystem` claims the arm and gripper joints from
Gazebo's physics engine and wires them to the ros2_control position interfaces,
so `JointTrajectoryController` commands now drive real Gazebo joints.

### 2. `src/jackal_ar4_description/urdf/jackal_ar4.urdf.xacro`

**What changed:** Added two new declarations inside the existing `is_sim` block.

- Added `gazebo_controllers` xacro arg (defaults to the installed
  `ros2_controllers.yaml` from `jackal_ar4_moveit_config`).
- Added `namespace` xacro arg.
- Added the `gz_ros2_control::GazeboSimROS2ControlPlugin` Gazebo plugin block
  (already present in `config/jackal_ar4.urdf.xacro` but absent from the URDF
  file actually used by the launch).

The plugin is conditional on `is_sim:=true` and takes the `gazebo_controllers`
path so Gazebo boots the embedded controller_manager with the correct config.

### 3. `src/jackal_ar4_description/launch/gazebo.launch.py`

**What changed:** Three edits.

1. **Pass `gazebo_controllers` to xacro** — the resolved absolute path to
   `ros2_controllers.yaml` is now forwarded as a xacro argument so the
   gz_ros2_control plugin can find its config at runtime.

2. **Removed standalone `ros2_control_node`** — the gz_ros2_control Gazebo
   plugin hosts its own controller_manager internally. The standalone node was
   winning the CM service race and preventing the plugin's CM from ever
   activating, leaving Gazebo joints unclaimed.

3. **Added `--controller-manager-timeout 30` to all spawners** — allows
   spawners to retry while the plugin's CM finishes initialising after the robot
   is spawned into complex worlds (tpr, room_with_walls, etc.).

---

## Data Flow After Fix

```
MoveIt (RViz)
    │  FollowJointTrajectory action
    ▼
move_group
    │  trajectory goal
    ▼
arm_controller / ar_gripper_controller  (JointTrajectoryController)
    │  position commands via ros2_control interfaces
    ▼
gz_ros2_control/GazeboSimSystem  ◄── NEW: replaces mock_components
    │  joint effort/position applied to Gazebo physics
    ▼
Gazebo Harmonic  (arm and gripper joints move in simulation)
    │  joint states fed back through GazeboSimSystem
    ▼
joint_state_broadcaster  →  /joint_states  →  RViz visualisation
```

---

### 4. `src/jackal_ar4_description/launch/gazebo.launch.py` — fix 2 (bringup crash)

**What changed:** Added `GZ_SIM_SYSTEM_PLUGIN_PATH` to the environment passed to
the Gazebo subprocess.

`libgz_ros2_control-system.so` is installed at `/opt/ros/jazzy/lib` by
`ros-jazzy-gz-ros2-control`, but Gazebo's plugin loader searches
`GZ_SIM_SYSTEM_PLUGIN_PATH` — not the generic library path. ROS 2's `setup.bash`
sets this variable in interactive shells, but it is not reliably inherited by
the `ExecuteProcess` Gazebo subprocess in a launch context.

The path is derived at launch time via `get_package_prefix('gz_ros2_control')`
so it stays correct if the package is ever relocated, and any existing
`GZ_SIM_SYSTEM_PLUGIN_PATH` from the shell is preserved by appending to it.

---

## Rebuild Required

```bash
cd /home/jplombardi/ros2_ws/TPR
colcon build --symlink-install --packages-select jackal_ar4_description
source install/setup.bash
```

## Verification

After launch, confirm:

```bash
# All three controllers active, hardware: gz_ros2_control (not mock)
ros2 control list_hardware_interfaces

# Arm and gripper interfaces should show as "available" and "claimed"
# Expected prefix: JackalAR4Hardware — gz_ros2_control/GazeboSimSystem

# Spawner success check
ros2 control list_controllers
# Expected: joint_state_broadcaster [active], arm_controller [active], ar_gripper_controller [active]
```

In RViz, use the Motion Planning panel to plan and execute an arm trajectory —
the Gazebo arm should now follow in real time.
