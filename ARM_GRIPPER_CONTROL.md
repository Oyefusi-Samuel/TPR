# Arm and Gripper Control: Development Notes

This document records the difficulties encountered integrating the AR4 arm and gripper into Gazebo Harmonic via `gz_ros2_control`, the intermediate steps taken to diagnose each problem, and the final configuration that resolves all issues.

---

## Finished Configuration (Summary)

**Arm.** The AR4 arm is controlled by a `joint_trajectory_controller/JointTrajectoryController` (`arm_controller`) operating on six revolute joints (`ar4_joint_1` through `ar4_joint_6`) via position command interfaces. The arm is bridged to Gazebo physics through a single `gz_ros2_control/GazeboSimSystem` plugin instance (`JackalAR4Hardware`) with `position_proportional_gain=2.0`, which converts position error into joint velocity commands at each 100 Hz control cycle. MoveIt 2 plans trajectories using OMPL and dispatches them to the JTC, which linearly interpolates waypoints so the K_p drive receives a changing command every cycle and remains active throughout motion.

**Gripper.** The gripper's two prismatic jaws (`ar4_gripper_jaw1_joint`, `ar4_gripper_jaw2_joint`) are registered in the same `JackalAR4Hardware` block as the arm joints and driven by a dedicated `joint_trajectory_controller/JointTrajectoryController` (`ar_gripper_controller`). The gripper JTC is configured with `goal_time: 0.0` (fire-and-forget), meaning it returns success as soon as the trajectory clock expires regardless of final jaw position — this prevents a jaw stalled by a grasped object from ever failing the action. Both jaw joints carry a ±2 mm buffer on their hard stops (`lower=-0.002 m`, `upper=0.016 m`) so the nominal operating positions (closed = 0.0 m, open = 0.014 m) are never coincident with an ODE joint limit constraint.

---

## Difficulties and Fixes

### 1. Two GazeboSimSystem Instances: Joint Ownership Race

**Problem.** The original URDF registered the arm joints in one `ros2_control` block (`JackalAR4Hardware`) and the gripper jaws in a separate block (`GripperHardware`), each loading its own `gz_ros2_control/GazeboSimSystem` plugin. Gazebo Harmonic's GazeboSimSystem plugin scans all joints on the model non-deterministically at startup. With two instances running on the same model, each scan could claim jaw1 or jaw2 in any order. The result was that whichever instance "stole" a jaw from the other left that jaw with no active command interface, permanently freezing it. The failure alternated between jaws across sim restarts, making it appear random.

**Fix.** Removed the `GripperHardware` `ros2_control` block entirely. Added `ar4_gripper_jaw1_joint` and `ar4_gripper_jaw2_joint` directly into the `JackalAR4Hardware` block alongside the arm joints. A single plugin instance now owns all actuated joints on the model with no race condition.

---

### 2. ForwardCommandController Did Not Drive Gripper Joints

**Problem.** After unifying the hardware block, the gripper was still controlled by a `forward_command_controller/ForwardCommandController` (FCC) publishing `Float64MultiArray` position commands. The jaws remained motionless even when a correct command (e.g., `[0.014, 0.014]`) was continuously published at 20 Hz. Confirmed over 21 simulation-seconds — neither jaw moved.

**Root cause.** `GazeboSimSystem` drives a position-command joint by computing:

```
error       = (joint_position - joint_position_cmd) × update_rate
target_vel  = −K_p × error
```

This velocity drive is applied **only when the command interface value changes between control cycles**. The FCC writes a single constant value to the command buffer and holds it indefinitely. From GazeboSimSystem's perspective the delta between cycles is zero after the first tick, so no velocity is ever generated and the joint stays frozen.

**Fix.** Replaced the FCC with a `joint_trajectory_controller/JointTrajectoryController`. The JTC interpolates a single-point trajectory and writes a linearly changing command to the interface at every 100 Hz control cycle. GazeboSimSystem sees a non-zero delta every cycle and keeps the K_p drive active throughout the full motion.

**Accompanying changes:**
- `ros2_controllers.yaml`: changed `ar_gripper_controller` type from `ForwardCommandController` to `JointTrajectoryController`; added full JTC parameter block with `goal_time: 0.0`.
- `pick_and_place.py`: replaced `Float64MultiArray` publisher with a `control_msgs/action/FollowJointTrajectory` ActionClient; rewrote `_set_gripper()` to build and send a single-point JTC trajectory.

---

### 3. Jaw1 Locked at Spawn by ODE Lower-Limit Constraint

**Problem.** After switching to the JTC, jaw2 responded correctly to open/close commands but jaw1 remained stuck at position ≈ 0.0, immovable until the arm itself physically jostled it. Observed via `ros2 topic echo /joint_states`:

```
jaw1: -9.051746e-15    (slightly below 0)
jaw2:  0.0             (exactly at 0)
```

**Root cause.** The joint's `lower` limit was `0`. The Gazebo ODE physics engine applies a hard constraint whenever a joint position is at or below its lower limit: it clamps the joint and resists **any** velocity applied to it — including velocity in the correct opening (position-increasing) direction. Jaw1 consistently spawned at ~−9×10⁻¹⁵ due to floating-point noise in ODE's spawn state initialization, placing it fractionally below `lower=0`. The constraint activated immediately and locked jaw1 for the lifetime of the simulation until external force moved it above the limit.

**Fix.** Changed `lower="0"` to `lower="-0.002"` for both jaw joints in `ar_gripper_macro.xacro`. The nominal closed position (0.0 m) is now 2 mm above the hard stop. The ODE constraint is never active at spawn time regardless of floating-point noise.

---

### 4. Jaw2 Locked at Upper Limit After Opening

**Problem.** With the lower-limit fix in place, Step 1 (open) succeeded — both jaws reached 0.0134 m. However, after the arm moved in steps 2–3, the jaws were reported at 0.0140 m entering Step 4 (close). Jaw1 closed normally, but jaw2 remained at 0.0140 for the entire close duration, producing a settle-timeout warning.

**Root cause.** Arm vibration during trajectory execution physically nudged jaw2 fractionally past the upper limit of 0.014 m (e.g., 0.01401 m). The same ODE constraint mechanism that locked jaw1 at the lower limit now locked jaw2 at the upper limit, resisting the K_p velocity drive in the closing (position-decreasing) direction. Jaw1 happened to land at ~0.01399 m (just below the limit) and was unaffected.

**Fix.** Changed `upper="0.014"` to `upper="0.016"` for both jaw joints. The nominal open position (0.014 m) is now 2 mm below the hard stop. Arm-motion vibration cannot push either jaw above the new upper limit under normal operation, and the ODE constraint no longer activates after an open command.

---

## File Changes Reference

| File | Change |
|---|---|
| `src/ar4/ar4_description/urdf/gripper/ar_gripper_macro.xacro` | `lower` 0 → −0.002; `upper` 0.014 → 0.016 for both jaw joints |
| `src/jackal_ar4_description/urdf/jackal_ar4.ros2_control.xacro` | Removed `GripperHardware` block; merged jaw joints into `JackalAR4Hardware` |
| `src/jackal_ar4_moveit_config/config/ros2_controllers.yaml` | `ar_gripper_controller` type: FCC → JTC; added JTC config with `goal_time: 0.0` |
| `src/jackal_ar4_goals/jackal_ar4_goals/pick_and_place.py` | Replaced FCC publisher with `FollowJointTrajectory` ActionClient; rewrote `_set_gripper()` |
