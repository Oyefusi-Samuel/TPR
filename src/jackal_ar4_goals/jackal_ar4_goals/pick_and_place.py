#!/usr/bin/env python3
"""pick_and_place.py — Pick-and-place executive for the Jackal AR4 arm.

Coordinates are in the **ar4_base_link** frame (the arm's fixed base link,
mounted on the Jackal chassis).

CLI usage
---------
# XYZ pick, default rear-bin place:
  ros2 run jackal_ar4_goals pick_and_place 0.25 0.0 0.05

# Full 6-DOF pick (x y z qx qy qz qw), default place:
  ros2 run jackal_ar4_goals pick_and_place 0.25 0.0 0.05 0.0 1.0 0.0 0.0

# XYZ pick with explicit XYZ place:
  ros2 run jackal_ar4_goals pick_and_place 0.25 0.0 0.05 --place -0.35 0.0 0.20

# Full 6-DOF pick and explicit 6-DOF place:
  ros2 run jackal_ar4_goals pick_and_place 0.25 0.0 0.05 0.0 1.0 0.0 0.0 \\
      --place -0.35 0.0 0.20 0.0 1.0 0.0 0.0

Library usage (array iterator)
-------------------------------
  import rclpy
  from jackal_ar4_goals.pick_and_place import PickAndPlaceNode, pick_and_place_sequence

  rclpy.init()
  node = PickAndPlaceNode()

  picks = [(0.25, 0.0, 0.05), (0.20, 0.10, 0.05), (0.30, -0.10, 0.05)]
  results = pick_and_place_sequence(node, picks)

  node.destroy_node()
  rclpy.shutdown()

Approach motion
---------------
Before descending to the pick pose the arm moves to a waypoint APPROACH_DIST
metres above it along the gripper's approach axis (local +Z of the end-effector
reversed, so the arm comes in from above).  The same waypoint is used for
retreat after grasping.

Gripper orientation for XYZ-only input
---------------------------------------
When only (x, y, z) is provided the gripper is commanded to point straight down.
The quaternion GRASP_DOWN_QUAT encodes this in ar4_base_link frame.  If the
gripper appears sideways or upside-down, adjust that constant empirically using:
  ros2 run tf2_tools view_frames
  ros2 run tf2_ros tf2_echo ar4_base_link ar4_link_6

Default place location
----------------------
DEFAULT_PLACE positions the gripper over a bin at the rear of the Jackal with
the gripper pointing outward (-X of ar4_base_link).
Adjust to match the actual bin position once placed in the simulation.
"""

import argparse
import enum
import sys
import threading
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    BoundingVolume,
    Constraints,
    JointConstraint,
    MoveItErrorCodes,
    OrientationConstraint,
    PositionConstraint,
    WorkspaceParameters,
)
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


# ── Pick result classification ─────────────────────────────────────────────────

class PickResult(enum.Enum):
    """Return value of pick_and_place() and _move_arm().

    bool() is True only for SUCCESS, so existing code of the form
    ``if not node.pick_and_place(...):`` continues to work unchanged.
    """
    SUCCESS     = 'success'
    UNREACHABLE = 'unreachable'   # no IK solution or goal in collision
    FAILED      = 'failed'        # planning timeout or other transient error

    def __bool__(self):
        return self is PickResult.SUCCESS


# ── Arm / gripper configuration ────────────────────────────────────────────────

PLANNING_GROUP = 'ar4_arm'
EEF_LINK       = 'ar4_link_6'
PLANNING_FRAME = 'ar4_base_link'

GRIPPER_JOINTS  = ['ar4_gripper_jaw1_joint', 'ar4_gripper_jaw2_joint']
GRIPPER_JOINT   = GRIPPER_JOINTS[0]   # monitored for skip/settle logic
GRIPPER_JOINT2  = GRIPPER_JOINTS[1]
GRIPPER_OPEN    = 0.014   # metres — matches SRDF 'open'  state
GRIPPER_CLOSED  = 0.0     # metres — matches SRDF 'closed' state
GRIPPER_TOL     = 0.002   # skip command if already within this tolerance
# Trajectory duration sent to the gripper JTC.  JTC linearly interpolates
# from current position to target over this many SIM seconds.  At 0.2x
# realtime that is 5 wall-seconds.  Longer = slower jaws, shorter = faster.
# GazeboSimSystem needs the command to CHANGE each control cycle to apply
# its velocity drive — that is exactly what JTC's trajectory tracking does.
GRIPPER_TRAJ_SIM_SECS = 1.0

# Pre-grasp / post-grasp retreat distance along the approach axis.
APPROACH_DIST  = 0.10    # metres

# All six arm joints in order, and their home (all-zeros) positions.
# Used by the recovery path to escape an invalid start state.
ARM_JOINTS          = [
    'ar4_joint_1', 'ar4_joint_2', 'ar4_joint_3',
    'ar4_joint_4', 'ar4_joint_5', 'ar4_joint_6',
]
ARM_HOME_POSITIONS  = [0.0] * 6   # matches SRDF "home" group state

# Planning workspace bounds.
# These are specified in the PLANNING_FRAME (ar4_base_link) but MoveIt
# transforms them to the global planning-scene frame (odom/map) before use.
# That transform shifts the Z origin by the arm's mounting height (~0.38 m),
# so a tight Z floor in ar4_base_link frame maps to a higher absolute Z and
# can exclude valid negative-Z poses.
# Using a large symmetric box avoids that clipping while still suppressing
# the "planning volume not specified" warning from ValidateWorkspaceBounds.
WS_MIN = (-2.0, -2.0, -2.0)   # (x, y, z) metres — large enough to never clip
WS_MAX = ( 2.0,  2.0,  2.0)   # (x, y, z) metres

# ── Gripper-down orientation ───────────────────────────────────────────────────
# Desired orientation of ar4_link_6 so the gripper faces straight down
# (-Z of ar4_base_link frame).
#
# The AR4's joint_6 has rpy="0 -1.5708 0" in the URDF, meaning the link_6
# Z axis (approach direction) points outward from the wrist at home pose.
# A 180° rotation about X maps that Z axis to point downward:
#   q = (w=0, x=1, y=0, z=0)
#
# If the gripper orientation looks wrong in practice, verify with:
#   ros2 run tf2_ros tf2_echo ar4_base_link ar4_link_6
# and update this constant to match the desired downward pose.
# Verified empirically via:
#   ros2 run tf2_ros tf2_echo ar4_base_link ar4_link_6
# At home (all joints = 0) the gripper points straight down.
# tf2_echo reported (xyzw): [0.707, 0.707, 0.000, 0.000]
# Rotation matrix at home:
#   [ 0  1  0 ]   local X → world +Y
#   [ 1  0  0 ]   local Y → world +X
#   [ 0  0 -1 ]   local Z (approach axis) → world -Z  ✓ down
GRASP_DOWN_QUAT = dict(w=0.0, x=0.707, y=0.707, z=0.0)

# Outward orientation: gripper horizontal, approach axis pointing in world -X
# (away from the robot front, into the rear bin).
# Rotation matrix with local Z → world -X:
#   [  0   0  -1 ]
#   [ -1   0   0 ]
#   [  0   1   0 ]
# Quaternion solved from that matrix (xyzw): [0.5, -0.5, -0.5, 0.5]
GRASP_OUTWARD_QUAT = dict(w=0.5, x=0.5, y=-0.5, z=-0.5)

# ── Default place pose (rear bin) ──────────────────────────────────────────────
# Full 7-tuple (x, y, z, qx, qy, qz, qw) in ar4_base_link frame.
# Position: 30 cm behind, 5 cm left, 20 cm up from the arm base.
# Orientation: gripper horizontal, pointing outward from the robot base (-X).
DEFAULT_PLACE = (
    -0.3, 0.05, 0.2,
    GRASP_OUTWARD_QUAT['x'],   # 0.5
    GRASP_OUTWARD_QUAT['y'],   # -0.5
    GRASP_OUTWARD_QUAT['z'],   # -0.5
    GRASP_OUTWARD_QUAT['w'],   # 0.5
)

# ── MoveIt error-code classification ──────────────────────────────────────────
_WORKSPACE_ERRORS = frozenset({
    MoveItErrorCodes.NO_IK_SOLUTION,
    MoveItErrorCodes.GOAL_IN_COLLISION,
    MoveItErrorCodes.START_STATE_IN_COLLISION,
    MoveItErrorCodes.INVALID_GOAL_CONSTRAINTS,
    MoveItErrorCodes.GOAL_VIOLATES_PATH_CONSTRAINTS,
    MoveItErrorCodes.GOAL_CONSTRAINTS_VIOLATED,
})
_PLANNING_ERRORS = frozenset({
    MoveItErrorCodes.PLANNING_FAILED,
    MoveItErrorCodes.INVALID_MOTION_PLAN,
    MoveItErrorCodes.MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE,
    MoveItErrorCodes.TIMED_OUT,
})

# Synchronous poll interval
_POLL_INTERVAL = 0.02   # seconds


# ══════════════════════════════════════════════════════════════════════════════

class PickAndPlaceNode(Node):
    """Long-lived node that executes pick-and-place cycles.

    Stays alive between calls so that the MoveGroup connection overhead is
    paid only once.  Call pick_and_place() as many times as needed, or use
    pick_and_place_sequence() to drive it with a list of targets.
    """

    def __init__(self):
        super().__init__('pick_and_place')

        self._arm_client     = ActionClient(self, MoveGroup, 'move_action')
        # JointTrajectoryController for the gripper — same controller type as
        # the arm.  JTC sends linearly-interpolated trajectory points at 100 Hz,
        # which means GazeboSimSystem sees a CHANGING command value every cycle
        # and applies its K_p velocity drive each step.  A constant FCC command
        # is NOT enough — GazeboSimSystem only drives joints when the command
        # value changes between read/write cycles.
        self._gripper_client = ActionClient(
            self, FollowJointTrajectory,
            '/ar_gripper_controller/follow_joint_trajectory',
        )

        # Cache latest joint states so we can skip gripper commands when
        # the jaw is already at the target position, and to detect when all
        # joints have settled (velocity ≈ 0) before sending the next move.
        self._joint_positions  = {}
        self._joint_velocities = {}
        self._js_lock = threading.Lock()
        self.create_subscription(JointState, '/joint_states', self._js_cb, 10)

        # Spin in a background thread so _sync_action() can poll futures
        # from the main thread without deadlocking.
        self._executor = MultiThreadedExecutor()
        self._executor.add_node(self)
        self._spin_thread = threading.Thread(
            target=self._executor.spin, daemon=True,
        )
        self._spin_thread.start()

        self._wait_for_servers()

    # ── Public API ─────────────────────────────────────────────────────────────

    def pick_and_place(self, pick, place=None):
        """Execute one pick-and-place cycle.

        Parameters
        ----------
        pick  : tuple (x, y, z) or (x, y, z, qx, qy, qz, qw)
                  Target pick pose in ar4_base_link frame.
        place : tuple (x, y, z) or (x, y, z, qx, qy, qz, qw) or None
                  Target place pose.  None → default rear-bin pose.

        Returns
        -------
        bool  True on full success, False on any failure.
        """
        try:
            pick_pose  = _to_pose(pick)
            place_pose = _to_pose(place) if place is not None \
                         else _to_pose(DEFAULT_PLACE)
        except ValueError as exc:
            self.get_logger().error(f'Invalid pose argument: {exc}')
            return PickResult.FAILED

        pre_pick = _approach_waypoint(pick_pose, APPROACH_DIST)

        # ── 1. Ensure gripper is open ────────────────────────────────────────
        self.get_logger().info('Step 1/6 — Opening gripper')
        if not self._set_gripper(GRIPPER_OPEN):
            self.get_logger().error('Failed to open gripper — aborting')
            return PickResult.FAILED

        # ── 2. Pre-grasp approach ────────────────────────────────────────────
        self.get_logger().info(
            f'Step 2/6 — Pre-grasp approach  '
            f'({pre_pick.position.x:.3f}, '
            f'{pre_pick.position.y:.3f}, '
            f'{pre_pick.position.z:.3f})'
        )
        arm_result = self._move_arm(pre_pick)
        if not arm_result:
            return arm_result
        self._wait_for_settle()

        # ── 3. Descend to pick pose ──────────────────────────────────────────
        self.get_logger().info(
            f'Step 3/6 — Descending to pick  '
            f'({pick_pose.position.x:.3f}, '
            f'{pick_pose.position.y:.3f}, '
            f'{pick_pose.position.z:.3f})'
        )
        arm_result = self._move_arm(pick_pose)
        if not arm_result:
            return arm_result
        self._wait_for_settle()

        # ── 4. Close gripper ─────────────────────────────────────────────────
        self.get_logger().info('Step 4/6 — Closing gripper (grasping)')
        if not self._set_gripper(GRIPPER_CLOSED):
            self.get_logger().error('Failed to close gripper — aborting')
            return PickResult.FAILED

        # ── 5. Retreat along approach axis ───────────────────────────────────
        self.get_logger().info('Step 5/6 — Retreating from pick')
        if not self._move_arm(pre_pick):
            self.get_logger().warn('Retreat failed — attempting place anyway')
        self._wait_for_settle()

        # ── 6. Move to place and release ─────────────────────────────────────
        self.get_logger().info(
            f'Step 6/6 — Moving to place  '
            f'({place_pose.position.x:.3f}, '
            f'{place_pose.position.y:.3f}, '
            f'{place_pose.position.z:.3f})'
        )
        arm_result = self._move_arm(place_pose)
        if not arm_result:
            # Drop whatever is in the gripper rather than leaving the arm
            # in an unknown state holding trash.
            self._set_gripper(GRIPPER_OPEN)
            return arm_result

        self.get_logger().info('Releasing — opening gripper')
        self._set_gripper(GRIPPER_OPEN)

        self.get_logger().info('Pick-and-place cycle complete')
        return PickResult.SUCCESS

    # ── Internal: arm motion ───────────────────────────────────────────────────

    def _move_arm(self, pose: Pose, _retry: bool = False) -> PickResult:
        """Send a Cartesian pose goal to MoveGroup. Blocks until done.

        If MoveIt returns START_STATE_INVALID (a joint has overshot its limit)
        this method automatically recovers the arm to home via the direct
        joint-trajectory controller, then retries the goal once.
        """
        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name                    = PLANNING_GROUP
        req.num_planning_attempts           = 10
        req.allowed_planning_time           = 15.0
        req.pipeline_id                     = 'ompl'
        req.max_velocity_scaling_factor     = 0.8
        req.max_acceleration_scaling_factor = 0.8

        # Explicit workspace bounds suppress the "planning volume not specified"
        # warning and prevent ValidateWorkspaceBounds from clipping valid poses.
        ws = WorkspaceParameters()
        ws.header.frame_id  = PLANNING_FRAME
        ws.min_corner.x, ws.min_corner.y, ws.min_corner.z = WS_MIN
        ws.max_corner.x, ws.max_corner.y, ws.max_corner.z = WS_MAX
        req.workspace_parameters = ws

        # Position constraint — 1 cm tolerance box around target
        pos_c = PositionConstraint()
        pos_c.header.frame_id = PLANNING_FRAME
        pos_c.link_name       = EEF_LINK
        box = SolidPrimitive()
        box.type       = SolidPrimitive.BOX
        box.dimensions = [0.01, 0.01, 0.01]
        pos_c.constraint_region.primitives.append(box)
        pos_c.constraint_region.primitive_poses.append(pose)
        pos_c.weight = 1.0

        # Orientation constraint — 0.1 rad tolerance; z-axis tolerance wider
        # so the planner can spin the wrist freely about the approach axis.
        ori_c = OrientationConstraint()
        ori_c.header.frame_id            = PLANNING_FRAME
        ori_c.link_name                  = EEF_LINK
        ori_c.orientation                = pose.orientation
        ori_c.absolute_x_axis_tolerance  = 0.1
        ori_c.absolute_y_axis_tolerance  = 0.1
        ori_c.absolute_z_axis_tolerance  = 3.14159   # free wrist rotation
        ori_c.weight                     = 1.0

        c = Constraints()
        c.position_constraints.append(pos_c)
        c.orientation_constraints.append(ori_c)
        req.goal_constraints.append(c)

        result = self._sync_action(self._arm_client, goal, timeout_sec=60.0)
        if result is None:
            self.get_logger().error('MoveGroup action rejected or timed out')
            return PickResult.FAILED

        ec = result.result.error_code.val
        if ec == MoveItErrorCodes.SUCCESS:
            return PickResult.SUCCESS

        # START_STATE_INVALID (-26): a joint overshot its limit in simulation
        # and MoveIt will reject every subsequent plan.  Recover by driving the
        # arm to home via the raw controller, then retry this goal once.
        if ec == MoveItErrorCodes.START_STATE_INVALID and not _retry:
            if self._recover_to_home():
                return self._move_arm(pose, _retry=True)

        if ec in _WORKSPACE_ERRORS:
            self.get_logger().error(
                f'Pose is outside the arm workspace or in collision  '
                f'(MoveItErrorCode {ec}).  '
                f'Target was ({pose.position.x:.3f}, '
                f'{pose.position.y:.3f}, {pose.position.z:.3f}) '
                f'in {PLANNING_FRAME}.'
            )
            return PickResult.UNREACHABLE
        elif ec in _PLANNING_ERRORS:
            self.get_logger().error(
                f'Planner failed to find a path  (MoveItErrorCode {ec}).  '
                f'Consider increasing allowed_planning_time or '
                f'num_planning_attempts, or checking for scene obstacles.'
            )
            return PickResult.FAILED
        else:
            self.get_logger().error(f'MoveGroup returned error code {ec}')
            return PickResult.FAILED

    # ── Internal: start-state recovery ────────────────────────────────────────

    def _recover_to_home(self) -> bool:
        """Plan and execute a move to home (all joints = 0) via MoveGroup.

        Called when a joint has overshot its software limit and the previous
        planning call returned START_STATE_INVALID.  MoveGroup's
        CheckStartStateBounds adapter clamps the out-of-bounds start state
        to the nearest valid value before planning, so this call succeeds even
        when a joint is several radians past its limit (the move_group node
        must have start_state_max_bounds_error set large enough — 2.0 in our
        launch file covers any realistic Gazebo overshoot).

        Using MoveGroup (not a direct joint-trajectory) keeps the motion
        time-optimal and avoids the arm-controller timeout that occurs when
        Gazebo's physics hard-stop blocks a raw trajectory.
        """
        self.get_logger().warn(
            'START_STATE_INVALID — recovering to home via MoveGroup joint goal'
        )
        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name                    = PLANNING_GROUP
        req.num_planning_attempts         = 5
        req.allowed_planning_time         = 15.0
        req.pipeline_id                   = 'ompl'
        req.max_velocity_scaling_factor   = 0.4   # slower for a recovery move
        req.max_acceleration_scaling_factor = 0.4

        c = Constraints()
        for joint_name, position in zip(ARM_JOINTS, ARM_HOME_POSITIONS):
            jc = JointConstraint()
            jc.joint_name     = joint_name
            jc.position       = float(position)
            jc.tolerance_above = 0.05
            jc.tolerance_below = 0.05
            jc.weight         = 1.0
            c.joint_constraints.append(jc)
        req.goal_constraints.append(c)

        result = self._sync_action(self._arm_client, goal, timeout_sec=60.0)
        if result is None:
            self.get_logger().error('Recovery to home timed out')
            return False

        ec = result.result.error_code.val
        if ec != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(f'Recovery to home failed with code {ec}')
            return False

        self.get_logger().info('Recovery complete — arm is at home')
        return True

    # ── Internal: gripper ──────────────────────────────────────────────────────

    def _set_gripper(self, target_pos: float) -> bool:
        """Command both gripper jaws to target_pos (metres) via JTC.

        Why JTC and not ForwardCommandController:
          GazeboSimSystem's position_proportional_gain drive computes
          velocity = K_p * (cmd - state) and applies it each physics step.
          It only applies the drive when the command value CHANGES between
          cycles.  FCC holds a constant value → no delta → joint frozen.
          JTC interpolates a trajectory at 100 Hz, so the command value
          changes every control cycle → K_p drive is active every step.

        goal_time: 0.0 means fire-and-forget — the action returns SUCCESS as
        soon as the trajectory clock expires.  We then poll joint positions
        to detect when both jaws have physically arrived or stalled.

        Completion:
          - Success: both jaws within GRIPPER_TOL of target.
          - Stall: both jaws stopped before reaching target (object blocking
            a close command) — this is intentional, not a failure.
        """
        with self._js_lock:
            pos1 = self._joint_positions.get(GRIPPER_JOINT)
            pos2 = self._joint_positions.get(GRIPPER_JOINT2)

        # Only skip if BOTH jaws are already within tolerance.
        if (pos1 is not None and pos2 is not None
                and abs(pos1 - target_pos) < GRIPPER_TOL
                and abs(pos2 - target_pos) < GRIPPER_TOL):
            self.get_logger().info(
                f'Gripper already at jaw1={pos1:.4f} jaw2={pos2:.4f} m — skipping'
            )
            return True

        j1_str = f'{pos1:.4f}' if pos1 is not None else '?'
        j2_str = f'{pos2:.4f}' if pos2 is not None else '?'
        self.get_logger().info(
            f'Gripper → {target_pos:.4f} m  (jaw1={j1_str}  jaw2={j2_str})'
        )

        # Build a single-point trajectory.
        traj = JointTrajectory()
        traj.joint_names = list(GRIPPER_JOINTS)
        pt = JointTrajectoryPoint()
        pt.positions  = [float(target_pos), float(target_pos)]
        pt.velocities = [0.0, 0.0]
        # time_from_start in SIM seconds.  At RT≈0.2 this is ~5 wall-seconds.
        # JTC interpolates from current position to target over this window,
        # emitting a new command every 10 ms control cycle.
        secs = int(GRIPPER_TRAJ_SIM_SECS)
        nsecs = int((GRIPPER_TRAJ_SIM_SECS - secs) * 1e9)
        pt.time_from_start = Duration(sec=secs, nanosec=nsecs)
        traj.points = [pt]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        # No per-goal tolerances — goal_time:0.0 in YAML means fire-and-forget.

        # Send and wait for the action server to accept + execute the trajectory.
        # With goal_time=0 the result arrives when the trajectory clock expires.
        result = self._sync_action(
            self._gripper_client, goal,
            timeout_sec=GRIPPER_TRAJ_SIM_SECS / 0.1 + 10.0,  # generous wall budget
        )
        if result is None:
            self.get_logger().warn('Gripper JTC action timed out — polling anyway')

        # Poll until both jaws arrive at target or stall against an object.
        deadline      = time.monotonic() + 20.0
        prev1         = pos1 if pos1 is not None else 0.0
        prev2         = pos2 if pos2 is not None else 0.0
        motion_seen_1 = False
        motion_seen_2 = False
        stall_count   = 0

        while time.monotonic() < deadline:
            time.sleep(_POLL_INTERVAL)
            with self._js_lock:
                p1 = self._joint_positions.get(GRIPPER_JOINT,  prev1)
                p2 = self._joint_positions.get(GRIPPER_JOINT2, prev2)
                v1 = abs(self._joint_velocities.get(GRIPPER_JOINT,  0.0))
                v2 = abs(self._joint_velocities.get(GRIPPER_JOINT2, 0.0))

            if abs(p1 - prev1) > 5e-4:
                motion_seen_1 = True
            if abs(p2 - prev2) > 5e-4:
                motion_seen_2 = True

            # Success: both jaws at target.
            if (abs(p1 - target_pos) < GRIPPER_TOL
                    and abs(p2 - target_pos) < GRIPPER_TOL):
                self.get_logger().info(
                    f'Gripper reached {target_pos:.4f} m  '
                    f'jaw1={p1:.4f} jaw2={p2:.4f}'
                )
                return True

            # Stall: both jaws have moved AND are now stopped away from target.
            if motion_seen_1 and motion_seen_2 and v1 < 0.002 and v2 < 0.002:
                stall_count += 1
                if stall_count >= 3:
                    self.get_logger().info(
                        f'Gripper stalled at jaw1={p1:.4f} jaw2={p2:.4f} '
                        f'(target {target_pos:.4f}) — object or limit'
                    )
                    return True
            else:
                stall_count = 0

            prev1, prev2 = p1, p2

        self.get_logger().warn(
            f'Gripper settle timeout  jaw1={prev1:.4f} jaw2={prev2:.4f}'
        )
        return True  # timeout is not fatal — continue the sequence

    # ── Internal: joint state subscriber ──────────────────────────────────────

    def _js_cb(self, msg: JointState):
        with self._js_lock:
            for name, pos, vel in zip(msg.name, msg.position, msg.velocity):
                self._joint_positions[name]  = pos
                self._joint_velocities[name] = vel

    def _wait_for_settle(self,
                         joints=None,
                         vel_threshold: float = 0.01,
                         timeout_sec: float = 15.0) -> None:
        """Block until all arm joint velocities drop below vel_threshold.

        Replaces fixed time.sleep() calls between moves.  In practice joints
        settle within 100–300 ms after a trajectory ends; this method returns
        as soon as they do, so the next move starts with no unnecessary delay.

        Parameters
        ----------
        joints        : iterable of joint names to monitor (default: ARM_JOINTS)
        vel_threshold : max absolute velocity considered 'stopped' (rad/s)
        timeout_sec   : fall-through safety limit if joints never settle
        """
        if joints is None:
            joints = ARM_JOINTS
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            with self._js_lock:
                vels = [abs(self._joint_velocities.get(j, 1.0)) for j in joints]
            if all(v < vel_threshold for v in vels):
                return
            time.sleep(_POLL_INTERVAL)

    # ── Internal: synchronous action helper ───────────────────────────────────

    def _sync_action(self, client, goal, timeout_sec=30.0):
        """Send an action goal and block until the result arrives.

        The node's MultiThreadedExecutor drives all callbacks in the background
        thread, so polling futures here is safe and deadlock-free.

        Returns the action result, or None on rejection / timeout.
        """
        send_future = client.send_goal_async(goal)
        if not _poll(send_future, timeout_sec):
            self.get_logger().error('Timed out waiting for goal acceptance')
            return None

        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            return None

        result_future = goal_handle.get_result_async()
        if not _poll(result_future, timeout_sec):
            self.get_logger().error('Timed out waiting for action result')
            goal_handle.cancel_goal_async()
            return None

        return result_future.result()

    # ── Internal: server readiness ─────────────────────────────────────────────

    def _wait_for_servers(self, timeout_sec=20.0):
        self.get_logger().info('Waiting for MoveGroup action server ...')
        if not self._arm_client.wait_for_server(timeout_sec=timeout_sec):
            raise RuntimeError(
                'MoveGroup action server not available after '
                f'{timeout_sec:.0f} s — is move_group running?'
            )
        self.get_logger().info('Waiting for gripper JTC action server ...')
        if not self._gripper_client.wait_for_server(timeout_sec=timeout_sec):
            raise RuntimeError(
                'Gripper JTC action server not available after '
                f'{timeout_sec:.0f} s — is ar_gripper_controller running?'
            )
        self.get_logger().info('Action servers ready')


# ══════════════════════════════════════════════════════════════════════════════
# Module-level helpers
# ══════════════════════════════════════════════════════════════════════════════

def pick_and_place_sequence(node: PickAndPlaceNode, picks, place=None):
    """Drive a list of pick locations through a single live node.

    Parameters
    ----------
    node  : PickAndPlaceNode — already-initialised, long-lived node.
    picks : iterable of (x, y, z) or (x, y, z, qx, qy, qz, qw) tuples,
            or geometry_msgs.msg.Pose objects.
    place : optional place target in the same formats; None → default rear bin.

    Returns
    -------
    list[PickResult]  per-pick results.  bool(result) is True only for SUCCESS.

    Example
    -------
    picks = [(0.25, 0.0, 0.05), (0.20, 0.10, 0.08)]
    results = pick_and_place_sequence(node, picks)
    failed  = [p for p, r in zip(picks, results) if not r]
    """
    results = []
    picks = list(picks)
    for i, pick in enumerate(picks):
        node.get_logger().info(f'--- Sequence pick {i + 1}/{len(picks)} ---')
        results.append(node.pick_and_place(pick, place))
    return results


def _to_pose(value) -> Pose:
    """Convert a coordinate tuple or existing Pose to geometry_msgs.msg.Pose.

    Accepted formats
    ----------------
    (x, y, z)                  — XYZ, gripper pointing straight down
    (x, y, z, qx, qy, qz, qw) — full 6-DOF pose
    geometry_msgs.msg.Pose     — returned as-is
    """
    if isinstance(value, Pose):
        return value

    vals = tuple(value)

    if len(vals) == 3:
        x, y, z = vals
        p = Pose()
        p.position.x    = float(x)
        p.position.y    = float(y)
        p.position.z    = float(z)
        p.orientation.w = GRASP_DOWN_QUAT['w']
        p.orientation.x = GRASP_DOWN_QUAT['x']
        p.orientation.y = GRASP_DOWN_QUAT['y']
        p.orientation.z = GRASP_DOWN_QUAT['z']
        return p

    if len(vals) == 7:
        x, y, z, qx, qy, qz, qw = vals
        p = Pose()
        p.position.x    = float(x)
        p.position.y    = float(y)
        p.position.z    = float(z)
        p.orientation.x = float(qx)
        p.orientation.y = float(qy)
        p.orientation.z = float(qz)
        p.orientation.w = float(qw)
        return p

    raise ValueError(
        f'pose must be (x,y,z) or (x,y,z,qx,qy,qz,qw) — got {len(vals)} values'
    )


def _approach_waypoint(pose: Pose, dist: float) -> Pose:
    """Return a waypoint dist metres above pose along the approach axis.

    The approach axis is the gripper's local +Z rotated into the planning frame
    by the goal orientation quaternion.  The waypoint is in the *opposite*
    direction (behind the tip), so the arm enters the pick zone from above.

    For the default downward-pointing gripper:
      local +Z  = world -Z  →  waypoint.z = pose.z + dist  (above the target)
    """
    q = pose.orientation
    w, x, y, z = q.w, q.x, q.y, q.z

    # Rotate the local +Z unit vector (0, 0, 1) by the quaternion.
    # This gives the approach direction in the planning frame.
    ax = 2.0 * (x * z + w * y)
    ay = 2.0 * (y * z - w * x)
    az = 1.0 - 2.0 * (x * x + y * y)

    # Waypoint is in the negative approach direction (arm comes from above).
    p = Pose()
    p.position.x  = pose.position.x - ax * dist
    p.position.y  = pose.position.y - ay * dist
    p.position.z  = pose.position.z - az * dist
    p.orientation = pose.orientation
    return p


def _poll(future, timeout_sec: float) -> bool:
    """Block until future is done, or until timeout. Returns True if done."""
    deadline = time.monotonic() + timeout_sec
    while not future.done():
        if time.monotonic() > deadline:
            return False
        time.sleep(_POLL_INTERVAL)
    return True


# ══════════════════════════════════════════════════════════════════════════════
# CLI entry point
# ══════════════════════════════════════════════════════════════════════════════

def _build_parser():
    parser = argparse.ArgumentParser(
        prog='pick_and_place',
        description=(
            'Execute a single pick-and-place cycle on the AR4 arm.\n\n'
            'All coordinates are in the ar4_base_link frame.\n\n'
            'Pick formats:\n'
            '  x y z                — XYZ, gripper pointing straight down\n'
            '  x y z qx qy qz qw   — full 6-DOF end-effector pose\n\n'
            'Place (--place) accepts the same formats.\n'
            'Omit --place to use the hardcoded rear-bin default.'
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        'pick', nargs='+', type=float, metavar='VAL',
        help='3 values (x y z) or 7 values (x y z qx qy qz qw)',
    )
    parser.add_argument(
        '--place', nargs='+', type=float, default=None, metavar='VAL',
        help='Place location: 3 or 7 floats. Default: hardcoded rear bin.',
    )
    return parser


def main(argv=None):
    parser = _build_parser()
    args = parser.parse_args(sys.argv[1:] if argv is None else argv)

    if len(args.pick) not in (3, 7):
        parser.error(f'pick requires 3 or 7 values, got {len(args.pick)}')
    if args.place is not None and len(args.place) not in (3, 7):
        parser.error(f'--place requires 3 or 7 values, got {len(args.place)}')

    rclpy.init()
    node = PickAndPlaceNode()
    try:
        success = node.pick_and_place(
            pick  = tuple(args.pick),
            place = tuple(args.place) if args.place else None,
        )
        sys.exit(0 if success else 1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
