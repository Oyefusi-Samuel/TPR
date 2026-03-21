#!/usr/bin/env python3
"""workspace_executive.py — ROS 2 action server for workspace pick-and-place.

Accepts an ordered array of trash pick poses (in ar4_base_link frame) and an
optional place pose, then executes the picks serially.  Returns a classified
result so the mission planner can decide whether to replan.

Action
------
  Name : workspace_pick_and_place
  Type : jackal_ar4_interfaces/action/WorkspacePickAndPlace

  Goal
    geometry_msgs/Pose[] pick_poses   — trash items in ar4_base_link frame
    geometry_msgs/Pose   place_pose   — where to deposit; ignored when
    bool                 use_default_place  — True → use DEFAULT_PLACE

  Feedback (published after each item attempt)
    int32  current_index   — zero-based index just attempted
    int32  total           — total picks in this workspace
    string status          — human-readable step description

  Result
    bool   success            — True only if ALL items were picked and placed
    int32  picked_count
    int32  unreachable_count  — items with no IK or goal in collision
    int32[] unreachable_indices
    int32  failed_count       — items with planning timeout / transient error
    int32[] failed_indices

Orientation convention
----------------------
Pick poses sent with identity (0,0,0,1) or zero (0,0,0,0) orientation are
treated as "no orientation specified" and the default downward-pointing gripper
quaternion (GRASP_DOWN_QUAT) is substituted.  Provide any other quaternion to
fully specify a 6-DOF approach direction.

Cancellation
------------
Honoured between pick-and-place cycles (a single cycle runs to completion
before the cancel is checked).  On cancel: opens the gripper and moves the
arm to the home pose.

Concurrent goals
----------------
A second goal while one is executing is rejected.

Frame convention
----------------
All poses must already be in ar4_base_link frame.  The upstream coordinate-
translation service is responsible for converting world-frame detections.
"""

import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse

from geometry_msgs.msg import Pose as RosPose
from jackal_ar4_interfaces.action import WorkspacePickAndPlace
from jackal_ar4_goals.pick_and_place import (
    GRASP_DOWN_QUAT,
    GRIPPER_OPEN,
    PickAndPlaceNode,
    PickResult,
)


def _apply_default_orientation(pose):
    """Return pose with GRASP_DOWN_QUAT substituted when no orientation was given.

    Treats both the zero quaternion (0,0,0,0) and the identity quaternion
    (0,0,0,1) as "unspecified".  The ROS CLI and many clients default to
    (0,0,0,1) when only a position is provided, so catching identity avoids
    the approach waypoint being computed in the wrong direction.
    """
    o = pose.orientation
    is_zero = (
        abs(o.x) < 1e-9 and abs(o.y) < 1e-9
        and abs(o.z) < 1e-9 and abs(o.w) < 1e-9
    )
    is_identity = (
        abs(o.w - 1.0) < 1e-6 and abs(o.x) < 1e-6
        and abs(o.y) < 1e-6 and abs(o.z) < 1e-6
    )
    if is_zero or is_identity:
        p = RosPose()
        p.position = pose.position
        p.orientation.x = GRASP_DOWN_QUAT['x']
        p.orientation.y = GRASP_DOWN_QUAT['y']
        p.orientation.z = GRASP_DOWN_QUAT['z']
        p.orientation.w = GRASP_DOWN_QUAT['w']
        return p
    return pose


class WorkspaceExecutiveNode(PickAndPlaceNode):
    """Action server that wraps PickAndPlaceNode for batch workspace execution."""

    def __init__(self):
        super().__init__()

        # Guard against concurrent workspace goals.
        self._busy = False
        self._busy_lock = threading.Lock()

        self._ws_server = ActionServer(
            self,
            WorkspacePickAndPlace,
            'workspace_pick_and_place',
            execute_callback=self._execute_cb,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
        )
        self.get_logger().info(
            'WorkspaceExecutive ready — action: workspace_pick_and_place'
        )

    # ── Goal / cancel callbacks ────────────────────────────────────────────────

    def _goal_cb(self, goal_request):
        if not goal_request.pick_poses:
            self.get_logger().warn('Rejecting empty workspace goal')
            return GoalResponse.REJECT
        with self._busy_lock:
            if self._busy:
                self.get_logger().warn(
                    'Already executing a workspace — rejecting new goal'
                )
                return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_cb(self, goal_handle):
        self.get_logger().info('Cancel requested — will honour after current pick')
        return CancelResponse.ACCEPT

    # ── Execute callback ───────────────────────────────────────────────────────

    def _execute_cb(self, goal_handle):
        """Top-level execute callback.

        goal_handle terminal methods (succeed / canceled) are called here,
        immediately before return, so that rclpy receives the populated result
        object at the same time as the status change.
        """
        with self._busy_lock:
            self._busy = True
        try:
            result, was_cancelled = self._run_workspace(goal_handle)
        finally:
            with self._busy_lock:
                self._busy = False

        if was_cancelled:
            goal_handle.canceled()
        else:
            goal_handle.succeed()
        return result

    def _run_workspace(self, goal_handle):
        """Execute all picks in the workspace.

        Returns
        -------
        (WorkspacePickAndPlace.Result, was_cancelled: bool)
        """
        goal  = goal_handle.request
        picks = goal.pick_poses
        place = None if goal.use_default_place else goal.place_pose
        total = len(picks)

        picked_indices      = []
        unreachable_indices = []
        failed_indices      = []

        for i, raw_pose in enumerate(picks):

            # ── Check for cancellation between items ─────────────────────────
            if goal_handle.is_cancel_requested:
                self.get_logger().info(
                    f'Cancel honoured at item {i}/{total} — cleaning up'
                )
                self._cancel_cleanup()
                result = self._build_result(
                    picked_indices, unreachable_indices, failed_indices
                )
                return result, True

            # ── Publish feedback ─────────────────────────────────────────────
            fb = WorkspacePickAndPlace.Feedback()
            fb.current_index = i
            fb.total         = total
            fb.status        = f'Attempting pick {i + 1}/{total}'
            goal_handle.publish_feedback(fb)

            self.get_logger().info(f'--- Workspace item {i + 1}/{total} ---')

            # ── Normalise orientation — identity/zero → GRASP_DOWN_QUAT ─────
            pick_pose = _apply_default_orientation(raw_pose)

            # ── Execute one pick-and-place cycle ─────────────────────────────
            pick_result = self.pick_and_place(pick_pose, place)

            if pick_result == PickResult.SUCCESS:
                picked_indices.append(i)
            elif pick_result == PickResult.UNREACHABLE:
                self.get_logger().warn(
                    f'Item {i} unreachable (no IK or goal in collision) — skipping'
                )
                unreachable_indices.append(i)
            else:  # FAILED
                self.get_logger().warn(
                    f'Item {i} failed (planning/timeout/collision) — skipping'
                )
                failed_indices.append(i)

        # ── All items processed ──────────────────────────────────────────────
        result = self._build_result(
            picked_indices, unreachable_indices, failed_indices
        )
        self.get_logger().info(
            f'Workspace complete — picked={result.picked_count} '
            f'unreachable={result.unreachable_count} '
            f'failed={result.failed_count}'
        )
        return result, False

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _build_result(self, picked, unreachable, failed):
        r = WorkspacePickAndPlace.Result()
        r.picked_count        = len(picked)
        r.unreachable_count   = len(unreachable)
        r.unreachable_indices = list(unreachable)
        r.failed_count        = len(failed)
        r.failed_indices      = list(failed)
        r.success             = (r.unreachable_count == 0 and r.failed_count == 0)
        return r

    def _cancel_cleanup(self):
        """Open gripper and return arm to home — called on cancellation."""
        self.get_logger().info('Cancel cleanup: opening gripper')
        self._set_gripper(GRIPPER_OPEN)
        self.get_logger().info('Cancel cleanup: moving arm to home')
        self._recover_to_home()


# ══════════════════════════════════════════════════════════════════════════════
# Entry point
# ══════════════════════════════════════════════════════════════════════════════

def main(argv=None):
    rclpy.init(args=argv)
    node = WorkspaceExecutiveNode()
    try:
        node.get_logger().info(
            'Workspace executive running — waiting for goals on '
            '"workspace_pick_and_place"'
        )
        while rclpy.ok():
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
