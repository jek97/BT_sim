#!/usr/bin/env python3
"""
move_to_node.py

Hosts MoveTo (amiga_interfaces/action/MoveTo) -- the ROS2-action form of
problog_project's MoveTo BT node. This IS the wrapper the user asked
about: MoveTo's own "trajectory" is a chained-cubic-Bezier control point
list (PlanPath's own output format), but Nav2's `controller_server`
FollowPath action -- the "extremely low level" Nav2 primitive this
simulation uses for MoveTo, deliberately bypassing planner_server/
bt_navigator/recoveries -- takes a nav_msgs/Path (a plain sequence of
stamped poses), not Bezier control points. So this node:

  1. Samples control_points into an (x, y, yaw) polyline (bezier.py,
     analytic tangent -> yaw, not finite-differenced), and builds a
     nav_msgs/Path from it.
  2. Sends that Path as a FollowPath goal to Nav2's controller_server,
     forwarding its own feedback (distance_to_goal) back out as this
     action's own feedback.
  3. While FollowPath runs, polls `triggers` (a real but partial subset
     of schema.yaml's own MoveTo triggers vocabulary -- see
     TRIGGER_FUNCTOR_TO_CONDITION below) against condition_service_node's
     EvaluateCondition service, and cancels FollowPath the moment one
     fires, reporting that trigger as the Result's own `reason` --
     mirroring MoveTo's own "halts on whichever of Triggers occurs
     earliest" contract.

NOT carried over from problog_project's own MoveTo semantics (see
schema.yaml's own triggers port description for the full vocabulary):
  - The AUTOMATIC collision/battery-depleted triggers bt_to_prolog.py
    injects into every leg on the Prolog side. There is no equivalent
    injection here -- only whatever `triggers` this action's own Goal
    actually lists gets checked.
  - line_of_sight_clear(...)/crosses_segment(...) (the Bug0/Bug2
    boundary-leave triggers) -- EvaluateCondition's own LineOfSightClear
    needs a goal point this action's Goal has no slot for; add one if a
    future scenario needs Bug-algorithm legs driven through this
    wrapper.
  - A structural guard derived from a tree's own ReactiveSequence
    siblings (schema.yaml's own "CONTROL-FLOW GUARD DERIVATION" note) --
    that is a BT.cpp-tree-structure concept with no meaning at this
    node's own level; it belongs in whatever future BT.cpp MoveTo leaf
    calls this action, not here.

Caveat: this node has not been exercised against a live Nav2
controller_server in this session (no ROS2/Nav2 environment available
here to run it against) -- the Bezier sampling
(bezier.py) is unit-tested and verified, but the FollowPath
integration itself should be smoke-tested against your own sim before
relying on it.
"""
import math
import re

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav2_msgs.action import FollowPath

from amiga_interfaces.action import MoveTo
from amiga_interfaces.srv import EvaluateCondition
from amiga_ros2_planners.bezier import sample_bezier_chain

# e.g. "battery_below(20)" -> ("battery_below", "20"). Matches the
# semicolon-separated syntax schema.yaml's own MoveTo.triggers port
# documents, minus the semicolons (this action's own Goal carries
# triggers as a plain string[] already -- see MoveTo.action's own
# header).
TRIGGER_PATTERN = re.compile(r"^(\w+)\(\s*([-+]?[0-9]*\.?[0-9]+)\s*\)$")

# Only the threshold-only conditions -- see this module's own docstring
# for what's deliberately not supported (line_of_sight_clear/
# crosses_segment need a goal point this action's Goal has no slot for).
TRIGGER_FUNCTOR_TO_CONDITION = {
    "obstacle_in_bound": "ObstacleInBound",
    "obstacle_on_path": "ObstacleOnPath",
    "battery_below": "BatteryBelow",
    "battery_over": "BatteryOver",
    "battery_equal": "BatteryEqual",
}


class MoveToNode(Node):
    def __init__(self):
        super().__init__("move_to_node")

        self.declare_parameter("reference_frame", "map")
        self.declare_parameter("samples_per_segment", 10)
        self.declare_parameter("trigger_poll_period_s", 0.5)
        self.declare_parameter("follow_path_action", "follow_path")
        self.declare_parameter("controller_id", "")

        # Both the FollowPath client and the condition-evaluation calls
        # need to complete WHILE this action's own execute callback is
        # still running -- a ReentrantCallbackGroup plus the ActionServer's
        # own default of running each goal's execute callback on its own
        # thread is what makes the blocking spin_until_future_complete
        # calls below safe rather than deadlocking against this same
        # node's main spin.
        self._cb_group = ReentrantCallbackGroup()
        self._follow_path_client = ActionClient(
            self, FollowPath, self.get_parameter("follow_path_action").value,
            callback_group=self._cb_group)
        self._condition_client = self.create_client(
            EvaluateCondition, "evaluate_condition", callback_group=self._cb_group)

        self._action_server = ActionServer(
            self, MoveTo, "move_to", self._execute,
            cancel_callback=lambda _goal_handle: CancelResponse.ACCEPT,
            callback_group=self._cb_group)

        self.get_logger().info("move_to_node ready on 'move_to'")

    def _parse_triggers(self, triggers):
        parsed = []
        for trigger in triggers:
            match = TRIGGER_PATTERN.match(trigger.strip())
            if not match:
                self.get_logger().warn(
                    f"unrecognized trigger syntax, ignoring: '{trigger}'")
                continue
            functor, value = match.group(1), float(match.group(2))
            condition = TRIGGER_FUNCTOR_TO_CONDITION.get(functor)
            if condition is None:
                self.get_logger().warn(
                    f"trigger '{trigger}' not supported by move_to_node "
                    f"(supported: {sorted(TRIGGER_FUNCTOR_TO_CONDITION)}), ignoring")
                continue
            parsed.append((trigger, condition, value))
        return parsed

    def _check_triggers(self, parsed_triggers):
        """The first trigger string that currently evaluates true, or
        None. One EvaluateCondition call per trigger per poll -- fine at
        the sub-ten-triggers scale this vocabulary supports."""
        for original, condition, threshold in parsed_triggers:
            if not self._condition_client.service_is_ready():
                continue
            request = EvaluateCondition.Request()
            request.condition = condition
            request.threshold = threshold
            future = self._condition_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
            response = future.result()
            if response is not None and response.result:
                return original
        return None

    def _build_path(self, control_points):
        samples = sample_bezier_chain(
            control_points, self.get_parameter("samples_per_segment").value)
        path = Path()
        path.header.frame_id = self.get_parameter("reference_frame").value
        path.header.stamp = self.get_clock().now().to_msg()
        for x, y, yaw in samples:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            path.poses.append(pose)
        return path

    def _execute(self, goal_handle):
        goal = goal_handle.request
        result = MoveTo.Result()

        control_points = [(p.x, p.y) for p in goal.control_points]
        try:
            path = self._build_path(control_points)
        except ValueError as exc:
            self.get_logger().error(f"bad control_points: {exc}")
            goal_handle.abort()
            result.reason, result.status = "aborted", False
            return result

        if not self._follow_path_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(
                f"FollowPath action server "
                f"'{self.get_parameter('follow_path_action').value}' unavailable")
            goal_handle.abort()
            result.reason, result.status = "aborted", False
            return result

        follow_goal = FollowPath.Goal()
        follow_goal.path = path
        controller_id = self.get_parameter("controller_id").value
        if controller_id:
            follow_goal.controller_id = controller_id

        send_future = self._follow_path_client.send_goal_async(
            follow_goal,
            feedback_callback=lambda fb: self._on_follow_path_feedback(fb, goal_handle))
        rclpy.spin_until_future_complete(self, send_future)
        follow_path_goal_handle = send_future.result()
        if follow_path_goal_handle is None or not follow_path_goal_handle.accepted:
            self.get_logger().error("FollowPath goal rejected")
            goal_handle.abort()
            result.reason, result.status = "aborted", False
            return result

        parsed_triggers = self._parse_triggers(list(goal.triggers))
        get_result_future = follow_path_goal_handle.get_result_async()
        poll_period = self.get_parameter("trigger_poll_period_s").value

        fired_trigger = None
        while not get_result_future.done():
            rclpy.spin_once(self, timeout_sec=poll_period)
            if get_result_future.done():
                break
            if goal_handle.is_cancel_requested:
                cancel_future = follow_path_goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=5.0)
                rclpy.spin_until_future_complete(self, get_result_future, timeout_sec=5.0)
                goal_handle.canceled()
                result.reason, result.status = "canceled", False
                return result
            if parsed_triggers:
                fired_trigger = self._check_triggers(parsed_triggers)
                if fired_trigger is not None:
                    cancel_future = follow_path_goal_handle.cancel_goal_async()
                    rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=5.0)
                    rclpy.spin_until_future_complete(self, get_result_future, timeout_sec=5.0)
                    break

        if fired_trigger is not None:
            goal_handle.succeed()
            result.reason, result.status = fired_trigger, False
            return result

        follow_path_result = get_result_future.result()
        status = follow_path_result.status
        if status == GoalStatus.STATUS_SUCCEEDED:
            goal_handle.succeed()
            result.reason, result.status = "completed", True
        elif status == GoalStatus.STATUS_CANCELED:
            goal_handle.canceled()
            result.reason, result.status = "canceled", False
        else:
            goal_handle.abort()
            result.reason, result.status = "aborted", False
        return result

    def _on_follow_path_feedback(self, feedback_msg, goal_handle):
        fb = MoveTo.Feedback()
        fb.distance_to_goal = feedback_msg.feedback.distance_to_goal
        goal_handle.publish_feedback(fb)


def main(args=None):
    rclpy.init(args=args)
    node = MoveToNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
