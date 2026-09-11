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
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav2_msgs.action import FollowPath
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from std_msgs.msg import String

from amiga_interfaces.action import MoveTo
from amiga_interfaces.srv import EvaluateCondition
from amiga_ros2_planners.bezier import sample_bezier_chain
from amiga_ros2_planners.pose import PoseProvider

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
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("samples_per_segment", 10)
        # Also bounds how quickly a BT-side cancel (e.g. a ReactiveSequence
        # guard like BatteryOver failing) is even noticed here -- see
        # _execute()'s own cancel-handling comment for the full latency
        # budget this feeds into.
        self.declare_parameter("trigger_poll_period_s", 0.2)
        # This node deliberately bypasses Nav2's own NavigateToPose/
        # bt_navigator pipeline (see this module's own docstring) --
        # unlike this repo's OTHER move actions (MoveToGPSLocation/
        # MoveToTreeID/etc, all of which go through bt_navigator's own
        # recovery behaviors: spin/backup/wait/clear-costmaps), a bare
        # FollowPath goal here has NO recovery layer of its own. If
        # controller_server ever stops making progress for a reason
        # none of `triggers` covers (e.g. a stale local_costmap
        # observation source -- see sim_camera_shim.py's own respawn
        # comment for one concrete way that happens), this walk would
        # otherwise run forever with no way for the BT tree holding it
        # to ever recover. This is a blunt backstop, not a substitute
        # for Nav2's own recoveries: it only ends a walk that has
        # already run unreasonably long, it doesn't try to fix
        # anything first.
        self.declare_parameter("max_walk_duration_s", 120.0)
        self.declare_parameter("follow_path_action", "follow_path")
        self.declare_parameter("controller_id", "")
        # tool_action_node's own tracked "currently equipped tool"
        # changes this walk's own desired speed -- config.yaml's
        # tool.equipped.<tool>.speed (problog_problem.tool_params's own
        # "speed" dict; "free" is motion.speed itself, same "no
        # separate key for no tool" convention that function's own
        # docstring explains). Applied via a LIVE parameter set on
        # controller_server before each goal (see _apply_tool_speed
        # below) -- FollowPath.action itself has no per-goal speed
        # field, so this is the only way to change it at runtime short
        # of editing nav2_params.yaml and restarting the stack.
        self.declare_parameter("tool_speed_free_mps", 0.5)
        self.declare_parameter("tool_speed_cart_mps", 0.5)
        self.declare_parameter("tool_speed_plow_mps", 0.5)
        # DeployTool/RetractTool's own THIRD speed state -- see
        # tool_action_node.py's own module docstring. Defaults to that
        # SAME kind's own regular speed above (config.yaml's own
        # tool.equipped.<kind>.deployed_speed, defaulting to
        # tool.equipped.<kind>.speed if not separately overridden) --
        # problog_problem.tool_params already resolves that default,
        # this node just reads whatever it's handed.
        self.declare_parameter("tool_speed_cart_deployed_mps", 0.5)
        self.declare_parameter("tool_speed_plow_deployed_mps", 0.5)
        self.declare_parameter("tool_deployed_topic", "tool_deployed")
        self.declare_parameter(
            "controller_server_set_parameters_service",
            "controller_server/set_parameters")
        self.declare_parameter("tool_state_topic", "tool_state")

        # local_costmap (which controller_server's FollowPath needs a
        # working state estimate to run against) looks up base_link->odom,
        # published by Gazebo's diff_drive_controller -- a plugin that can
        # load several seconds after this node starts. Used to gate
        # sending a FollowPath goal below: without it, controller_server
        # has no real robot state, and observed behavior is NOT "wait/
        # fail" but a false-positive instant "Reached the goal!" with the
        # robot never actually moving.
        self._odom_pose = PoseProvider(
            self,
            self.get_parameter("odom_frame").value,
            self.get_parameter("base_frame").value,
        )

        # Both the FollowPath client and the condition-evaluation calls
        # need to complete WHILE this action's own execute callback is
        # still running -- a ReentrantCallbackGroup plus the ActionServer's
        # own default of running each goal's execute callback on its own
        # thread is what lets this node wait on ANOTHER call's future
        # from inside _execute() without deadlocking against this same
        # node's own main spin.
        #
        # That waiting is done with this module's own _wait_for_future()
        # (plain future.done() polling + time.sleep), deliberately NOT
        # rclpy.spin_once()/spin_until_future_complete(self, ...): those
        # top-level helpers each do their own executor.add_node(self),
        # reassigning this node's own .executor out from under whatever
        # already owns it (main()'s MultiThreadedExecutor here). Called
        # occasionally that's mostly harmless, but _execute() calls one
        # of them on every trigger-poll tick of every walk, and the
        # cancellation path fires two more back-to-back -- concurrent
        # re-entrant spins like that are a known way for an executor to
        # lose track of a node's callbacks. Observed effect: the very
        # first walk of a mission always worked, but the walk
        # immediately after the first BatteryOver-triggered cancel never
        # produced a single piece of feedback again for the rest of that
        # mission. _wait_for_future() never touches .executor at all --
        # it just polls, so the externally-owned MultiThreadedExecutor
        # remains the only thing that ever actually spins this node.
        self._cb_group = ReentrantCallbackGroup()
        self._follow_path_client = ActionClient(
            self, FollowPath, self.get_parameter("follow_path_action").value,
            callback_group=self._cb_group)
        self._condition_client = self.create_client(
            EvaluateCondition, "evaluate_condition", callback_group=self._cb_group)
        self._set_params_client = self.create_client(
            SetParameters,
            self.get_parameter("controller_server_set_parameters_service").value,
            callback_group=self._cb_group)

        # "free" is tool_action_node's own initial state and its own
        # first (latched) publish -- correct default even if this node
        # subscribes before tool_action_node exists at all (no
        # InstallTool/UninstallTool in this mission's own tree).
        self._equipped_tool = "free"
        self.create_subscription(
            String, self.get_parameter("tool_state_topic").value,
            self._on_tool_state, 10, callback_group=self._cb_group)

        # "false" is tool_action_node's own initial state -- see
        # _on_tool_state's own comment above for why a default here
        # matters even before tool_action_node's first latched publish.
        self._deployed = False
        self.create_subscription(
            String, self.get_parameter("tool_deployed_topic").value,
            self._on_tool_deployed, 10, callback_group=self._cb_group)

        self._action_server = ActionServer(
            self, MoveTo, "move_to", self._execute,
            cancel_callback=lambda _goal_handle: CancelResponse.ACCEPT,
            callback_group=self._cb_group)

        self.get_logger().info("move_to_node ready on 'move_to'")

    def _on_tool_state(self, msg):
        self._equipped_tool = msg.data

    def _on_tool_deployed(self, msg):
        self._deployed = msg.data == "true"

    def _apply_tool_speed(self, controller_id):
        """Best-effort: sets <controller>.desired_linear_vel on
        controller_server to this walk's own tool-appropriate speed
        (see the constructor's own comment on tool_speed_*_mps).
        Never aborts the walk over this -- a speed override that
        doesn't take (service not up yet, wrong controller_id, an
        older Nav2 without this exact param name) just means the walk
        runs at whatever speed controller_server was already
        configured with, not a reason to fail the whole action."""
        if self._deployed and self._equipped_tool in ("cart", "plow"):
            speed = self.get_parameter(
                f"tool_speed_{self._equipped_tool}_deployed_mps").value
        else:
            speed = {
                "free": self.get_parameter("tool_speed_free_mps").value,
                "cart": self.get_parameter("tool_speed_cart_mps").value,
                "plow": self.get_parameter("tool_speed_plow_mps").value,
            }.get(self._equipped_tool)
        if speed is None:
            return
        if not self._set_params_client.service_is_ready():
            self.get_logger().warn(
                "move_to_node: controller_server's set_parameters "
                "service not ready, walking at whatever speed it's "
                "already configured with")
            return
        controller_name = controller_id or "FollowPath"
        param = Parameter()
        param.name = f"{controller_name}.desired_linear_vel"
        param.value = ParameterValue(
            type=ParameterType.PARAMETER_DOUBLE, double_value=float(speed))
        request = SetParameters.Request(parameters=[param])
        future = self._set_params_client.call_async(request)
        self._wait_for_future(future, timeout_sec=1.0)

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

    @staticmethod
    def _wait_for_future(future, timeout_sec, poll_interval_s=0.02):
        """Block up to timeout_sec for `future` to resolve, WITHOUT
        spinning this node ourselves -- see the constructor's own
        comment on why rclpy.spin_once()/spin_until_future_complete(
        self, ...) are unsafe to call from here. Whatever callback
        fulfills `future` runs on the externally-owned
        MultiThreadedExecutor's own worker threads regardless of
        whether this thread spins or not; this just waits for it.
        Returns whether it resolved in time (mirrors
        spin_until_future_complete's own success/timeout distinction,
        since callers here branch on that)."""
        deadline = time.monotonic() + timeout_sec
        while not future.done():
            if time.monotonic() >= deadline:
                return False
            time.sleep(poll_interval_s)
        return True

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
            self._wait_for_future(future, timeout_sec=1.0)
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

        # See this node's own constructor comment on why -- without a real
        # base_link->odom transform yet, controller_server has no state to
        # run FollowPath against and reports a false-positive instant
        # "Reached the goal!" rather than actually driving anywhere.
        if not self._odom_pose.wait_ready():
            self.get_logger().error(
                "move_to_node: no odom pose after startup timeout, aborting")
            goal_handle.abort()
            result.reason, result.status = "aborted", False
            return result

        follow_goal = FollowPath.Goal()
        follow_goal.path = path
        controller_id = self.get_parameter("controller_id").value
        if controller_id:
            follow_goal.controller_id = controller_id

        self._apply_tool_speed(controller_id)

        # wait_for_server only confirms the action is visible on the ROS
        # graph, which for a Nav2 lifecycle node (controller_server) can be
        # true well before it's actually ACTIVE -- the mission starts
        # ticking as soon as it's delivered, independent of Nav2's own
        # lifecycle bring-up sequence (see run_problog_problem.launch.py's
        # own docstring), so the very first goal here can land in that gap
        # and get rejected outright by an inactive controller_server. Retry
        # for a while rather than aborting on the first rejection -- once
        # lifecycle_manager finishes activating controller_server (a
        # one-time startup event), goals go through normally.
        follow_path_goal_handle = None
        deadline = time.monotonic() + 30.0
        while follow_path_goal_handle is None or not follow_path_goal_handle.accepted:
            send_future = self._follow_path_client.send_goal_async(
                follow_goal,
                feedback_callback=lambda fb: self._on_follow_path_feedback(fb, goal_handle))
            self._wait_for_future(send_future, timeout_sec=30.0)
            follow_path_goal_handle = send_future.result()
            if follow_path_goal_handle is not None and follow_path_goal_handle.accepted:
                break
            if time.monotonic() >= deadline:
                self.get_logger().error("FollowPath goal rejected")
                goal_handle.abort()
                result.reason, result.status = "aborted", False
                return result
            self.get_logger().warn(
                "FollowPath goal rejected (controller_server likely not "
                "active yet), retrying...")
            time.sleep(0.5)

        parsed_triggers = self._parse_triggers(list(goal.triggers))
        get_result_future = follow_path_goal_handle.get_result_async()
        poll_period = self.get_parameter("trigger_poll_period_s").value
        max_walk_duration = self.get_parameter("max_walk_duration_s").value
        walk_start = time.monotonic()

        fired_trigger = None
        while not get_result_future.done():
            time.sleep(poll_period)
            if get_result_future.done():
                break
            if time.monotonic() - walk_start > max_walk_duration:
                self.get_logger().error(
                    f"MoveTo: no result after {max_walk_duration}s, "
                    "cancelling (see max_walk_duration_s's own comment "
                    "-- this walk has no Nav2 recovery layer of its own)")
                cancel_future = follow_path_goal_handle.cancel_goal_async()
                self._wait_for_future(cancel_future, timeout_sec=2.0)
                self._wait_for_future(get_result_future, timeout_sec=2.0)
                goal_handle.abort()
                result.reason, result.status = "timeout", False
                return result
            if goal_handle.is_cancel_requested:
                # bt.cpp's own MoveTo leaf (BT::RosActionNode::halt())
                # waits for THIS action's own result before a containing
                # ReactiveSequence/Fallback can move on (e.g. to problog's
                # own GoHome branch once BatteryOver fails) -- worst case
                # here is trigger_poll_period_s (noticing the cancel) plus
                # these two timeouts, so keep them tight; Nav2's own
                # cancel ack/result normally arrive in well under a
                # second on a local controller_server.
                cancel_future = follow_path_goal_handle.cancel_goal_async()
                self._wait_for_future(cancel_future, timeout_sec=2.0)
                self._wait_for_future(get_result_future, timeout_sec=2.0)
                goal_handle.canceled()
                result.reason, result.status = "canceled", False
                return result
            if parsed_triggers:
                fired_trigger = self._check_triggers(parsed_triggers)
                if fired_trigger is not None:
                    cancel_future = follow_path_goal_handle.cancel_goal_async()
                    self._wait_for_future(cancel_future, timeout_sec=5.0)
                    self._wait_for_future(get_result_future, timeout_sec=5.0)
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
