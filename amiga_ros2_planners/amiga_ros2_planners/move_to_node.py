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
  3. While FollowPath runs, polls an ALWAYS-ON safety cutoff
     (SafetyMonitor -- battery_state/chassis_contact_<side>, see
     safety_monitor.py) and cancels FollowPath the moment it trips.

This action takes only the input needed to execute the walk
(control_points) and reports only success/failure (`status`) -- no
configurable triggers port, no reason string: a BT.cpp leaf calling this
only ever needs SUCCESS/FAILURE/RUNNING (RUNNING is inherent in the
action still being active), so that's all this hands back.

Caveat: this node has not been exercised against a live Nav2
controller_server in this session (no ROS2/Nav2 environment available
here to run it against) -- the Bezier sampling
(bezier.py) is unit-tested and verified, but the FollowPath
integration itself should be smoke-tested against your own sim before
relying on it.
"""
import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav2_msgs.action import FollowPath
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from std_msgs.msg import String

from amiga_interfaces.action import MoveTo
from amiga_ros2_planners.bezier import sample_bezier_chain
from amiga_ros2_planners.ploughing import cell_index
from amiga_ros2_planners.pose import PoseProvider
from amiga_ros2_planners.safety_monitor import SafetyMonitor


class MoveToNode(Node):
    def __init__(self):
        super().__init__("move_to_node")

        self.declare_parameter("reference_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        # "simplified" branch: read the robot's live position straight off
        # amiga_ros2_gazebo's ground_truth_node.py (Ignition's own
        # PosePublisher, no EKF in the loop) instead of the map->base_link
        # tf2 transform. Empty string restores the tf2 lookup. Only
        # _pose (reference_frame) below uses this -- _odom_pose still reads
        # odom_frame->base_frame off tf2, unrelated to map localization.
        self.declare_parameter("pose_topic", "ground_truth/pose")
        self.declare_parameter("samples_per_segment", 10)
        # How often the poll loop below wakes up to check for a cancel,
        # the safety cutoff, or the max-duration backstop -- also bounds
        # how quickly a BT-side cancel is noticed here (see _execute()'s
        # own cancel-handling comment for the full latency budget this
        # feeds into).
        self.declare_parameter("poll_period_s", 0.2)
        # This node deliberately bypasses Nav2's own NavigateToPose/
        # bt_navigator pipeline (see this module's own docstring) --
        # unlike this repo's OTHER move actions (MoveToGPSLocation/
        # MoveToTreeID/etc, all of which go through bt_navigator's own
        # recovery behaviors: spin/backup/wait/clear-costmaps), a bare
        # FollowPath goal here has NO recovery layer of its own. If
        # controller_server ever stops making progress for a reason
        # the safety cutoff doesn't cover (e.g. a stale local_costmap
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
        # ploughed/3 (basic_action_theory.pl) -- the cell-indexed
        # fluent PloughedAt/PloughedBetween query. This problem's own
        # config.yaml ploughing.cell_size (problog_problem.
        # ploughing_params); 1.0 here is just a harmless default for a
        # mission that never configures ploughing.cell_size at all (no
        # PloughedAt/PloughedBetween in its own tree either, so the
        # exact value is moot then). MUST match condition_service_node's
        # own plough_cell_size param, or a cell boundary could land on
        # different (Cx,Cy) indices between the two.
        self.declare_parameter("plough_cell_size", 1.0)
        self.declare_parameter("ploughed_cells_topic", "ploughed_cells")

        # ALWAYS-ON safety cutoff -- see safety_monitor.py's own module
        # docstring. Params match condition_service_node's own
        # battery_topic/contact_topic_*/contact_stale_after_s names so
        # one launch arg set covers both.
        self.declare_parameter("battery_topic", "battery_state")
        self.declare_parameter("battery_depleted_threshold_pct", 0.0)
        self.declare_parameter("contact_topic_front", "chassis/contact_front")
        self.declare_parameter("contact_topic_back", "chassis/contact_back")
        self.declare_parameter("contact_topic_left", "chassis/contact_left")
        self.declare_parameter("contact_topic_right", "chassis/contact_right")
        self.declare_parameter("contact_stale_after_s", 0.5)
        self._safety = SafetyMonitor(
            self,
            battery_topic=self.get_parameter("battery_topic").value,
            battery_depleted_threshold_pct=self.get_parameter(
                "battery_depleted_threshold_pct").value,
            contact_topic_front=self.get_parameter("contact_topic_front").value,
            contact_topic_back=self.get_parameter("contact_topic_back").value,
            contact_topic_left=self.get_parameter("contact_topic_left").value,
            contact_topic_right=self.get_parameter("contact_topic_right").value,
            contact_stale_after_s=self.get_parameter("contact_stale_after_s").value,
        )

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
        # reference_frame pose -- see _mark_ploughed_here below. A
        # SEPARATE PoseProvider from _odom_pose above: that one exists
        # only to gate sending a FollowPath goal (odom_frame->base_frame
        # needs to exist yet), this one is the same reference_frame
        # (typically "map") pose every OTHER condition/query in this
        # package (condition_service_node.py, tool_action_node.py) reads
        # positions in -- ploughed cells must line up with THOSE, not
        # with odom.
        self._pose = PoseProvider(
            self,
            self.get_parameter("reference_frame").value,
            self.get_parameter("base_frame").value,
            pose_topic=self.get_parameter("pose_topic").value,
        )

        # The FollowPath client needs to complete WHILE this action's own
        # execute callback is still running -- a ReentrantCallbackGroup
        # plus the ActionServer's own default of running each goal's
        # execute callback on its own thread is what lets this node wait
        # on ANOTHER call's future from inside _execute() without
        # deadlocking against this same node's own main spin.
        #
        # That waiting is done with this module's own _wait_for_future()
        # (plain future.done() polling + time.sleep), deliberately NOT
        # rclpy.spin_once()/spin_until_future_complete(self, ...): those
        # top-level helpers each do their own executor.add_node(self),
        # reassigning this node's own .executor out from under whatever
        # already owns it (main()'s MultiThreadedExecutor here). Called
        # occasionally that's mostly harmless, but _execute() calls one
        # of them on every poll tick of every walk, and the cancellation
        # path fires two more back-to-back -- concurrent re-entrant spins
        # like that are a known way for an executor to lose track of a
        # node's callbacks. Observed effect: the very first walk of a
        # mission always worked, but the walk immediately after the
        # first safety-cutoff-triggered cancel never produced a single
        # piece of feedback again for the rest of that mission.
        # _wait_for_future() never touches .executor at all -- it just
        # polls, so the externally-owned MultiThreadedExecutor remains
        # the only thing that ever actually spins this node.
        self._cb_group = ReentrantCallbackGroup()
        self._follow_path_client = ActionClient(
            self, FollowPath, self.get_parameter("follow_path_action").value,
            callback_group=self._cb_group)
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

        # ploughed/3 -- see _mark_ploughed_here's own docstring. Latched
        # (TRANSIENT_LOCAL) so condition_service_node's own
        # PloughedAt/PloughedBetween handlers get the CURRENT full set
        # immediately on subscribing, regardless of node startup order
        # -- same convention sample_service_node's own sample_values
        # topic already uses.
        ploughed_cells_qos = QoSProfile(depth=1)
        ploughed_cells_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        ploughed_cells_qos.reliability = ReliabilityPolicy.RELIABLE
        self._ploughed_cells_pub = self.create_publisher(
            String, self.get_parameter("ploughed_cells_topic").value,
            ploughed_cells_qos)
        self._ploughed_cells = set()
        self._publish_ploughed_cells()

        self._action_server = ActionServer(
            self, MoveTo, "move_to", self._execute,
            cancel_callback=lambda _goal_handle: CancelResponse.ACCEPT,
            callback_group=self._cb_group)

        self.get_logger().info("move_to_node ready on 'move_to'")

    def _on_tool_state(self, msg):
        self._equipped_tool = msg.data

    def _on_tool_deployed(self, msg):
        self._deployed = msg.data == "true"

    def _publish_ploughed_cells(self):
        self._ploughed_cells_pub.publish(
            String(data=json.dumps(sorted(self._ploughed_cells))))

    def _mark_ploughed_here(self):
        """ploughed/3 (basic_action_theory.pl): marks the macro-cell
        under the robot's own CURRENT position as ploughed. Called
        periodically through a walk (see _execute's own poll loop) --
        NOT problog_project's own bracket-sampled-along-the-nominal-
        spline approach (there is no synthetic noisy spline here, Nav2
        drives the robot for real), so this samples the genuinely
        actual path instead, at the same cadence the poll loop already
        runs at. Caller (_execute) is responsible for only calling this
        while the walk started with the plow both hitched AND deployed
        -- see basic_action_theory.pl's own ploughed/3 note
        (hitch(plow,SPrev), deployed(SPrev)) for why: this method
        itself does not re-check either, so it can also be called once
        more right after a walk ends to catch the final resting cell."""
        xy = self._pose.get_xy()
        if xy is None:
            return
        cell_size = self.get_parameter("plough_cell_size").value
        cell = (cell_index(xy[0], cell_size), cell_index(xy[1], cell_size))
        if cell not in self._ploughed_cells:
            self._ploughed_cells.add(cell)
            self._publish_ploughed_cells()

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
            result.status = False
            return result

        if not self._follow_path_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(
                f"FollowPath action server "
                f"'{self.get_parameter('follow_path_action').value}' unavailable")
            goal_handle.abort()
            result.status = False
            return result

        # See this node's own constructor comment on why -- without a real
        # base_link->odom transform yet, controller_server has no state to
        # run FollowPath against and reports a false-positive instant
        # "Reached the goal!" rather than actually driving anywhere.
        if not self._odom_pose.wait_ready():
            self.get_logger().error(
                "move_to_node: no odom pose after startup timeout, aborting")
            goal_handle.abort()
            result.status = False
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
                result.status = False
                return result
            self.get_logger().warn(
                "FollowPath goal rejected (controller_server likely not "
                "active yet), retrying...")
            time.sleep(0.5)

        get_result_future = follow_path_goal_handle.get_result_async()
        poll_period = self.get_parameter("poll_period_s").value
        max_walk_duration = self.get_parameter("max_walk_duration_s").value
        walk_start = time.monotonic()

        # ploughed/3 (basic_action_theory.pl): TRUE for this leg's own
        # whole span iff the plow was BOTH hitched AND deployed right
        # before this walk started -- checked ONCE here, not re-checked
        # per poll, matching hitch(plow,SPrev)/deployed(SPrev)'s own
        # "provably constant for the whole span of one walk" invariant
        # (tool state can only change between BT leaves, via a
        # sequential InstallTool/DeployTool/etc leaf, never DURING a
        # MoveTo). See _mark_ploughed_here's own docstring for the
        # marking itself -- called here at start, every poll, and once
        # more (in `finally`) after the walk ends, so the leg's start
        # AND final resting cell are both covered even on an early
        # cancel/cutoff/timeout, not just a full completion.
        should_plough = self._deployed and self._equipped_tool == "plow"
        if should_plough:
            self._mark_ploughed_here()

        try:
            safety_tripped = False
            while not get_result_future.done():
                time.sleep(poll_period)
                if should_plough:
                    self._mark_ploughed_here()
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
                    result.status = False
                    return result
                if goal_handle.is_cancel_requested:
                    # bt.cpp's own MoveTo leaf (BT::RosActionNode::halt())
                    # waits for THIS action's own result before a containing
                    # ReactiveSequence/Fallback can move on -- worst case
                    # here is poll_period_s (noticing the cancel) plus
                    # these two timeouts, so keep them tight; Nav2's own
                    # cancel ack/result normally arrive in well under a
                    # second on a local controller_server.
                    cancel_future = follow_path_goal_handle.cancel_goal_async()
                    self._wait_for_future(cancel_future, timeout_sec=2.0)
                    self._wait_for_future(get_result_future, timeout_sec=2.0)
                    goal_handle.canceled()
                    result.status = False
                    return result
                # ALWAYS-ON safety cutoff -- checked every poll (see
                # safety_monitor.py).
                if self._safety.tripped() is not None:
                    safety_tripped = True
                    cancel_future = follow_path_goal_handle.cancel_goal_async()
                    self._wait_for_future(cancel_future, timeout_sec=5.0)
                    self._wait_for_future(get_result_future, timeout_sec=5.0)
                    break

            if safety_tripped:
                goal_handle.succeed()
                result.status = False
                return result

            follow_path_result = get_result_future.result()
            status = follow_path_result.status
            if status == GoalStatus.STATUS_SUCCEEDED:
                goal_handle.succeed()
                result.status = True
            elif status == GoalStatus.STATUS_CANCELED:
                goal_handle.canceled()
                result.status = False
            else:
                goal_handle.abort()
                result.status = False
            return result
        finally:
            if should_plough:
                self._mark_ploughed_here()

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
