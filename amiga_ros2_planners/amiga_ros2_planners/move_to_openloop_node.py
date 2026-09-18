#!/usr/bin/env python3
"""
move_to_openloop_node.py

An OPEN-LOOP alternative backend for MoveTo (amiga_interfaces/action/
MoveTo) -- the SAME action interface move_to_node.py hosts, but never
dials Nav2's controller_server FollowPath, and never reads the robot's
actual position/heading at all (no tf2, no odometry subscription
anywhere in this file). Instead:

  1. Samples control_points into an (x, y, yaw) polyline, exactly like
     move_to_node.py does (bezier.sample_bezier_chain).
  2. Converts that polyline into a sequence of constant (linear_x,
     angular_z) commands, one per consecutive waypoint pair, each
     timed to cover that segment's own distance/yaw change at this
     walk's own tool-appropriate speed (open_loop_trajectory.
     build_velocity_segments) -- a pure feedforward computation, done
     ONCE up front from the plan alone.
  3. Publishes those commands directly to `cmd_vel` at a fixed control
     rate, advancing a purely NOMINAL (dead-reckoned) pose in software
     via open_loop_trajectory.integrate_unicycle_step -- never
     comparing it against where the robot actually ends up. If the
     real robot drifts from the plan (wheel slip, an inflated/actual
     mismatch, anything), this backend has no way to notice or correct
     for it -- that IS "open loop": Nav2's own controller_server runs a
     genuine PURSUIT controller (continuously replanning velocity from
     the actual measured pose vs. the path), which this deliberately
     does not.
  4. While running, still polls an ALWAYS-ON safety cutoff
     (SafetyMonitor -- battery_state/chassis_contact_<side>, see
     safety_monitor.py) and cancels the walk the moment it trips. Note
     that check is against the robot's REAL sensors, not this backend's
     own nominal pose -- "open loop" here describes how the WALK ITSELF
     is driven, not whether a real collision/battery event can still
     stop it.
  5. Also mirrors move_to_node.py's own tool-speed selection (tool_
     state/tool_deployed topics -> tool_speed_<kind>[_deployed]_mps)
     and ploughed-cell marking (ploughing.py) -- using this walk's own
     NOMINAL position for marking, since that's the only position this
     backend ever computes; see _mark_ploughed_here's own docstring.

Same ROS2 action name ("move_to" by default, `move_to_action_name`
param) as move_to_node.py's own MoveTo action -- a drop-in alternative
backend, chosen at launch time (planners.launch.py's own
`move_to_backend` arg: "nav2" (default, move_to_node.py) or "openloop"
(this node)), never both at once against the same action name.

WHY THIS EXISTS: Nav2's controller_server needs a working base_link->
odom transform (from Gazebo's diff_drive_controller) and a live
local_costmap before FollowPath will actually drive anywhere -- real
startup-ordering machinery move_to_node.py already works around (see
its own docstring). This backend needs NONE of that: it only needs
`cmd_vel` to reach the robot at all (this repo's own /cmd_vel ->
diff_drive_controller/cmd_vel_unstamped bridge, sim_twist_control.py),
so it can drive a mission before Nav2 has finished coming up, or in a
setup with no Nav2 stack running at all. The tradeoff is exactly what
"open loop" always costs: no correction for real-world drift, ever.

Caveat: not exercised against a live Gazebo/ros2_control diff_drive_
controller in this session (no ROS2 environment available here) --
open_loop_trajectory.py's own math (segment timing, unicycle
integration) is unit-tested and verified in isolation; the cmd_vel
publishing/action-server plumbing itself should be smoke-tested against
your own sim before relying on it, same caveat move_to_node.py's own
docstring already carries for FollowPath.

Same interface as move_to_node.py: takes only the input needed to
execute the walk (control_points) and reports only success/failure
(`status`) -- see MoveTo.action's own header.
"""
import json
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

from amiga_interfaces.action import MoveTo
from amiga_ros2_planners.bezier import sample_bezier_chain
from amiga_ros2_planners.open_loop_trajectory import (
    build_velocity_segments, integrate_unicycle_step,
)
from amiga_ros2_planners.ploughing import cell_index
from amiga_ros2_planners.safety_monitor import SafetyMonitor


class MoveToOpenLoopNode(Node):
    def __init__(self):
        super().__init__("move_to_openloop_node")

        self.declare_parameter("move_to_action_name", "move_to")
        self.declare_parameter("cmd_vel_topic", "cmd_vel")
        self.declare_parameter("samples_per_segment", 10)
        self.declare_parameter("control_period_s", 0.1)
        # Rotation-rate cap for build_velocity_segments -- see that
        # function's own docstring (also bounds a pure "spin toward the
        # next waypoint's own heading" segment's own speed, since
        # nothing else constrains angular_z otherwise).
        self.declare_parameter("max_angular_speed_rps", 1.0)
        # Same names/defaults as move_to_node.py's own tool_speed_*
        # params, so a launch file can hand this node the SAME values
        # with no translation needed.
        self.declare_parameter("tool_speed_free_mps", 0.5)
        self.declare_parameter("tool_speed_cart_mps", 0.5)
        self.declare_parameter("tool_speed_plow_mps", 0.5)
        self.declare_parameter("tool_speed_cart_deployed_mps", 0.5)
        self.declare_parameter("tool_speed_plow_deployed_mps", 0.5)
        self.declare_parameter("tool_state_topic", "tool_state")
        self.declare_parameter("tool_deployed_topic", "tool_deployed")
        # Same meaning/default as move_to_node.py's own -- MUST match
        # condition_service_node's own plough_cell_size.
        self.declare_parameter("plough_cell_size", 1.0)
        self.declare_parameter("ploughed_cells_topic", "ploughed_cells")

        # ALWAYS-ON safety cutoff -- see safety_monitor.py's own module
        # docstring / move_to_node.py's own identical params for the
        # full rationale.
        self.declare_parameter("battery_topic", "battery_state")
        self.declare_parameter("battery_depleted_threshold_pct", 0.0)
        self.declare_parameter("contact_topic_front", "chassis/contact_front")
        self.declare_parameter("contact_topic_back", "chassis/contact_back")
        self.declare_parameter("contact_topic_left", "chassis/contact_left")
        self.declare_parameter("contact_topic_right", "chassis/contact_right")
        self.declare_parameter("contact_stale_after_s", 0.5)

        self._cb_group = ReentrantCallbackGroup()
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
        self._cmd_vel_pub = self.create_publisher(
            Twist, self.get_parameter("cmd_vel_topic").value, 10)

        # "free"/False are tool_action_node's own initial published
        # values -- see move_to_node.py's own identical constructor
        # comment for why these particular defaults matter even before
        # this node's own subscriptions receive their first message.
        self._equipped_tool = "free"
        self.create_subscription(
            String, self.get_parameter("tool_state_topic").value,
            self._on_tool_state, 10, callback_group=self._cb_group)
        self._deployed = False
        self.create_subscription(
            String, self.get_parameter("tool_deployed_topic").value,
            self._on_tool_deployed, 10, callback_group=self._cb_group)

        # ploughed/3 -- see move_to_node.py's own identical publisher
        # for the full rationale (this node's own copy of the SAME
        # ploughed_cells topic, since only one MoveTo backend runs at a
        # time -- see this module's own docstring).
        ploughed_cells_qos = QoSProfile(depth=1)
        ploughed_cells_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        ploughed_cells_qos.reliability = ReliabilityPolicy.RELIABLE
        self._ploughed_cells_pub = self.create_publisher(
            String, self.get_parameter("ploughed_cells_topic").value,
            ploughed_cells_qos)
        self._ploughed_cells = set()
        self._publish_ploughed_cells()

        self._action_server = ActionServer(
            self, MoveTo, self.get_parameter("move_to_action_name").value, self._execute,
            cancel_callback=lambda _goal_handle: CancelResponse.ACCEPT,
            callback_group=self._cb_group)

        self.get_logger().info(
            f"move_to_openloop_node ready on "
            f"'{self.get_parameter('move_to_action_name').value}' (open-loop, no Nav2)")

    def _on_tool_state(self, msg):
        self._equipped_tool = msg.data

    def _on_tool_deployed(self, msg):
        self._deployed = msg.data == "true"

    def _publish_ploughed_cells(self):
        self._ploughed_cells_pub.publish(
            String(data=json.dumps(sorted(self._ploughed_cells))))

    def _mark_ploughed(self, x, y):
        """ploughed/3 -- see move_to_node.py's own _mark_ploughed_here
        docstring for the full rationale. Marks against the NOMINAL
        (dead-reckoned) position, since that is the only position this
        open-loop backend ever computes -- unlike move_to_node.py's own
        version, which marks against the robot's REAL tf2 pose."""
        cell_size = self.get_parameter("plough_cell_size").value
        cell = (cell_index(x, cell_size), cell_index(y, cell_size))
        if cell not in self._ploughed_cells:
            self._ploughed_cells.add(cell)
            self._publish_ploughed_cells()

    def _resolve_speed(self):
        """This walk's own tool-appropriate linear speed -- same
        selection logic as move_to_node.py's own _apply_tool_speed,
        just returned as a plain float instead of pushed into a Nav2
        controller_server parameter (there is no controller_server
        here)."""
        if self._deployed and self._equipped_tool in ("cart", "plow"):
            return self.get_parameter(
                f"tool_speed_{self._equipped_tool}_deployed_mps").value
        return {
            "free": self.get_parameter("tool_speed_free_mps").value,
            "cart": self.get_parameter("tool_speed_cart_mps").value,
            "plow": self.get_parameter("tool_speed_plow_mps").value,
        }.get(self._equipped_tool, self.get_parameter("tool_speed_free_mps").value)

    @staticmethod
    def _wait_for_future(future, timeout_sec, poll_interval_s=0.02):
        """See move_to_node.py's own _wait_for_future -- identical
        rationale (never spin this node re-entrantly from inside an
        ActionServer execute callback running under main()'s own
        MultiThreadedExecutor)."""
        deadline = time.monotonic() + timeout_sec
        while not future.done():
            if time.monotonic() >= deadline:
                return False
            time.sleep(poll_interval_s)
        return True

    def _publish_cmd_vel(self, linear_x, angular_z):
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self._cmd_vel_pub.publish(twist)

    def _stop(self):
        self._publish_cmd_vel(0.0, 0.0)

    def _execute(self, goal_handle):
        goal = goal_handle.request
        result = MoveTo.Result()

        control_points = [(p.x, p.y) for p in goal.control_points]
        try:
            waypoints = sample_bezier_chain(
                control_points, self.get_parameter("samples_per_segment").value)
        except ValueError as exc:
            self.get_logger().error(f"bad control_points: {exc}")
            goal_handle.abort()
            result.status = False
            return result

        speed = self._resolve_speed()
        max_angular = self.get_parameter("max_angular_speed_rps").value
        segments = build_velocity_segments(waypoints, speed, max_angular)

        final_x, final_y, _final_yaw = waypoints[-1]
        should_plough = self._deployed and self._equipped_tool == "plow"
        x, y, yaw = waypoints[0]
        if should_plough:
            self._mark_ploughed(x, y)

        if not segments:
            # Degenerate "already there" chain -- nothing to command.
            goal_handle.succeed()
            result.status = True
            return result

        control_period = self.get_parameter("control_period_s").value

        segment_idx = 0
        segment_elapsed = 0.0
        last_tick = time.monotonic()

        try:
            while segment_idx < len(segments):
                time.sleep(control_period)

                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.status = False
                    return result
                # ALWAYS-ON safety cutoff (see safety_monitor.py).
                if self._safety.tripped() is not None:
                    goal_handle.succeed()
                    result.status = False
                    return result

                now = time.monotonic()
                remaining_dt = now - last_tick
                last_tick = now

                # Advance the NOMINAL pose across however many segment
                # boundaries this one tick spans (a coarse
                # control_period relative to a short segment shouldn't
                # silently lose distance -- see this module's own
                # docstring on why exact multi-segment consumption
                # matters for feedback/ploughing accuracy).
                while remaining_dt > 1.0e-9 and segment_idx < len(segments):
                    duration, linear_x, angular_z = segments[segment_idx]
                    remaining_in_segment = duration - segment_elapsed
                    step = min(remaining_dt, remaining_in_segment)
                    x, y, yaw = integrate_unicycle_step(x, y, yaw, linear_x, angular_z, step)
                    segment_elapsed += step
                    remaining_dt -= step
                    if segment_elapsed >= duration - 1.0e-9:
                        segment_idx += 1
                        segment_elapsed = 0.0

                if should_plough:
                    self._mark_ploughed(x, y)

                if segment_idx < len(segments):
                    _duration, linear_x, angular_z = segments[segment_idx]
                    self._publish_cmd_vel(linear_x, angular_z)

                fb = MoveTo.Feedback()
                fb.distance_to_goal = ((final_x - x) ** 2 + (final_y - y) ** 2) ** 0.5
                goal_handle.publish_feedback(fb)

            if should_plough:
                self._mark_ploughed(final_x, final_y)
            goal_handle.succeed()
            result.status = True
            return result
        finally:
            self._stop()


def main(args=None):
    rclpy.init(args=args)
    node = MoveToOpenLoopNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
